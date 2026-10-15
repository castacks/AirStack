#!/usr/bin/env python3
"""Frozen semantic development probe on a private warm worker. No robot dispatch."""
import argparse
import base64
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import sys
import time
from urllib.request import Request, urlopen
import uuid
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from rrm.cosmos_reason2 import _extract_json_object, parse_cosmos_candidate
from rrm.cosmos_worker_client import CosmosWorkerClient
from rrm.intent_grounding_fixtures import specification
from rrm.intent_resolution import resolve_task_intent
from rrm_cosmos_reason2 import load_context_payload


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def sha(value):
    return hashlib.sha256(value).hexdigest()


def save(path,value):
    with path.open('xb') as stream:stream.write(canonical(value)+b'\n')


def neutral_png():
    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    pixels=(b'\0'+b'\x80\x80\x80'*480)*300
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',480,300,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(pixels))+chunk(b'IEND',b'')


def build_request(case, image, index):
    # Whitelist only context and transport fields: no adjudication, candidate scope,
    # expected goal or independent resolver result is sent to the model.
    return dict(cycle_id=uuid.uuid4().hex,step_index=index,observation_sha256=sha(image),
                context=json.loads(canonical(case['context'])),image_base64=base64.b64encode(image).decode())


def assess(case,response):
    context=load_context_payload(case['context'])
    resolution=resolve_task_intent(context,candidate_entity_ids=tuple(case['candidate_entity_ids']),scope_ref=case['scope_ref'])
    raw=response.get('candidate',{}).get('raw_response') if isinstance(response.get('candidate'),dict) else None
    valid_text=isinstance(raw,str) and bool(raw.strip())
    local=parse_cosmos_candidate(raw,context) if valid_text else None
    local_status=local.status.value if local else 'MALFORMED'
    try:declared=_extract_json_object(raw) if valid_text else {}
    except (ValueError,TypeError):declared={}
    goal=declared.get('grounded_goal')
    if not isinstance(goal,dict):goal={}
    status=declared.get('status')
    expected=case['adjudication']
    if expected['status']=='READY':
        actions=declared.get('actions',[])
        match=(status=='READY' and local_status=='ACCEPTED'
            and goal.get('name')=='near' and goal.get('subject')=='$self'
            and goal.get('obj')==expected['target'] and goal.get('negated',False) is False
            and bool(actions) and all(a.get('verb')=='NAVIGATE_TO' and a.get('targets')==[expected['target']]
                                       for a in actions if isinstance(a,dict)))
    elif expected['status']=='NEEDS_CLARIFICATION':
        refs=declared.get('ambiguity_refs')
        match=(status=='NEEDS_CLARIFICATION' and local_status=='NEEDS_CLARIFICATION'
               and isinstance(refs,list) and bool(refs) and all(isinstance(x,str) and x.strip() for x in refs))
    else:
        match=status=='UNSUPPORTED' and local_status=='UNSUPPORTED'
    bound=None
    if resolution.binding is not None:
        bound=parse_cosmos_candidate(raw,replace(context,intent_binding=resolution.binding)) if valid_text else None
        pipeline='PROPOSAL' if bound and bound.status.value=='ACCEPTED' else 'HELD_MODEL_MISMATCH'
    else:pipeline=resolution.status
    return dict(reference_status=resolution.status,reference_reasons=resolution.reasons,
        reference_target=resolution.binding.grounded_goal.obj if resolution.binding else None,
        binding=resolution.binding.model_dump(mode='json') if resolution.binding else None,
        reference_matches_label=(resolution.status==expected['status'] and
            (resolution.binding.grounded_goal.obj if resolution.binding else None)==expected['target']),
        declared_model_status=status,declared_model_target=goal.get('obj'),local_schema_status=local_status,
        local_reasons=local.reasons if local else ('missing_model_text',),model_matches_authored_intent=bool(match),pipeline_status=pipeline,
        guarded_candidate=bound.model_dump(mode='json') if bound else None,execution_dispatch=False)


def source_identity():
    paths=sorted((ROOT/'rrm').glob('*.py'))+[Path(__file__),ROOT/'scripts/rrm_cosmos_reason2.py']
    return {p.relative_to(ROOT).as_posix():sha(p.read_bytes()) for p in paths}


def rpc(url,body=None):
    request=Request(url,data=canonical(body) if body is not None else None,
                    headers={'Content-Type':'application/json','Accept':'application/json'})
    with urlopen(request,timeout=60) as response:
        raw=response.read(4*1024*1024+1)
        if len(raw)>4*1024*1024:raise ValueError('worker_response_too_large')
        if response.status!=200:raise ValueError('worker_status_not_ok')
    return json.loads(raw)


def validate_response(request,response):
    if response.get('execution_dispatch') is not False:raise ValueError('worker_dispatch_contract')
    for name in ('cycle_id','step_index','observation_sha256'):
        if type(response.get(name)) is not type(request[name]) or response.get(name)!=request[name]:
            raise ValueError('worker_response_identity_mismatch')


def summarise(records):
    completed=[r for r in records if r['status']=='RESPONDED']
    groups={}
    for expected in ('READY','NEEDS_CLARIFICATION','UNSUPPORTED'):
        selected=[r for r in records if r['adjudication']['status']==expected]
        answered=[r for r in selected if r['status']=='RESPONDED']
        groups[expected]=dict(scheduled=len(selected),responded=len(answered),
            model_matches=sum(r['assessment']['model_matches_authored_intent'] for r in answered))
    return dict(scheduled=len(records),responded=len(completed),
        errors=sum(r['status']=='ERROR' for r in records),not_run=sum(r['status']=='NOT_RUN' for r in records),
        reference_matches=sum(r['assessment']['reference_matches_label'] for r in completed),
        model_matches=sum(r['assessment']['model_matches_authored_intent'] for r in completed),
        groups=groups,model_schema_accepted=sum(r['assessment']['local_schema_status']=='ACCEPTED' for r in completed),
        guarded_proposals=sum(r['assessment']['pipeline_status']=='PROPOSAL' for r in completed),
        held_model_mismatches=sum(r['assessment']['pipeline_status']=='HELD_MODEL_MISMATCH' for r in completed),
        clarification_holds=sum(r['assessment']['pipeline_status']=='NEEDS_CLARIFICATION' for r in completed),
        unsupported_holds=sum(r['assessment']['pipeline_status']=='UNSUPPORTED' for r in completed))


def run(output,url):
    CosmosWorkerClient(url,timeout_s=60)  # Reuse URL constraints; no live-cycle/robot method is invoked.
    output.mkdir(parents=True,exist_ok=False)
    source=source_identity()
    corpus=specification()
    save(output/'corpus.json',corpus)
    image=neutral_png()
    with (output/'neutral.png').open('xb') as stream:stream.write(image)
    capabilities=rpc(url+'/v1/capabilities')
    save(output/'worker-before.json',capabilities)
    if capabilities.get('execution_dispatch') is not False or '/v1/propose' not in capabilities.get('routes',[]):
        raise ValueError('worker_not_proposal_only')
    records=[]
    stop_requests=False
    for index,case in enumerate(corpus['cases']):
        folder=output/case['case_id']
        folder.mkdir()
        request=build_request(case,image,index)
        save(folder/'request.json',request)
        record=dict(case_id=case['case_id'],adjudication=case['adjudication'],request_sha256=sha(canonical(request)),
                    execution_dispatch=False)
        if stop_requests:
            record.update(status='NOT_RUN',reason='stopped_after_transport_or_identity_error')
        else:
            started=time.monotonic()
            try:
                response=rpc(url+'/v1/propose',request)
                save(folder/'response.json',response)
                validate_response(request,response)
                assessment=assess(case,response)
                record.update(status='RESPONDED',assessment=assessment,
                              worker_reported_inference_wall_s=response.get('inference_wall_s'))
            except Exception as error:
                record.update(status='ERROR',error_type=type(error).__name__)
                stop_requests=True  # No retry or queue of further inference after uncertain transport state.
            record['rpc_and_assessment_wall_s']=time.monotonic()-started
        save(folder/'assessment.json',record)
        records.append(record)
        print(json.dumps(dict(case=case['case_id'],status=record['status'],
            assessment={k:v for k,v in record.get('assessment',{}).items()
                        if k in ('reference_status','declared_model_status','declared_model_target','local_schema_status','pipeline_status','model_matches_authored_intent')})),flush=True)
    after=rpc(url+'/v1/capabilities')
    save(output/'worker-after.json',after)
    report=dict(written_at_utc=datetime.now(timezone.utc).isoformat(),corpus_sha256=sha(canonical(corpus)),
        scope='REAL_WORKER_SYNTHETIC_SEMANTIC_DEVELOPMENT',execution_dispatch=False,
        model_identity_attested=False,semantic_prompt_identity_attested=False,generation_settings_attested=False,
        source_unchanged=source==source_identity(),worker_capabilities_unchanged=canonical(capabilities)==canonical(after),
        summary=summarise(records),records=records)
    save(output/'report.json',report)
    save(output/'manifest.json',dict(source=source,artifacts={p.relative_to(output).as_posix():sha(p.read_bytes())
        for p in sorted(output.rglob('*')) if p.is_file()}))
    print(json.dumps(report['summary']),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--worker-url',required=True)
    args=parser.parse_args()
    run(args.output,args.worker_url.rstrip('/'))


if __name__=='__main__':main()
