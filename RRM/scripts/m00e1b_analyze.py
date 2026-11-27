#!/usr/bin/env python3
"""Offline interpretations of E1B passive evidence; no runtime APIs or authority."""
from decimal import Decimal
from datetime import datetime,timezone
import argparse
import json
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1B'

def start_ticks(text):
    # comm may contain spaces/parentheses: field22 is index19 after the final ') '.
    end=text.rfind(') ')
    if end<0:raise ValueError('Invalid proc stat record')
    fields=text[end+2:].split()
    if len(fields)<20 or not fields[19].isdigit():raise ValueError('Missing process start ticks')
    return int(fields[19])

def derive(discovery,artifacts,host):
    if not isinstance(host['clock_ticks_per_second'],int) or isinstance(host['clock_ticks_per_second'],bool) or host['clock_ticks_per_second']<=0:raise ValueError('Invalid tick rate')
    all_processes=[(c,p) for c in discovery['containers'] for p in c['processes']]
    def one(comm):
        rows=[(c,p) for c,p in all_processes if p['comm']==comm]
        if len(rows)!=1:raise ValueError('Missing/ambiguous '+comm)
        return rows[0]
    cpx,px=one('px4');cri,ri=one('robot_interface')
    ticks=start_ticks(px['stat_raw'])-start_ticks(ri['stat_raw'])
    if ticks<0:raise ValueError('Reversed expected birth ordering')
    # Namespace membership and host proc supply one host boot; this is not UTC mapping.
    if not discovery['host'].get('boot_id'):raise ValueError('Missing common host boot')
    bindings=[]
    for c in discovery['containers']:
        owners={}
        for p in artifacts['process_artifacts']:
            if p['container']!=c['name']:continue
            for fd in p.get('selected_fd_targets',[]):
                match=re.fullmatch(r'socket:\[(\d+)\]',fd['target'])
                if match:owners.setdefault(match[1],[]).append(dict(pid=p['host_pid'],comm=p['comm'],fd=fd['fd']))
        for table in c['socket_tables']:
            for s in table.get('entries',[]):
                if s['local_port'] in {4561,14581,14541,49100} or s['remote_port']==4561:
                    bindings.append(dict(container=c['name'],protocol=table['protocol'],socket=s,owners=owners.get(s['inode'],[]),limitation='Instantaneous namespace socket/FD metadata; no payload inspected, source/remote peer or actuation safety not authenticated'))
    tty={c['name']:[d['name'] for d in c['devices_metadata'].get('entries',[]) if re.fullmatch(r'ttyS\d+',d['name'])] for c in discovery['containers']}
    lines=cpx['existing_log_excerpt']['selected_lines']
    event_times=[]
    for needle in ['Simulation App Startup Complete','PX4 node at']:
        matching=[line for line in lines if needle in line]
        if len(matching)!=1:break
        match=re.match(r'^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.([0-9]{9})Z ',matching[0])
        if not match:raise ValueError('Invalid existing-log receipt timestamp')
        second=int(datetime.strptime(match[1],'%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc).timestamp())
        event_times.append(Decimal(second)+Decimal('0.'+match[2]))
    log_delta=None
    if len(event_times)==2:
        if event_times[1]<event_times[0]:raise ValueError('Reversed existing-log events')
        log_delta=str(event_times[1]-event_times[0])
    return dict(target_designation='UNVERIFIED',startup_trials_executed=0,whole_unresolved_prerequisites_closed=[],
                newly_read_existing_log_app_to_graph_s=log_delta,
                existing_log_scope='New read of old Docker receipt events, not a new episode; missing/ambiguous tail events remain null',
                observed_process_birth_separation_s=str(Decimal(ticks)/Decimal(host['clock_ticks_per_second'])),
                process_birth_scope='Current host proc start ticks, same boot; no wall-clock readiness duration or new startup episode',
                serial_node_metadata=tty,socket_ownership=bindings,simulator_isolation='UNVERIFIED',live_instrumentation_ready=False,execution_authorized=False,
                interpretation='Metadata establishes current configuration/exposure and selected installed artifact identity, not runtime safety, connected physical hardware, mission state or qualification')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');args=p.parse_args()
    result=derive(*[json.loads((E/name).read_text()) for name in ['passive-discovery.json','installed-artifacts.json','host-and-lifecycle.json']])
    text=json.dumps(result,indent=2)+'\n';out=E/'discovery-findings.json'
    if args.check:
        if out.read_text()!=text:raise ValueError('Discovery interpretation not reproducible')
    else:
        with out.open('x') as f:f.write(text)
    print(json.dumps({'observed_birth_separation_s':result['observed_process_birth_separation_s'],'closed_whole_prerequisites':0,'target_designation':'UNVERIFIED','execution_authorized':False}))
if __name__=='__main__':main()
