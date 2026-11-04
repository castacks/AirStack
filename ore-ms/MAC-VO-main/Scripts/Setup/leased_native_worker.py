"""Native expert with mapped shared encoded lease released before acknowledgment."""
import contextlib
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import time
import traceback


def worker(camera,options,pipe):
    from Scripts.Setup.leased_transport import BoundedConnection,mapped_payload,lease_identity
    pipe=BoundedConnection(pipe)
    output=Path(options['output'])/'experts'/camera
    output.mkdir(parents=True,exist_ok=False)
    with (output/'worker.log').open('x',buffering=1) as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        system=None;recorder=None;uncertainty=None;processed=[];calls=[];phases=[];report={};context={'source':None,'group':None}
        try:
            import numpy as np
            import torch
            from Scripts.Setup import run_td3_execution_v3 as r
            from Scripts.Setup.arbiter_event_recorder import ArbiterEventRecorder
            from Scripts.Setup.td3_uncertainty import UncertaintyRecorder
            from Scripts.Setup.sparse_feedback import execute_request
            torch.manual_seed(0);np.random.seed(0)
            sequence=r.TD3ManifestSequence(Path(options['dataset']),Path(options['manifest']),camera,acknowledge_provisional_geometry=True)
            rows=sequence.rows;assert len(rows)==options['frames']
            _,config=r.load_config(r.ROOT/'Config/Experiment/MACVO/MACVO_Performant.yaml')
            config={'Odometry':config['Odometry']};odom=config['Odometry']
            odom['args'].update(mapping=False,profile=False);odom['frontend']['type']='FlowFormerCovFrontend'
            checkpoint=r.ROOT.parent/'models/macslam'/('MACVO_FrontendCov.pth' if camera=='rgb' else 'MACT_Offroad.pth')
            odom['frontend']['args'].update(weight=str(checkpoint),enc_dtype='fp32',dec_dtype='fp32')
            odom['optimizer']['args']['parallel']=False
            (output/'effective-config.json').write_text(json.dumps(config,indent=2)+'\n')
            report={'started_at_utc':datetime.now(timezone.utc).isoformat(),'modality':camera,'status':'RUNNING',
                    'coordinate_frame':'rectified left camera NED; identity body/sensor transform',
                    'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                    'torch':torch.__version__,'gpu':torch.cuda.get_device_name(0)}
            system=r.MACVO.from_config(r.asNamespace(config))
            def measure_frontend(name):
                original=getattr(system.Frontend,name)
                def measured(*args,**kwargs):
                    torch.cuda.synchronize();start=time.perf_counter_ns();status='FAILED'
                    try:
                        result=original(*args,**kwargs);status='PASS';return result
                    finally:
                        torch.cuda.synchronize();end=time.perf_counter_ns()
                        calls.append({'kind':'frontend_'+('depth' if name=='estimate_depth' else 'pair'),
                                      'start_ns':start,'end_ns':end,'duration_ns':end-start,'status':status})
                setattr(system.Frontend,name,measured)
            measure_frontend('estimate_depth');measure_frontend('estimate_pair')
            uncertainty=UncertaintyRecorder(system,output/'uncertainty')
            class StreamingRecorder(ArbiterEventRecorder):
                def _emit(self,kind,**fields):
                    super()._emit(kind,**fields)
                    pipe.send({'kind':'observer_event','event':json.loads(self.events[-1]),'invocation_source':context['source'],'delivery_group':context['group']})
            recorder=StreamingRecorder(system,output/'arbiter-events',camera,record_root_initialization=True)
            first=system.Frontend.estimate_depth(sequence[0].stereo)
            depth=first.depth.detach().cpu().numpy();covariance=first.cov.detach().cpu().numpy()
            valid=np.isfinite(depth)&(depth>0)&np.isfinite(covariance)&(covariance>0)
            if not valid.any():raise RuntimeError('no finite positive first-depth samples')
            np.savez_compressed(output/'first-depth.npz',depth=depth,covariance=covariance,valid=valid)
            for source in range(len(rows)):
                pipe.send({'kind':'ready','source':source})
                command=pipe.recv()
                if not isinstance(command,tuple) or len(command)!=4 or command[:2]!=('step',source) or type(command[2]) is not bool:
                    raise RuntimeError('invalid source-group batch command')
                requested,descriptor=command[2],command[3]
                if source==0 and not requested:raise ValueError('native root cannot be skipped')
                with mapped_payload(descriptor,camera,source) as payload:
                    batch=[]
                    if payload is not None:
                        if not requested or payload['source']!=source-1 or not processed or payload['source']<=processed[-1]:
                            raise ValueError('invalid retained native ordering')
                        batch.append((payload['source'],payload))
                    if requested:batch.append((source,None))
                    for invocation,encoded in batch:
                        if processed and invocation<=processed[-1]:raise ValueError('native source order must strictly increase')
                        context.update(source=invocation,group=source)
                        if encoded is None:
                            frame=sequence[invocation];frame.idx=[len(processed)]
                        else:
                            from Scripts.Setup.decode_retained import retained_frame
                            frame=retained_frame(sequence,invocation,encoded,len(processed))
                        torch.cuda.synchronize();start=time.perf_counter_ns();status='FAILED'
                        try:system.run(frame);status='PASS'
                        finally:
                            torch.cuda.synchronize();end=time.perf_counter_ns()
                            phases.append({'kind':'run','source':invocation,'delivery_group':source,
                                           'buffered':encoded is not None,'sensor_time_ns':int(frame.time_ns[0]),
                                           'start_ns':start,'end_ns':end,'duration_ns':end-start,'status':status})
                        processed.append(invocation)
                pipe.send({'kind':'completed','source':source,'executed':requested,'buffered_count':int(descriptor is not None),'processed_count':len(processed),'lease_ack':lease_identity(descriptor)})
            pipe.send({'kind':'drain_ready'})
            if pipe.recv()!=('drain',):raise RuntimeError('unexpected drain command')
            context.update(source=None,group=None)
            before=system.graph.frames.data['need_interp'].tensor.detach().cpu().numpy().copy()
            torch.cuda.synchronize();start=time.perf_counter_ns();system.terminate();torch.cuda.synchronize();end=time.perf_counter_ns()
            phases.append({'kind':'terminate','start_ns':start,'end_ns':end,'duration_ns':end-start,'status':'PASS'})
            after=system.graph.frames.data['need_interp'].tensor.detach().cpu().numpy().copy()
            uncertainty.finish()
            np.savez_compressed(output/'tracking-flags.npz',before_refinement=before,after_refinement=after)
            poses=system.graph.frames.data['pose'].tensor.detach().cpu().numpy()
            times=system.graph.frames.data['time_ns'].tensor.detach().cpu().numpy().astype(np.int64)
            if poses.shape!=(len(processed),7) or not np.isfinite(poses).all():raise RuntimeError('incomplete/nonfinite sparse trajectory')
            if not np.allclose(np.linalg.norm(poses[:,3:],axis=1),1.,atol=1e-5):raise RuntimeError('nonunit trajectory quaternion')
            expected=[rows[i][camera+'_left']['time_ns'] for i in processed]
            if times.reshape(-1).tolist()!=expected:raise RuntimeError('sparse trajectory source-time mismatch')
            np.savez_compressed(output/'tensor-map.npz',**system.graph.serialize())
            np.savez_compressed(output/'trajectory.npz',poses_camera_ned=poses,time_ns=times)
            mapping={'camera':camera,'source_indices':processed,'source_frame_count':len(rows),
                     'input_rows':[rows[i] for i in processed]}
            (output/'source-mapping.json').write_text(json.dumps(mapping,indent=2)+'\n')
            matches=int(system.graph.match.index.tensor.numel());report['retained_matches']=matches
            if not matches:raise RuntimeError('no visual matches retained')
            report.update(status='PASS_EXECUTION_ONLY',completed_frames=len(processed),source_frames=len(rows))
            pipe.send({'kind':'finished','processed_count':len(processed)})
        except BaseException as error:
            report.update(status='FAILED',error_type=type(error).__name__,error=str(error))
            pipe.send({'kind':'worker_error','traceback':traceback.format_exc()})
        finally:
            try:
                if system is not None and not system.terminated:system.terminate()
                if recorder is not None:report['arbiter_event_recording']=recorder.finish()
                (output/'native-cost.json').write_text(json.dumps({'frontend_calls':calls,'native_phases':phases,'processed_source_indices':processed},indent=2)+'\n')
                (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            finally:
                (output/'control-channel.json').write_text(json.dumps(pipe.report(),indent=2)+'\n')
                pipe.close()
