"""Bounded observation-only PhysX contact reports for UAV fixture qualification.
API: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/physics/rb_physics.html
"""
import json,math,time,hashlib
from pathlib import Path


class UavContactCapture:
    def __init__(self,directory):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.file=(self.directory/'contacts.jsonl').open('x',buffering=1)
        self.source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.subscription=None;self.bodies=[];self.count=0;self.error=None;self.next_status=0.;self.sim_time=None;self.callbacks=0;self.steps=0;self.last_callback=None;self.metadata_done=False;self.metadata_attempts=0;self.metadata_error=None

    def step(self,stage,sim_time):
        self.sim_time=sim_time;self.steps+=1
        try:
            if self.subscription is None:
                from pxr import PhysxSchema,UsdPhysics,Usd
                from omni.physx import get_physx_simulation_interface
                root=stage.GetPrimAtPath('/World/base_link')
                if root:
                    for prim in Usd.PrimRange(root):
                        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            api=PhysxSchema.PhysxContactReportAPI.Apply(prim)
                            api.CreateThresholdAttr().Set(0.)
                            self.bodies.append(str(prim.GetPath()))
                if self.bodies:self.subscription=get_physx_simulation_interface().subscribe_contact_report_events(self.report)
        except Exception as e:self.error=repr(e)
        try:
            if self.subscription is not None and not self.metadata_done and self.metadata_attempts<50:
                self.metadata_attempts+=1
                try:
                    from pxr import Usd,UsdGeom,UsdPhysics
                    from omni.isaac.dynamic_control import _dynamic_control
                    dc=_dynamic_control.acquire_dynamic_control_interface();body_values=[]
                    for path in sorted(set(self.bodies)):
                        handle=dc.get_rigid_body(path)
                        if not handle:raise ValueError('rigid-body handle unavailable: '+path)
                        props=dc.get_rigid_body_properties(handle)
                        body_prim=stage.GetPrimAtPath(path)
                        attrs={a.GetName():str(a.Get()) for a in body_prim.GetAttributes() if any(k in a.GetName().lower() for k in ('gravity','kinematic','rigidbody','mass'))}
                        local_com=getattr(props,'cMassLocalPose',None)
                        body_values.append(dict(path=path,center_of_mass_local_m=[float(getattr(getattr(local_com,'p',local_com),c)) for c in 'xyz'] if local_com is not None else None,mass_kg=float(props.mass),diagonal_inertia_kg_m2=[float(getattr(props.moment,c)) for c in 'xyz'],usd_attributes=attrs))
                    colliders=[]
                    cache=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide])
                    for prim in stage.Traverse():
                        if prim.HasAPI(UsdPhysics.CollisionAPI):
                            attrs={a.GetName():dict(value=str(a.Get()),authored=a.HasAuthoredValueOpinion()) for a in prim.GetAttributes() if any(k in a.GetName().lower() for k in ('collision','offset','approximation'))}
                            bound=cache.ComputeWorldBound(prim).ComputeAlignedRange()
                            colliders.append(dict(path=str(prim.GetPath()),attributes=attrs,usd_world_bounds_min=list(bound.GetMin()),usd_world_bounds_max=list(bound.GetMax())))
                    value=dict(wall_unix=time.time(),sim_time_s=sim_time,body_properties=body_values,total_mass_kg=sum(v['mass_kg'] for v in body_values),
                               colliders=colliders,scope='live body properties; USD bounds may be stale under Fabric and are not contact separations')
                    (self.directory/'physics-metadata.json').write_text(json.dumps(value,indent=2));self.metadata_done=True;self.metadata_error=None
                except Exception as e:
                    self.metadata_error=repr(e)
                    (self.directory/'metadata-error.json').write_text(json.dumps(dict(wall_unix=time.time(),error=repr(e),attempts=self.metadata_attempts)))
            if time.monotonic()>=self.next_status:
                self.next_status=time.monotonic()+1
                states=[]
                try:
                    from omni.isaac.dynamic_control import _dynamic_control
                    dc=_dynamic_control.acquire_dynamic_control_interface()
                    for path in sorted(set(self.bodies)):
                        pose=dc.get_rigid_body_pose(dc.get_rigid_body(path))
                        states.append(dict(path=path,position_m=[float(getattr(pose.p,c)) for c in 'xyz'],quaternion_xyzw=[float(getattr(pose.r,c)) for c in 'xyzw']))
                except Exception as e:states=[dict(error=repr(e))]
                value=dict(body_states=states,source_sha256=self.source_sha256,wall_unix=time.time(),sim_time_s=sim_time,subscribed=self.subscription is not None,
                           bodies=sorted(set(self.bodies)),contact_headers=self.count,error=self.error,limit_reached=self.count>=30000,
                           metadata_complete=self.metadata_done,metadata_error=self.metadata_error,post_steps=self.steps,callback_invocations=self.callbacks,last_callback_wall=self.last_callback)
                p=self.directory/'status.json.tmp';p.write_text(json.dumps(value,indent=2));p.replace(self.directory/'status.json')
        except Exception as e:self.error=repr(e)

    def report(self,headers,data):
        self.callbacks+=1;self.last_callback=time.time()
        try:
            from pxr import PhysicsSchemaTools
            for h in headers:
                if self.count>=30000:return
                v=dict(callback_index=self.callbacks,wall_unix=time.time(),sim_time_s_last_post_step=self.sim_time,event_type=int(h.type),event_name=str(h.type),
                       actor0=str(PhysicsSchemaTools.intToSdfPath(h.actor0)),actor1=str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
                       collider0=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),collider1=str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
                       contact_count=int(h.num_contact_data))
                impulses=[]
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    impulses.append(math.sqrt(sum(float(x)**2 for x in data[i].impulse)))
                v['separations_m']=[float(data[i].separation) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                v['positions_m']=[[float(x) for x in data[i].position] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                v['normals']=[[float(x) for x in data[i].normal] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                v['impulse_magnitudes']=impulses;self.file.write(json.dumps(v)+'\n');self.count+=1
        except Exception as e:self.error=repr(e)
