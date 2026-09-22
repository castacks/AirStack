"""Planner-independent PhysX contact and nominal spherical clearance oracle."""
import math
from pxr import PhysxSchema, Usd, UsdPhysics
import omni.physx
from isaacsim.sensors.physics import _sensor


class SceneOracle:
    def __init__(self, stage, root='/World/base_link', radius=.25):
        self.root=root;self.radius=radius;self.bodies=[]
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
                self.bodies.append(str(prim.GetPath()))
        if not self.bodies:raise RuntimeError('no drone rigid bodies for contact reporting')
        self.sensor=_sensor.acquire_contact_sensor_interface()
        self.query=omni.physx.get_physx_scene_query_interface()
        self.armed=False;self.collision=None;self.last_clearance=-1.
        self.status={'available':True,'armed':False,'collision':None,'bodies':self.bodies}

    def external(self,path):
        return bool(path) and path!=self.root and not path.startswith(self.root+'/')

    def overlap(self,point,radius):
        found=[]
        def hit(result):
            path=str(result.collision)
            if self.external(path):found.append(path);return False
            return True
        self.query.overlap_sphere(float(radius),tuple(map(float,point)),hit,False)
        return bool(found)

    def clearance(self,point,maximum=5.):
        # Query the actual triangle colliders, not just three box bounds. The
        # sphere is a nominal vehicle envelope, not the exact collision hull.
        if not self.overlap(point,maximum):return maximum-self.radius,True
        lo,hi=0.,maximum
        for _ in range(13):
            mid=(lo+hi)/2
            if self.overlap(point,mid):hi=mid
            else:lo=mid
        return lo-self.radius,False

    def update(self,t,position):
        if position[2]>.3:self.armed=True
        contacts=set()
        for body in self.bodies:
            for contact in self.sensor.get_rigid_body_raw_data(body):
                values=list(contact)
                names=[self.sensor.decode_body_name(values[i]) for i in (2,3)]
                contacts.update(p for p in names if self.external(p))
        if self.armed and contacts and self.collision is None:
            self.collision={'sim_time':t,'objects':sorted(contacts),'position':list(map(float,position))}
        self.status.update(armed=self.armed,contacts=sorted(contacts),collision=self.collision,sim_time=t)
        if t-self.last_clearance>=.1:
            distance,censored=self.clearance(position)
            self.status.update(clearance_m=distance,clearance_censored=censored,
                clearance_method='PhysX mesh distance minus nominal 0.25m sphere (not exact hull)',clearance_sim_time=t)
            self.last_clearance=t
        return self.status
