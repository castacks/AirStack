"""Ground/object coupling, material continuity and static-cache contracts."""
import math
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from disaster import quake_suburban_ground as ground
from disaster import quake_suburban_interactions as response


def traces():
    return [dict(id='test',points=[(-10.,0.),(10.,0.)],width=.4,
                 band=3.,heave=.4,drop=.2,depth=.8)]


def test_displacement_sign_and_falloff():
    assert ground.fault_sample(0,1,traces())['z']>0
    assert ground.fault_sample(0,-1,traces())['z']<0
    assert ground.fault_sample(0,0,traces())['opening']
    assert ground.fault_sample(0,10,traces())['z']==0
    assert ground.fault_sample(0,0,[])['z']==0


def test_rupture_bounds_scale_beyond_the_250m_review():
    import random
    trace=ground._trace(300.,0.,0.,80.,random.Random(4),0,
                        (-500.,-500.,500.,500.))
    assert max(x for x,_ in trace['points'])>350.
    assert all(-499.<x<499. and -499.<y<499. for x,y in trace['points'])


def test_zero_rate_soft_soil_does_not_author_liquefaction_traces():
    soil=dict(center=[0.,0.],rx_m=60.,ry_m=40.,angle_deg=0.,rate=0.)
    assert ground.traces_for_scene([],soil,10)==[]


def test_rigid_footprint_bridges_opening():
    p=response.support_plane(0,0,1,2,traces())
    assert p['affected'] and p['gy']>0
    assert p['dz']>-.2  # Not the 80 cm-deep fissure floor.
    assert p['tilt_deg']>0
    assert not response.support_plane(0,20,1,1,traces())['affected']


def test_plane_tilt_has_correct_world_direction_under_rotated_parent():
    from pxr import Gf,Usd,UsdGeom
    stage=Usd.Stage.CreateInMemory()
    parent=UsdGeom.Xform.Define(stage,'/P')
    parent.AddRotateZOp().Set(73.)
    parent.AddScaleOp().Set(Gf.Vec3f(2.))
    prim=UsdGeom.Xform.Define(stage,'/P/Object').GetPrim()
    before=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    plane=dict(gx=.1,gy=.2,dz=.3)
    delta=response.plane_matrix(0,0,0,plane)
    response.world_delta(prim,delta)
    after=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    for p in (Gf.Vec3d(0),Gf.Vec3d(1,0,0),Gf.Vec3d(0,1,0)):
        assert np.allclose(after.Transform(p),delta.Transform(before.Transform(p)))
    assert delta.Transform(Gf.Vec3d(0,1,0))[2]>.3
    assert delta.Transform(Gf.Vec3d(0,-1,0))[2]<.3


def test_collapse_footprint_is_actual_stock_not_radius():
    spec=dict(x=8.,y=0.,z=.1,l=3.,w=.15,t=.1,yaw=0.,pitch=0.,roll=0.)
    h=dict(id='h',x=10.,y=20.,yaw=90.,debris_specs=[spec])
    from shapely.geometry import Point
    shape=response.collapse_footprints([h])[0][1]
    assert shape.contains(Point(10.,28.))
    assert not shape.contains(Point(18.,20.))
    assert not shape.contains(Point(11.,28.))


def test_ground_reuses_real_material_and_uv_and_cuts_original(monkeypatch):
    from pxr import Usd,UsdGeom,UsdShade
    from shapely.geometry import box
    from disaster import quake_flow as qf
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage,'/World')
    mat=UsdShade.Material.Define(stage,'/World/RoadMaterial')
    soil=UsdShade.Material.Define(stage,'/World/SoilMaterial')
    coeff=np.array([[.5,0],[0,.25],[.1,.2]])
    ground._mesh(stage,'/World/ground/road',box(-15,-5,15,5),
                 lambda x,y:.1,str(mat.GetPath()),coeff)
    monkeypatch.setattr(qf,'materials',lambda *a:{})
    monkeypatch.setattr(qf,'_c_look_at',lambda *a,**k:soil)
    report=ground.author(stage,'/World',traces())
    assert report['pieces']>10
    source=ground.read_surfaces(stage,'/World/ground')[0]
    assert source['geometry'].area<300
    for p in stage.GetPrimAtPath('/World/quake_ground').GetChildren():
        if not p.GetName().startswith('surface_piece'):
            continue
        assert UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0].GetPath()==mat.GetPath()
        mesh=UsdGeom.Mesh(p)
        st=UsdGeom.PrimvarsAPI(mesh).GetPrimvar('st').Get()
        for xyz,uv in zip(mesh.GetPointsAttr().Get(),st):
            assert np.allclose(np.array([xyz[0],xyz[1],1.])@coeff,uv,atol=2e-6)


def test_replacement_debris_uses_thin_suburban_stock():
    from disaster import quake_suburban_bake as bake
    from detail import modular_house as mh
    style=next(iter(mh.STYLES))
    h=dict(style=style,seed=42,mode='partial_collapse',side='S',W=15.,D=10.)
    specs=bake.debris_specs(h,19)
    assert len(specs)>2*209
    assert sum(s['l']*s['w'] for s in specs)>19*14*1.7
    assert all(.005<s['t']<.10 for s in specs)
    assert all(s['class'] in ('board','deck','joist','sheathing','siding','stud') for s in specs)


def test_failed_bearing_does_not_leave_upper_wall_and_floor_floating():
    from disaster.quake_suburban_bake import module_failures
    items=[dict(category='house_'+role,z_m=z) for role,z in
           [('floor',0),('wall',0),('wall',0),('floor',3.5),('wall',3.5),('roof',7.)]]
    bounds=[(-5,-5,-.28,5,5,0),(-5,-5.1,0,5,-4.9,3.5),
            (-5,4.9,0,5,5.1,3.5),(-5,-5,3.22,5,5,3.5),
            (-5,4.9,3.5,5,5.1,7),(-5,-5,7,5,5,10)]
    broken,extra=module_failures(items,bounds,dict(mode='partial_collapse',side='S',W=10,D=10))
    assert 0 not in broken and 2 not in broken
    assert {1,3,4,5}<=broken and extra>=1


def test_cracked_band_tapers_instead_of_square_cut_end():
    from shapely.geometry import LineString
    envelope=ground.trace_envelope(traces()[0])
    middle=envelope.intersection(LineString([(0,-10),(0,10)])).length
    end=envelope.intersection(LineString([(-9.8,-10),(-9.8,10)])).length
    assert 0<end<middle*.7


def test_review_sky_is_textured_neutral_noon_dome():
    from pxr import Usd,UsdLux
    from tools.suburban_quake_build import configure_sky
    stage=Usd.Stage.CreateInMemory()
    url='/tmp/noon_grass.hdr'
    configure_sky(stage,dict(sky=url,sky_intensity=3000,sky_color=[1.,1.,1.]))
    dome=UsdLux.DomeLight.Get(stage,'/World/DomeLight')
    assert dome.GetTextureFileAttr().Get().path==url
    assert dome.GetIntensityAttr().Get()==3000
    assert tuple(dome.GetColorAttr().Get())==(1.,1.,1.)
    assert dome.GetExposureAttr().Get()==0


def test_placement_scope_is_repaired_before_foundation_tilt():
    from pxr import Gf,Usd,UsdGeom
    import scene_generator as sg
    from disaster.quake_suburban_bake import restore_structure_frame
    from disaster import quake_flow as qf
    stage=Usd.Stage.CreateInMemory()
    sg.apply_placements(stage,[],'/House/structure',1.)
    child=UsdGeom.Cube.Define(stage,'/House/structure/wall').GetPrim()
    m=dict(cx=0.,cy=0.,yaw=0.,W=15.,D=10.,z0=0.,top=10.)
    matrix,geometry=qf._c_tilt_matrix(m,'E',6.,.32,max_drop_m=1.35,max_rise_m=.9)
    shell=UsdGeom.Xformable(stage.GetPrimAtPath('/House/structure'))
    shell.AddTransformOp().Set(matrix)
    # Negative control: a saved nonzero matrix on Scope is ignored by USD.
    assert UsdGeom.XformCache().GetLocalToWorldTransform(child).TransformDir(Gf.Vec3d(0,0,1))==Gf.Vec3d(0,0,1)
    restore_structure_frame(stage,'/House/structure')
    up=UsdGeom.XformCache().GetLocalToWorldTransform(child).TransformDir(Gf.Vec3d(0,0,1)).GetNormalized()
    assert math.degrees(math.acos(up[2]))==pytest.approx(geometry['tilt'])
    low=matrix.Transform(Gf.Vec3d(7.5,0,0))[2]
    high=matrix.Transform(Gf.Vec3d(-7.5,0,0))[2]
    assert low<0<high


def test_real_surface_cells_preserve_skin_without_thick_caps():
    import trimesh
    from disaster.quake_suburban_fragments import surface_cells
    mesh=trimesh.Trimesh(vertices=[[-4,-2,0],[4,-2,0],[4,2,0],[-4,2,0]],
                         faces=[[0,1,2],[0,2,3]],process=False)
    cells=surface_cells(mesh,np.random.default_rng(81),24)
    assert len(cells)>8
    assert sum(c.area for c in cells)==pytest.approx(mesh.area,rel=.02)
    assert max(c.extents[2] for c in cells)<1e-6
    assert max(max(c.extents) for c in cells)<3.5


def test_actual_fracture_footprints_reach_fence_queries():
    from shapely.geometry import Point
    from disaster.quake_suburban_interactions import collapse_footprints
    h=dict(x=10,y=20,yaw=90,fracture_footprints=[[(1,0),(3,0),(3,2),(1,2)]])
    shape=collapse_footprints([h])[0][1]
    assert shape.contains(Point(9,22))


def test_cache_key_ignores_world_pose_but_not_recipe():
    from disaster import quake_suburban_bake as bake
    h=dict(style='cottage',palette='red',mode='foundation',side='N',seed=13)
    assert bake.cache_key(h)==bake.cache_key(dict(h,x=100,y=-30,yaw=75))
    assert bake.cache_key(h)!=bake.cache_key(dict(h,side='S'))


def test_cache_key_deduplicates_sides_only_when_geometry_is_identical():
    from disaster import quake_suburban_bake as bake
    base=dict(style='cottage',palette='wood_white',seed=13)
    for mode in ('pristine','collapse'):
        assert bake.cache_key(dict(base,mode=mode,side='N')) == \
               bake.cache_key(dict(base,mode=mode,side='W'))
    assert bake.cache_key(dict(base,mode='foundation',side='N')) != \
           bake.cache_key(dict(base,mode='foundation',side='W'))


def test_export_gate_checks_final_points_and_does_not_hide_failed_solve():
    from disaster.quake_suburban_bake import validate_settled_export
    r=dict(converged=True,still_moving=0,below_grade_pts_worst=-.101,at_rest_ok=False)
    assert validate_settled_export(r,0.)['post_clamp_verified']
    assert not r['at_rest_ok']
    for changes,z in [(dict(still_moving=1),0.),(dict(converged=False),0.),
                      (dict(no_collider=['/bad']),0.),(dict(clamp_failed=1),0.),
                      (dict(below_grade_pts_worst=-.21),0.),({},-.03),({},float('nan'))]:
        with pytest.raises(RuntimeError):
            validate_settled_export(dict(r,**changes),z)
