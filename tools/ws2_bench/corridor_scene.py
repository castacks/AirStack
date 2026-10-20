"""Isaac/USD construction of matte, textured corridors from saved geometry."""
from pathlib import Path
import math


def textures(folder,seed,style='hallway'):
    import numpy as np
    from PIL import Image,ImageDraw
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(seed)
    result={}
    palette=[('wall',(184,174,155)),('floor',(98,110,119)),('block',(130,94,62))]
    if style=='hallway_daylight':palette.append(('ceiling',(222,222,215)))
    if style.startswith('paper_lab'):palette=[('wall',(181,184,184)),('floor',(69,96,72)),('block',(47,49,51))]
    if style=='paper_lab_low':palette=[('wall',(115,120,122)),('floor',(52,85,58)),('block',(35,37,39))]
    if style=='hallway_black':palette=[('wall',(184,174,155)),('floor',(98,110,119)),('block',(35,37,39))]
    for kind,base in palette:
        pixels=np.clip(np.array(base)[None,None,:]+rng.normal(0,4,(512,512,1)),0,255).astype('uint8')
        im=Image.fromarray(pixels);d=ImageDraw.Draw(im)
        if style.startswith('paper_lab') or (style=='hallway_black' and kind=='block'):
            if kind=='wall':
                for x in (0,511):d.line((x,0,x,512),fill=(126,130,132),width=3)
            elif kind=='block':
                d.rectangle((8,8,503,503),outline=(91,94,96),width=7)
                for y in (95,415):d.line((8,y,503,y),fill=(67,70,72),width=4)
        elif kind=='ceiling':
            for x in range(0,512,256):d.line((x,0,x,512),fill=(175,177,173),width=2)
            for y in range(0,512,256):d.line((0,y,512,y),fill=(175,177,173),width=2)
        elif kind=='floor':
            for x in range(0,512,128):d.line((x,0,x,512),fill=(65,72,78),width=3)
            for y in range(0,512,128):d.line((0,y,512,y),fill=(65,72,78),width=3)
        elif kind=='wall':
            for x in (0,256,511):d.line((x,0,x,512),fill=(131,124,111),width=3)
            d.rectangle((0,450,511,511),fill=(85,79,69))
            d.rectangle((35,120,195,310),fill=(124,118,101),outline=(69,64,54),width=5)
            d.rectangle((48,133,182,297),fill=(192,181,151))
        else:
            for y in range(0,512,32):d.line((0,y,512,y),fill=(105,72,45),width=2)
            for x in (4,507):d.line((x,0,x,512),fill=(60,48,39),width=7)
            if style=='hallway_daylight':
                # Wood grain and cabinet handles make the obstacle a readable
                # furnishing, while keeping its original collision geometry.
                for _ in range(65):
                    x=int(rng.integers(0,512));y=int(rng.integers(0,512))
                    d.arc((x-80,y-5,x+80,y+5),0,180,fill=(115,80,50),width=1)
                d.rounded_rectangle((435,215,449,290),radius=5,fill=(55,55,52))
        dest=folder/f'{style}_{kind}_{seed}.png';im.save(dest);result[kind]=dest
    return result


class CorridorScene:
    def __init__(self,stage,spec,seed,texture_folder):
        from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
        from corridors import geometry,point_at
        self.stage=stage;self.data=geometry(spec,seed)
        stage.DefinePrim('/World/Corridor','Xform')
        files=textures(texture_folder,seed,self.data['spec']['style']);self.materials={}
        for name,file in files.items():
            path='/World/Corridor/Materials/'+name
            mat=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/Surface')
            shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.9)
            shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            tx=UsdShade.Shader.Define(stage,path+'/Texture');tx.CreateIdAttr('UsdUVTexture')
            tx.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(file)))
            tx.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
            tx.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat');tx.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
            tx.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
            uv=UsdShade.Shader.Define(stage,path+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            tx.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(),'rgb')
            self.materials[name]=mat
        for i,w in enumerate(self.data['walls']):self.box(f'Wall_{i:03}',w,'wall')
        for i,b in enumerate(self.data['obstacles']):self.box(f'Obstacle_{i:02}',b,'block')
        xs=[p[0] for p in self.data['route']];ys=[p[1] for p in self.data['route']]
        horizontal=[('Floor',-.12,'floor')]
        if self.data['spec']['style'].startswith('hallway'):
            horizontal.append(('Ceiling',3.32,'ceiling' if 'ceiling' in self.materials else 'wall'))
        for name,z,material in horizontal:
            self.box(name,dict(center=[(min(xs)+max(xs))/2,(min(ys)+max(ys))/2,z],
                              size=[max(xs)-min(xs)+12,max(ys)-min(ys)+12,.24],yaw=0),material)
        if self.data['spec']['style']=='paper_lab_low':
            # Distant arena enclosure supplies a visible indoor background above
            # the low partitions, as in the source experiment's laboratory.
            x0,x1=min(xs)-6,max(xs)+6;y0,y1=min(ys)-6,max(ys)+6
            for i,(center,size) in enumerate([
                ([(x0+x1)/2,y0,2.5],[x1-x0,.2,5.]),
                ([(x0+x1)/2,y1,2.5],[x1-x0,.2,5.]),
                ([x0,(y0+y1)/2,2.5],[.2,y1-y0,5.]),
                ([x1,(y0+y1)/2,2.5],[.2,y1-y0,5.])]):
                self.box('Arena_'+str(i),dict(center=center,size=size,yaw=0),'wall')
        self.lights=[]
        self.light_multiplier=3. if self.data['spec']['style']=='hallway_daylight' else 1.
        distances=range(0,math.ceil(self.data['route_length_m'])+3,3) if self.data['spec']['style'].startswith('hallway') else []
        for i,d in enumerate(distances):
            xy,_=point_at(self.data['route'],min(d,self.data['route_length_m']))
            if self.data['spec']['style']=='hallway_daylight':
                # A downward ceiling panel stays entirely below the ceiling;
                # a large sphere intersecting it can be self-occluded in RTX.
                lamp=UsdLux.RectLight.Define(stage,f'/World/Corridor/Light_{i:02}')
                lamp.CreateWidthAttr(1.5);lamp.CreateHeightAttr(1.5)
            else:
                lamp=UsdLux.SphereLight.Define(stage,f'/World/Corridor/Light_{i:02}')
                lamp.CreateRadiusAttr(.4)
            lamp.CreateIntensityAttr(1800.*self.light_multiplier)
            UsdLux.ShadowAPI.Apply(lamp.GetPrim()).CreateShadowEnableAttr(False)
            UsdGeom.Xformable(lamp).AddTranslateOp().Set(Gf.Vec3d(*xy,2.9))
            self.lights.append(lamp)

    def illumination(self,intensity):
        for lamp in self.lights:lamp.GetIntensityAttr().Set(float(intensity)*self.light_multiplier)

    def box(self,name,box,material):
        from pxr import UsdGeom,UsdShade,UsdPhysics,Sdf,Gf
        sx,sy,sz=box['size'];v=[(-sx/2,-sy/2,-sz/2),(sx/2,-sy/2,-sz/2),(sx/2,sy/2,-sz/2),(-sx/2,sy/2,-sz/2),
                               (-sx/2,-sy/2,sz/2),(sx/2,-sy/2,sz/2),(sx/2,sy/2,sz/2),(-sx/2,sy/2,sz/2)]
        faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        vertices=[v[i] for face in faces for i in face];uv=[]
        for width,height in [(sx,sy),(sx,sy),(sx,sz),(sy,sz),(sx,sz),(sy,sz)]:
            uv.extend([(0,0),(width/2,0),(width/2,height/3.2),(0,height/3.2)])
        mesh=UsdGeom.Mesh.Define(self.stage,'/World/Corridor/'+name)
        mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr([4]*6);mesh.CreateFaceVertexIndicesAttr(list(range(24)))
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(uv)
        x=UsdGeom.Xformable(mesh);x.AddTranslateOp().Set(Gf.Vec3d(*box['center']));x.AddRotateZOp().Set(math.degrees(box['yaw']))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(self.materials[material])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        return mesh

    def patch_corners(self,size,mount=None):
        from corridors import patch_corners
        return patch_corners(self.data,size,mount)
