"""Cut a textured disc out of the Google tiles and export it as USD (Blender script).

    blender -b data/blender_data/disaster_city.blend --python tiles_extract.py -- cx cy radius out.usd

For features no drone clip covers: the tile photogrammetry is the only source.
World coordinates are kept; textures are written next to the USD.
"""
import sys
import bpy, bmesh

cx, cy, r = map(float, sys.argv[sys.argv.index("--") + 1:][:3])
out = sys.argv[-1]
src = bpy.data.objects["Google 3D Tiles.001"]
bm = bmesh.new(); bm.from_mesh(src.data)
bm.transform(src.matrix_world)
bmesh.ops.delete(bm, geom=[f for f in bm.faces
                           if max((v.co.x - cx) ** 2 + (v.co.y - cy) ** 2 for v in f.verts) > r * r], context="FACES")
bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context="VERTS")
me = bpy.data.meshes.new("cut"); bm.to_mesh(me); bm.free()
for i, mat in enumerate(src.data.materials): me.materials.append(mat)
ob = bpy.data.objects.new("cut", me); bpy.context.scene.collection.objects.link(ob)
# drop materials no remaining face uses, so only their textures get exported
used = {p.material_index for p in me.polygons}
for i in reversed(range(len(me.materials))):
    if i not in used: me.materials.pop(index=i)
# Blosm's tile materials are Emission(image) -- the USD exporter only reads a
# Principled BSDF, so give each a plain one with the same image as base colour.
for i, mat in enumerate(me.materials):
    img = next(n.image for n in mat.node_tree.nodes if n.type == "TEX_IMAGE")
    new = bpy.data.materials.new(f"tile_{i}"); new.use_nodes = True
    nt = new.node_tree; bsdf = nt.nodes["Principled BSDF"]
    tex = nt.nodes.new("ShaderNodeTexImage"); tex.image = img
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 1.0
    me.materials[i] = new
for o in bpy.context.scene.objects: o.select_set(o is ob)
bpy.context.view_layer.objects.active = ob
bpy.ops.wm.usd_export(filepath=out, selected_objects_only=True, export_materials=True,
                      export_textures_mode="NEW", export_animation=False, convert_orientation=False)
print(f"tiles_extract: {len(me.polygons)} faces, {len(me.materials)} materials -> {out}")
