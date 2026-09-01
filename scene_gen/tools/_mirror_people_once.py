import os, omni.client as oc
SRC="omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/People/Assets"
DST="/isaac-sim/AirStack/scene_gen/assets/people"
n=[0,0]
def mirror(src, dst):
    res, entries = oc.list(src)
    assert str(res)=="Result.OK", (src, res)
    os.makedirs(dst, exist_ok=True)
    for e in entries or []:
        s=src+"/"+e.relative_path; d=os.path.join(dst, e.relative_path)
        if e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN:
            mirror(s, d)
        else:
            if os.path.exists(d) and os.path.getsize(d)==e.size:
                n[1]+=1; continue
            r, _, content = oc.read_file(s)
            assert str(r)=="Result.OK", (s, r)
            with open(d,"wb") as fh: fh.write(memoryview(content))
            n[0]+=1
mirror(SRC, DST)
print("mirrored %d file(s), %d already current" % tuple(n))
