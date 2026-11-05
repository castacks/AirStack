"""Add read-only inline accessors; native class layout/library remain unchanged."""
from pathlib import Path
import hashlib
p=Path('/opt/ORB_SLAM3/include/System.h')
original=p.read_text();anchor='private:\n\n    void SaveAtlas'
assert original.count(anchor)==1
Path('/opt/ore-ms-final-adapter/System.h.original').write_text(original)
addition='''public:
    Tracking* ExportTracker() const { return mpTracker; }
    Atlas* ExportAtlas() const { return mpAtlas; }
    LocalMapping* ExportLocalMapper() const { return mpLocalMapper; }
    LoopClosing* ExportLoopCloser() const { return mpLoopCloser; }

private:

    void SaveAtlas'''
p.write_text(original.replace(anchor,addition))
print('Original System.h SHA256',hashlib.sha256(original.encode()).hexdigest())
