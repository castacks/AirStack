"""Verify/install the supplied patch in the version-controlled asset directory."""
import argparse, hashlib, json, shutil, struct
from pathlib import Path

HERE = Path(__file__).resolve().parent

def install(source):
    data = Path(source).read_bytes()
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('Expected a PNG image')
    width, height = struct.unpack('!II', data[16:24])
    if not (1 <= width <= 4096 and 1 <= height <= 4096):
        raise ValueError('Invalid patch dimensions')
    target = HERE/'assets/learned_patch.png'
    expected = json.loads((HERE/'assets/patch_manifest.json').read_text())
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected['sha256']:
        raise ValueError('Patch differs from the registered Rui asset; register its provenance explicitly first')
    target.parent.mkdir(exist_ok=True)
    if Path(source).resolve() != target: shutil.copyfile(source,target)
    print(json.dumps(dict(path=str(target),sha256=digest,width=width,height=height)))

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path)
    install(p.parse_args().source)
