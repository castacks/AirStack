"""Register extra frames into an existing recon_sfm.py model.

    ~/.venvs/recon/bin/python recon_extend.py data/recon/b01 Bh B07,B08,B09,B10,B02,B03,A02,A03,A06

For stretches that failed at 1 fps (dark interiors): drop denser frames into
<dir>/images/<folder>/, and this extracts their features, matches them to each
other (+-12 neighbours in name order) and to every registered image whose clip
prefix is listed, then resumes incremental mapping from the largest model.
Writes <dir>/sparse_ext/.
"""
import sys
from pathlib import Path
import pycolmap

d, folder, prefixes = Path(sys.argv[1]).resolve(), sys.argv[2], sys.argv[3].split(",")
db, images = d / "database.db", d / "images"
new = sorted(f"{folder}/{p.name}" for p in (images / folder).glob("*.jpg"))
pycolmap.extract_features(db, images, image_names=new, camera_mode=pycolmap.CameraMode.PER_FOLDER,
                          reader_options=pycolmap.ImageReaderOptions(camera_model="OPENCV"))

base = max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
reg = sorted(im.name for im in pycolmap.Reconstruction(base).images.values())
old = [n for n in reg if n.split("/")[1][:3] in prefixes]
pairs = {(a, b) for i, a in enumerate(new) for b in new[i + 1:i + 13]} | {(a, b) for a in new for b in old}
(d / "pairs_ext.txt").write_text("".join(f"{a} {b}\n" for a, b in sorted(pairs)))
print(f"{len(new)} new images, {len(old)} registered partners, {len(pairs)} pairs")
pycolmap.match_image_pairs(db, pairing_options=pycolmap.ImportedPairingOptions(match_list_path=str(d / "pairs_ext.txt")))

out = d / "sparse_ext"; out.mkdir(exist_ok=True)
opts = pycolmap.IncrementalPipelineOptions(); opts.ba_refine_principal_point = False
models = pycolmap.incremental_mapping(db, images, out, options=opts, input_path=base)
for i, m in models.items():
    got = sum(1 for im in m.images.values() if im.name.startswith(folder + "/"))
    print(f"model {i}: {m.num_reg_images()} images ({got}/{len(new)} of the new), {m.num_points3D()} points, "
          f"reproj {m.compute_mean_reprojection_error():.2f}px")
