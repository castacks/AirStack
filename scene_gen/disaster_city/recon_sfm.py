"""Sparse reconstruction (COLMAP) of one site feature from drone frames.

    ~/.venvs/recon/bin/python recon_sfm.py data/recon/rubble_west

Expects <dir>/images/<camera>/*.jpg -- one sub-folder per physical camera, so
each lens gets its own intrinsics (A = 12 MP wide-angle, B = the low-res one).
Writes <dir>/database.db and <dir>/sparse/<n>/ (largest model first in the log).
"""
import sys, time
from pathlib import Path
import pycolmap

d = Path(sys.argv[1]).resolve()
db, images, sparse = d / "database.db", d / "images", d / "sparse"
db.unlink(missing_ok=True)
sparse.mkdir(exist_ok=True)
t = time.time()

reader = pycolmap.ImageReaderOptions(camera_model="OPENCV")      # wide-angle lens: k1 k2 p1 p2
ext = pycolmap.FeatureExtractionOptions(max_image_size=2048)
ext.sift.max_num_features = 8192
pycolmap.extract_features(db, images, camera_mode=pycolmap.CameraMode.PER_FOLDER,
                          reader_options=reader, extraction_options=ext)
print(f"features {time.time() - t:.0f}s")

pycolmap.match_exhaustive(db)
print(f"matching {time.time() - t:.0f}s")

opts = pycolmap.IncrementalPipelineOptions()
opts.ba_refine_principal_point = False
models = pycolmap.incremental_mapping(db, images, sparse, options=opts)
n_img = sum(1 for _ in images.rglob("*.jpg"))
for i, m in sorted(models.items(), key=lambda kv: -kv[1].num_reg_images()):
    print(f"model {i}: {m.num_reg_images()}/{n_img} images, {m.num_points3D()} points, "
          f"reproj {m.compute_mean_reprojection_error():.2f}px")
    for c in m.cameras.values(): print("   ", c.model.name, c.width, c.height, [round(p, 4) for p in c.params])
print(f"done {time.time() - t:.0f}s")
