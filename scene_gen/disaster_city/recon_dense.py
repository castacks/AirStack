"""Dense reconstruction (COLMAP PatchMatch, GPU) on top of recon_sfm.py.

    ~/.venvs/recon/bin/python recon_dense.py data/recon/rubble_west [max_image_size] [model_dir]

Uses model_dir (default: the sparse model with the most registered images). To
densify a subset, write a model with the rest deregistered -- an image_names
list alone leaves PatchMatch reading files that were never undistorted. Undistorts into
<dir>/dense/, runs PatchMatch + fusion -> <dir>/dense/fused.ply. The depth and
normal maps are the disk hog (~40 MB/image at 1200 px) -- keep max_image_size low.
"""
import sys, time
from pathlib import Path
import pycolmap

d = Path(sys.argv[1]).resolve()
size = int(sys.argv[2]) if len(sys.argv) > 2 else 1200
best = Path(sys.argv[3]).resolve() if len(sys.argv) > 3 else \
    max((d / "sparse").iterdir(), key=lambda p: pycolmap.Reconstruction(p).num_reg_images())
dense = d / "dense"
t = time.time()

pycolmap.undistort_images(dense, best, d / "images",
                          undistort_options=pycolmap.UndistortCameraOptions(max_image_size=size))
print(f"undistort {time.time() - t:.0f}s  (model {best.name})")

pm = pycolmap.PatchMatchOptions(max_image_size=size, geom_consistency=True)
pycolmap.patch_match_stereo(dense, options=pm)
print(f"patch match {time.time() - t:.0f}s")

rec = pycolmap.stereo_fusion(dense / "fused.ply", dense, output_type="PLY")
print(f"fusion {time.time() - t:.0f}s -> {dense / 'fused.ply'}")
import shutil; shutil.rmtree(dense / "stereo"); shutil.rmtree(dense / "images")   # depth maps are ~15 MB/image
print("removed depth maps and undistorted images")
