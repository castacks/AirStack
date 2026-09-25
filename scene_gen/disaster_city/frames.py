"""Pull the frames a reconstruction is built from, out of the named clips.

    ~/.venvs/recon/bin/python frames.py rubble_west      # -> data/recon/rubble_west/images/{A,B}/
    ~/.venvs/recon/bin/python frames.py b01

One folder per physical camera (recon_sfm.py gives each its own intrinsics).
Camera A is 12 MP and is halved to 2028 px; camera B is used as is. The
recipes are the ones the shipped reconstructions were built from.
Clips: data/mocaps/clips/ (index + source times in clips.tsv).
"""
import subprocess, sys, tempfile
from pathlib import Path
import imageio_ffmpeg
from _paths import DATA, R

CLIPS = DATA / "mocaps/clips"
# (clip prefix, fps, optional (start_s, end_s), frame filter on the 1-based index, name prefix)
RECIPES = {
    "rubble_west": [
        ("A01", 2, None, lambda i: i >= 21, "A01_"), ("A05", 2, None, lambda i: i <= 42, "A05_"),
        ("B01", 2, None, lambda i: 21 <= i <= 109, "B01_"),
    ],
    "b01": [
        *[(c, 1, None, None, f"{c}_") for c in ("A02", "A03", "A06", "B02", "B03", "B04", "B07", "B08", "B09", "B10")],
        ("A01", 2, None, lambda i: 79 <= i <= 101 and i % 2, "A01_"), ("A05", 2, None, lambda i: i <= 41 and i % 2, "A05_"),
        ("B01", 2, None, lambda i: 93 <= i <= 111 and i % 2, "B01_"),
        # the dark interior stretches, at 4 fps (1 fps did not register): register with recon_extend.py
        ("B08", 4, (12, 34), None, "Bh/B08h_"), ("B10", 4, (2, 15), None, "Bh/B10h_"), ("B10", 4, (41, 52), None, "Bh/B10k_"),
    ],
}

subject = sys.argv[1]; out = R / subject / "images"
ff = imageio_ffmpeg.get_ffmpeg_exe()
for clip, fps, win, keep, name in RECIPES[subject]:
    src = next(CLIPS.glob(f"{clip}_*.mp4"))
    cam = name.split("/")[0] if "/" in name else clip[0]
    dst = out / cam; dst.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as t:
        vf = f"fps={fps}" + (",scale=2028:1520" if clip[0] == "A" else "")
        cut = ["-ss", str(win[0]), "-to", str(win[1])] if win else []
        subprocess.run([ff, "-nostdin", "-v", "error", *cut, "-i", str(src), "-vf", vf, "-q:v", "2", f"{t}/%04d.jpg"], check=True)
        n = 0
        for f in sorted(Path(t).glob("*.jpg")):
            i = int(f.stem)
            if keep is None or keep(i):
                width = 3 if (fps == 1 or "/" in name) else 4
                f.rename(dst / f"{name.split('/')[-1]}{i:0{width}d}.jpg"); n += 1
    print(f"{clip} @ {fps} fps{f' {win[0]}-{win[1]} s' if win else ''}: {n} frames -> {dst}")
