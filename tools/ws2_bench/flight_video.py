"""Record the viewer's live simulator/inference feeds, without desktop capture.

Both panels are 960x540 with aspect-preserving letterboxing. Capture starts at
the ROS takeoff send-goal marker, not simulator startup. Timestamps preserve the
actual asynchronous display cadence; no inference frames are synthesized.
"""
import io
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

from PIL import Image, ImageDraw, ImageFont, ImageOps


def encoder():
    path = os.environ.get('WS2_FFMPEG') or shutil.which('ffmpeg')
    if path:
        return path
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        raise RuntimeError('Recording needs ffmpeg or pip install imageio-ffmpeg') from None


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def panel(image, size=(960, 540)):
    result = Image.new('RGB', size, '#070c10')
    if image is not None:
        fitted = ImageOps.contain(image.convert('RGB'), size, Image.Resampling.LANCZOS)
        result.paste(fitted, ((size[0]-fitted.width)//2, (size[1]-fitted.height)//2))
    return result


class FlightVideo:
    def __init__(self, runtime, output, target, title, fps=20):
        self.runtime, self.output = Path(runtime), Path(output)
        self.target, self.title, self.fps = target, title, fps
        self.run_id = str(self.output.resolve())
        self.marker = self.output/'capture_events.jsonl'
        self.done = threading.Event()
        self.started = threading.Event()
        self.error = None
        self.frames = self.scene_updates = self.model_updates = 0
        self.start_time = self.stop_time = None
        self.last_scene = self.last_model = None
        self.stop_reason = None
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
        self.small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
        self.log = (self.output/'video_encoder.log').open('wb')
        self.process = subprocess.Popen([
            encoder(), '-hide_banner', '-loglevel', 'warning', '-y',
            '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '1920x640',
            '-framerate', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-threads', '4', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(self.output/'flight.mp4')],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log)
        self.thread = threading.Thread(target=self._run, name='flight-video', daemon=True)
        self.thread.start()

    def _takeoff(self):
        try:
            for line in self.marker.read_text().splitlines():
                marker = json.loads(line)
                if marker['event'] == 'takeoff_command_sent':
                    return marker
        except (OSError, ValueError, KeyError):
            pass
        return None

    def _image(self, path):
        try:
            return Image.open(io.BytesIO(path.read_bytes())).convert('RGB')
        except (OSError, ValueError):
            return None

    def _frame(self, now):
        scene = read_json(self.runtime/'live_view.json')
        model = read_json(self.runtime/'inference'/(self.target+'.json'))
        live = read_json(self.runtime/'live_episode.json')
        scene_valid = scene.get('run_id') == self.run_id
        model_valid = model.get('run_id') == self.run_id
        a = self._image(self.runtime/'live_view.jpg') if scene_valid else None
        b = self._image(self.runtime/'inference'/(self.target+'.jpg')) if model_valid else None
        canvas = Image.new('RGB', (1920, 640), '#101820')
        canvas.paste(panel(a), (0, 68)); canvas.paste(panel(b), (960, 68))
        d = ImageDraw.Draw(canvas)
        d.text((16, 7), self.title, font=self.font, fill='#eef4f6')
        d.text((16, 37), 'AirStack / Isaac Sim', font=self.small, fill='#b6c5cf')
        label = {'mononav':'MonoNav: RGB / ZoeDepth / TSDF / path',
                 'kim':'Kim et al.: RGB / FCRN / D3QN',
                 'pulp_dronet':'PULP-DroNet v3: image / steering / collision'}[self.target]
        d.text((976, 37), label, font=self.small, fill='#b6c5cf')
        if b is None:
            d.text((1200, 325), 'Model starts after takeoff', font=self.font, fill='#b6c5cf')
        age = now-model.get('wall_time', now) if model_valid else None
        phase = live.get('phase', '') if live.get('directory') == self.run_id else ''
        sim = scene.get('sim_time') if scene_valid else None
        d.text((16, 614), f'Elapsed {now-self.start_time:.1f}s  |  Simulation {sim or 0:.1f}s  |  {phase}',
               font=self.small, fill='#b6c5cf')
        if model_valid:
            d.text((976, 614), f'Inference frame {model.get("sequence", "?")}  |  Last update {age:.1f}s ago',
                   font=self.small, fill='#b6c5cf')
        if scene_valid and scene.get('wall_time') != self.last_scene:
            self.scene_updates += 1; self.last_scene = scene.get('wall_time')
        if model_valid and model.get('sequence') != self.last_model:
            self.model_updates += 1; self.last_model = model.get('sequence')
        return canvas, dict(frame=self.frames, wall_time=now, sim_time=sim,
                            scene_wall_time=scene.get('wall_time') if scene_valid else None,
                            model_sequence=model.get('sequence') if model_valid else None,
                            model_wall_time=model.get('wall_time') if model_valid else None,
                            phase=phase)

    def _run(self):
        try:
            while not self.done.is_set():
                marker = self._takeoff()
                if marker:
                    self.start_time = marker['wall_time']; self.started.set(); break
                self.done.wait(.01)
            if not self.started.is_set():
                return
            with (self.output/'video_frames.jsonl').open('w') as trace:
                while True:
                    now = time.time()
                    end = self.stop_time if self.done.is_set() else now
                    due = max(1, math.ceil((end-self.start_time)*self.fps))
                    if self.frames < due:
                        frame, meta = self._frame(now)
                        self.process.stdin.write(frame.tobytes())
                        trace.write(json.dumps(meta)+'\n')
                        if self.frames == 0: frame.save(self.output/'video_first.jpg', quality=90)
                        if meta['phase']=='planner' and meta['model_sequence'] is not None and self.frames % (5*self.fps)==0:
                            frame.save(self.output/'video_preview.jpg', quality=90)
                        self.frames += 1
                        self.final_frame = frame
                    elif self.done.is_set():
                        break
                    else:
                        self.done.wait(min(.01, 1/self.fps))
        except Exception as exc:
            self.error = str(exc)
        finally:
            if self.process.stdin:
                self.process.stdin.close()

    def stop(self, reason):
        self.stop_time = time.time(); self.stop_reason = reason; self.done.set()
        self.thread.join(timeout=60)
        if self.thread.is_alive():
            self.process.kill(); self.thread.join(timeout=5)
            self.error = self.error or 'Recorder failed to finalize'
        try:
            code = self.process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            self.process.kill(); code = self.process.wait()
            self.error = self.error or 'Encoder finalization timed out'
        self.log.close()
        if hasattr(self, 'final_frame'):
            self.final_frame.save(self.output/'video_last.jpg', quality=90)
        result = dict(file='flight.mp4', width=1920, height=640, panel_size=[960,540], fps=self.fps,
                      frames=self.frames, duration_s=self.frames/self.fps,
                      start_wall_time=self.start_time, stop_wall_time=self.stop_time,
                      start_event='takeoff_command_sent', stop_event=reason,
                      scene_updates=self.scene_updates, model_updates=self.model_updates,
                      encoder_exit=code, error=self.error)
        result['valid'] = bool(self.start_time and self.frames and self.scene_updates > 1
                               and self.model_updates > 0 and code == 0 and not self.error)
        (self.output/'video.json').write_text(json.dumps(result, indent=2))
        return result
