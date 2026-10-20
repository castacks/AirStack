import json
from pathlib import Path
from queue import Queue, Full
from threading import Thread


class Recorder:
    def __init__(self, path, capacity=256):
        self.file = Path(path).open('x', buffering=1)
        self.queue = Queue(maxsize=capacity)
        self.error = None
        self.thread = Thread(target=self.write, daemon=True)
        self.thread.start()


    def append(self, record):
        if self.error is not None:
            raise OSError('Flight recorder failed: '+str(self.error))
        try:
            self.queue.put_nowait(json.dumps(record, allow_nan=False)+'\n')
        except Full as exc:
            raise OSError('Flight recorder queue full') from exc


    def write(self):
        try:
            while True:
                line = self.queue.get()
                try:
                    if line is None:
                        return
                    self.file.write(line)
                finally:
                    self.queue.task_done()
        except OSError as exc:
            self.error = exc
        finally:
            try:
                self.file.close()
            except OSError as exc:
                self.error = exc


    def close(self):
        if self.thread.is_alive():
            try:
                self.queue.put(None, timeout=1.)
            except Full as exc:
                raise OSError('Recorder shutdown queue full') from exc
            self.thread.join(timeout=2.)
            if self.thread.is_alive():
                raise OSError('Recorder shutdown exceeded deadline')
        if self.error is not None:
            raise OSError('Flight recorder failed: '+str(self.error))
