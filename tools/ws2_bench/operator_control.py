"""Campaign-scoped operator requests, never executable shell commands."""
import json,time
from pathlib import Path

class UserStop(Exception):pass

def request(path):
    if path is None:return 'run'
    try:return json.loads(Path(path).read_text()).get('action','run')
    except (OSError,ValueError):return 'run'

def wait_between_trials(path,publish):
    while request(path)=='pause':
        publish('paused');time.sleep(.2)
    if request(path)=='stop':raise UserStop('operator requested stop')
