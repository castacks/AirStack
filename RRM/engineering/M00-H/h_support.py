"""Strict serialization and identity for offline H components."""
import hashlib
import json
from pathlib import Path


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def sha_bytes(value):return hashlib.sha256(value).hexdigest()

def file_sha(path):return sha_bytes(Path(path).read_bytes())


def decode(raw):
    def unique(pairs):
        out={}
        for key,value in pairs:
            if key in out:raise ValueError('Duplicate JSON key')
            out[key]=value
        return out
    return json.loads(raw,object_pairs_hook=unique,
                      parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
