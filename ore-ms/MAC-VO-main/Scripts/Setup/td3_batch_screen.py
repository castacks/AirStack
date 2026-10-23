"""Bounded Box ZIP source-image screening; no learned inference or VO changes."""
import argparse, concurrent.futures, hashlib, io, json, subprocess, zipfile
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

STREAMS = dict(rgb_left='multisense_left_rect_color', rgb_right='multisense_right_rect',
               thermal_left='thermal_left_processed', thermal_right='thermal_right_processed')


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


class RemoteZIP(io.RawIOBase):
    def __init__(self, command, size, cache):
        self.command, self.size, self.cache, self.pos = command, size, cache, 0
        self.blocks = {}; self.requests = []
        cache.mkdir(parents=True, exist_ok=True)

    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos
    def seek(self, offset, whence=0):
        self.pos = offset if whence == 0 else self.pos+offset if whence == 1 else self.size+offset
        if self.pos < 0: raise ValueError('Negative seek')
        return self.pos

    def fetch(self, start, count):
        path = self.cache/f'{start}-{count}.bin'
        if not path.exists():
            r = subprocess.run([*self.command, '--offset', str(start), '--count', str(count)], capture_output=True, timeout=240)
            if r.returncode or len(r.stdout) != count:
                raise RuntimeError(f'Authenticated range failed: offset={start}, count={count}; private diagnostics suppressed')
            path.write_bytes(r.stdout)
        data = path.read_bytes()
        if len(data) != count: raise ValueError('Truncated range cache')
        return start, count, data

    def prefetch(self, infos):
        spans = []
        for i in sorted(infos, key=lambda x:x.header_offset):
            start = i.header_offset
            end = min(self.size, start+30+len(i.filename.encode())+len(i.extra)+i.compress_size+256)
            if spans and start-spans[-1][1] <= 65536 and end-spans[-1][0] <= 8*1024*1024:
                spans[-1][1] = max(spans[-1][1], end)
            else: spans.append([start,end])
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            for start, count, data in pool.map(lambda s:self.fetch(s[0],s[1]-s[0]), spans):
                self.blocks[start,count] = data
                self.requests.append([start,count])

    def read(self, count=-1):
        count = min(self.size-self.pos, count if count >= 0 else self.size-self.pos)
        if count <= 0: return b''
        for (start,n), data in self.blocks.items():
            if start <= self.pos and self.pos+count <= start+n:
                result = data[self.pos-start:self.pos-start+count]; self.pos += count
                return result
        start = self.pos
        _, n, data = self.fetch(start,count)
        self.blocks[start,n] = data; self.requests.append([start,n]); self.pos += n
        return data


def timeline(text, stream, names):
    records = []; seen = set()
    for index, value in enumerate(text.splitlines()):
        if not value.strip(): continue
        stamp = Decimal(value.strip())
        if not stamp.is_finite(): continue
        ns = int(stamp*1000000000)
        name = f'sensors/{stream}/{index:08d}.png'
        if ns not in seen and name in names:
            records.append(dict(index=index, time_ns=ns, path=name)); seen.add(ns)
    return sorted(records, key=lambda r:r['time_ns'])


def windows(rows):
    out = []
    if not rows: return out
    lower, upper = rows[0]['time_ns'], rows[-1]['time_ns']+1
    for third in range(3):
        lo = lower+(upper-lower)*third//3; hi = lower+(upper-lower)*(third+1)//3
        selected = []
        for i in range(len(rows)-19):
            block = rows[i:i+20]
            if block[0]['time_ns'] < lo or block[-1]['time_ns'] >= hi: continue
            if all(0 < b['time_ns']-a['time_ns'] <= 150000000 for a,b in zip(block,block[1:])):
                selected = block; break
        out.append(dict(third=third, rows=selected, available=bool(selected)))
    return out


def nearest(source, target):
    import bisect
    if not source or not target: return []
    ts = [r['time_ns'] for r in target]; out = []
    for r in source:
        j = bisect.bisect_left(ts,r['time_ns'])
        candidates = [k for k in (j-1,j) if 0 <= k < len(ts)]
        k = min(candidates,key=lambda k:(abs(ts[k]-r['time_ns']),k))
        out.append(ts[k]-r['time_ns'])
    return out


def acquire(args):
    out = Path(args.output); out.mkdir(parents=True,exist_ok=True)
    base = ['rclone','--config',args.config,'--box-root-folder-id',args.folder]
    r = subprocess.run([*base,'lsjson','ore_ms:','--files-only'],capture_output=True,check=True)
    listing = json.loads(r.stdout); save(out/'box-listing.json',listing)
    for name in args.clips:
        clip = out/name; clip.mkdir(exist_ok=True)
        info = next(i for i in listing if i['Name'] == name+'.zip')
        reader = RemoteZIP([*base,'cat','ore_ms:'+name+'.zip','--contimeout','15s','--timeout','90s','--retries','2'],info['Size'],clip/'range-cache')
        with zipfile.ZipFile(reader) as z:
            entries = z.infolist(); names = [i.filename for i in entries]
            if len(names) != len(set(names)) or any(Path(n).is_absolute() or '..' in Path(n).parts for n in names): raise ValueError('Unsafe ZIP inventory')
            roots = set(n.split('/')[0] for n in names)
            if roots != {name}: raise ValueError(f'Unexpected ZIP roots {roots}')
            relnames = {n.split('/',1)[1] for n in names if '/' in n}
            save(clip/'archive-index.json',[dict(name=i.filename,size=i.file_size,compressed_size=i.compress_size,crc=i.CRC,offset=i.header_offset) for i in entries])
            wanted = [i for i in entries if i.filename.endswith('/timestamps.txt') and any('/'+s+'/' in i.filename for s in STREAMS.values())]
            reader.prefetch(wanted); files = {}
            def extract(i):
                data = z.read(i); rel = i.filename.split('/',1)[1]; path=clip/'source'/rel
                if path.exists() and path.read_bytes()!=data: raise ValueError('Existing source changed')
                path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
                files[rel]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),zip_crc=i.CRC)
                return data
            for i in wanted: extract(i)
            streams = {}
            for key,stream in STREAMS.items():
                p=clip/'source/sensors'/stream/'timestamps.txt'
                rows=timeline(p.read_text(),stream,relnames) if p.exists() else []
                scout=[rows[round(j*(len(rows)-1)/11)] for j in range(12)] if rows else []
                lines=p.read_text().splitlines() if p.exists() else []
                streams[key]=dict(rows=rows,windows=windows(rows),scouts=scout,
                    timestamp_rows=len(lines), nonfinite_timestamps=sum(not Decimal(v.strip()).is_finite() for v in lines if v.strip()),
                    excluded_rows=len(lines)-len(rows))
            selected={r['path'] for key,s in streams.items() if key.endswith('left') for r in s['scouts']+[r for w in s['windows'] for r in w['rows']]}
            plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),source=info,streams=streams,
                      selection='12 evenly spaced unique acquisitions and first eligible20-frame <=150ms block per chronological third, independently per modality; selected before decoding',
                      timing_ns={key:nearest(streams[a]['rows'],streams[b]['rows']) for key,a,b in [('rgb_stereo','rgb_left','rgb_right'),('thermal_stereo','thermal_left','thermal_right'),('thermal_minus_rgb','rgb_left','thermal_left')]})
            save(clip/'plan.json',plan)
            image_infos=[z.getinfo(name+'/'+rel) for rel in sorted(selected)]
            print(name,'entries',len(entries),'selected images',len(image_infos),flush=True)
            reader.prefetch(image_infos)
            for i in image_infos: extract(i)
            save(clip/'extraction.json',dict(files=files,requests=reader.requests,whole_archive_hash_verified=False))
            print(name,'acquisition complete',flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--folder',default='428559630945');p.add_argument('--output',required=True);p.add_argument('clips',nargs='+');acquire(p.parse_args())

if __name__=='__main__':main()
