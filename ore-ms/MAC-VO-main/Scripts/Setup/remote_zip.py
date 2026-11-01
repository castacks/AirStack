"""Bounded seekable HTTPS range reads for selected members of large public ZIPs."""
import hashlib
import io
import urllib.parse
import urllib.request
import uuid


class HTTPRangeFile(io.RawIOBase):
    def __init__(self, url, size, *, record=None, max_read=8*1024*1024, budget=192*1024*1024):
        if urllib.parse.urlsplit(url).scheme != 'https' or type(size) is not int or size <= 0:
            raise ValueError('HTTPS URL and positive exact archive size required')
        self.url, self.size, self.position = url, size, 0
        self.record, self.max_read, self.budget = record, max_read, budget
        self.transferred = 0

    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.position

    def seek(self, offset, whence=io.SEEK_SET):
        if whence not in (io.SEEK_SET, io.SEEK_CUR, io.SEEK_END):
            raise ValueError('invalid seek mode')
        value = offset + (self.position if whence == io.SEEK_CUR else self.size if whence == io.SEEK_END else 0)
        if not 0 <= value <= self.size:
            raise ValueError('seek outside archive')
        self.position = value
        return value

    def read(self, size=-1):
        size = self.size-self.position if size == -1 else min(size, self.size-self.position)
        if size == 0: return b''
        if size < 0 or size > self.max_read or self.transferred+size > self.budget:
            raise ValueError('range read exceeds declared byte budget')
        start, end = self.position, self.position+size-1
        separator = '&' if '?' in self.url else '?'
        # A public nonce avoids stale cached redirect/range responses. Never log
        # the final signed redirect URL; it is not part of source provenance.
        request = urllib.request.Request(self.url+separator+'ore_ms_range='+uuid.uuid4().hex,
            headers={'Range': f'bytes={start}-{end}', 'Accept-Encoding':'identity', 'Cache-Control':'no-cache'})
        with urllib.request.urlopen(request, timeout=45) as response:
            if response.status != 206 or response.headers.get('Content-Range') != f'bytes {start}-{end}/{self.size}':
                raise ValueError('server did not honor exact byte range')
            body = response.read(size+1)
            if len(body) != size:
                raise ValueError('range body length mismatch')
            if self.record:
                self.record({'public_url':self.url, 'start':start, 'end':end, 'total_size':self.size,
                    'bytes':len(body), 'sha256':hashlib.sha256(body).hexdigest(),
                    'final_host':urllib.parse.urlsplit(response.url).hostname,
                    'etag':response.headers.get('ETag')})
        self.transferred += size
        self.position += size
        return body
