"""Actual encoded transport retention, independent of policy/neural processing."""
import hashlib
from pathlib import Path
import time
from Scripts.Setup.transition_feedback import input_available

class EncodedInputBuffer:
    def __init__(self,root,rows,perturb):
        self.root=Path(root).resolve();self.rows=rows;self.perturb=perturb
        self.entries={c:{} for c in ['rgb','thermal']};self.latest={c:-1 for c in self.entries}
        self.audit=[];self.ingress_bytes=0;self.query_bytes=0;self.io_ns=0;self.peak_bytes=0

    def advance(self,camera,source):
        if source!=self.latest[camera]+1:raise ValueError('source arrivals must advance one group')
        start=time.perf_counter_ns();self.latest[camera]=source
        expired=[i for i in self.entries[camera] if i<source-1]
        for i in expired:del self.entries[camera][i]
        metadata={'camera':camera,'source':source,'expired_sources':expired,'stored':False}
        if input_available(camera,source,self.perturb):
            images={}
            for side in ['left','right']:
                record=self.rows[source][camera+'_'+side];path=(self.root/record['path']).resolve()
                if not path.is_relative_to(self.root):raise ValueError('retained input leaves dataset')
                data=path.read_bytes();images[side]={'data':data,'path':record['path'],
                    'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'time_ns':record['time_ns']}
                self.ingress_bytes+=len(data)
            entry={'source':source,'camera':camera,'images':images};self.entries[camera][source]=entry
            metadata.update(stored=True,images={side:{k:v for k,v in x.items() if k!='data'} for side,x in images.items()})
        metadata['retained_sources']=sorted(self.entries[camera])
        self.audit.append(metadata);self.io_ns+=time.perf_counter_ns()-start
        current=sum(len(image['data']) for entries in self.entries.values() for item in entries.values() for image in item['images'].values())
        self.peak_bytes=max(self.peak_bytes,current)

    def retrieve(self,camera,source,delivery_group):
        if source!=delivery_group-1 or self.latest[camera]!=delivery_group:
            raise ValueError('only the immediately prior input at this group may be queried')
        payload=self.entries[camera].get(source)
        if payload:self.query_bytes+=sum(len(x['data']) for x in payload['images'].values())
        return payload

    def report(self):
        return {'retention':'current and immediately prior source frame, encoded stereo bytes',
            'ingress_bytes':self.ingress_bytes,'retrieved_bytes':self.query_bytes,
            'peak_retained_bytes':self.peak_bytes,'buffer_io_ns':self.io_ns,'arrivals':self.audit}


def missing_previous_source(current,last_processed,selected,delivered,used):
    if used>=2 or not selected or not delivered or current<2 or last_processed!=current-2:return None
    return current-1


def register_retained(feedback,camera,source):
    pair=source-1
    if source!=feedback.last_source[camera]+1 or pair in feedback.bindings[camera] or pair in feedback.cache.health[camera]:
        raise ValueError('retained source must be a previously unprocessed next input')
    feedback.cache.requested[camera].add(pair)
    return feedback.declare(camera,source)
