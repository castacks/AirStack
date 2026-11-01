import io
import json
import zipfile

import cv2
import numpy as np
import pytest

from Scripts.Setup.prepare_tartanair_flow import prepare_pair
from Scripts.Setup.remote_zip import HTTPRangeFile


def test_exact_range_zip_and_crc(monkeypatch):
    storage=io.BytesIO()
    with zipfile.ZipFile(storage,'w',compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('frame.npy',b'known-source-labels'*200)
    content=storage.getvalue();records=[]
    def open_response(request,timeout):
        a,b=map(int,request.get_header('Range').removeprefix('bytes=').split('-'))
        response=io.BytesIO(content[a:b+1]);response.status=206
        response.headers={'Content-Range':f'bytes {a}-{b}/{len(content)}','ETag':'fixed'}
        response.url='https://cdn.example/object?secret-signature=do-not-record'
        return response
    monkeypatch.setattr('urllib.request.urlopen',open_response)
    remote=HTTPRangeFile('https://public.example/immutable-revision.zip',len(content),record=records.append)
    with zipfile.ZipFile(remote) as archive:assert archive.read('frame.npy')==b'known-source-labels'*200
    assert records and all('secret-signature' not in json.dumps(r) for r in records)


@pytest.mark.parametrize('failure',['status','range','length'])
def test_bad_range_responses_rejected(monkeypatch,failure):
    def reply(request,timeout):
        value=io.BytesIO(b'ab' if failure=='length' else b'abc');value.status=200 if failure=='status' else 206
        value.headers={'Content-Range':'bytes 1-3/10' if failure=='range' else 'bytes 0-2/10'}
        return value
    monkeypatch.setattr('urllib.request.urlopen',reply)
    with pytest.raises(ValueError):HTTPRangeFile('https://example/a',10).read(3)


@pytest.mark.parametrize('failure',['read','budget','seek','scheme'])
def test_range_bounds_before_network(failure):
    with pytest.raises(ValueError):
        stream=HTTPRangeFile('http://example/a' if failure=='scheme' else 'https://example/a',10,max_read=4,budget=2)
        if failure=='read':stream.read(5)
        elif failure=='budget':stream.read(3)
        elif failure=='seek':stream.seek(-1)


def test_preparation_preserves_pixels_flows_and_mask_polarity(tmp_path):
    bgr=np.full((64,64,3),[3,100,255],dtype=np.uint8)
    for name in ('a.png','b.png'):assert cv2.imwrite(str(tmp_path/name),bgr)
    flow=np.full((64,64,2),[-1.25,2.5],dtype=np.float32);np.save(tmp_path/'flow.npy',flow)
    mask=np.zeros((64,64),dtype=np.uint8);mask[0,:3]=[1,100,200];np.save(tmp_path/'mask.npy',mask)
    output=tmp_path/'pair.npz';result=prepare_pair(tmp_path/'a.png',tmp_path/'b.png',tmp_path/'flow.npy',tmp_path/'mask.npy',output)
    with np.load(output) as data:
        assert data['image1'].dtype==np.float32 and data['valid_mask'].dtype==np.bool_
        assert np.array_equal(data['image1'][:,0,0],np.array([255,100,3],dtype=np.float32)/np.float32(255))
        assert np.array_equal(data['flow'],flow.transpose(2,0,1))
        assert np.array_equal(data['valid_mask'][0],mask==0)
    assert result['source_valid_pixels']==64*64-3
    with pytest.raises(FileExistsError):prepare_pair(None,None,None,None,output)
