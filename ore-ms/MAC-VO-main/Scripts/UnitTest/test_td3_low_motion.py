import cv2
import numpy as np
from Scripts.Setup.td3_low_motion import lk_track,track_masks


def texture():
    rng=np.random.default_rng(3)
    return cv2.GaussianBlur(rng.integers(0,256,(160,192),dtype=np.uint8),(3,3),0)


def test_lk_recovers_known_translation_and_identity():
    first=texture();uv=np.array([[x,y] for y in range(48,120,16) for x in range(48,152,16)],np.float32)
    for shift in [[0,0],[2,-1]]:
        second=cv2.warpAffine(first,np.array([[1.,0,shift[0]],[0,1,shift[1]]]),(192,160))
        end,back,ok,reverse,_,_=lk_track(first,second,uv)
        good,_,_,_=track_masks(uv,end,back,ok,reverse,192,160)
        assert good.sum()>=.9*len(uv)
        np.testing.assert_allclose(np.median(end[good]-uv[good],axis=0),shift,atol=.03)


def test_constant_image_has_no_valid_tracks():
    image=np.zeros((160,192),np.uint8);uv=np.array([[64.,64.],[80,80]],np.float32)
    end,back,ok,reverse,_,_=lk_track(image,image,uv)
    assert not track_masks(uv,end,back,ok,reverse,192,160)[0].any()


def test_forward_backward_masks_boundaries_status_and_nan():
    uv=np.array([[50.,50.]]*6);end=np.array([[50.,50.],[50,50],[50,50],[31,50],[50,50],[np.nan,50]])
    back=uv+np.array([[1.,0],[.5,0],[1.01,0],[0,0],[0,0],[0,0]])
    valid,tight,_,_=track_masks(uv,end,back,[1,1,1,1,0,1],[1]*6,192,160)
    assert valid.tolist()==[True,True,False,False,False,False]
    assert tight.tolist()==[False,True,False,False,False,False]
