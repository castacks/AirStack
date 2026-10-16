import math
import numpy as np
import xml.etree.ElementTree as ET
from camera_urdf import pitched_description,rotation


def test_pitch_is_in_flu_not_rotated_body_frame():
    source='''<robot name="test"><joint name="body" type="fixed"><parent link="base_link"/><child link="body"/><origin xyz="0 0 0" rpy="0 0 1.5707963267948966"/></joint><joint name="mount" type="fixed"><parent link="body"/><child link="base_link_ZED_X"/><origin xyz="0 -0.2 -0.05" rpy="0 0 0"/></joint></robot>'''
    result=ET.fromstring(pitched_description(source,20))
    origin=result.find("joint[@name='mount']/origin")
    rpy=[float(v) for v in origin.get('rpy').split()]
    a=rotation([0,0,math.pi/2]);actual=a@rotation(rpy)@a.T
    np.testing.assert_allclose(actual,rotation([0,math.radians(20),0]),atol=1e-10)
    assert origin.get('xyz')=='0 -0.2 -0.05'
    assert result.find("joint[@name='body']/origin").get('rpy')=='0 0 1.5707963267948966'
