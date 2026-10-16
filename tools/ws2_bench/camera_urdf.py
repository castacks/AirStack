"""Create an episode-local URDF matching the Isaac camera's FLU pitch.

Never modify the installed robot description. Body frames in the exported Iris
URDF are rotated relative to FLU, so adding pitch directly to the mount joint
would rotate the camera around the wrong axis.
"""
import argparse
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotation(rpy):
    r,p,y=rpy;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def pitched_description(xml, degrees):
    if not math.isfinite(degrees) or not 0<=degrees<=30:raise ValueError('Camera pitch must be 0..30 degrees')
    root=ET.fromstring(xml)
    parents={j.find('child').get('link'):j for j in root.findall('joint')}
    joint=parents['base_link_ZED_X'];link=joint.find('parent').get('link')
    chain=[]
    while link!='base_link':
        j=parents[link];chain.append(j);link=j.find('parent').get('link')
    parent_rotation=np.eye(3)
    for j in reversed(chain):
        parent_rotation=parent_rotation@rotation([float(v) for v in j.find('origin').get('rpy','0 0 0').split()])
    origin=joint.find('origin');old=rotation([float(v) for v in origin.get('rpy','0 0 0').split()])
    new=parent_rotation.T@rotation([0,math.radians(degrees),0])@parent_rotation@old
    rpy=[math.atan2(new[2,1],new[2,2]),math.asin(float(np.clip(-new[2,0],-1,1))),math.atan2(new[1,0],new[0,0])]
    origin.set('rpy',' '.join(f'{v:.12g}' for v in rpy))
    return ET.tostring(root,encoding='unicode')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--pitch',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    from ament_index_python.packages import get_package_share_directory
    import xacro
    name=Path(os.environ.get('URDF_FILE','iris_with_sensors.pegasus.robot.urdf'))
    if name.is_absolute():source=name
    elif len(name.parts)>1:source=Path(get_package_share_directory(name.parts[0])).joinpath(*name.parts[1:])
    else:source=Path(get_package_share_directory('robot_descriptions'))/'iris/urdf'/name
    expanded=xacro.process_file(str(source),mappings={'robot_name':os.environ.get('ROBOT_NAME','robot_1')}).toxml()
    args.output.write_text(pitched_description(expanded,args.pitch))
    print(f'Episode camera pitch {args.pitch:g}deg FLU; source {source}; output {args.output}')
