import numpy as np
import time
from skimage import measure

from argparse import Namespace
def get_camera_K(width, height, fov):
    """Returns a camera matrix from image size and fov."""
    cx = (width - 1.) / 2.
    cy = (height - 1.) / 2.
    fx = (width / 2.) / np.tan(np.deg2rad(fov / 2.))
    camera_matrix = {'cx': cx, 'cy': cy, 'fx': fx, 'fy': fx}
    camera_matrix = Namespace(**camera_matrix)
    return camera_matrix

def to_numpy(tensor):
    if isinstance(tensor, np.ndarray):
        return tensor
    return tensor.detach().cpu().numpy()
    
def find_big_connect(image):
    img_label, num = measure.label(image, connectivity=2, return_num=True)#输出二值图像中所有的连通域
    props = measure.regionprops(img_label)#输出连通域的属性，包括面积等
    # print("img_label.shape: ", img_label.shape) # 480*480
    resMatrix = np.zeros(img_label.shape)
    tmp_area = 0
    for i in range(0, len(props)):
        if props[i].area > tmp_area:
            tmp = (img_label == i + 1).astype(np.uint8)
            resMatrix = tmp
            tmp_area = props[i].area 
    
    return resMatrix