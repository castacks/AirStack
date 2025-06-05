import numpy as np
from typing import overload

from rclpy.node import Node
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from typing import TYPE_CHECKING
import os, sys

from .MessageFactory import to_image

# Add the src directory to the Python path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'src'))
sys.path.insert(0, src_path)
if TYPE_CHECKING:
    # To make static type checker happy : )
    from src.Module import IFrontend, IMatcher, IStereoDepth
    from src.DataLoader import StereoData
else:
    from Module import IFrontend, IMatcher, IStereoDepth
    from DataLoader import StereoData


class DisparityPublisher(IFrontend):
    def __init__(self, node: Node, internal_module: IFrontend, publish_topic: str, frame_id: str):
        self.ros2_node       = node
        self.internal_module = internal_module
        self.publish_topic   = publish_topic
        self.publisher       = node.create_publisher(Image, publish_topic, qos_profile=1)
        self.frame_id        = frame_id
        self.curr_timestamp: Time | None = None
    
    @property
    def provide_cov(self) -> tuple[bool, bool]: return self.internal_module.provide_cov
    
    def init_context(self): return None
    
    @overload
    def estimate(self, frame_t1: None, frame_t2: StereoData) -> tuple[IStereoDepth.Output, None]: ...
    @overload
    def estimate(self, frame_t1: StereoData, frame_t2: StereoData) -> tuple[IStereoDepth.Output, IMatcher.Output]: ...
    
    def estimate(self, frame_t1: StereoData | None, frame_t2: StereoData) -> tuple[IStereoDepth.Output, IMatcher.Output | None]:
        depth, match = self.internal_module.estimate(frame_t1, frame_t2)
        
        if (depth.disparity is not None) and (self.curr_timestamp is not None):
            disparity_msg = to_image(
                depth.disparity[0].permute(1, 2, 0).cpu().numpy().astype(np.uint16), 
                self.frame_id,
                self.curr_timestamp,
                encoding="mono16"
            )
            self.publisher.publish(disparity_msg)
        
        return depth, match
