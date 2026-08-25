import numpy as np
import torch
import torch.nn.functional as F
import os
import time
import torchvision
import supervision as sv

from PIL import Image

from ultralytics import YOLO
from ultralytics import SAM

 
# # Set up some path used in this script
# # Assuming all checkpoint files are downloaded as instructed by the original GSA repo
# if "GSA_PATH" in os.environ:
#     GSA_PATH = os.environ["GSA_PATH"]
# else:
#     raise ValueError("Please set the GSA_PATH environment variable to the path of the GSA repo. ")
    
# # GroundingDINO config and checkpoint
# import sys
# TAG2TEXT_PATH = os.path.join(GSA_PATH, "Tag2Text")
# EFFICIENTSAM_PATH = os.path.join(GSA_PATH, "EfficientSAM")
# sys.path.append(GSA_PATH) # This is needed for the following imports in this file
# sys.path.append(TAG2TEXT_PATH) # This is needed for some imports in the Tag2Text files
# sys.path.append(EFFICIENTSAM_PATH)


class Object_Detection_and_Segmentation():
    r""" YOLO
    
    Args:
    """
    def __init__(self, args, classes, device):
        self.args = args
        self.device = device
      
        # Env-overridable so an offline container can point at pre-baked weights
        # rather than have ultralytics fetch them into the cwd on first use.
        self.sam_predictor = SAM(
            os.environ.get('CONAVGPT2_SAM_WEIGHTS', 'mobile_sam.pt')).to(self.device)

        # Initialize a YOLO-World model
        self.yolo_model_w_classes = YOLO(
            os.environ.get('CONAVGPT2_YOLO_WORLD_WEIGHTS', 'yolov8l-world.pt')).to(self.device)
        
        self.yolo_model_w_classes.set_classes(classes)
        
    def detect(self, image):
        
        # UltraLytics YOLO
        yolo_s_time = time.time()
        with torch.no_grad():
            # yolo_results_w_classes = self.yolo_model_w_classes(image, conf=0.1, verbose=False)
            yolo_results_w_classes = self.yolo_model_w_classes.predict(image, conf=0.1, verbose=False)
        # print(yolo_results_w_classes)
        yolo_e_time = time.time()

        confidences = yolo_results_w_classes[0].boxes.conf.cpu().numpy()
        detection_class_ids = yolo_results_w_classes[0].boxes.cls.cpu().numpy().astype(int)
        xyxy_tensor = yolo_results_w_classes[0].boxes.xyxy
        xyxy_np = xyxy_tensor.cpu().numpy()
        # print('yolo: %.3f秒'%(yolo_e_time - yolo_s_time)) 

        detections = sv.Detections(
            xyxy=xyxy_np,
            confidence=confidences,
            class_id=detection_class_ids,
            mask=None,
        )

        if len(confidences) > 0:

            # UltraLytics SAM
            with torch.no_grad():
                sam_out = self.sam_predictor.predict(image, bboxes=xyxy_tensor, verbose=False)
            masks_tensor = sam_out[0].masks.data

            masks_np = masks_tensor.cpu().numpy()
            
            detections = sv.Detections(
                xyxy=xyxy_np,
                confidence=confidences,
                class_id=detection_class_ids,
                mask=masks_np,
            )
            sam_e_time = time.time()
            # print('sam: %.3f秒'%(sam_e_time - sam_s_time)) 
            

        return detections
