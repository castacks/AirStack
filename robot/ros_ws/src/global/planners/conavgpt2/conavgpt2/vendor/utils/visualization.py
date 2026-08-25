import math
from typing import Iterable
import dataclasses
from PIL import Image, ImageDraw, ImageFont
import cv2
import os

import numpy as np
from typing import List, Union
import skimage.morphology
from PIL import Image
from conavgpt2.vendor.constants import color_palette, coco_categories, category_to_id

import supervision as sv
from supervision.draw.color import Color, ColorPalette

# Copied from https://github.com/concept-graphs/concept-graphs/     
def vis_result_fast(
    image: np.ndarray, 
    detections: sv.Detections, 
    classes: List[str], 
    color: Union[Color, ColorPalette] = ColorPalette.default(),
    instance_random_color: bool = False,
    draw_bbox: bool = True,
) -> np.ndarray:
    '''
    Annotate the image with the detection results. 
    This is fast but of the same resolution of the input image, thus can be blurry. 
    '''
    # Annotators
    bounding_box_annotator = sv.BoundingBoxAnnotator(
        color=color,
        thickness=1  # Thickness of bounding box lines
    )
    label_annotator = sv.LabelAnnotator(
        text_scale=0.3,
        text_thickness=1,
        text_padding=2,
    )
    mask_annotator = sv.MaskAnnotator(
        color=color
    )
    
    # Generate labels
    labels = [
        f"{classes[class_id]} {confidence:0.2f}" 
        for _, _, confidence, class_id, _, _
        in detections
    ]
    
    if instance_random_color:
        # Generate random colors for each instance
        detections = dataclasses.replace(detections)
        detections.class_id = np.arange(len(detections))
        
    # Apply mask annotations
    annotated_image = mask_annotator.annotate(scene=image.copy(), detections=detections)
    
    # Apply bounding box annotations
    if draw_bbox:
        annotated_image = bounding_box_annotator.annotate(scene=annotated_image, detections=detections)
        
        # Apply text labels separately
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)
    
    return annotated_image

def init_vis_image(goal_name, action = 0):
    vis_image = np.ones((537, 1165, 3)).astype(np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (20, 20, 20)  # BGR
    thickness = 2

    text = "Observations" 
    textsize = cv2.getTextSize(text, font, fontScale, thickness)[0]
    textX = (640 - textsize[0]) // 2 + 15
    textY = (50 + textsize[1]) // 2
    vis_image = cv2.putText(vis_image, text, (textX, textY),
                            font, fontScale, color, thickness,
                            cv2.LINE_AA)

    text = "Find {}  Action {}".format(goal_name, str(action))
    textsize = cv2.getTextSize(text, font, fontScale, thickness)[0]
    textX = 640 + (480 - textsize[0]) // 2 + 30
    textY = (50 + textsize[1]) // 2
    vis_image = cv2.putText(vis_image, text, (textX, textY),
                            font, fontScale, color, thickness,
                            cv2.LINE_AA)

    # draw outlines
    color = [100, 100, 100]
    vis_image[49, 15:655] = color
    vis_image[49, 670:1150] = color
    vis_image[50:530, 14] = color
    vis_image[50:530, 655] = color
    vis_image[50:530, 669] = color
    vis_image[50:530, 1150] = color
    vis_image[530, 15:655] = color
    vis_image[530, 670:1150] = color


#     # draw legend
#     lx, ly, _ = legend.shape
#     vis_image[537:537 + lx, 155:155 + ly, :] = legend

    return vis_image

def draw_line(start, end, mat, steps=25, w=1):
    for i in range(steps + 1):
        x = int(np.rint(start[0] + (end[0] - start[0]) * i / steps))
        y = int(np.rint(start[1] + (end[1] - start[1]) * i / steps))
        mat[x - w:x + w, y - w:y + w] = 1
    return mat

def init_multi_vis_image(goal_name, multi_color, s_x = 537, s_y = 670):
    vis_image = np.ones((s_x, s_y, 3)).astype(np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (20, 20, 20)  # BGR
    thickness = 2

    text = "Find {}".format(goal_name) 
    textsize = cv2.getTextSize(text, font, fontScale, thickness)[0]
    textX = 50
    textY = (50 + textsize[1]) // 2
    vis_image = cv2.putText(vis_image, text, (textX, textY),
                            font, fontScale, color, thickness,
                            cv2.LINE_AA)

    for i in range(len(multi_color)):
        text = "Agent {}".format(i) 
        vis_image = cv2.putText(vis_image, text, (textX+200+150*i, textY),
                                font, fontScale, multi_color[i], thickness,
                                cv2.LINE_AA)
    # draw outlines
    color = [100, 100, 100]
    # vis_image[49, 15:495] = color
    # vis_image[50:530, 14] = color
    # vis_image[50:530, 495] = color
    # vis_image[530, 15:495] = color


#     # draw legend
#     lx, ly, _ = legend.shape
#     vis_image[537:537 + lx, 155:155 + ly, :] = legend

    return vis_image


def get_contour_points(pos, origin, size=20):
    x, y, o = pos
    pt1 = (int(x) + origin[0],
           int(y) + origin[1])
    pt2 = (int(x + size / 1.5 * np.cos(o + np.pi * 4 / 3)) + origin[0],
           int(y + size / 1.5 * np.sin(o + np.pi * 4 / 3)) + origin[1])
    pt3 = (int(x + size * np.cos(o)) + origin[0],
           int(y + size * np.sin(o)) + origin[1])
    pt4 = (int(x + size / 1.5 * np.cos(o - np.pi * 4 / 3)) + origin[0],
           int(y + size / 1.5 * np.sin(o - np.pi * 4 / 3)) + origin[1])

    return np.array([pt1, pt2, pt3, pt4])

EPS = 1e-4
def write_number(image, pose, number):
    
    pil_image = Image.fromarray(image)
        
    # add the number on the image
    # Initialize drawing context
    draw = ImageDraw.Draw(pil_image)
    
    # 1. Draw the main number as a rectangle.
    font_size_main = 30
    try:
        font_main = ImageFont.truetype("arial.ttf", font_size_main)
    except IOError:
        font_main = ImageFont.load_default(font_size_main)

    text_width = 20
    text_height = 35
    padding = 3
    position = (3, 3)  # Adjust position as needed

    # Define the rectangle coordinates
    rect_x0 = position[0] - padding
    rect_y0 = position[1] - padding
    rect_x1 = position[0] + text_width + padding
    rect_y1 = position[1] + text_height + padding

    # Draw the white rectangle
    draw.rectangle([rect_x0, rect_y0, rect_x1, rect_y1], fill="white")

    # Add text to image
    draw.text(position, str(number), fill="red", font=font_main)

    # 2. Draw circles for each pose point.
    circle_radius = 12
    try:
        font_pose = ImageFont.truetype("arial.ttf", 15)
    except IOError:
        font_pose = ImageFont.load_default(15)

    drawn_centers = []
    def push_away(px, py, existing_x, existing_y, radius):
        """Push point (px, py) away from (existing_x, existing_y) just enough to not overlap."""
        dist = math.dist((px, py), (existing_x, existing_y))
        # If already not overlapping or same point, do nothing
        if dist >= 2 * radius:
            return px, py

        # Calculate overlap distance
        overlap = 2 * radius - dist
        # Direction from existing circle to new circle
        dx = px - existing_x
        dy = py - existing_y
        # If dx,dy is zero, pick a random small direction
        if dx == 0 and dy == 0:
            dx, dy = 1e-3, 0
        length = math.hypot(dx, dy)

        # Normalize direction, move 'overlap/2' away 
        # (or some fraction, depending how you want them spaced)
        nx = dx / length
        ny = dy / length
        px += nx * (overlap / 2)
        py += ny * (overlap / 2)
        return px, py
    
    for i, (px, py, pz) in enumerate(pose):
        # py = 480-py
        moved = True
        while moved:
            moved = False
            for (ex, ey) in drawn_centers:
                dist = math.dist((px, py), (ex, ey))
                if dist + EPS < 2 * circle_radius:
                    # push away
                    px, py = push_away(px, py, ex, ey, circle_radius)
                    moved = True
                
                
        # Circle bounding box
        x0 = px - circle_radius
        y0 = py - circle_radius
        x1 = px + circle_radius
        y1 = py + circle_radius

        # Draw the black-filled circle with a white outline
        draw.ellipse(
            [x0, y0, x1, y1],
            fill="white",
            outline="black",  # optional outline color
            width=2           # outline thickness
        )

        # Text in the center
        index_str = "R"+str(i)
        # Use textbbox or font.getsize
        bbox_pose = draw.textbbox((0, 0), index_str, font=font_pose)
        text_width_pose = bbox_pose[2] - bbox_pose[0]
        text_height_pose = bbox_pose[3] - bbox_pose[1]

        text_x_pose = px - text_width_pose / 2
        text_y_pose = py - text_height_pose +2

        draw.text((text_x_pose, text_y_pose), index_str, fill="black", font=font_pose)

        drawn_centers.append((px, py))
    
    return pil_image

def write_number_full(image, pose, number):
    
    pil_image = Image.fromarray(image)
        
    # add the number on the image
    # Initialize drawing context
    draw = ImageDraw.Draw(pil_image)
    
    # 1. Draw the main number as a rectangle.
    font_size_main = 30
    try:
        font_main = ImageFont.truetype("arial.ttf", font_size_main)
    except IOError:
        font_main = ImageFont.load_default(font_size_main)

    text_width = 20
    text_height = 35
    padding = 3
    position = (3, 3)  # Adjust position as needed

    # Define the rectangle coordinates
    rect_x0 = position[0] - padding
    rect_y0 = position[1] - padding
    rect_x1 = position[0] + text_width + padding
    rect_y1 = position[1] + text_height + padding

    # Draw the white rectangle
    draw.rectangle([rect_x0, rect_y0, rect_x1, rect_y1], fill="white")

    # Add text to image
    draw.text(position, str(number), fill="red", font=font_main)

    # 2. Draw circles for each pose point.
    circle_radius = 12
    try:
        font_pose = ImageFont.truetype("arial.ttf", 15)
    except IOError:
        font_pose = ImageFont.load_default(15)

    drawn_centers = []
    def push_away(px, py, existing_x, existing_y, radius):
        """Push point (px, py) away from (existing_x, existing_y) just enough to not overlap."""
        dist = math.dist((px, py), (existing_x, existing_y))
        # If already not overlapping or same point, do nothing
        if dist >= 2 * radius:
            return px, py

        # Calculate overlap distance
        overlap = 2 * radius - dist
        # Direction from existing circle to new circle
        dx = px - existing_x
        dy = py - existing_y
        # If dx,dy is zero, pick a random small direction
        if dx == 0 and dy == 0:
            dx, dy = 1e-3, 0
        length = math.hypot(dx, dy)

        # Normalize direction, move 'overlap/2' away 
        # (or some fraction, depending how you want them spaced)
        nx = dx / length
        ny = dy / length
        px += nx * (overlap / 2)
        py += ny * (overlap / 2)
        return px, py
    
    for i, (px, py, pz) in enumerate(pose):
        # py = 480-py
        moved = True
        while moved:
            moved = False
            for (ex, ey) in drawn_centers:
                dist = math.dist((px, py), (ex, ey))
                if dist + EPS < 2 * circle_radius:
                    # push away
                    px, py = push_away(px, py, ex, ey, circle_radius)
                    moved = True
                
                
        # Circle bounding box
        x0 = px - circle_radius
        y0 = py - circle_radius
        x1 = px + circle_radius
        y1 = py + circle_radius

        # Draw the black-filled circle with a white outline
        draw.ellipse(
            [x0, y0, x1, y1],
            fill="white",
            outline="black",  # optional outline color
            width=2           # outline thickness
        )

        # Text in the center
        index_str = "R"+str(i)
        # Use textbbox or font.getsize
        bbox_pose = draw.textbbox((0, 0), index_str, font=font_pose)
        text_width_pose = bbox_pose[2] - bbox_pose[0]
        text_height_pose = bbox_pose[3] - bbox_pose[1]

        text_x_pose = px - text_width_pose / 2
        text_y_pose = py - text_height_pose +2

        draw.text((text_x_pose, text_y_pose), index_str, fill="black", font=font_pose)

        drawn_centers.append((px, py))
    
    return pil_image

def Visualize(args, step, pose_pred, map_pred, exp_pred, goal_name, visited_vis, map_edge, goal_map, top_view_map, episode_n=0, rank=0):
    sem_map = np.zeros(map_pred.shape)

    map_mask = np.rint(map_pred) == 1
    exp_mask = np.rint(exp_pred) == 1
    edge_mask = map_edge >0

    sem_map[exp_mask] = 2
    sem_map[map_mask] = 1

    for i in range(args.num_agents):
        sem_map[visited_vis[i] == 1] = 3+i
        selem = skimage.morphology.disk(4)
        goal_mat = 1 - skimage.morphology.binary_dilation(
            goal_map[i], selem) != True

        goal_mask = goal_mat == 1
        sem_map[goal_mask] = 3+i
            
    sem_map[edge_mask] = 3

    color_pal = [int(x * 255.) for x in color_palette]
    sem_map_vis = Image.new("P", (sem_map.shape[1],
                                    sem_map.shape[0]))
    sem_map_vis.putpalette(color_pal)
    sem_map_vis.putdata(sem_map.flatten().astype(np.uint8))
    sem_map_vis = sem_map_vis.convert("RGB")
    sem_map_vis = np.flipud(sem_map_vis)

    sem_map_vis = sem_map_vis[:, :, [2, 1, 0]]
    sem_map_vis = cv2.resize(sem_map_vis, (480, 480),
                                interpolation=cv2.INTER_NEAREST)

    color = []
    for i in range(args.num_agents):
        color.append((int(color_palette[11+3*i] * 255),
                    int(color_palette[10+3*i] * 255),
                    int(color_palette[9+3*i] * 255)))

    vis_image = init_multi_vis_image(goal_name, color, 537, 980)

    vis_image[50:530, 15:495] = sem_map_vis
    top_view_map_nor = cv2.resize(top_view_map, (480, 480),
                                interpolation=cv2.INTER_NEAREST)
    vis_image[50:530, 500:980] = np.flipud(top_view_map_nor)

    for i in range(args.num_agents):
        agent_arrow = get_contour_points(pose_pred[i], origin=(15, 50), size=10)

        cv2.drawContours(vis_image, [agent_arrow], 0, color[i], -1)

    if args.visualize:
        # Displaying the image
        cv2.imshow("episode_{}".format(rank), vis_image)
        cv2.waitKey(1)
    
    if args.print_images:
        dump_dir = "{}/dump/{}".format(args.dump_location, args.nav_mode)
        ep_dir = '{}/episodes_multi/{}/eps_{}/'.format(
            dump_dir, rank, episode_n)
        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)
        fn = ep_dir + 'Merged_Vis-{}.png'.format(step)
        cv2.imwrite(fn, vis_image)

    return vis_image