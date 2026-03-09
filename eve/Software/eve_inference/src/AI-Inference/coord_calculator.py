import math as m
import yaml
import numpy as np
from transformers.models.regnet.modeling_regnet import RegNetXLayer
from ultralytics.models.sam.modules.blocks import CXBlock
from PIL import Image
from typing import List, Tuple
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent  # adjust based on your folder structure
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from project_paths import CONFIG_COORD_FILE

'''VARIABLES from Drone'''
camera_height = 100 # in m


########################
# Calculations
########################
def pixel_to_camera(config: dict, img_width: int, img_height: int, bbox_midpoints: List[Tuple[float, float]], camera_height: int) -> List[Tuple[float, float]]:

    '''VARIABLES from yaml file'''
    camera_roll_angle  = config['camera_setting']['camera_roll_angle']
    f = config['camera_setting']['focal_length']
    h_s_width = config['camera_setting']['horizontal_sensorwidth']
    v_s_width = config['camera_setting']['vertical_sensorwidth']

    '''Field of View: Image Width and Height'''
    distance_x_direction = camera_height / m.cos(m.radians(camera_roll_angle))  # direct distance from camera in direction of positive x-axis
    horizontal_width = distance_x_direction * (h_s_width / f)  # in m
    vertical_height = distance_x_direction * (v_s_width / f)  # in m

    '''Midpoint Pixel'''
    mid_pix = (img_height / 2, img_width/2) # calculate the Pixel in the middle of the image
    
    '''Position of object center in Camera Coordinates'''
    distance_camera_mid_pix = m.tan(m.radians(camera_roll_angle)) * camera_height # distance from camera to midpoint of image in positive x-axis (suggestion: the camera is only rotated around the x-axis)
    vertical_height_pixel_ratio = vertical_height / img_height # m/pixel
    horizontal_width_pixel_ratio = horizontal_width / img_width # m/pixel

    '''Distance between Object midpoint and image midpoint'''
    camera_coords = []

    for obj_center in bbox_midpoints:
        dx_pixels = obj_center[0] - mid_pix[0] # horizontal (equivalent to y-axis of camera coordinate system)
        dy_pixels = obj_center[1] - mid_pix[1] # vertical (equivalent to x-axis of camera coordinate system)

        Cx = distance_camera_mid_pix + vertical_height_pixel_ratio * dy_pixels
        Cy = horizontal_width_pixel_ratio * dx_pixels

        camera_coords.append((Cx, Cy))

    return camera_coords


def camera_to_drone(config: dict) -> np.array:
    # get the translation
    tx = config['drone_to_camera_coordinate_system']['translation_x']
    ty = config['drone_to_camera_coordinate_system']['translation_y']
    tz = config['drone_to_camera_coordinate_system']['translation_z']

    T = np.eye(4)
    T[:3, 3] = [tx, ty, tz] # Translation

    # get the rotations angles
    rx = config['drone_to_camera_coordinate_system']['rotation_x'] # roll
    ry = config['drone_to_camera_coordinate_system']['rotation_y'] # pitch
    rz = config['drone_to_camera_coordinate_system']['rotation_z'] # yaw

    # Rotation matrices
    Rx = np.array([[1, 0, 0, 0],
                   [0, np.cos(rx), -np.sin(rx), 0],
                   [0, np.sin(rx), np.cos(rx), 0],
                   [0, 0, 0, 1]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry), 0],
                   [0, 1, 0, 0],
                   [-np.sin(ry), 0, np.cos(ry), 0],
                   [0, 0, 0, 1]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0, 0],
                   [np.sin(rz), np.cos(rz), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    R = Rz @ Ry @ Rx # all Rotations together

    transform_matric = np.eye(4)
    transform_matric[:3, :3] = R[:3, :3]  # Rotation
    transform_matric[:3, 3] = [tx, ty, tz]  # Translation

    return transform_matric


def drone_to_world(config: dict) -> np.array:
    # Translation (Drone in World coordinate system)
    tx = None
    ty = None
    tz = None

    # Rotation (roll, pitch, yaw)
    rx = None
    ry = None
    rz = None

    # Rotation matric
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(rx), -np.sin(rx), 0],
        [0, np.sin(rx), np.cos(rx), 0],
        [0, 0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry), 0],
        [0, 1, 0, 0],
        [-np.sin(ry), 0, np.cos(ry), 0],
        [0, 0, 0, 1]
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0, 0],
        [np.sin(rz), np.cos(rz), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    R = Rz @ Ry @ Rx # all rotations matrices together

    transform_matric = np.eye(4)
    transform_matric[:3, :3] = R[:3, :3]  # Rotation
    transform_matric[:3, 3] = [tx, ty, tz]  # Translation

    return transform_matric

def drone_to_GPS():
    pass




















