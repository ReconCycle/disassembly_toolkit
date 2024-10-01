from unified_planning.shortcuts import *
from unified_planning.exceptions import UPValueError
#from action_prediction_interface.pddl_utils.pddl_utils import get_all_objects_of_class, generate_valid_args_regex
import os
import time
import cv2
import PIL
from PIL import Image
import copy
from shapely import Polygon
import copy
from typing import List
from dataclasses import dataclass
from typing import Union, List
import numpy as np
import json
import rospy

from context_action_framework.types import Label, Detection
from disassembly_pipeline.utils.tf_utils import TFManager
from robotblockset_python.transformations import x2t
from context_action_framework.types import Detection
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.skills.visual_servoing import VisualServoing


class EstimateBatteryContactPose(BaseSkill):
    def __init__(self):
        0

    def on_enter(self,
                 detections: List[Detection],
                 image: PIL.Image.Image,
                 machine_base_frame: str = 'cnc_machine_home',
                 safe_height_above_drill_point_in_meters = 0.01,
                 drill_depth_in_meters = 0.0,
                 hardcoded_drill_point_raise_z_in_meters = 0,
                 ):

        example_detections = copy.deepcopy(detections)
    
        all_drill_points, base_frame, image_with_drawn_drillpoints = get_battery_solder_contacts_drill_points(detections = example_detections,
                                                                                                                    image = image)
        # Get drill points in whatever base frame
        drill_tasks = [DrillTask(base_frame_of_start_drill_point = base_frame,
                                safe_height_above_start_drill_point = safe_height_above_drill_point_in_meters,
                                start_drill_point_in_meters = point.endpoint_to_drill_at - [0, 0, hardcoded_drill_point_raise_z_in_meters],
                                drill_depth_in_meters = drill_depth_in_meters) for point in all_drill_points]

        drill_tasks = convert_drill_task_list_to_machine_base_frame(drill_task_list = drill_tasks, machine_base_frame = machine_base_frame)
        try:
            drilling_gcode = generate_drilling_gcode(drill_tasks, machine_base_frame = machine_base_frame)
        except ValueError as e:
            print(e)
            drilling_gcode = None
        
        return dict(
                    drill_tasks = drill_tasks,
                    image_with_drawn_drillpoints = image_with_drawn_drillpoints,
                    drilling_gcode = drilling_gcode)
    def execute(self):
        pass
    
    def on_exit(self):
        pass

@dataclass
class DrillPointCandidate:
    origin_point: List = None  # u,v point in img coords. Battery edge-point from which, together with normed_vector_to_endpoint, we construct the 
    # point to drill at, some distance from the battery.
    origin_point_3d: List = None  # x, y, z coords in some frame
    normed_vector_to_endpoint: List = None  # Vector (u,v) that shows direction from origin point (e.g. battery edge) towards where to drill.
    endpoint_to_drill_at_3d: List = None  # x,y,z
    endpoint_to_drill_at_2d: List = None  # u,v 

@dataclass
class DrillTask:
    base_frame_of_start_drill_point: str  # 'panda_1/panda_1_link0'  # Base frame in which drill_point_above_in_meters is defined

    safe_height_above_start_drill_point: float  # delta-Z Height above start_drill_point_in_meters, where to initially move and is considered safe
    start_drill_point_in_meters: List[float]  # Point to drill at - x,y,z coordinates in base_frame, defined in meters.
    drill_depth_in_meters: float  # Depth to drill (below the start_drill_point_in_meters)

    def __post_init__(self):
        # Variable validity checking
        if (self.safe_height_above_start_drill_point < 0) or (self.drill_depth_in_meters < 0):
            raise ValueError("safe_height_above_start_drill_point and drill_depth_in_meters must be positive!")

        if len(self.start_drill_point_in_meters) != 3:
            raise ValueError(f"start_drill_point_in_meters should be of length 3, i.e. [x, y, z] coordinates. Input value was of len {len(start_drill_point_in_meters)}")

def get_battery_solder_contacts_drill_points(detections: List[Detection],
                                             image: PIL.Image.Image = None):
    """ 
    Assumptions:
    - only one battery
    - battery is enclosed within some larger object (e.g. smoke_detector_internals)
    
    """

    example_detections = copy.deepcopy(detections)

    base_frame = example_detections[0].parent_frame
    
    battery_detection = None
    # Object which encloses the battery/parent object. Used to select most likely solder points (usually towards center of object)
    enclosing_detection = None
    center_device = None

    # Find battery and pop it from example detections
    for i, d in enumerate(example_detections):
        if d.label == Label.battery:
            battery_detection = d
            example_detections.pop(i) # Remove from original dict
            break
    if battery_detection is None:
        raise ValueError("Did not find battery among vision system detections!")
    
    enclosing_detection = get_enclosing_detection(candidate_detection = battery_detection,
                            possible_enclosing_detections = example_detections)
    
    if enclosing_detection is not None:
        print(f"""Enclosing detection label: {enclosing_detection.label.name}""")
        center_device = enclosing_detection.center_px
    else:
        print("Did not find enclosing object. Drill points will be less accurately determined")
    
    center_battery = battery_detection.center_px
    
    polygon_battery = battery_detection.polygon_px
    polygon_battery = polygon_battery.minimum_rotated_rectangle
    bbox_battery = [i for i in polygon_battery.boundary.coords]

    bbox_battery = np.zeros((5,2))
    # Debug
    
    bbox_battery[0:-1, :] = battery_detection.obb_px
    bbox_battery[-1, :] = bbox_battery[0, :] # Last point is same as zeroth point, as shapely.polygon.bounds.coords is defined

    bbox_3d_battery = np.zeros((5,3)) # ((5,3))
    bbox_3d_battery[0:-1, :] = battery_detection.obb_3d[0:4, :]
    bbox_3d_battery[-1, :] = bbox_3d_battery[0, :] # Last point is same as zeroth point, as shapely.polygon.bounds.coords is defined

    principal_axes = get_principal_axes_of_bbox(bbox_battery)
            
    midpoints, midpoints_3d = get_short_edges_midpoints_from_rectangle_bbox(bbox_battery, bbox_3d_battery)

    side_contact_points_and_direction_vectors = get_battery_solder_contact_points_from_short_edge_midpoints(midpoints = midpoints,
                                                                                                            midpoints_3d = midpoints_3d,
                                                                                                            bbox_center = center_battery,
                                                                                                            principal_axes= principal_axes)

    outside_contact_points_and_direction_vectors = get_battery_solder_contact_points(bbox_battery = bbox_battery,
                                                                                     bbox_battery_3d = bbox_3d_battery,
                                                                                     bbox_center = center_battery,
                                                                                     principal_axes = principal_axes)
    all_points = side_contact_points_and_direction_vectors + outside_contact_points_and_direction_vectors

    # It's more likely battery contacts are positioned closer to device center
    # Sort by distance from device center
    if center_device is not None:
        def sorting_criteria(item):
            return np.linalg.norm(item.origin_point - center_device)
        all_points = sorted(all_points, key=sorting_criteria)

    # Based on direction vectors and drill diameter, move point in direction vector by suitable distance.
    drill_diameter_mm = 6
    drill_diameter_m = drill_diameter_mm / 1000
    
    dist_to_move = drill_diameter_m / 2
    
    # Image coord frame to 
    R_image_frame_to_camera_base_frame = np.eye(3)
    
    # Determine final point
    for i in all_points:
        origin_point_3d = i.origin_point_3d
        
        normed_direction_vector_3d = np.zeros(3)
        normed_direction_vector_3d[0:2] = i.normed_vector_to_endpoint
    
        direction_vector_3d = np.array([np.cos(normed_direction_vector_3d[0]) * dist_to_move,
                               np.sin(normed_direction_vector_3d[1]) * dist_to_move,
                               0])
        
        final_drill_point_T = origin_point_3d + R_image_frame_to_camera_base_frame@direction_vector_3d
        i.endpoint_to_drill_at = final_drill_point_T

    
    image_with_drawn_drillpoints = None
    if image is not None:
        image_with_drawn_drillpoints = draw_on_image(image = image,
                                                     drill_points = all_points,
                                                     bbox_battery = bbox_battery,
                                                     midpoints = midpoints,
                                                     principal_axes = principal_axes,
                                                     center_battery = center_battery)

    return all_points, base_frame, image_with_drawn_drillpoints

def get_enclosing_detection(candidate_detection: Detection, possible_enclosing_detections: List[Detection]):
    """ Given a candidate detection and possible_enclosing_detections, find if any detection encloses the candidate_detection

    Returns:
    if found, the enclosing detection
    else, None"""
    
    candidate_polygon = candidate_detection.polygon
    for possible_enclosing_detection in possible_enclosing_detections:
        is_within = candidate_polygon.within(possible_enclosing_detection.polygon)
        if is_within:
            return possible_enclosing_detection
    return None

def get_principal_axes_of_bbox(bbox):
    # Calculate the covariance matrix
    bbox = bbox[0:4]
    cov_matrix = np.cov(bbox, rowvar=False)
    
    # Perform eigenvalue decomposition
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # The eigenvectors are the principal axes
    principal_axes = eigenvectors
    principal_axes[0] = principal_axes[0][::-1] # x,y are inverted in numpy coords compared to cv2 image coords.
    principal_axes[1] = principal_axes[1][::-1]
    
    # In our case the "major" ax is along the long side of the battery, for which the variance is smaller and so the eigenvalue is smaller.
    if eigenvalues[0]>eigenvalues[1]:
        principal_axes = principal_axes[::-1]

    return principal_axes

def get_short_edges_midpoints_from_rectangle_bbox(bbox, bbox_3d):
    """ From bbox, find 2 point-pairs which are closest together (e.g. short edges of rectangle)
    Return midpoints of these 2 point-pairs.
    """
    assert len(bbox) == 5, "Expecting shapely polygon where last bbox point is same as zeroth point"

    point_pair_a, point_pair_a_3d, point_pair_b, point_pair_b_3d = get_short_edges(bbox, bbox_3d)

    midpoint_1 = get_midpoint(point_pair_a[0], point_pair_a[1])
    midpoint_1_3d =  get_midpoint(point_pair_a_3d[0], point_pair_a_3d[1])
    
    midpoint_2 =  get_midpoint(point_pair_b[0], point_pair_b[1])
    midpoint_2_3d =  get_midpoint(point_pair_b_3d[0], point_pair_b_3d[1])

    midpoints = [midpoint_1, midpoint_2]
    
    midpoints_3d = [midpoint_1_3d, midpoint_2_3d]

    return midpoints, midpoints_3d

def get_short_edges(bbox, bbox_3d):
    """ From bbox, find 2 point-pairs which are closest together (e.g. short edges of rectangle)
    Return midpoints of these 2 point-pairs.
    """
    assert len(bbox) == 5, "Expecting shapely polygon where last bbox point is same as zeroth point"
    distances = []
    for i in range(0, len(bbox)-1):
        dist = np.linalg.norm(np.array(bbox[i]) - np.array(bbox[i+1]))
        
        distances.append(dist)

    two_shortest_distances_arg_idx = np.argsort(distances)
    two_shortest_distances_arg_idx = two_shortest_distances_arg_idx[0:2]
    # points are idx TO idx+1
    point_pair_a = bbox[two_shortest_distances_arg_idx[0]], bbox[two_shortest_distances_arg_idx[0] + 1 ]
    point_pair_a_3d = bbox_3d[two_shortest_distances_arg_idx[0]], bbox_3d[two_shortest_distances_arg_idx[0] + 1 ]

    point_pair_b = bbox[two_shortest_distances_arg_idx[1]], bbox[two_shortest_distances_arg_idx[1] + 1 ]
    point_pair_b_3d = bbox_3d[two_shortest_distances_arg_idx[1]], bbox_3d[two_shortest_distances_arg_idx[1] + 1 ]
    
    return point_pair_a, point_pair_a_3d, point_pair_b, point_pair_b_3d

def get_midpoint(point_a, point_b):
    if not isinstance(point_a, np.ndarray): point_a = np.array(point_a)
    if not isinstance(point_b, np.ndarray): point_b = np.array(point_b)

    return (point_a + point_b) / 2

def get_battery_solder_contact_points_from_short_edge_midpoints(midpoints, midpoints_3d, bbox_center, principal_axes) -> List[DrillPointCandidate]:
    
    distance = 20 # px
    out_list = []
    
    major_principal_ax = principal_axes[0]
    for i, point in enumerate(midpoints):
        bbox_center_to_point = - bbox_center + point
        if np.dot(major_principal_ax, bbox_center_to_point) < 0:
            major_principal_ax = - major_principal_ax

        drill_point_candidate = DrillPointCandidate(origin_point = point,
                                                    origin_point_3d = midpoints_3d[i],
                                                    normed_vector_to_endpoint = major_principal_ax,
                                                    endpoint_to_drill_at_2d = point + major_principal_ax * distance
        )

        out_list.append(drill_point_candidate)
    return out_list
        
def get_battery_solder_contact_points(bbox_battery, bbox_battery_3d, bbox_center, principal_axes) -> List[DrillPointCandidate]:
    minor_principal_ax = principal_axes[1]
    
    distance = 20 # px

    out_list = []
    
    for i, point in enumerate(bbox_battery[0:-1]):

        # Determine vector from bbox center to this point
        bbox_center_to_point = - bbox_center + point

        # If the bbox_center_to_point and principal ax vector ARE collinear, just add principal axes to get end-point directions.
        # else invert direction of principal axes
        #for principal_ax in principal_axes:
        if np.dot(minor_principal_ax, bbox_center_to_point) < 0:
            minor_principal_ax = - minor_principal_ax

        drill_point_candidate = DrillPointCandidate(origin_point = point,
                                                    origin_point_3d = bbox_battery_3d[i],
                                                    normed_vector_to_endpoint = minor_principal_ax,
                                                    endpoint_to_drill_at_2d = point + minor_principal_ax * distance
        )
        
        out_list.append(drill_point_candidate)

    return out_list

def draw_on_image(image: PIL.Image.Image,
                  drill_points: List[DrillPointCandidate],
                  bbox_battery: List = None,
                  midpoints: List = None,
                  principal_axes: List = None,
                  center_battery: List = None):

    test_img = copy.deepcopy(image)
    cv_img = np.array(test_img)

    # Draw end points
    for i, point_dict in enumerate(drill_points):
        sample_endpoint = point_dict.endpoint_to_drill_at_2d
        cv2.circle(cv_img,tuple(sample_endpoint.astype(int)), 10 - i, (255,0,0), -1)
    
    # Draw battery edge points
    if bbox_battery is not None:
        for pt in bbox_battery:
            pt_int = tuple(np.array(pt).astype(int))
            cv2.circle(cv_img,pt_int, 5, (0,0,255), -1)

    # Draw midpoints on both short battery edges (where plus and minus terminals are)
    if midpoints is not None:
        for midpoint in midpoints:
            cv2.circle(cv_img,tuple(midpoint.astype(int)), 5, (0,255,0), -1)

    if (principal_axes is not None) and (center_battery is not None):
        ax_len = [100, 50] # major, minor axes
        start_pt = center_battery
        for i, ax in enumerate(principal_axes):
            pt1 = start_pt.astype(int)
            pt2 = start_pt + ax * ax_len[i]
            pt2 = pt2.astype(int)
            
            pt1 = tuple(pt1)
            pt2 = tuple(pt2)
            cv2.line(img = cv_img, pt1 = pt1, pt2 = pt2, color = (255,255,255), thickness = 5)
    test_img = Image.fromarray(cv_img)
    
    return test_img
    
def generate_drilling_gcode(drill_tasks: List[DrillTask],
                            rapid_move_speed_mm_per_s = 1000,
                            feed_move_speed_mm_per_s = 100,
                             machine_base_frame: str = '/cnc_machine_home'):
    
    gcode_list = []
    
    for d in drill_tasks:
        if d.base_frame_of_start_drill_point != machine_base_frame:
            raise ValueError(f"Drill points are not in machine base frame ({machine_base_frame})! Convert them beforehand.") 

        point_start_drill = np.array(d.start_drill_point_in_meters)
        # Convert to machine base frame

        point_above = point_start_drill + [0,0, d.safe_height_above_start_drill_point]
        point_start_drill = point_start_drill
        point_end_drill = point_start_drill + [0,0, -d.drill_depth_in_meters]

        # Convert to milimeters
        pt_above_mm, pt_start_drill, pt_end_drill_mm = [convert_point_in_meters_to_milimeters(i).astype(int) for i in [point_above, point_start_drill, point_end_drill]]

        gcode_list.append(f'G00X{pt_above_mm[0]}Y{pt_above_mm[1]}Z{pt_above_mm[2]}F{rapid_move_speed_mm_per_s}')  # Fast move above 
        gcode_list.append(f'G01X{pt_start_drill[0]}Y{pt_start_drill[1]}Z{pt_start_drill[2]}F{feed_move_speed_mm_per_s}')  # Feed Move to start drill loc
        gcode_list.append(f'G01X{pt_end_drill_mm[0]}Y{pt_end_drill_mm[1]}Z{pt_end_drill_mm[2]}F{feed_move_speed_mm_per_s}')  # Feed move to end drill loc
        gcode_list.append(f'G00X{pt_above_mm[0]}Y{pt_above_mm[1]}Z{pt_above_mm[2]}F{rapid_move_speed_mm_per_s}')  # Feed move above
        
    return gcode_list

def convert_point_in_meters_to_milimeters(point: np.array):
    new_point = copy.deepcopy(point)
    new_point[0:3] = new_point[0:3] * 1000
    return new_point

def get_cutting_parameters():
    """ 
    Analyze an (image, List[context_action_framework.Detection])
    to determine CNC milling/cutting strategy.
    """
    CUTTING_STRATEGIES = ['EDGE_CUT', 'DRILL']

def transform_cartesian_position_to_new_frame(cartesian_position: Union[List, np.array], parent_frame: str, new_frame: str, tf2x_function: TFManager.tf2x):
    dx = tf2x_function(parent_frame = parent_frame, child_frame = new_frame)
    dT = x2t(dx)

    if not isinstance(cartesian_position, np.ndarray): cartesian_position = np.array(cartesian_position)
    if not isinstance(dx, np.ndarray): dx = np.array(dx)

    T_in_old_frame = np.eye(4)
    T_in_old_frame[0:3, -1] = cartesian_position

    cartesian_position_in_new_frame = np.linalg.inv(dT) @ T_in_old_frame
    return cartesian_position_in_new_frame[0:3, -1]

def convert_drill_task_list_to_machine_base_frame(drill_task_list: List[DrillTask], machine_base_frame = None) -> List[DrillTask]:

    # Convert each drill point to machine base frame ( integrate into detection function )
    if not rospy.get_node_uri():
        rospy.init_node('testing_cnc', anonymous = False)
    tfmanager = TFManager()
    for d in drill_task_list:
        if d.base_frame_of_start_drill_point != machine_base_frame:
            d.start_drill_point_in_meters = transform_cartesian_position_to_new_frame(cartesian_position = d.start_drill_point_in_meters,
                                                                                      parent_frame = d.base_frame_of_start_drill_point,
                                                                                      new_frame = machine_base_frame,
                                                                                      tf2x_function = tfmanager.tf2x)
            d.base_frame_of_start_drill_point = machine_base_frame
    return drill_task_list