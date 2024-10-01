# Send TF2 transformacijo - Izriše v rviz

import tf2_ros
import geometry_msgs.msg
import rospy
import time
import numpy as np

""" Set of utilities to determine whether an object is safe to be picked up. """

def get_visiontable_pickup_safety(tf2x, object_vision_name:str):
    0
    name_hca = 'hca_back_vision_table_zero'
    name_visiontablelimit = 'vision_table/safepickuplim'
    
    hca_to_vision_safe = tf2x(parent_frame = name_visiontablelimit, child_frame=object_vision_name)
    
    x = hca_to_vision_safe[0] ; y =  hca_to_vision_safe[1]
    if not ((0 < x < 0.35) and (-0.2 < y < 0.2)):
        rospy.loginfo("Unsafe pickup position! {} {}".format(x,y))
        return 0
    return 1

#def get_cutter_battery_safety(tf2x):
def get_position_within_four_bboxes(tf2x, origin_frame, b1, b2, b3, b4, target_frame, printing = True):
    """ Given the transforms, check if battery is within a safe area so that it can be handled later.
    Returns an integer, used downstream to determine valid battery grasp pose.

    # B1 and B2 are two inner bboxes 
    # B3 and B4 are two outer bboxes

    Args:
        tf2x: obj
            tf2x object from disassembly_pipeline.tf_utils
        origin_frame : str
            ROS frame of origin

    Returns:

    Example call:
    >>> get_positions_within_four_bboxes(origin_frame = "cutter",
        b1 = "cutter/plate_inedge1",
        b2 = "cutter/plate_inedge2",
        b3 = "cutter/plate_outedge1",
        b4 = "cutter/plate_outedge2",
        target_frame = "battery_realsense")
    """

    # Get these 4 limit tfs.
    xi1 = tf2x(parent_frame = origin_frame, child_frame=b1)
    xi2 = tf2x(parent_frame = origin_frame, child_frame=b2)
    xo1 = tf2x(parent_frame = origin_frame, child_frame=b3)
    xo2 = tf2x(parent_frame = origin_frame, child_frame=b4)
    
    # Get the battery TF
    x_battery = tf2x(parent_frame = origin_frame, child_frame=target_frame)

    if (xi2[0]<x_battery[0]<xi1[0]) and (xi2[1]<x_battery[1]<xi1[1]):
        if printing: print("Center Center")
        return 1
    elif xi1[1]<x_battery[1]:
        if printing: print("Left edge")
        return 2 
    elif (xi1[0]<x_battery[0]<xo1[0]) and (xi2[1]<x_battery[1]<xi1[1]):
        if printing: print("Top Center")
        return 3
    elif (xi1[0]<x_battery[0]<xo1[0]) and (xo2[1]<x_battery[1]<xi2[1]):
        if printing: print("Top Right")
        return 4 
    elif (xi2[0]<x_battery[0]<xi1[0]) and (xo2[1]<x_battery[1]<xi2[1]):
        if printing: print("Center Right")
        return 5
    elif (xo2[0]<x_battery[0]<xi2[0]) and (xo2[1]<x_battery[1]<xi2[1]):
        if printing: print("Bottom right")
        return 6
    elif (xo2[0]<x_battery[0]<xi2[0]) and (xi2[1]<x_battery[1]<xi1[1]):
        if printing: print("Bottom center")
        return 7
    else:
        if printing: print("Not on plate")
        return 0
