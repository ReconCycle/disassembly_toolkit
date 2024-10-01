#!/usr/bin/env python
import numpy as np
import copy

from robotblockset_python.transformations import x2t, rot_z, r2q, x2t
from disassembly_pipeline.utils.tf_utils import TFManager
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections

from context_action_framework.vision_interface import VisionInterface
"""Utilities for generating g-code for cutting of smoke detectors. """


def generate_battery_contact_cutting_gcode(device_center_in_cnc_machine_home_frame_mm = [-52, -162, 0],
                                           device_center_to_battery_contact_offset_x_y_y_mm = [10, 25, 0],
                                           rotation_deg = 0,
                                           safe_height_above_z = 20,
                                           initial_move_speed = 1000,
                                           feed_speed = 300,
                                           enable_spindle = False):
    """ 
    Generate G-code to cut 2 battery contacts (e.g. within Fumonic smoke detector) which are (at 0 deg rotation) symmetric around X axis of CNC machine.
    When rotation is input and is other than 0, the G-code is rotated around the CNC machine Z axis.
    Parameters:
    :device_center_in_cnc_machine_home_frame_mm
    :device_center_to_battery_contact_offset_x_y_y_mm : x,y offset of battery contacts in regard to device center.
    
    Example use:
    >>> gcode = generate_battery_contact_cutting_gcode(device_center_in_cnc_machine_home_frame_mm = [-300, -100, 0],
                                                       device_center_to_battery_contact_offset_x_y_y_mm = [10, 10, 0],
                                                       rotation_deg = 30, enable_spindle = True)
    """

    
    DEVICE_CENTER_IN_CNC_MACHINE_FRAME_MM = device_center_in_cnc_machine_home_frame_mm
    CENTER_TO_BATTERY_CONTACT_OFFSET_X_Y_Z_MM = device_center_to_battery_contact_offset_x_y_y_mm
    ROTATION = rotation_deg

    offset_1 = CENTER_TO_BATTERY_CONTACT_OFFSET_X_Y_Z_MM
    offset_2 = copy.deepcopy(CENTER_TO_BATTERY_CONTACT_OFFSET_X_Y_Z_MM) # battery contacts are symmetric in y, around x axis
    offset_2[1] *= -1
    
    DEVICE_CENTER_IN_CNC_MACHINE_FRAME_MM = np.array(DEVICE_CENTER_IN_CNC_MACHINE_FRAME_MM)
    CENTER_TO_BATTERY_CONTACT_OFFSET_X_Y_Z_MM = np.array(CENTER_TO_BATTERY_CONTACT_OFFSET_X_Y_Z_MM)
    
    battery_contact_position_1 = DEVICE_CENTER_IN_CNC_MACHINE_FRAME_MM + rot_z(ROTATION, unit='deg')@offset_1
    battery_contact_position_2 = DEVICE_CENTER_IN_CNC_MACHINE_FRAME_MM + rot_z(ROTATION, unit='deg')@offset_2
    bc1_mm = battery_contact_position_1.astype(int)
    bc2_mm = battery_contact_position_2.astype(int)

    # G-code generation
    gcode = []
    # Move above first contact
    gcode.append(f'G00X{bc1_mm[0]}Y{bc1_mm[1]}Z{bc1_mm[2] + safe_height_above_z}F{initial_move_speed}')
    # Enable spindle
    if enable_spindle: gcode.append('S1000M3')
    # Drill into first contact
    gcode.append(f'G01X{bc1_mm[0]}Y{bc1_mm[1]}Z{bc1_mm[2]}F{feed_speed}')
    # Move back up above first contact
    gcode.append(f'G00X{bc1_mm[0]}Y{bc1_mm[1]}Z{bc1_mm[2] + safe_height_above_z}F{initial_move_speed}')
    # Move above second contact
    gcode.append(f'G00X{bc2_mm[0]}Y{bc2_mm[1]}Z{bc2_mm[2] + safe_height_above_z}F{initial_move_speed}')
    # Drill into second contact
    gcode.append(f'G01X{bc2_mm[0]}Y{bc2_mm[1]}Z{bc2_mm[2]}F{feed_speed}')
    # Move back up above second contact
    gcode.append(f'G00X{bc2_mm[0]}Y{bc2_mm[1]}Z{bc2_mm[2] + safe_height_above_z}F{initial_move_speed}')
    # Turn off spindle
    if enable_spindle: gcode.append('M5')

    return gcode

def circular_cut_gcode_parametric(device_center_in_cnc_base_frame_mm = [-52, -162, -7],
                                  radius_mm = 38,
                                  safe_height_above_device_center_mm = 30,
                                  fast_move_feed = 1300,
                                  feed_speed = 600,
                                  enable_spindle = False):
    """ 
    Generate gcode to perform a circular cut in machine XY plane
    Args:

    Example call:
    >>> cnc_cut_skill = CNCCutSmokeDetector(init_ros_node = False)
    >>> gcode = circular_cut_gcode_parametric(device_center_in_cnc_base_frame_mm = [-313, -149, -10])
    >>> cnc_cut_skill.cnc_cl.call_server(gcode = gcode)"""
    
    # Move sequence:
    # 1. initial_point_above
    # 2. (if enable_spindle: turn on spindle)
    # 3. initial_cut_point
    # 4. (perform circular cut, end up at initial_cut_point)
    # 5. initial_point above
    # 6. (if enable_spindle: turn off spindle)
    def deg_to_rad(deg):
        return deg * np.pi / 180
    
    device_center = np.array(device_center_in_cnc_base_frame_mm)
    initial_cut_point = copy.deepcopy(device_center)
    initial_cut_point[0] += radius_mm

    # Generate final cut point so we can cut a bit more than 360 degrees
    final_cut_point = copy.deepcopy(device_center)
    final_cut_point[1] -= radius_mm
    #final_cut_point_dx = round(radius_mm * np.cos(deg_to_rad(cutting_arc_degrees - 360)))
    #final_cut_point_dy = -round(radius_mm * np.sin(deg_to_rad(cutting_arc_degrees - 360)))
    #final_cut_point = copy.deepcopy(device_center)
    #final_cut_point[0] += final_cut_point_dx
    #final_cut_point[1] += final_cut_point_dy
    # Circle center in relation to final_cut_point
    #NEW_I = -final_cut_point_dx
    #NEW_J = -final_cut_point_dy
    #print(initial_cut_point, final_cut_point)

    initial_point_above = copy.deepcopy(initial_cut_point)
    initial_point_above[2] += safe_height_above_device_center_mm
    final_point_above = copy.deepcopy(final_cut_point)
    final_point_above[2] += safe_height_above_device_center_mm

    gcode = []
    # 1. Initial move to above starting position
    gcode.append(f"""G01X{initial_point_above[0]}Y{initial_point_above[1]}Z{initial_point_above[2]}F{fast_move_feed}""")
    # Start spindle
    if enable_spindle: gcode.append('S1000M3')
    # 2. initial_cut_point - Move down into device
    gcode.append(f"""G01X{initial_cut_point[0]}Y{initial_cut_point[1]}Z{initial_cut_point[2]}F{feed_speed}""")
    # 3. perform_circular_cut
    gcode.append(f"""G17G02X{initial_cut_point[0]}Y{initial_cut_point[1]}I-{radius_mm}J0F{feed_speed}""")
    gcode.append(f"""G17G02X{final_cut_point[0]}Y{final_cut_point[1]}I-{radius_mm}J0F{feed_speed}""")

    # 4. initial_point_above
    gcode.append(f"""G01X{final_point_above[0]}Y{final_point_above[1]}Z{final_point_above[2]}F{fast_move_feed}""")

    # Stop spindle
    if enable_spindle: gcode.append('M5\n')


    return gcode

def cut_smokedet_hekatron_tab(device_center_position,
                              battery_cover_rotation_deg,
                              rectangle_dimension,
                              center_to_rectangle_offset,
                              cnc_table_base_frame = "table_cnc",
                              cnc_machine_home_frame = 'cnc_machine_home',
                              cnc_table_base_to_cnc_machine_home = None,
                              tf_manager = None,
                              feed_rate = 200,
                              initial_raise_above_mm = 20,
                              initial_move_feed = 1000,
                              enable_spindle = False):
    """ Function to cut the smoke detector Hekatron. It's parametric. If SendTf is provided, the target frames will be shown in ROS.
    Args:
    -------------------------------------
    center_T: (4,4) Pose matrix of the center of the smoke detector/chuck in the cnc_table_base_frame.
    
    rectangle_dimension: (a,b) dimension of the battery cover rectangle of the hekatron smoke detector.
    
    center_to_rectangle_offset: (a,b,c) dimension of offset from center_T to center of battery box cover
    (used to construct bboxes).
    
    cnc_table_base_frame: Name of CNC table (usually table_cnc)
    
    cnc_machine_home_frame: Name of CNC machine home(homing) frame
    
    sendTf: sendTf object from disassembly cycle manager
    
    tf2x: tf2x object from disassembly
    
    feed_rate : Feed rate for the cutting moves
    
    initial_raise_above: First point will be raised above the cutting points by this distance.
    
    initial_move_feed: Feed rate for the first move which moves above the first cutting point.
    
    
    
    """    
    center_T = np.eye(4)
    center_T[0:3, -1] = device_center_position
    center_T[0:3, 0:3] = rot_z(battery_cover_rotation_deg, unit='deg')@center_T[0:3, 0:3]

    #assert center_T.shape == (4,4), "center_T shape must be (4,4)."
    assert len(rectangle_dimension) == 2, "rectangle_dimension must be of shape (2)."
    assert feed_rate >0, "Feed rate must be positive."
    assert feed_rate < 2000, "For cutting, feed rate should be less than 500."
    
    # If TF is not give, we must look it up
    if cnc_table_base_to_cnc_machine_home is None:
        cnc_table_base_to_cnc_machine_home = tf_manager.tf2x(parent_frame = cnc_table_base_frame,
                                                child_frame = cnc_machine_home_frame)
        cnc_table_base_to_cnc_machine_home = x2t(cnc_table_base_to_cnc_machine_home)
    
    # Construct relative TF from smokedetector center to battery box center
    bat_bbox_center_dT = np.eye(4)
    bat_bbox_center_dT[0:3, -1] = center_to_rectangle_offset
    # Construct absolute TF of battery box center
    bat_bbox_center_T = center_T@bat_bbox_center_dT
    
    # Relative TF from battery box center to bbox edges
    bat_bbox_a_dT = np.eye(4)
    bat_bbox_b_dT = np.eye(4)
    bat_bbox_c_dT = np.eye(4)
    bat_bbox_d_dT = np.eye(4)
    
    bat_bbox_a_name = 'hek_batbox_edge_a'
    bat_bbox_b_name = 'hek_batbox_edge_b'
    bat_bbox_c_name = 'hek_batbox_edge_c'
    bat_bbox_d_name = 'hek_batbox_edge_d'


    bat_bbox_a_dT[0:3, -1] = -rectangle_dimension[0]/2, +rectangle_dimension[1]/2, 0
    bat_bbox_b_dT[0:3, -1] = -rectangle_dimension[0]/2, -rectangle_dimension[1]/2, 0
    bat_bbox_c_dT[0:3, -1] = +rectangle_dimension[0]/2, -rectangle_dimension[1]/2, 0
    bat_bbox_d_dT[0:3, -1] = +rectangle_dimension[0]/2, +rectangle_dimension[1]/2, 0

    bat_bbox_a = bat_bbox_center_T@bat_bbox_a_dT
    bat_bbox_b = bat_bbox_center_T@bat_bbox_b_dT
    bat_bbox_c = bat_bbox_center_T@bat_bbox_c_dT
    bat_bbox_d = bat_bbox_center_T@bat_bbox_d_dT

    if tf_manager is not None:
        # Draw center TF
        #sendTf(p = center_T[0:3, -1], q = r2q(center_T[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
        #       child_frame = "center_hek")
        
        # Draw battery box center tf
        #sendTf(p = bat_bbox_center_T[0:3, -1], q = r2q(bat_bbox_center_T[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
        #       child_frame = "center_hek_batbox")

        # Draw square bbox TFs
        tf_manager.SendTransform2tf(p = bat_bbox_a[0:3, -1], q = r2q(bat_bbox_a[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_a_name)
        
        tf_manager.SendTransform2tf(p = bat_bbox_b[0:3, -1], q = r2q(bat_bbox_b[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_b_name)
        
        tf_manager.SendTransform2tf(p = bat_bbox_c[0:3, -1], q = r2q(bat_bbox_c[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_c_name)
        
        tf_manager.SendTransform2tf(p = bat_bbox_d[0:3, -1], q = r2q(bat_bbox_d[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_d_name)

    
        #sendTf(p=cnc_table_base_to_cnc_machine_home[0:3, -1], q = r2q(cnc_table_base_to_cnc_machine_home[0:3, 0:3]), \
        #       parent_frame = cnc_table_base_frame, child_frame = cnc_machine_home_frame)
    
    # Get all points in relation to cnc machine zero frame.
    a_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_a)[0:3,-1]
    
    above_a = copy.deepcopy(a_in_cnc)
    above_a[2] +=initial_raise_above_mm / 1000
    
    b_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_b)[0:3,-1]
    c_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_c)[0:3,-1]
    d_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_d)[0:3,-1]

    #print(a_in_cnc)
    #print(b_in_cnc)
    #print(c_in_cnc)
    #print(d_in_cnc)
    
    out_gcode = []
    all_commands = [above_a, a_in_cnc, b_in_cnc, c_in_cnc, d_in_cnc, a_in_cnc, above_a]
    # Append initial
    # Append initial
    if enable_spindle:
        out_gcode.append('S1000M3')

    i=0
    for element in all_commands:
        feed_rate_tmp = int(feed_rate)
        
        if (i==0) or (i==len(all_commands)-1):
            # For first point above the smokedet, and for last point where we raise above the 
            # smoke detector, increase the feed rate since these are not cutting moves.
            feed_rate_tmp = int(initial_move_feed)
            
        gcode_line = "G01X{}Y{}Z{}F{}\n".format(int(1000*element[0]), int(1000*element[1]), int(1000*element[2]), \
                                                feed_rate_tmp)
        out_gcode.append(gcode_line)
        i+=1

    if enable_spindle:
        out_gcode.append('M5\n')
    
    return out_gcode

def cut_hca(n_passes = 1, do_cut = True):
    """ 
    Generate gcode to cut HCA within the CNC HCA vise

    Args:
    do_cut: If true, append Gcode to start spindle, and stop it at the end.
    n_passes: how many cut passes to do. If 1, just perform one cut to the right (CNC frame 'cnc_machine_home' negative x axis)
    If more, do cut to right, move down, then cut to left, ...


    """
    TRAVEL_SPEED = 1000
    CUT_SPEED = 180

    DZ_PER_PASS = 15 # Depth of cut per single pass
    N_PASSES = n_passes
    
    above_cut_start_position = [-150, -40, 35] # Position on left side/more positive cnc_machine_home x_axis
    above_cut_finish_position = [-200, -40, 35] # Position on right side/ negative cnc_machine_home x_axis

    gcode = []
    gcode.append(f'G01X{above_cut_start_position[0]}Y{above_cut_start_position[1]}Z{above_cut_start_position[2]}F{TRAVEL_SPEED}') # move from home to initial position
    
    # Turn on spindle
    if do_cut: gcode.append('S1000M3')
        
    for i in range(0, N_PASSES):
        current_dz_from_start_pose = int(DZ_PER_PASS * (i+1))
        # Pass starts on left side
        if not i%2:
            move_down_gcode = f'G01X{above_cut_start_position[0]}Y{above_cut_start_position[1]}Z{above_cut_start_position[2]- current_dz_from_start_pose}F{TRAVEL_SPEED}'
            gcode.append(move_down_gcode)
            # cutting move - Move to right
            cut_to_right_gcode = f'G01X{above_cut_finish_position[0]}Y{above_cut_finish_position[1]}Z{above_cut_finish_position[2]- current_dz_from_start_pose}F{CUT_SPEED}'
            gcode.append(cut_to_right_gcode)
        # Pass starts on right side
        else:
            move_down_gcode = f'G01X{above_cut_finish_position[0]}Y{above_cut_finish_position[1]}Z{above_cut_finish_position[2]- current_dz_from_start_pose}F{TRAVEL_SPEED}'
            gcode.append(move_down_gcode)
            cut_to_left_gcode = f'G01X{above_cut_start_position[0]}Y{above_cut_start_position[1]}Z{above_cut_start_position[2]- current_dz_from_start_pose}F{CUT_SPEED}'
            gcode.append(cut_to_left_gcode)
        
        
    # Move above final position either to above_cut_start_position or above_cut_finish_position, depending on N passes (where we finish)
    if not N_PASSES%2:
        # Will end up below above_cut_finish_position
        gcode.append(f'G01X{above_cut_start_position[0]}Y{above_cut_start_position[1]}Z{above_cut_start_position[2]}F{TRAVEL_SPEED}')
    else:
        # Will end up below above_cut_start_position
        gcode.append(f'G01X{above_cut_finish_position[0]}Y{above_cut_finish_position[1]}Z{above_cut_finish_position[2]}F{TRAVEL_SPEED}')
        
    # Turn off spindle
    if do_cut: gcode.append('M5')

    return gcode
