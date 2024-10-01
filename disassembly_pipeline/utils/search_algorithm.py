#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
import time
import os
import numpy as np
import copy
import rospy
#import sys
#sys.path.append('.robotblockset_python')

from robotblockset_python.robots import robot
from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *

from disassembly_pipeline.Levering_block_v3 import move_until_contact

   
def search_for_gap(robot,
                   starting_cartesian_position = None,
                   velocity_move_to_starting_position = 0.05,
                   task_space = 'World',
                   unit_dp_gap_direction = [0,0,-1],
                   unit_dp_orthogonal_to_gap= [0, -1, 0],
                   v_gap_direction = 0.01,
                   dz_threshold = 0.01,
                   dp_step = 0.005):
    """ Search algorithm to detect the gap.
    Currently, moves in 1D (with dp_step) and tries to find gap in dz direction.
    dz_threshold = 0.01 # Threshold to consider gap detected. Hardcoded threshold. Can be improved
    dp_step = 0.005 # How much to move horizontally per attempt to detect the gap. Hardcoded, can be improved.
    
    """
    #assert task_space in ['World', 'Tool']
    assert task_space in ['World'] # For tool task space, the success condition is not yet implemented.
    
    r = robot
    
    ts = robot.tsamp
        
    # dp = v * dt
    dp_gap_direction = (np.array(unit_dp_gap_direction) * v_gap_direction) * ts
    
    dp_orthogonal_to_gap = np.array(unit_dp_orthogonal_to_gap) * dp_step
    
    #print(ts, "dp", dp_gap_direction, "vel", dp_gap_direction/ts)
    #print(ts, dp_orthogonal_to_gap)
    
    # Start algorithm
    if r._control_strategy != "CartesianImpedance":
        r.Switch_controller(start_controller = 'CartesianImpedance')
        
    r.ResetCurrentTarget() # Error recovery already does resetcurrenttarget ? 
    r.error_recovery()
    r.SetCartesianStiff_helper(m=1, n=0.75)
    
    if starting_cartesian_position is not None:
        # Start at initial point, if specified. Otherwise, start wherever robot currently is.
        # If not there already, move
        r.GetState()
        p_err = np.linalg.norm(np.abs(r.x[0:3] - starting_cartesian_position[0:3]))
        if p_err > 0.01:
            rospy.loginfo("GapSearch moving to initial position")
            move_time = p_err / velocity_move_to_starting_position
            r.CMove(x = starting_cartesian_position, t = move_time)
        else:rospy.loginfo("GapSearch already at initial position")
    r.GetState()
    starting_cartesian_position = np.array(copy.deepcopy(r.x))
    # Start the search loop
    success = 0
    while success == 0:
        try:
            # Move down until contact detected. 
            # move_until_contact
            # If dz > dz_threshold : success
            # Else: Move back up, then move in 1D space and attempt to detect gap again
            recorded_F, success = move_until_contact(robot=r,
                       task_space = task_space,
                       mv_unit_direction = unit_dp_gap_direction,
                       mv_velocity = v_gap_direction,
                       rot_unit_direction= None,
                       rot_velocity = 0.01,
                       dt = None,
                       axes_to_monitor = [0,0,1,1,0,0],
                       max_allowed_values = [13,13,13,2,2,2],
                       use_abs = True,
                       max_allowed_t = 5,
                       allowed_tool_angles = (None,None),
                       zeroing_FT = [0, 0,0,0,0,0],
                       min_F_measurement_dt = 0.01)
            r.GetState()
            final_position = np.array(copy.deepcopy(r.x))
            
            dp = (final_position[0:3] - starting_cartesian_position[0:3]) * np.abs(unit_dp_gap_direction) # Zero out the coordinates which we dont care about (X and Y usually)
            dp = np.max(np.abs(dp))
            if dp > dz_threshold:
                # We detected the gap
                success = 1
                rospy.loginfo("Search for gap succes: dp = {}".format(dp))
                
            else:
                rospy.loginfo("Search for gap failure: dp = {}".format(dp))
                # Return to Position back above, from which we started the search.

                r.GetState()
                p_err = np.linalg.norm(np.abs(r.x[0:3] - starting_cartesian_position[0:3]))
                move_time = p_err / velocity_move_to_starting_position
                r.CMove(x = starting_cartesian_position, t = move_time)

                # Next position from where we will try to find gap
                starting_cartesian_position[0:3] = starting_cartesian_position[0:3] + dp_orthogonal_to_gap

                r.GetState()
                p_err = np.linalg.norm(np.abs(r.x[0:3] - starting_cartesian_position[0:3]))
                move_time = p_err / velocity_move_to_starting_position
                r.CMove(x = starting_cartesian_position, t = 3*move_time)
                                
        except KeyboardInterrupt:
            break
