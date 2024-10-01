
import matplotlib.pyplot as plt
import rospy
import time
import os
import numpy as np
import copy
from robotblockset_python.transformations import *
from franka_msgs.msg import FrankaState as fs # import because break condition for move_until_contact checks if robot entered reflex (r.state.robot_mode != fs.ROBOT_MODE_MOVE).
last_estimate_if_contact_print_time = 0

def move_until_contact(robot,
                       task_space = 'World',
                       mv_unit_direction = [1, 0, 0],
                       mv_velocity = 0.05,
                       rot_unit_direction= None,
                       rot_velocity = 0.01,
                       dt = None,
                       axes_to_monitor = [0,0,1,1,0,0],
                       max_allowed_values = [13,13,13,2,2,2],
                       use_abs = True,
                       max_allowed_t = 10,
                       allowed_tool_angles = (None,None),
                       zeroing_FT = [0, 0,0,0,0,0],
                       min_F_measurement_dt = 0.01,
                       use_goto_T = False,
                       sendTf = None,
                       additional_break_fns = {},
                       **kwargs):
    
    """Function to move in world coords until detecting contact based on either Forces or Moments 
    Input args:
                max_allowed_t     max amount of time that movement will take, if no contact detected after we return 0
                
                additional_break_fns: any function in this list will be called each loop. If it returns 1, break out and stop."""

    assert task_space in ["World", "Tool"]

    r = robot

    # Set low me
    prev_verbose = copy.deepcopy(r._verbose)
    r._verbose = 1 # So CMove_started msg is not shown

    sendTf_parent_frame = r.Base_link_name
    sendTf_child_frame_name = 'MOVE_UNTIL_CONTACT'
    
    if dt is None:dt = r.tsamp
    
    if mv_unit_direction is None:
        mv_unit_direction = [0,0,0]

    mv_direction = (np.array(mv_unit_direction) * mv_velocity) * dt

    if rot_unit_direction is None:
        rot_unit_direction = [0,0,0]

    rot_direction = (np.array(rot_unit_direction) * rot_velocity) * dt
    
    start_t = time.time()
    last_F_sample_t = start_t
    T = copy.deepcopy(r.T)
    contact = 0
    detected_success = 0
    F_arr = np.zeros((6,50000))
    F_n = 0
    forces, feeling_contact = estimate_if_contact(robot = r, axes_to_monitor= axes_to_monitor, use_abs = use_abs, max_allowed_values = max_allowed_values, zeroing_FT = zeroing_FT)

    # chkros() used to be in the while loop
    while (not feeling_contact):
        r.GetState()

        #T = copy.deepcopy(r.T)

        cur_t = time.time()
        #if self.estimate_if_contact(axes_to_monitor, use_abs = use_abs, max_allowed_values =max_allowed_values)
        #self.robot.CMoveFor([0,0,-0.005], t=0.5, task_space = task_space)
        
        #if mv_direction is None:
        cur_rpy = r2rpy(T[0:3,0:3], out= 'deg')
        cur_rpy += rot_direction
        
        T[0:3, 0:3] = rpy2r(cur_rpy, unit='deg', out='R')
    
        if task_space == 'World':
            T[0:3, 3] += mv_direction
        elif task_space == 'Tool':
            mv_T = np.eye(4)
            mv_T[0:3, 3] = mv_direction
            T = T@mv_T
        
        if use_goto_T:
            r.GoTo_T(T, v = np.zeros(6), FT = np.zeros(6), task_space = 'World', last_move = True)
        else:
            r.CMove(x=T,t=dt, task_space = 'World')

            #sendTf.SendTransform2tf(p = T[0:3, -1], q = r2q(T[0:3, 0:3]), parent_frame = sendTf_parent_frame, child_frame = sendTf_child_frame_name)

        #self.robot.GoTo_T(T=T, v =np.zeros((1,6)), FT = np.zeros((1,6)), wait=dt, varargs = None)

        # Break 1 - timeout
        if (cur_t-start_t) > max_allowed_t:
            print("move_until_contact breaking due to max_allowed_t")

            r._verbose = prev_verbose
            return F_arr, 0
        
        # Break 2 - robot reflex
        if r.state.robot_mode not in [fs.ROBOT_MODE_MOVE]:
                
            r._verbose = prev_verbose
            print("Move_until_contact aborting due to some robot reflex.")
            return F_arr, 0
        
        # Break 3 - all the additional_break_fns
        for key in additional_break_fns.keys():
            fn_output = key(**additional_break_fns[key])
            if fn_output == 1:
                print("Move_until_contact Breaking due to fn {0} with args:\n{1}".format(key, additional_break_fns[key]))
                r._verbose = prev_verbose
                return F_arr, 0

        
        # Sample Forces and append them to array.
        if cur_t - last_F_sample_t > min_F_measurement_dt:
            #o = chkros())
            if allowed_tool_angles[0] is not None:
                #r.GetState()
                Rz = r.R[2,2]
                if (Rz > allowed_tool_angles[0]) or (Rz < allowed_tool_angles[1]):
                    print("move until contact breaking due to reaching min angle:", Rz)
                    detected_success = 1

                    r._verbose = prev_verbose
                    return F_arr, detected_success
            
            forces, feeling_contact = estimate_if_contact(robot = r, axes_to_monitor= axes_to_monitor, use_abs = use_abs, max_allowed_values = max_allowed_values, zeroing_FT = zeroing_FT)
            detected_success = feeling_contact
            F_arr[:,F_n] = forces
            F_n+=1
            #F_arr.append(forces)
            last_F_sample_t = cur_t
        
    print("move until contact breaking, success detected")
    r._verbose = prev_verbose
    return F_arr, detected_success

def estimate_if_contact(robot,
                        axes_to_monitor = [0,0,1, 1, 0, 0],
                        max_allowed_values = [10, 10, 10, 1.5, 1.5, 1.5],
                        use_abs = True,
                        zeroing_FT = [0,0,0,0,0,0],
                        min_print_dt = 0.1):
        """ Function to check if robot end effector is in contact with stuff"""
        global last_estimate_if_contact_print_time # Not the prettiest but then we don't have to keep this function within a class.
        
        forces_desc = ['F_x', 'F_y', 'F_z', 'M_x', 'M_y', 'M_z']
        forces_unit = ['N', 'N', 'N', 'Nm', 'Nm', 'Nm'] 
        
        r = robot
        
        r.GetState() # Refresh forces
        tmp_F_arr = np.array(getattr(r.state, 'K_F_ext_hat_K')) # can also use O_F_ext_hat_K

        
        tmp_F_arr = tmp_F_arr-zeroing_FT
            
        if use_abs: tmp_F_arr = np.abs(tmp_F_arr)
        
        #print(forces[3:])
        for i in range(0,6):
            force_val = tmp_F_arr[i]
            
            if (axes_to_monitor[i]) and (force_val> max_allowed_values[i]):
                print("Stopping cuz %s = %.3f %s"%(forces_desc[i], force_val, forces_unit[i]))        
                return tmp_F_arr, 1 
            
        tt = time.time()
        if tt-last_estimate_if_contact_print_time > min_print_dt:
            tmp_F_arr = np.array(tmp_F_arr, dtype = np.float16)
            #print("Fs:", tmp_F_arr)
            last_estimate_if_contact_print_time = tt
        ## If no conditions are hit, smooth sailing, return 0
        return tmp_F_arr, 0