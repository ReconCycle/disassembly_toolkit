#! /bin/python3

# Autoreload modules during import so we don't have to restart the kernel after changing the code
import numpy as np
import sys
import matplotlib.pyplot as plt

import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import rospy

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper
from robotblockset_python.panda_ros import record_movement_in_while_loop

from disassembly_pipeline.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray
from disassembly_pipeline.tf_utils import GenericTransformListener
from disassembly_pipeline.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.multithreading import do_concurrently
from disassembly_pipeline.disassembly_blocks import MoveBlock
from disassembly_pipeline.cell_init_utils import CellManager

from disassembly_pipeline.saved_positions import p1_q1_init,p2_q1_init
from disassembly_pipeline.saved_positions import p1_q_vice_grip_hca, p1_q_above_vice_grip_hca,  p1_q1_above_sl
from disassembly_pipeline.saved_positions import p2_q_next_to_cutter, p2_q_in_cutter#, p2_q_pinpushinit, p2_q_pinpush_position

from disassembly_pipeline.demos_for_guests import demo_armwave, demo_kamera_cobra, demo_fastmove
from disassembly_pipeline.disassembly_cycle import Disassembly_cycle

def pick_up_from_vise_fn(robot_obj, x_nad_vise, dz=None, dz_fino=None):

    x_grip = [0.47870341, -0.52939923, 0.18828454, 0.02858605, -0.49992496, 0.84466692, 0.18919744]
    
    q_grip = [2.0260219896717513, -0.9128211119089101, -2.3124214543125086, -1.153660093734437, -0.36173234054584996, 1.5503414589926845, 2.478482675637407]
    r = robot_obj
    
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller =  'CartesianImpedance')
    
    r.error_recovery()
    #r.SetCartesianStiff_helper(m=0.8, n=0.7)
    #q_nad_vise = (0.16139887329383146, 0.6462348884219338, -1.0226668910824868, -1.5660149424574674, 0.7766294913142671, 1.7667423487220826, 1.777937873617677)

    # brez nepoterebnih rotacij poberi s poševno nagnjeno roko, griper zaprt do 0.5
    
    if np.sum(np.abs((np.array(r._actual_int.x[0:3]) - x_nad_vise[0:3]))) > 0.03:
        t_move = 5
    else:
        t_move = 2

    r.CMove(x_nad_vise, t_move, qdot_max_factor=0.45, qddot_max_factor = 0.45) # Tukaj preveri v kateri poziciji smo
    r.SetCartesianStiff_helper(m=0.8, n=0.7)
    
    r.gripper.close(0.65, sleep=True)
    r.gripper.close(0.55)
    
    # pejdi 6 cm dol
    # r.CMoveFor([0,0,-dz], 3)

    x_grip[0] -= 0.005
    x_grip[1] -= 0.005
    x_grip[2] -= 0.015
    r.CMove(x_grip, 3)

    # zapri griper
    r.gripper.close(0.90, sleep=True)

    # nastavi cart stiffnes nazaj na začetno vrednost 
    r.SetCartesianStiff_helper(m=1.2, n=0.9)
    
    # dvigni gor
    # r.CMoveFor([0,0,dz], 3, task_space = 'World')
    r.CMove(x_nad_vise, 3)
    return 0

def predstavitev_dan_odprtih_vrat(robot_obj, cycle_obj = None,
                                  dummy = None,
                                  go_to_init = True,
                                  pick_up_and_put_into_vise = False,
                                  rotate_vise = False,
                                  pick_up_from_vise = False,
                                  place_on_table = False,
                                  return_to_init = False,
                                  p2_pick_up_battery = False,
                                  do_dummy = False
                                 ):

    #if 1:
    cy = cycle_obj

    try:
    
        ### SAVED POSITIONS
        q_init = (0.128779, -1.318009, -0.570318, -2.275162, -0.846657, 1.038001, 0.429413) 

        #x_nad_vise = [0.478736, -0.536,  0.246156,  0.104992, -0.487868,  0.855668,  0.137088]
        x_nad_vise = [0.478736-0.005, -0.536,  0.246156,  0.104992, -0.487868,  0.855668,  0.137088]

        q_nad_vise = (0.181628, 0.047478, -0.206520, -2.090579, -0.006855, 2.084698, 2.146783)

        ### END SAVED POSITIONS

        cy.prepare_vision(activate_realsense=None, activate_basler = False)

        assert robot_obj.Name == 'panda_1'
        r = robot_obj

        r.tsamp = 1/200.
        r.error_recovery()
        if r._control_strategy != 'JointPositionTrajectory':
            r.ResetCurrentTarget() # So that _command_int.x will be where this controller starts PID
            r.Switch_controller(start_controller =  'JointPositionTrajectory')

        if go_to_init:
            #r.GetState();r.JMove(r._actual_int.q, 0.5)
            r.JMove(q_init,1.5, qddot_max_factor=0.3, qdot_max_factor= 0.3)
            
        if pick_up_and_put_into_vise:
            # Maybe assert we're somewhere safe ?
            r.gripper.open()
            if dummy:
                hca_x = [ 6.625383e-01, -3.324111e-02,  0.000000e+00,  6.123007e-17, -9.999628e-01, -8.619745e-03,5.278074e-19]
                hca_x = np.array(hca_x)
                cy.p1_pickup_hca_and_put_into_vice(init_safe = False, stop_after_raising = True,
                                                    ignore_bad_rotation = True, hca_x = hca_x)
                
            else:
                cy.p1_pickup_hca_and_put_into_vice(init_safe = False,
                                                    ignore_bad_rotation = True,
                                                    stop_after_raising = True)

            handle_pneumatics_jaws(ud =cy.ud, mainjaws_st= cy.mainjaws_st, sidejaws_st = cy.sidejaws_st, command=  'close')
            time.sleep(1)
            
        if pick_up_from_vise:
            r.gripper.open()
            handle_pneumatics_jaws(ud =cy.ud, mainjaws_st= cy.mainjaws_st, sidejaws_st = cy.sidejaws_st, command=  'open')
            # Assert close to initial pickup position
            pick_up_from_vise_fn(robot_obj = r,
                                x_nad_vise = x_nad_vise,
                                dz = 0.08,
                                dz_fino=0.02)
            
        if place_on_table:

            cy.prepare_vision(activate_basler = False)
            # Assert close to position above table
            #q, qdot,t  = perform_DMP(robot_obj = p1, space = 'cartesian')
            x_between = np.array([ 0.652749, -0.198482,  0.197319,  0.037327, -0.976034,  0.116913,  0.179711])
            #x_between_2 = np.array([ 0.714559,  0.028313,  0.045903,  0.0301  , -0.981963,  0.1431  ,  0.11985 ])
            x_end = np.array([ 0.714559,  0.0,  0.045903,  0.0301  , -0.981963,  0.1431  ,  0.11985 ])

            x_end_possibilities = {0 : [0,0], 1: [-0.02, -0.02], 2 : [-0.04, -0.04]} # To debug random drop we will set a fixed drop position out of a list
            index_to_choose = np.random.randint(low=0, high = 2)
            #index_to_choose = 1
            print("Chose index {} for random drop position".format(index_to_choose))
            x_end[0:2] += x_end_possibilities[index_to_choose]
            #x_end[0] -= 0.04 * np.random.randint(low = 0, high = 10) / 10   # Randomize the x position in regards to robot base C.S, by up to 4 cm
            #x_end[1] -= 0.04 * np.random.randint(low = 0, high = 10) / 10   # Randomize the y position in regards to robot base C.S, by up to 4 cm

            r.ResetCurrentTarget()
            if r._control_strategy != 'JointPositionTrajectory':
                r.Switch_controller(start_controller =  'JointPositionTrajectory')
            #r.GetState(); r.JMove(r._actual_int.q, 0.5)

            path = np.stack([r._command_int.x, x_between, x_end])
            r.CPath_new(path,t=5)
            r.gripper.open(0.55, sleep=True)
            
        if return_to_init:
            if r._control_strategy != 'JointPositionTrajectory':
                r.Switch_controller(start_controller =  'JointPositionTrajectory')
            r.JMove(q_init,3, qddot_max_factor=0.2, qdot_max_factor= 0.2)

        if p2_pick_up_battery:
            cy.p2_pick_up_battery(get_into_camera_position = True, safe_to_camera = True, pick_up_battery = True, return_to_init = True, sleep_t = 1.5, qdot_max_factor = 0.4, qddot_max_factor = 0.4)

    except rospy.ROSInterruptException:
        rospy.loginfo("Got keyboard interrupt") 
    return 0
    
    
    # Randomly place on table

#q, qdot,t = perform_DMP(robot_obj = p1, space = 'cartesian')

def initialize_system():
    ### Panda 1 init
    p1 = panda_ros('panda_1', init_node= True, init_frankadesk_gripper_TCP = True,
                start_controller = 'position_joint_trajectory_controller')

    p1.SetCollisionBehavior(F=50, T= 20, tq = 30)
    p1.error_recovery()
    sh_grp = SofthandGripper()
    p1.gripper = sh_grp
    p1.gripper.open(1)

    #p1.CMoveFor([0,0,0.1],3)

    rospy.loginfo("RECONCYCLE CELL INIT START")
    cellmanager = CellManager()
    j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st, sidejaws_st, \
    slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st, tfread_st,  get_next_action_st= cellmanager.init_cell()
    rospy.loginfo("RECONCYCLE CELL INIT COMPLETE")

    class r:
        def __init__(self, ns):
            self.Name = ns
            
    p2 = r(ns = 'panda_2')
    # Panda 2 init
    start_p2 = 0
    if start_p2:
        p2 = panda_ros('panda_2', start_controller = 'position_joint_trajectory_controller', init_frankadesk_gripper_TCP = True)
        p2.SetCollisionBehavior(F=50, T= 20, tq = 30)
        p2.error_recovery()
        sc_grp = VariableStiffnessGripper()
        p2.gripper = sc_grp
        p2.gripper.open()

        p2.GetState()
        p2.ResetCurrentTarget()
        p2.JMove(q = p2._actual_int.q, t = 0.2, traj='poly')

    cy = Disassembly_cycle(p1, p2, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st,
                        sidejaws_st, slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st,
                        tfread_st, get_next_action_st)

    rospy.set_param("/vision/basler/publish_labeled_img", True)
    rospy.set_param("/vision/realsense/publish_labeled_img", True)
    rospy.set_param("/vision/realsense/publish_depth_img", False)
    rospy.set_param("/vision/realsense/publish_cluster_img", False)
    rospy.set_param("/vision/realsense/do_gap_detection", False)

    cy.prepare_pneumatics()
    cy.set_cycle_speed('fast')

    #handle_pneumatics_jaws(ud =cy.ud, mainjaws_st= cy.mainjaws_st, sidejaws_st = cy.sidejaws_st, command=  'close')
    return p1, p2, cy

if __name__ == '__main__':

    p1, p2, cy = initialize_system()
    n_loop_replays = 5
    current_n_replay = 0
    while current_n_replay < n_loop_replays:
        current_n_replay +=1
        rospy.loginfo("Current n: {} / {}".format(current_n_replay, n_loop_replays)) 
        predstavitev_dan_odprtih_vrat(robot_obj = p1,
                                    cycle_obj = cy,
                                    go_to_init = True,
                                    pick_up_and_put_into_vise = True,
                                    rotate_vise = True, 
                                    pick_up_from_vise = True,
                                    place_on_table = True,
                                    return_to_init = True,
                                    p2_pick_up_battery = False,
                                    dummy = False
                                    )
