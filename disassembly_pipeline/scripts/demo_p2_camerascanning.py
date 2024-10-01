#!/usr/bin/env python
# coding: utf-8

# %%

# Autoreload modules during import so we don't have to restart the kernel after changing the code
import numpy as np
import sys
import matplotlib.pyplot as plt

# Set print options for numpy arrays
#np.set_printoptions(precision=6,linewidth=110)

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper

import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import rospy

import warnings
warnings.filterwarnings('ignore')

#from disassembly_pipeline.robot_init_utils import init_robot
from disassembly_pipeline.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray
from disassembly_pipeline.tf_utils import GenericTransformListener
from disassembly_pipeline.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.multithreading import do_concurrently
from disassembly_pipeline.disassembly_blocks import MoveBlock

from disassembly_pipeline.saved_positions import p1_q1_init,p2_q1_init
from disassembly_pipeline.saved_positions import p1_q_vice_grip_hca, p1_q_above_vice_grip_hca,  p1_q1_above_sl
from disassembly_pipeline.saved_positions import p2_q_next_to_cutter, p2_q_in_cutter#, p2_q_pinpushinit, p2_q_pinpush_position

from disassembly_pipeline.disassembly_cycle import Disassembly_cycle

from disassembly_pipeline.demos_for_guests import demo_scanning

if __name__ == "__main__":

    # Panda 1 init
    #p1 = init_robot('panda_1', start_controller = 'cartesian_impedance_controller')
    #p1 = panda_ros('panda_1', init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
    #p1.SetCollisionBehavior(F=50, T= 20, tq = 30)
    #sh_grp = SofthandGripper()
    #p1.gripper = sh_grp
    #p1.gripper.open(1)
    #p1.ResetCurrentTarget()
    #p1.JMove(p1_q1_init, t = 2, max_vel=0.5, max_acc= 0.5)
    #p1.GetState()
    #p1.JMove(p1.q, 0.1)
    class r:
        def __init__(self):
            self.Name = 'kk'
    p1 = r()
    
    # Panda 2 init
    #p2 = init_robot('panda_2', start_controller = 'joint_impedance_controller')
    p2 = panda_ros('panda_2', start_controller = 'position_joint_trajectory_controller', init_frankadesk_gripper_TCP = True)
    p2.SetCollisionBehavior(F=50, T= 20, tq = 30)
    p2.error_recovery()
    sc_grp = VariableStiffnessGripper()
    p2.gripper = sc_grp
    p2.gripper.open()

    p2.GetState()
    p2.ResetCurrentTarget()
    p2.JMove(q = p2._actual_int.q, t = 0.2, traj='poly')
    
    from disassembly_pipeline.cell_init_utils import CellManager

    # MAKE SURE panda_1 or panda_2 are initialized before calling this ! (they call init_node() which is required!)
    cellmanager = CellManager()

    j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st, sidejaws_st, \
    slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st, tfread_st,  get_next_action_st= cellmanager.init_cell()
    
    from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
    # This will actually move the robots
    #use_moveit = True
    use_moveit = False
    cy = Disassembly_cycle(p1, p2, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st,
                        sidejaws_st, slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st,
                        tfread_st, get_next_action_st, use_moveit=use_moveit)

    rospy.set_param("/vision/basler/publish_labeled_img", True)
    rospy.set_param("/vision/realsense/publish_labeled_img", True)
    rospy.set_param("/vision/realsense/publish_depth_img", False)
    rospy.set_param("/vision/realsense/publish_cluster_img", False)
    rospy.set_param("/vision/realsense/do_gap_detection", False)

    cy.prepare_pneumatics()
    #cy.prepare_vision(activate_basler = False, activate_realsense = False)
    #p1.tsamp = 0.005
    cy.set_cycle_speed('fast')

    #do_concurrently([[cy.robot_go_to_init_and_open_gripper, {'robot_obj':p1}],[cy.robot_go_to_init_and_open_gripper, {'robot_obj':p2}]], wait_until_task_completion=True)
        
    #hca_type = cy.p1_pickup_hca_and_put_into_vice(init_safe = True)
    #cy.p1_handle_hca_frame_to_bin() 
    #cy.p2_handle_hca_frame_to_bin() 

    #cy.p2_perform_pinpush(hca_type='qundis')
    #hca_type = 'kalo'
    #cy.p2_perform_levering(hca_type = 'qundis', direction_deg = 90, adaptive = True)
    #cy.p2_handle_pcb_to_cutter()
    #cy.robot_flip_hca(robot = p2, init_safe = True)
    #cy.prepare_vision(activate_basler = False, activate_realsense = True)
    #cy.p2_pick_up_battery(get_into_camera_position = True, safe_to_camera = False, pick_up_battery = True, return_to_init=True,
    #                     sleep_t = 1.5)
    # cy.set_cycle_speed('slow')

    #cy.perform_cycle_using_action_predictor()
    #cy.perform_cycle(force_pinpush = False, allow_pinpush = False, double_tap_vise = False, rotate_vise = False,
    #                 adaptive_levering = True, activate_cutter = False)
    
    # Run demo 
    #predstavitev_za_gospode(p1 = p1, p2 = p2, n_retries = 1, standard_t = 2, robot_to_run = 'panda_1')
    #predstavitev_za_gospode(p1 = p1, p2 = p2, n_retries = 1, standard_t = 2, robot_to_run = 'panda_2')
    try:
        demo_scanning(cy=cy, robot =p2)
    except KeyboardInterrupt:
        pass#return 0

