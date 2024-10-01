#!/usr/bin/env python
# coding: utf-8

# %%

# Autoreload modules during import so we don't have to restart the kernel after changing the code
import numpy as np
import sys
import matplotlib.pyplot as plt

# Set print options for numpy arrays
np.set_printoptions(precision=6,linewidth=110)

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

from disassembly_pipeline.robot_init_utils import init_robot
from disassembly_pipeline.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray
from disassembly_pipeline.disassembly_utils import get_correct_hand_orientation_from_hca_tf
from disassembly_pipeline.tf_utils import GenericTransformListener
from disassembly_pipeline.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.multithreading import do_concurrently
from disassembly_pipeline.disassembly_blocks import MoveBlock
from disassembly_pipeline.move_utils import time_to_move_to_x

from disassembly_pipeline.saved_positions import p1_q1_init,p2_q1_init
from disassembly_pipeline.saved_positions import p1_q_vice_grip_hca, p1_q_above_vice_grip_hca,  p1_q1_above_sl
from disassembly_pipeline.saved_positions import p2_q_next_to_cutter, p2_q_in_cutter#, p2_q_pinpushinit, p2_q_pinpush_position

from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
from disassembly_pipeline.cell_init_utils import CellManager



if __name__ == "__main__":

    # Panda 1 init
    #p1 = init_robot('panda_1', start_controller = 'cartesian_impedance_controller')
    p1 = panda_ros('panda_1', init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
    p1.SetCollisionBehavior(F=50, T= 20, tq = 30)
    sh_grp = SofthandGripper()
    p1.gripper = sh_grp
    p1.gripper.open(1)
    #p1.ResetCurrentTarget()
    #p1.JMove(p1_q1_init, t = 2, max_vel=0.5, max_acc= 0.5)
    p1.GetState()
    p1.JMove(p1.q, 0.1)

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


    # MAKE SURE panda_1 or panda_2 are initialized before calling this ! (they call init_node() which is required!)
    cellmanager = CellManager()
    j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st, sidejaws_st, \
    slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st, tfread_st,  get_next_action_st= cellmanager.init_cell()

    

    from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
    # This will actually move the robots

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
    
    # Run the cycle
    cy.perform_cycle(force_pinpush = False, allow_pinpush = False, double_tap_vise = False, rotate_vise = False,
                 adaptive_levering = True, activate_cutter = False)
    
