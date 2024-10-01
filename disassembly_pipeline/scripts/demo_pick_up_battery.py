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

import warnings

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

if __name__ == '__main__':

    class Rob:
        def __init__(self, ns):
            self.Name = 'panda_1'
    p1 = Rob('panda_1')
    # Panda 2 init
    p2 = panda_ros('panda_2', start_controller = 'position_joint_trajectory_controller', init_frankadesk_gripper_TCP = True, init_node = True)
    #p2.SetCollisionBehavior(F=50, T= 20, tq = 30)
    p2.error_recovery()
    sc_grp = VariableStiffnessGripper()
    p2.gripper = sc_grp
    p2.gripper.open()

    p2.GetState()
    p2.ResetCurrentTarget()
    p2.JMove(q = p2._actual_int.q, t = 0.2, traj='poly')
    
    cellmanager = CellManager()
    j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st, sidejaws_st, \
    slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st, tfread_st,  get_next_action_st= cellmanager.init_cell()

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

    cy.p2_pick_up_battery(get_into_camera_position = True, safe_to_camera = True, pick_up_battery = True, return_to_init = True, sleep_t = 1.5, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
    """
    n_loop_replays = 2
    current_n_replay = 0
    while current_n_replay < n_loop_replays:
        current_n_replay +=1

        predstavitev_dan_odprtih_vrat(robot_obj = p1,
                                    cycle_obj = cy,
                                    go_to_init = True,
                                    pick_up_and_put_into_vise = True,
                                    pick_up_from_vise = True,
                                    place_on_table = True,
                                    return_to_init = False,
                                    dummy = False
                                    )"""
