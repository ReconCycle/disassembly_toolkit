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

from disassembly_pipeline.demos_for_guests import predstavitev_za_gospode



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
    p2.JMove(q = p2._command_int.q, t = 0.2, traj='poly')
    
    # Run demo 
    #predstavitev_za_gospode(p1 = p1, p2 = p2, n_retries = 1, standard_t = 2, robot_to_run = 'panda_1')
    #predstavitev_za_gospode(p1 = p1, p2 = p2, n_retries = 1, standard_t = 2, robot_to_run = 'panda_2')
    try:
        predstavitev_za_gospode(p1 = p1, p2 = p2, n_retries = 3, standard_t = 3.5, robot_to_run = 'both')
    except KeyboardInterrupt:
        pass #return 0

