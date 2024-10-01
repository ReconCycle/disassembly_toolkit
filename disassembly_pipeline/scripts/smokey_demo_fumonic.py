import numpy as np
import json
import os
import sys
import matplotlib.pyplot as plt

# Set print options for numpy arrays
np.set_printoptions(precision=6,linewidth=110, suppress = True)

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.grippers import ToolChanger, ThreeJawChuck, dualVacuumGripper, VariableStiffnessGripper

import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import rospy

import warnings
warnings.filterwarnings('ignore')

from disassembly_pipeline.utils.Levering_block_v3 import move_until_contact
from disassembly_pipeline.utils.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.utils.tf_utils import GenericTransformListener
from disassembly_pipeline.utils.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.utils.change_tool import DropCurrentAndPickUpNewTool, DetectAndGrabSmokeDetector
from disassembly_pipeline.utils.multithreading import do_concurrently
from disassembly_pipeline.skills.disassembly_skills import Move
from disassembly_pipeline.utils.disassembly_manager import DisassemblyManager, DummyRobot
##############################################################################################################################################################################

pose_database_file = open('../pose_database.json')
pose_db = json.load(pose_database_file)
chuck = ThreeJawChuck()
vac_gripper = dualVacuumGripper()

# Robot to initialize
robots = ['panda_1']

if 'panda_1' in robots:
    # Panda 1 init
    p1 = panda_ros('panda_1', init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
    p1.Stop_controller()
    p1.SetNewEEConfig(tool_name='tc_and_adapter.json', restart=False)
    # p1.SetNewEEConfig(tool_name='tc_and_3jaw_chuck.json', restart=False)
    # p1.SetNewEEConfig(tool_name='tc_and_dual_vacuum_gripper.json', restart=False)
    p1.SetJointImpedanceFranka(np.array([10000]*7), restart=False)
    p1.SetCollisionBehavior(F=70, T=20, tq=30, restart=False)
    p1.Start_controller()
    p1.error_recovery()
    tc = ToolChanger()
    # tc.open()
    p1.GetState()
else:
    p1 = DummyRobot(name='panda_1')

if 'panda_2' in robots:
    p2 = panda_ros('panda_2', init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
    p2.Stop_controller()
    p2.SetNewEEConfig(tool_name='tc_and_adapter.json', restart=False)
    # p2.SetNewEEConfig(tool_name='tc_and_3jaw_chuck.json', restart=False)
    # p2.SetNewEEConfig(tool_name='tc_and_dual_vacuum_gripper.json', restart=False)
    p2.SetJointImpedanceFranka(np.array([10000]*7), restart=False)
    p2.SetCollisionBehavior(F=70, T=20, tq=30, restart=False)
    p2.Start_controller()
    p2.error_recovery()
    sc_grp = VariableStiffnessGripper()
    p2.gripper = sc_grp
    p2.gripper.open()
    p2.GetState()
else:
    p2 = DummyRobot(name='panda_2')


cyc_manager = DisassemblyManager(p1=p1, p2=p2)
pneum_manager = cyc_manager.pm
pneum_manager.prepare_pneumatics()

DropCurrentAndPickUpNewTool(p1, pose_db, tc, current_tool=None, new_tool='3JAWCHCK')
chuck.open()

# EITHER THIS
DetectAndGrabSmokeDetector(p1, chuck, cyc_manager, pose_db, smoke_detector='fumonic')
########################################################################
# OR THIS
# p1.JMove(pose_db['demo_poses']['smoke_detector_pickup_q'], t=2)

# smokedet_pickup_x = pose_db['demo_poses']['smoke_detector_pickup_x']
# smokedet_pickup_offset = np.array([0, 0, 0.1, 0, 0, 0, 0])

# p1.CMove(np.array(smokedet_pickup_x) + smokedet_pickup_offset, t=2)
# p1.CMove(smokedet_pickup_x, t=1)
# chuck.close()
# time.sleep(0.5)
########################################################################

# Go above CNC (joint move)
p1.JMove(pose_db['cnc_table']['above_chuck_fumonic_q']['p1'], t=2)

# Get the pose where to place CNC
smokedet_cnc_place_x = pose_db['cnc_table']['inside_chuck_fumonic_x']
smokedet_cnc_place_offset = np.array([0, 0, 0.1, 0, 0, 0, 0])

# Open chuck in the CNC
pneum_manager.handle_cnc_chuck('open')

# Place the smoke detector into CNC (cartesian move)
p1.CMove(np.array(smokedet_cnc_place_x) + smokedet_cnc_place_offset, t=2)
p1.CMove(smokedet_cnc_place_x, t=1)
chuck.open()
p1.CMove(np.array(smokedet_cnc_place_x) + smokedet_cnc_place_offset, t=2)

# Close the CNC chuck
pneum_manager.handle_cnc_chuck('close')

# Drop 3-finger gripper and pick up the vacuum gripper
DropCurrentAndPickUpNewTool(p1, pose_db, tc, current_tool='3JAWCHCK', new_tool='2VACGRPR')

import ipdb; ipdb.set_trace()

# Go above CNC again
p1.JMove(pose_db['cnc_table']['above_chuck_fumonic_q']['p1'], t=2)

# Get battery poses
battery_cnc_x = pose_db['demo_poses']['battery_in_cnc_x']
battery_cnc_offset = np.array([0, 0, 0.05, 0, 0, 0, 0])

# Go to the battery and activate vacuum
p1.CMove(np.array(battery_cnc_x) + battery_cnc_offset, t=1)
p1.CMove(battery_cnc_x, t=1)
#import ipdb; ipdb.set_trace()
vac_gripper.grasp()
p1.CMove(np.array(battery_cnc_x) + battery_cnc_offset, t=1)

# Go to dropbox and release vacuum
p1.JMove(pose_db['demo_poses']['battery_drop_q'], t=3)
vac_gripper.open()
time.sleep(0.2)

# Place back the vacuum gripper
DropCurrentAndPickUpNewTool(p1, pose_db, tc, current_tool='2VACGRPR', new_tool=None)

# This impossible calculation exits script
1/0

################## OLD STUFF ##################
###############################################
###############################################
###############################################

# p1.JMove(pose_db['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
# x_list = []
# x_list.append(p1._actual_int.x + np.array([0.2,0,0,0,0,0,0]))
# x_list.append(p1._actual_int.x + np.array([0.2,0,-0.1,0,0,0,0]))
# x_list.append(p1._actual_int.x + np.array([0.2,-0.1,-0.1,0,0,0,0]))
# x_list = np.array(x_list)

# p1.CPath_new(x_list, t=6, dense=True)
# p1.JMove(pose_db['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
# p1.CPath_BEST(x_list, t=3)
