######################################
######################################
######################################
############ THIS IS THE BARE ########
##### MINIMUM CODE REQUIRED FOR ######
##### INITIALIZING THE CELL ##########
##### KEEP UPDATED ###################
######################################
######################################
######################################

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

p1_q_init = pose_db['q_init']['p1']
p2_q_init = pose_db['q_init']['p2']

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