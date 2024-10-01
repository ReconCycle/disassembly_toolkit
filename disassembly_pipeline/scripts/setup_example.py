import sys
import time
import copy
import json
import rospy
import warnings
import numpy as np
import tf2_ros as tf2
from rospy import Time
from franka_msgs.msg import FrankaState as fs

warnings.filterwarnings('ignore')

from robotblockset_python.transformations import t2x
from robotblockset_python.grippers import SofthandGripper, VariableStiffnessGripper
from ..utils.Levering_block_v2 import LeverBlock
from ..skills.disassembly_skills import Move
from ..utils.multithreading import do_concurrently
from ..utils.vision_utils import set_realsense_height
from ..utils.disassembly_manager import Disassembly_cycle
from ..utils.cell_init_utils import CellManager
from ..utils.robot_quick_init import initialize_robot, DummyRobot
from ..utils.vision_utils import VisionUtils, set_realsense_height, get_realsense_height
from ..utils.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice
from ..utils.pneumatics_utils import move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray

from ..robot_ee_settings.gripper_dict import gripper_dict
sys.path.append('/ros_ws/src/context_action_framework')

pose_database_file = open('../pose_database.json')
pose_database = json.load(pose_database_file)

############################################################
# Initialize workcell utilities
############################################################
cellmanager = CellManager()

j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, \
    mainjaws_st, sidejaws_st, slider_st, clamptray_st, rotate_holder_st, \
    cutter_st, cutter_airblower_st, tfread_st, get_next_action_st = cellmanager.init_cell()


############################################################
# Initialize robots
############################################################
tool_panda_1 = None
tool_panda_2 = None

p1 = initialize_robot('panda_1', tool_panda_1)
p2 = DummyRobot('panda_2')

############################################################
# Create 'cy' class for many useful functionalities
############################################################
cy = Disassembly_cycle(p1, p2, use_moveit=False, initialize_using_cellmanager = True)

############################################################
# Prepare the workcell and robots
############################################################
cy.prepare_pneumatics()

import ipdb; ipdb.set_trace()