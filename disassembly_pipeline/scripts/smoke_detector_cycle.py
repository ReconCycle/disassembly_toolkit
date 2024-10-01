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
from ..utils.cell_init_utils import CellManager, initialize_robot
from ..utils.vision_utils import VisionUtils, set_realsense_height, get_realsense_height
from ..utils.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray

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
tool_panda_1 = "softhand"
tool_panda_2 = "variable_stiffness_gripper"

p1 = initialize_robot('panda_1', tool_panda_1)
p2 = initialize_robot('panda_2', tool_panda_2)

############################################################
# Create 'cy' class for many useful functionalities
############################################################
cy = Disassembly_cycle(p1, p2, use_moveit=False, initialize_using_cellmanager = True)

############################################################
# Prepare the workcell and robots
############################################################
cy.prepare_pneumatics()
cy.robot_go_to_init_and_open_gripper(p1)
cy.robot_go_to_init_and_open_gripper(p2)

############################################################
# Pickup the smokey smokey
############################################################
with p1 as robot:
    if robot._control_strategy != 'JointPositionTrajectory': 
        robot.ResetCurrentTarget()
        robot.Switch_controller(start_controller = 'JointPositionTrajectory')
    
    robot.error_recovery()
    
    robot.SetCartesianStiff_helper(m=0.9, n=0.85)  # Keep this in case we start using cart imp controller later
            
    # Set the nullspace
    neutral_q = (robot.q_max + robot.q_min) / 2
    neutral_q[3] = -2.553869
    robot.SetCartImpContNullspace(q=neutral_q,  k=[0, 0, 0, 4, 0, 0, 0])
    robot.SetCartesianStiff_helper(m=1.1, n=0.65)

    # Read pickup location and perform CMove
    offset_pickup_smoke = [0, 0, 0.1]
    x_pickup_smoke = t2x(pose_database['vision_table']['smoke_detector']['center'])
    x_pickup_smoke_above = x_pickup_smoke
    x_pickup_smoke_above[0:3] += offset_pickup_smoke
    robot.CMove(x_pickup_smoke_above, t=5, v_max_factor=0.4, a_max_factor=0.4)
    robot.CMove(x_pickup_smoke, t=5, v_max_factor=0.4, a_max_factor=0.4)
    robot.CMove(x_pickup_smoke_above, t=5, v_max_factor=0.4, a_max_factor=0.4)


    # Read battery location and perform CMove
    offset_pickup_battery = [0, 0, 0.1]
    x_pickup_battery = t2x(pose_database['vise_table']['smoke_detector']['battery'])
    x_pickup_battery_above = x_pickup_battery
    x_pickup_battery_above[0:3] += offset_pickup_battery
    robot.CMove(x_pickup_battery_above, t=5, v_max_factor=0.4, a_max_factor=0.4)
    robot.CMove(x_pickup_battery, t=5, v_max_factor=0.4, a_max_factor=0.4)
    robot.CMove(x_pickup_battery_above, t=5, v_max_factor=0.4, a_max_factor=0.4)
