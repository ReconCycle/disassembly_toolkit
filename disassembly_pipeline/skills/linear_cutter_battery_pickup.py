import time
import copy
import numpy as np
from robotblockset_python.transformations import *
from disassembly_pipeline.utils.pickup_utils import determine_optimal_grasp_pose_based_on_minimal_gripper_rotation_around_z_axis
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.skills.look_at_table import LookAtTable
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections


def linear_cutter_battery_pickup(robot_panda_2, cutter_module, tf_manager):
    GRASP_HEIGHT_Z = 0.01
    GRIPPER_ROTATION_Y = -10 # Deg
    MOVE_DURATION_TO_PICKUP = 2.5
    BATTERY_PICKUP_OFFSET = [0, -0.01, 0] # xyz offset from battery_center_T

    p2 = robot_panda_2

    skill_homing = RobotHoming()
    skill_look_at_table = LookAtTable()


    cutter_module.open_tray()
    skill_homing.on_enter(p2)
    skill_look_at_table.on_enter(robot = p2, location = 'table_cutter')

    p2.camera_vision_interface.enable_camera(True)
    time.sleep(1)

    all_detections = p2.camera_vision_interface.get_detections()
    centermost_battery = get_most_centered_detection_of_particular_class_from_detections(detections=all_detections, desired_detection_class= Label.battery)

    battery_x = tf_manager.tf2x(p2.Base_link_name, centermost_battery.tf_name)
    battery_T = x2t(battery_x)

    offset = np.array(BATTERY_PICKUP_OFFSET)
    offset_2 = copy.deepcopy(offset)
    offset_2[0:2] = -offset[0:2] # Only invert x and y, not z offset (dont wanna be moving down into the table)

    grasp_dt_1 = np.eye(4)
    grasp_dt_1[0:3, 0:3] = grasp_dt_1[0:3, 0:3]@rot_x(180, unit='deg')@rot_z(90, unit = 'deg')@rot_y(GRIPPER_ROTATION_Y, unit = 'deg')
    grasp_dt_1[0:3, -1] = offset

    grasp_dt_2 = np.eye(4)
    grasp_dt_2[0:3, 0:3] = grasp_dt_2[0:3, 0:3]@rot_x(180, unit='deg')@rot_z(-90, unit = 'deg')@rot_y(GRIPPER_ROTATION_Y, unit = 'deg')
    grasp_dt_2[0:3, -1] = offset_2

    grasp_T_1 = battery_T@grasp_dt_1
    grasp_T_2 = battery_T@grasp_dt_2

    final_grasp_T = determine_optimal_grasp_pose_based_on_minimal_gripper_rotation_around_z_axis(robot = p2, list_of_grasp_Ts=[grasp_T_1, grasp_T_2])['best_grasp_pose']

    final_grasp_T[2, -1] = GRASP_HEIGHT_Z
    final_above_grasp_T = copy.deepcopy(final_grasp_T)
    final_above_grasp_T[2, -1] += 0.05
    tf_manager.SendTransform2tf(p= final_grasp_T[0:3, -1], q = r2q(final_grasp_T[0:3, 0:3]), parent_frame = p2.Base_link_name, child_frame = 'battery_pickup')
