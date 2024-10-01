import numpy as np
import copy
import rospy 

from robotblockset_python.transformations import *
from disassembly_pipeline.poses.saved_positions import p2_q1_init
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections
from context_action_framework.types import Label

from robotblockset_python.robots import robot as Robot
from disassembly_pipeline.utils.tf_utils import TFManager
from disassembly_pipeline.utils.move_until_contact import move_until_contact
from robotblockset_python.transformations import *

import copy
import time


class WigglingBatteryRemoval(BaseSkill):
    def __init__(self, tf_manager):

        if tf_manager is None:
            tf_manager = TFManager()
        self.tf_manager = tf_manager

        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        out = self.on_enter()
        return out

    def on_enter(self, robot, vision_interface) -> SkillExecutionResult:

        vision_interface.enable_camera()
        time.sleep(1)

        all_detections = vision_interface.get_detections()
        centered_battery = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections,
                                                                                        desired_detection_class = Label.battery) 
        battery_in_robot_base_frame = self.tf_manager.tf2x(parent_frame = robot.Base_link_name, child_frame = centered_battery.tf_name)
        battery_T = x2t(battery_in_robot_base_frame)
        # Do wiggling
        rocking_grasp_object(robot = robot,
                        object_center_T = battery_T,
                        tf_manager = self.tf_manager)

        vision_interface.enable_camera(False)

        result = SkillExecutionResult()
        return result

    def execute(self):
        pass

    def on_exit(self):
        pass


def rocking_grasp_object(robot: Robot,
                         object_center_T: np.ndarray,
                         gripper_close_command = 3,
                         tf_manager: TFManager = None):
    """ Grasp the object (battery) to enable performing rocking battery removal. 
    
    object_center_T: (4,4) pose matrix of object (e.g. battery) which we want to grasp, defined in robot base frame.
    """

    initial_robot_controller = copy.deepcopy(robot._control_strategy)

    if not (isinstance(object_center_T, np.ndarray) and object_center_T.shape == (4,4)):
        raise ValueError("Invalid input arg object_center_T, should be np.array of shape (4,4))")
    # When battery x is given, initially the robot will move ABOVE battery_x by 
    Z_OFFSET_WHEN_MOVING_ABOVE = 0.05

    Z_OFFSET_WHEN_GRASPING = 0.00 #0.04
    Y_OFFSET_WHEN_GRASPING = 0.015
    GRIPPER_Y_ROTATION = -18 # Rotate gripper around Y axis, so it doesnt catch on smoke detector casing
    
    # Initial gripper offset from battery_x, so that we approach the battery from the side. battery_x is center of battery, and we don't want to 
    # hit the center with the gripper. Therefore an offset is specified for each gripper.
    dT_1 = np.eye(4)
    dT_2 = np.eye(4)
    dT_1[1, -1] += Y_OFFSET_WHEN_GRASPING
    dT_2[1, -1] -= Y_OFFSET_WHEN_GRASPING
    dT_1[2, -1] += Z_OFFSET_WHEN_GRASPING
    dT_2[2, -1] += Z_OFFSET_WHEN_GRASPING
    dT_1[0:3, 0:3] = dT_1[0:3, 0:3]@rot_x(180,unit='deg')@rot_z(-90, unit='deg')
    dT_2[0:3, 0:3] = dT_2[0:3, 0:3]@rot_x(180,unit='deg')@rot_z(90, unit='deg')

    # 2 possible grasp modes
    battery_grasp_T1 = object_center_T@dT_1
    battery_grasp_T2 = object_center_T@dT_2
    battery_grasp_T_modes = [battery_grasp_T1, battery_grasp_T2]

    poses_to_object_side_and_above = []
    poses_to_object_side = []
    for i in range(0, 2):
        # Generate robot poses
        pose_to_object_side_and_above = copy.deepcopy(battery_grasp_T_modes[i])
        pose_to_object_side = copy.deepcopy(battery_grasp_T_modes[i])

        pose_to_object_side_and_above[2, -1] += Z_OFFSET_WHEN_MOVING_ABOVE

        poses_to_object_side_and_above.append(pose_to_object_side_and_above)
        poses_to_object_side.append(pose_to_object_side)

    # Show frames
    if tf_manager is not None:
        sendTf = tf_manager.SendTransform2tf
        x1 = t2x(poses_to_object_side_and_above[0])
        sendTf(p = x1[0:3] , q = x1[3:], parent_frame = robot.Base_link_name, child_frame = 'rocking_pose_to_object_side_and_above')
        x2 = t2x(poses_to_object_side[0])
        sendTf(p = x2[0:3], q = x2[3:], parent_frame = robot.Base_link_name, child_frame = 'rocking_pose_to_object_side')
    
    # CHOOSE ONE GRASP POSE OUT OF TWO OPTIONS!
    side_q = []
    side_and_above_q = []
    for i in range(0, len(poses_to_object_side)):
        x_pose_to_object_side = t2x(poses_to_object_side[i])
        x_pose_to_object_side_and_above = t2x(poses_to_object_side_and_above[i])
        #print(x_pose_to_object_side)
        #print(x_pose_to_object_side_and_above)
        side_q.append(robot.x2q_invkin(x = x_pose_to_object_side, q_last = robot.q, dq_factor = 5))
        side_and_above_q.append(robot.x2q_invkin(x = x_pose_to_object_side_and_above, q_last = robot.q, dq_factor = 5))
        #print("Finished calculating invkin for idx {i}")

    if side_q[0] is not None and side_and_above_q[0] is not None:
        selection_idx = 0
    elif side_q[1] is not None and side_and_above_q[1] is not None:
        selection_idx = 1
    else:
        raise ValueError("No valid inverse kinematics solution found for both grasp poses")

    pose_to_object_side_and_above = poses_to_object_side_and_above[selection_idx]
    pose_to_object_side = poses_to_object_side[selection_idx]

    # Show frames
    if tf_manager is not None:
        sendTf = tf_manager.SendTransform2tf
        x1 = t2x(pose_to_object_side_and_above)
        sendTf(p = x1[0:3] , q = x1[3:], parent_frame = robot.Base_link_name, child_frame = 'rocking_pose_to_object_side_and_above')
        x2 = t2x(pose_to_object_side)
        sendTf(p = x2[0:3], q = x2[3:], parent_frame = robot.Base_link_name, child_frame = 'rocking_pose_to_object_side')

    # Robot moves
    r = robot
    r.error_recovery()
    r.Switch_controller(start_controller = 'CartesianImpedance')
    r.gripper.close(command = 0.2, stiffness = 1.3, sleep = False)
    r.gripper.close(command = 0.80, stiffness = 1.3, sleep = False)
    r.CMove(pose_to_object_side_and_above, 3)

    # Rotate a bit
    R = rot_y(GRIPPER_Y_ROTATION, unit = 'deg')
    r.CMoveFor(dx = R, t = 1, task_space = 'Tool')

    r.CMove(pose_to_object_side[0:3,-1], 2)

    # Move down until feeling contact
    ft = np.zeros((1,6))
    ft[0,:] = [0,0,-1, 0, 0, 0]
    r.CMoveFor([0,0,-0.02], 1)
    
    r.gripper.close(gripper_close_command)
    # Rotate back to vertical
    #R = rot_y(-GRIPPER_Y_ROTATION, unit = 'deg')
    #r.CMoveFor(dx = R, t = 1, task_space = 'Tool')

    # Show TF of battery again
    tf_manager.SendTransform2tf(p = object_center_T[0:3, -1], q = r2q(object_center_T[0:3, 0:3]), parent_frame = r.Base_link_name, child_frame = 'wiggling_battery_center')

    # Perform wiggling with robot 'r'
    # The function will grasp and wiggle for some seconds
    wiggle_motion(r = r,
                  force = 40,
                  tf_manager = tf_manager)

    r.CMove(pose_to_object_side_and_above, 3)
    r.CMoveFor([0,0,0.05], 1)

    # Move up and slightly release grip on battery
    # r.gripper.close(0.75)

    # Switch to original controller
    if r._control_strategy != initial_robot_controller:
        r.Switch_controller(start_controller = initial_robot_controller)

    return 0

def wiggle_motion(r,
                  rotate_wiggle_angle = 8,
                  wiggle_t = 0.25,
                  force = 10,
                  tf_manager = None):
    MAX_N_ROCKING = 20

    r.GetState()
    current_pose = r.x
    wiggle_pose_1 = x2t(current_pose)

    wiggle_pose_2 = copy.deepcopy(wiggle_pose_1)
    wiggle_pose_2[0:3, 0:3] = wiggle_pose_2[0:3, 0:3]@rot_z(rotate_wiggle_angle, unit='deg')

    dR_robot_base_to_EE = np.linalg.solve(wiggle_pose_1[0:3, 0:3], np.eye(3))

    FT_wiggle_1 = np.array([force, 0, 2, 0, 0, 0])
    FT_wiggle_2 = copy.deepcopy(FT_wiggle_1)
    FT_wiggle_2[0] = -FT_wiggle_2[0]

    FT_wiggle_1[0:3] = dR_robot_base_to_EE@FT_wiggle_1[0:3]
    FT_wiggle_2[0:3] = dR_robot_base_to_EE@FT_wiggle_2[0:3]

    assert r.gripper.Name == 'vsgripper'
    r.error_recovery()
    
    initial_control_strategy = copy.deepcopy(r._control_strategy)
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller = 'CartesianImpedance')
        
    r.SetCartesianStiff_helper(m = 1.2, n = 1.2)

    r.GetState()
    initial_robot_position = copy.deepcopy(r.x[0:3])
    for i in range(0, MAX_N_ROCKING):
        
        if tf_manager is not None:
            tf_manager.SendTransform2tf(p = wiggle_pose_1[0:3, -1] , q = r2q(wiggle_pose_1[0:3, 0:3]), parent_frame = r.Base_link_name, child_frame = 'wiggle_pose_1')
            tf_manager.SendTransform2tf(p = wiggle_pose_2[0:3, -1], q = r2q(wiggle_pose_2[0:3, 0:3]), parent_frame = r.Base_link_name, child_frame = 'wiggle_pose_2')
        r.CMove(x = wiggle_pose_1, t = wiggle_t, FT = FT_wiggle_1)
        # success_1, success_1_dP = rocking_complete_callback_based_on_position(robot = r, initial_x = initial_robot_position, threshold = 0.02, DEBUG = True)
        # success_force_1, success_force_1_avg_F_xy = rocking_complete_callback_based_on_force(robot = r, DEBUG = True)
        r.CMove(x = wiggle_pose_2, t = wiggle_t, FT = FT_wiggle_2)
        # success_2, success_2_dP = rocking_complete_callback_based_on_position(robot = r, initial_x = initial_robot_position, threshold = 0.02, DEBUG = True)
        # success_force_2, success_force_2_avg_F_xy = rocking_complete_callback_based_on_force(robot = r, DEBUG = True)
        # if success_1 or success_2:
        #     print("Rocking success detected due to position change, {}".format(max(success_1_dP, success_2_dP)))
        # if success_force_1 or success_force_2:
        #     print("Rocking success detected due low felt force in xy plane, {}".format(min(success_force_1_avg_F_xy, success_force_2_avg_F_xy)))
        #     break
    
    r.Switch_controller(start_controller = initial_control_strategy)

def rocking_complete_callback_based_on_position(robot, initial_x, threshold = 0.03, DEBUG = True):
    """ To detect when the battery is removed:
    - we look at the initial robot position. When after rocking, the position changes by more than threshold, we consider the movement completed.
    - we check that the gripper is not completely closed 

    Args:
    robot: robotblockset_python object
    initial_pose: initial pose upon which we applyFT
    threshold: absolute cartesian distance in meters between initial_pose and current (final) pose to consider the battery removed
    """
    r = robot
    
    r.GetState()
    initial_x = np.array(initial_x[0:3])
    cur_x = np.array(r._actual_int.x[0:3])

    difference = np.linalg.norm(initial_x - cur_x)
    
    if DEBUG:rospy.loginfo("rocking_complete_callback: difference between positions is {}".format(difference))

    if difference > threshold:
        return 1, difference
    
    return 0, difference

def rocking_complete_callback_based_on_force(robot, threshold_newtons = 2.5, DEBUG = True):
    """ To detect when the battery is removed:
    - we look at the initial robot position. When after rocking, the position changes by more than threshold, we consider the movement completed.
    - we check that the gripper is not completely closed 

    Args:
    robot: robotblockset_python object
    initial_pose: initial pose upon which we applyFT
    threshold: absolute cartesian distance in meters between initial_pose and current (final) pose to consider the battery removed
    """
    r = robot
    
    r.GetState()
    average_xy_force = np.linalg.norm(r.FT[0:2])

    if average_xy_force < threshold_newtons:
        return 1, average_xy_force

    if DEBUG:rospy.loginfo("rocking_complete_based_on_force: average force in x-y is {}".format(average_xy_force))

    return 0, average_xy_force


if __name__ == '__main__':
    """ This is example of how to run the battery removal. First you must init 
    cycle_manager and p2."""
    # 1. primer
    #battery_x = [ 0.441858, -0.105727,  0.080116,  0.022586, -0.740918,  0.668585, -0.05937 ]
    #battery_T = x2t(battery_x)
    #battery_T[0:3, 0:3] = np.eye(3)@rot_z(-90,unit='deg')

    # 2. primer
    battery_x = [ 0.441858, -0.055727,  0.160116,  0.022586, -0.740918,  0.668585, -0.05937 ]
    battery_T = x2t(battery_x)
    battery_T[0:3, 0:3] = np.eye(3)@rot_z(0,unit='deg')

    battery_x = t2x(battery_T)
    cyc_manager.sendTf(p = battery_x[0:3], q = battery_x[3:], parent_frame = r.Base_link_name)

    battery_T = x2t(battery_x)

    # hekatron_remove_battery(robot = p2, battery_T = battery_T, sendTf_fn = cyc_manager.sendTf)
