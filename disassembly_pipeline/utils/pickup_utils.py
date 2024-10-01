# For typehint
from robotblockset_python.robots import robot as Robot
from robotblockset_python.transformations import *
import numpy as np
import copy

def generate_dT_grasp_poses(r_p_y_rotations = [[180, 0, 0]], x_y_z_offset_list = [0,0,0]):
    """
    There are several possible poses in which to pick up and object, e.g. battery.
    Each pickup pose is defined by a relative dT in relation to object T.
    (final pickup T in robot frame) = (object_T in robot frame) @ dT 
    This function return a list dT values based on a list of (RPY rotations, x_y_z_offsets)
    dT = R | t, where R is RPY rotation, t is (x,y,z) offset

    Example call:
    >>> r_p_y_rotations = [[180, 0, 90], [180, 0, -90], [180, 0, 0]]
    >>> x_y_z_offset_list = [[0,-0.02,0], [0, 0.02, 0], [0.02, 0, 0]]
    >>> dTs = generate_dT_grasp_poses(r_p_y_rotations, x_y_z_offset_list)
    """
    
    out_dTs = []
    for x_y_z_offset, r_p_y_rotation in zip(x_y_z_offset_list, r_p_y_rotations):
        dT = np.eye(4)
        dT[0:3, 0:3] = rot_x(r_p_y_rotation[0], unit = 'deg')@rot_y(r_p_y_rotation[1], unit = 'deg')@rot_z(r_p_y_rotation[2], unit='deg')
        dT[0:3, -1] = np.array(x_y_z_offset)
        out_dTs.append(dT)
    return out_dTs

def generate_pickup_poses_based_on_dT(object_center_T_in_robot_frame, list_of_dTs):
    """
    Generate pickup poses in robot frame given an object_center_T in robot frame and a list of relative grasp poses (dTs) in relation to object frame.
    pickup_pose = object_center_T_in_robot_frame @ dT
    These poses can later be evaluated and a most suitable one can be selected to enable grasping without hitting joint limits.

    Example call:
    >>> from disassembly_pipeline.utils.tf_utils import TFManager
    >>> from context_action_framework.vision_interface import VisionInterface
    >>> detection = vision_interface.get_detections()
    >>> object_center_T_in_robot_frame = tf_manager.tf2x(robot_base_frame, detection.tf_name)

    >>> list_of_dTs = [np.eye(4)]
    >>> pickup_Ts = generate_pickup_poses_based_on_dT(object_center_T_in_robot_frame, list_of_dTs)
    >>> example_pickup_t = pickup_Ts[0]

    """
    pickup_Ts = []
    for dT in list_of_dTs:
        pickup_Ts.append(object_center_T_in_robot_frame@dT)
    return pickup_Ts

def determine_optimal_pickup_T_based_on_reachability(robot: Robot, list_of_pickup_Ts, also_check_grasp_pose_above = False, grasp_pose_above_height = 0.1):

    is_pickup_T_valid = np.zeros(len(list_of_pickup_Ts)) # Keep track of which ones are valid
    robot.GetState()

    criteria_for_all_poses = []

    for i, pickup_T in enumerate(list_of_pickup_Ts):
        # Check if robot can reach it and not hit joint limits 
        x_pose_to_object_side = t2x(pickup_T)
        q = robot.x2q_invkin(x = x_pose_to_object_side, q_last = robot.q, dq_factor = 5)
        outside_limits = robot.CheckJointLimits(q)
        
        criteria_ok = False
        if not(outside_limits):
            criteria_ok = True

        criteria_for_all_poses.append(criteria_ok)
    
    # Check if all values in criteria_for_all_poses are False
    criteria_for_all_poses = np.array(criteria_for_all_poses)
    if not np.any(criteria_for_all_poses):
        raise ValueError("No valid inverse kinematics solution found for both grasp poses")

    print("criteria for all poses:", criteria_for_all_poses)
    # pick up first element that is true
    pose_to_object_side = list_of_pickup_Ts[np.where(criteria_for_all_poses)[0][0]]
        
    return dict(best_grasp_pose = pose_to_object_side, criteria_for_all_poses = criteria_for_all_poses)

def determine_optimal_grasp_pose_based_on_minimal_gripper_rotation_around_z_axis(robot, 
                                                                                list_of_grasp_Ts):
    """ 
    Given a list of grasp poses T, find the one which rotates the gripper the least in Z axis, based on current gripper pose.
    """
    robot.GetState()
    cur_robot_T = copy.deepcopy(robot.T)

    z_rot_list = []
    for grasp_T in list_of_grasp_Ts:
        EE_to_grasp_pose_T = np.linalg.inv(cur_robot_T)@grasp_T
        rpy = r2rpy(EE_to_grasp_pose_T[0:3, 0:3])
        z_rot_list.append(abs(rpy[0]))
    best_grasp_pose = list_of_grasp_Ts[np.argmin(z_rot_list)]

    return dict(best_grasp_pose = best_grasp_pose, criteria_for_all_poses = z_rot_list)
