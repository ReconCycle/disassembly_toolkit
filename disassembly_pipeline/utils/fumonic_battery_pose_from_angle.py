from robotblockset_python.transformations import *
from robotblockset_python.robots import robot as Robot
import time
import copy
import numpy as np
from disassembly_pipeline.skills.look_at_table import LookAtTable
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.utils.cnc_utils import generate_battery_contact_cutting_gcode
from disassembly_pipeline.utils.tf_utils import TFManager
from context_action_framework.vision_interface import VisionInterface
from context_action_framework.types import Label
from disassembly_pipeline.utils.pickup_utils import determine_optimal_grasp_pose_based_on_minimal_gripper_rotation_around_z_axis
from disassembly_pipeline.utils.pickup_utils import determine_optimal_pickup_T_based_on_reachability
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections

def detect_battery_angle_and_perform_cnc_cut(eye_in_hand_camera_robot: Robot,
                                             battery_pickup_robot: Robot,
                                             cnc_client = None,
                                             tf_manager: TFManager = None,
                                             step_1_perform_homing: bool = True,
                                             step_2_look_at_table: bool = True,
                                             step_3_return_robot_to_home: bool = True,
                                             step_4_perform_cnc_contact_cutting: bool = True,
                                             step_5_pick_up_battery: bool = True,
                                             cnc_machine_frame = 'cnc_machine_home',
                                             battery_angle_frame = 'table_cnc_center'
                                             ):
    
    RAISE_CNC_CUTTING_Z_MM = 0 # For simulation, you can cut higher and not actually cut battery contacts

    skill_homing = RobotHoming()
    skill_look_at_table = LookAtTable()

    r = eye_in_hand_camera_robot
    if step_1_perform_homing:
        skill_homing.on_enter(robot= r)

    if cnc_client is not None: cnc_client.move_chuck('close')

    if step_2_look_at_table:
        skill_look_at_table.on_enter(robot = r, location = 'table_cnc', location_subframe = 'chuck_fumonic')

    r.camera_vision_interface.enable_camera(True)
    time.sleep(1)
    vision_interface = r.camera_vision_interface
    angle_deg = get_battery_rotation_within_smoke_detector_in_machine_z_axis(tf_manager = tf_manager, vision_interface = vision_interface)
    print("Detected battery angle: {}".format(angle_deg))

    if step_3_return_robot_to_home:
        r.CMoveFor(dx = [0, -0.2, 0], t = 2) 
        skill_homing.on_enter(r)

    # Convert battery angle in battery_angle_frame to cnc_machine_frame
    frames_dT = x2t(tf_manager.tf2x(cnc_machine_frame, battery_angle_frame))
    frame_difference_angle = np.rad2deg(r2rpy(frames_dT[0:3, 0:3]))[0]

    gcode = generate_battery_contact_cutting_gcode(device_center_in_cnc_machine_home_frame_mm = [-52, -162, -20 + RAISE_CNC_CUTTING_Z_MM],
                                               safe_height_above_z = 35,
                                               rotation_deg = angle_deg + frame_difference_angle,
                                               enable_spindle = True)

    if step_4_perform_cnc_contact_cutting:
        cnc_client.call_server(gcode = gcode, return_to_home = True)

    if step_5_pick_up_battery:
        # Check battery angle with panda_2

        # Determine battery pose in panda_1 base frame
        battery_x_y_z_offset_rotated, battery_parent_frame = fumonic_battery_pose_from_battery_angle(battery_angle = angle_deg, tf_manager = tf_manager)

        battery_T = battery_x_y_z_offset_rotated
        # Recalulate in pickup robot frame

        #p1_base_to_cnc_machine_home @ cnc_machine_home_to_battery_T = p1_base_to_battery_T
        p1_base_to_cnc_machine_home = x2t(tf_manager.tf2x(parent_frame = battery_pickup_robot.Base_link_name, child_frame = battery_parent_frame))
        p1_base_to_battery_T = p1_base_to_cnc_machine_home@battery_T

        battery_pickup_x_1, battery_pickup_x_2, battery_x_in_robot_frame = get_fumonic_battery_pickup_T(battery_x_in_robot_frame= t2x(p1_base_to_battery_T))

        ### DETERMINE OPTIMAL GRASP POSE
        # out_dict = determine_optimal_grasp_pose_based_on_minimal_gripper_rotation_around_z_axis(robot = battery_pickup_robot, 
        #                                                                              list_of_grasp_Ts = [x2t(battery_pickup_x_1), x2t(battery_pickup_x_2)])
        out_dict = determine_optimal_pickup_T_based_on_reachability(robot = battery_pickup_robot, 
                                                                    list_of_pickup_Ts = [x2t(battery_pickup_x_1), x2t(battery_pickup_x_2)])
        best_grasp_x = t2x(out_dict['best_grasp_pose'])
              
        ### END DETERMINE OPTIMAL GRASP POSE
        battery_pickup_x_1 = best_grasp_x


        tf_manager.SendTransform2tf(p = battery_pickup_x_1[0:3], q = battery_pickup_x_1[3:], parent_frame = battery_pickup_robot.Base_link_name, child_frame = 'battery_pickup')
        print("Battery pickup Rpy", r2rpy(q2r(battery_pickup_x_1[3:])))

        battery_pickup_x_1_above = copy.deepcopy(battery_pickup_x_1)
        battery_pickup_x_1_above[2] += 0.05

        #skill_homing.on_enter(battery_pickup_robot)
        # TODO delete GetStates if it doenst help
        battery_pickup_robot.error_recovery()
        battery_pickup_robot.CMove(battery_pickup_x_1_above, 3.5)
        battery_pickup_robot.GetState()
        print(np.linalg.norm(battery_pickup_robot.x_err[0:2]))
        battery_pickup_robot.gripper.close()
        battery_pickup_robot.CMove(battery_pickup_x_1, 3.5)
        battery_pickup_robot.GetState()
        print(np.linalg.norm(battery_pickup_robot.x_err[0:2]))
        battery_pickup_robot.error_recovery()
        time.sleep(1.5)
        print(np.linalg.norm(battery_pickup_robot.x_err[0:2]))
        battery_pickup_robot.CMove(battery_pickup_x_1_above, 1.5)
        battery_pickup_robot.GetState()
        print(np.linalg.norm(battery_pickup_robot.x_err[0:2]))
        battery_pickup_robot.error_recovery()

    return dict(gcode = gcode,
                battery_angle_deg = angle_deg,
                battery_T = battery_x_y_z_offset_rotated,
                battery_T_parent_frame = battery_parent_frame)

def get_battery_rotation_within_smoke_detector_in_machine_z_axis(tf_manager: TFManager,
                                                                 vision_interface: VisionInterface,
                                                                 machine_base_frame = 'table_cnc_center'):
    """ Get rotation of smoke detector battery in relation to smoke detector center, expressed in machine_base_frame's z-axis """
    # TODO Consider moving to disassembly_pipeline/utils/vision_utils.py

    all_detections = vision_interface.get_detections()
    battery = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections, desired_detection_class = Label.battery)
    smoke_detector_insides = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections, desired_detection_class = Label.smoke_detector_insides)

    battery_frame = battery.tf_name
    smoke_detector_insides_frame = smoke_detector_insides.tf_name
    
    bat_x = tf_manager.tf2x(parent_frame = machine_base_frame, child_frame = battery_frame)
    insides_x = tf_manager.tf2x(parent_frame = machine_base_frame, child_frame = smoke_detector_insides_frame)
    
    dx_insides_to_battery = -np.array(insides_x[0:3]) + np.array(bat_x[0:3])
    angle_rad = np.arctan2(dx_insides_to_battery[1], dx_insides_to_battery[0])
    angle_deg = angle_rad * 180 / np.pi

    return angle_deg

def fumonic_battery_pose_from_battery_angle(battery_angle = 0,
                                            machine_base_frame = 'table_cnc_center',
                                            tf_manager: TFManager = None,
                                            device_center_in_machine_frame_mm = [106, 141.7, 164],
                                            battery_x_y_z_offset_mm_at_0_deg_rotation = [20.5,0,0]):
    """ 
    device_center_in_machine_frame_mm = [106, 139.7, 164],    old center
    """
    #FIXED_ROTATION_OFFSET = +4  #Hardcoded since vision seems to have some error. This should ideally be set to 0.
    FIXED_ROTATION_OFFSET = +3
    R = rot_z(battery_angle, unit='deg')
    battery_x_y_z_offset_rotated = R@battery_x_y_z_offset_mm_at_0_deg_rotation

    final_battery_pose = device_center_in_machine_frame_mm + battery_x_y_z_offset_rotated
    final_battery_pose_m = final_battery_pose/1000
 
    battery_R = np.eye(3)@rot_z(90 + battery_angle + FIXED_ROTATION_OFFSET, unit='deg') # At 0 rotation, battery x axis is rotated 90 deg in relation to   

    q = r2q(battery_R)
    battery_T = np.eye(4)
    battery_T[0:3, 0:3] = q2r(q)
    battery_T[0:3, -1] = final_battery_pose_m
    if tf_manager is not None:
        tf_manager.SendTransform2tf(p = final_battery_pose_m, q= q, parent_frame = machine_base_frame, child_frame = 'battery_pose_fumonic')

    return battery_T, machine_base_frame

def get_fumonic_battery_pickup_T(battery_x_in_robot_frame):
    #OFFSET_X_Y_Z = [0.012, 0.008, 0]
    #OFFSET_X_Y_Z = [0.002, 0.005, 0]
    # Trying without the 0.01 offset
    OFFSET_X_Y_Z = np.array([0, 0, 0])
    OFFSET_X_Y_Z_2 = copy.deepcopy(OFFSET_X_Y_Z)
    OFFSET_X_Y_Z_2[0:2] = -OFFSET_X_Y_Z[0:2]

    battery_pickup_dT = np.eye(4)
    battery_T_in_robot_frame = x2t(battery_x_in_robot_frame)
        
    battery_pickup_dT_1 = np.eye(4)
    battery_pickup_dT_1[0:3,0:3] = rot_x(180, unit='deg')@rot_z(-90, unit='deg')
    battery_pickup_dT_1[0:3, -1] = OFFSET_X_Y_Z
    battery_pickup_dT_2 = np.eye(4)
    battery_pickup_dT_2[0:3,0:3] = rot_x(180, unit='deg')@rot_z(+90, unit='deg')
    battery_pickup_dT_2[0:3, -1] = OFFSET_X_Y_Z
    
    
    battery_pickup_T_1 = battery_T_in_robot_frame@battery_pickup_dT_1
    battery_pickup_x_1 = t2x(battery_pickup_T_1)
    
    battery_pickup_T_2 = battery_T_in_robot_frame@battery_pickup_dT_2
    battery_pickup_x_2 = t2x(battery_pickup_T_2)
    
    return battery_pickup_x_1, battery_pickup_x_2, battery_x_in_robot_frame