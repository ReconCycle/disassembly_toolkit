from disassembly_pipeline.workcell_modules.vision_module import VisionModule
from disassembly_pipeline.workcell_modules.panda_1_module import Panda1Module
from disassembly_pipeline.workcell_modules.panda_2_module import Panda2Module
from disassembly_pipeline.workcell_modules.cnc_module import CNCModule
from disassembly_pipeline.skills.drop_object import DropObject
from disassembly_pipeline.skills.pickup_object import PickupObject
from disassembly_pipeline.skills.pickup_detector_top import PickupDetectorTop
from disassembly_pipeline.skills.check_clamped_object import CheckClampedObject
from disassembly_pipeline.utils.tf_utils import TFManager, tf_obj2x
from disassembly_pipeline.skills.cnc_cut_smoke_detector import CNCCutSmokeDetector
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.skills.estimate_battery_contact_pose import EstimateBatteryContactPose
from disassembly_pipeline.skills.wiggling_battery_removal import rocking_grasp_object, WigglingBatteryRemoval
from disassembly_pipeline.skills.linear_cutter_battery_pickup import linear_cutter_battery_pickup
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections, ROSImageListener, check_if_detections_contain_label
from disassembly_pipeline.skills.exception_handling_skill_wrapper import ExceptionHandlingSkillWrapper
from disassembly_pipeline.skills.look_at_table import LookAtTable
from disassembly_pipeline.skills.jmove_above_table import JMoveAboveTable
from disassembly_pipeline.skills.linear_cutter_pcb_touchoff_and_cut import LinearCutterPCBTouchoffAndCut
from disassembly_pipeline.workcell_modules.vise_module import ViseModule
from disassembly_pipeline.workcell_modules.linear_pneumatic_cutter_module import LinearPneumaticCutterModule
from disassembly_pipeline.utils.Levering_block_v3 import move_until_contact
from disassembly_pipeline.utils.Levering_block_v3 import LeverBlock
from disassembly_pipeline.utils.multithreading import do_concurrently
from robotblockset_python.transformations import *
from robotblockset_python.transformations import x2t, r2q, rot_x, rot_y, rot_z, r2rpy, q2rpy
from context_action_framework.types import Label
import rospy
import numpy as np
import json
import time
import copy


def perform_hca_cycle(do_step_1_homing = True,
                      do_step_2_place_hca_into_vise = True,
                      do_step_3_levering = True,
                      do_step_4_move_to_cutter_do_touchoff_and_cut = True,
                      do_step_5_drop_pcb = True,
                      do_step_6_pick_up_and_drop_battery = True,
                      do_step_7_pick_up_hca_frame_from_vise_and_drop = True
                      ):
    skill_drop = DropObject(move_above_z = 0.08)

    if do_step_1_homing:
        skill_homing.on_enter(p1)
        skill_homing.on_enter(p2)
        vise_module.open()

    if do_step_2_place_hca_into_vise:
        # Get HCA detection
        detections = vi_basler.get_detections()
        hca_detection = get_most_centered_detection_of_particular_class_from_detections(detections=detections, desired_detection_class = Label.hca)
        skill_pickup.on_enter(detection = hca_detection, robot = p2)
        
        # Drop into table vise
        skill_drop = DropObject()
        skill_drop.on_enter(robot = p2, drop_location = 'table_vise', drop_object = hca_detection)
    if do_step_3_levering:
        
        # Look at table to detect gap
        skill_look_at_table.on_enter(robot = p2, location = 'table_vise', return_to_home = False)
        time.sleep(1)
        
        # Levering
        p2.error_recovery()
        x_above_levering = [-0.04780933, -0.60115153,  0.19001202,  0.04478305, -0.01314394,
                0.99878274,  0.01596091]

        r = p2

        #skill_jmove_above_table.on_enter(robot = p2, location= 'table_vise')
        r.gripper.open()
        
        vise_module.close()
        time.sleep(1)
        vise_module.open_side_jaws()

        r.CMove(x_above_levering, 1.2)
        
        p2.SetCollisionBehavior(F = 45, T = 5, restart = True)
        
        p2.error_recovery()
        p2.ResetCurrentTarget()
        skill_levering = LeverBlock(robot = p2, end_effector = 'softclaw', direction = 0, allowed_tool_angles = (None, None))
        skill_levering.on_enter(
                         robot = p2,
                         hca_type = 'kalo',
                         starting_cartesian_pose = [-0.04780933, -0.60115153,  0.21001202,  0.04478305, -0.01314394, 0.99878274,  0.01596091],
                         perform_adaptive_levering_using_known_positions = False,
                         step_1_move_above_init_pose = False,
                         step_2_perform_z_minus_search = True,
                         step_3_perform_x_plus_search = True,
                         step_4_perform_fulcrum_detect_rotation = True,
                         step_4_perform_PDMP = False,
                         linear_mv_velocity = 0.1,
                         rot_velocity = 12)

        p2.gripper.close(1.8)
        time.sleep(0.8)
        p2.CMoveFor([0,0,0.1], 1.1)
        p2.Switch_controller(start_controller = 'JointPositionTrajectory')
    if do_step_4_move_to_cutter_do_touchoff_and_cut:
        skill_move_pcb_to_cutter_and_cut_battery = LinearCutterPCBTouchoffAndCut()
        skill_move_pcb_to_cutter_and_cut_battery.on_enter(robot = p2, cutter_module = cutter_module, perform_cutting = True)
    if do_step_5_drop_pcb:
        skill_drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'pcb')
    if do_step_6_pick_up_and_drop_battery:
        HARDCODED_PICKUP = True
        if HARDCODED_PICKUP:
            cutter_module.open_tray()
            skill_homing.on_enter(p2)
            skill_look_at_table.on_enter(robot = p2, location = 'table_cutter')
            x_pickup = [ 0.50185321, -0.39844298,  0.00880007,  0.00508486,  0.97106678,
                   -0.07968814, -0.22506279]
            x_pickup_above = copy.deepcopy(x_pickup)
            x_pickup_above[2] += 0.15
            
            p2.CMove(x_pickup_above, 2)
            p2.CMove(x_pickup, 1.5)
            p2.gripper.close()
            p2.CMove(x_pickup_above, 2)
            skill_look_at_table.on_enter(robot = p2, location = 'table_cutter')
            skill_drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'battery', perform_initial_jmove= False)
        else:
            linear_cutter_battery_pickup(robot_panda_2 = p2, cutter_module = cutter_module, tf_manager = tf_manager)
    if do_step_7_pick_up_hca_frame_from_vise_and_drop:
        skill_homing.on_enter(p2)
        vise_module.open()
        skill_pickup.on_enter(robot = p2, object_class = 'kalo', pickup_location = 'table_vise')
        skill_drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'hca_frame', perform_initial_jmove = False)
        skill_homing.on_enter(p2)


def main():
    # Enable cameras
    realsense_img_listener_raw = ROSImageListener(image_topic = '/realsense/color/image_raw')
    realsense_img_listener_detections = ROSImageListener(image_topic = '/vision/realsense/colour')

    ENABLE_VISION = True
    if ENABLE_VISION:
        vision_module = VisionModule()
        vi_basler = vision_module.get_callable_submodules()['basler']['vision_interface']
        vi_realsense = vision_module.get_callable_submodules()['realsense']['vision_interface']
        vi_basler.enable_camera()
        vi_realsense.enable_camera()

    # Initialize robots
    p1_module = Panda1Module()
    p1_module.enable_airblock()
    p1 = p1_module.get_callable_submodules()['robot']
    p1.SetNewEEConfig('tc_and_3jaw_chuck.json')
    p1.SetLoggerLevel(level='info')

    p2_module = Panda2Module(activate_eye_in_hand_camera = ENABLE_VISION)
    p2 = p2_module.get_callable_submodules()['robot']
    p2.SetNewEEConfig('tc_and_vsg.json') # TODO add to Panda2Module, but then change toolchanger Pins so p2.tc doesnt open p1.tc
    p2.SetLoggerLevel(level='info')
    p2.SetVerbosityLevel(level='debug')
    p2.SetCollisionBehavior(F = np.array([45.0, 45.0, 45.0]), T = np.array([20.0, 20.0, 18.0]), tq = np.array([87, 87, 87, 87, 12, 12, 12])*0.95, restart = True)

    # Initialize TF manager
    tf_manager = TFManager()
    tf2x = tf_manager.tf2x
    sendTf = tf_manager.SendTransform2tf

    # Initialize skills
    skill_homing = RobotHoming()
    skill_pickup = PickupObject()
    pickup_top = PickupDetectorTop()
    skill_drop = DropObject(move_above_z = 0.08)
    check = CheckClampedObject(tf_manager = tf_manager)
    estimate_battery_pose_skill = EstimateBatteryContactPose()
    wiggling_skill = WigglingBatteryRemoval(tf_manager = tf_manager)
    skill_look_at_table = LookAtTable()
    skill_exception_handling_wrapper = ExceptionHandlingSkillWrapper()
    skill_jmove_above_table = JMoveAboveTable()
    skill_move_pcb_to_cutter_and_cut_battery = LinearCutterPCBTouchoffAndCut()

    vise_module = ViseModule(init_ros_node = True)
    cutter_module = LinearPneumaticCutterModule(init_ros_node = True)

    perform_hca_cycle() # Run the actual cycle

if __name__ == '__main__':
    main()