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
from disassembly_pipeline.utils.vision_utils import get_most_centered_detection_of_particular_class_from_detections, check_vision_is_working, ROSImageListener, check_if_detections_contain_label
from disassembly_pipeline.skills.exception_handling_skill_wrapper import ExceptionHandlingSkillWrapper
from disassembly_pipeline.skills.look_at_table import LookAtTable
from disassembly_pipeline.skills.jmove_above_table import JMoveAboveTable
from disassembly_pipeline.utils.fumonic_battery_pose_from_angle import detect_battery_angle_and_perform_cnc_cut
from disassembly_pipeline.utils.fumonic_battery_pose_from_angle import get_fumonic_battery_pickup_T
from disassembly_pipeline.utils.change_tool import change_tool
from robotblockset_python.transformations import *
import copy
from robotblockset_python.ros.grippers_ros import ThreeJawChuck, OffsetVacuumGripper
from robotblockset_python.transformations import x2t, r2q, rot_x, rot_y, rot_z, r2rpy, q2rpy
from context_action_framework.types import Label
import rospy
import numpy as npii
import json
import time


def perform_hekatron_cycle(step_1_put_smokedet_into_cnc_and_cut = False,
                           step_2_remove_battery_cover = False,
                           step_3_remove_battery_by_wiggling = False,
                           step_4_pickup_smokedet_and_drop_into_bin = False
                           ):
    if step_1_put_smokedet_into_cnc_and_cut:
        cnc_cut_skill.unclamp()
        home_all_robots()
        smoke_detector_detection = panda_1_pick_up_smoke_detector_from_table_vision_and_place_in_cnc()
        # Find battery cover rotation and do CNC cut
        hekatron_cnc_cut(smoke_detector_detection = smoke_detector_detection)
    if step_2_remove_battery_cover:
        # Look at it again and detect whether battery covered is seen or not.
        hekatron_remove_battery_cover()
    if step_3_remove_battery_by_wiggling:
        # Wiggling battery removal
        result = check.on_enter(robot = p2, location = None, object_to_check = None, return_to_home = False,
                                    only_move_to_position_and_ignore_detecting_objects = True)
        all_detections = vi_realsense.get_detections() # Detect either battery_covered or 
        if check_if_detections_contain_label(all_detections, Label.battery):
            # TODO add skill_exception_handling_wrapper
            panda_2_wiggling_battery_removal()
            # panda_2 drop battery
            panda_2_drop_battery()
    if step_4_pickup_smokedet_and_drop_into_bin:
        # Smoke detector dropping in 
        panda_1_pick_up_smoke_detector_from_cnc_and_place_on_table()
        home_all_robots()

def perform_fumonic_cycle(step_0_put_smokedet_into_cnc = False,
                          step_1_cut_smoke_detector = False,
                          step_2_remove_battery_cover_and_drop_to_bin = False,
                          step_3_do_toolchange = False,
                          step_4_do_homing_after_toolchange = False,
                          step_5_detect_battery_do_cut_and_remove_cover = False,
                          step_6_do_toolchange_to_3jaw = False,
                          step_7_pickup_smokedet_and_drop = False):
    cnc_cut_skill.unclamp()
    home_all_robots()
    if step_0_put_smokedet_into_cnc:
        smoke_detector = panda_1_pick_up_smoke_detector_from_table_vision_and_place_in_cnc()
        # Dummy look at skill
    if step_1_cut_smoke_detector:
        cnc_cut_skill.clamp()
    
        skill_look_at_table.on_enter(robot = p2,
                                     location = 'table_cnc',
                                     location_subframe = 'chuck_fumonic',
                                     return_to_init_q = True,
                                     return_to_home = True)
        # CNC Cut cover
        cnc_cut_skill.on_enter_fumonic(enable_spindle = True)
        # Open and close clamp in case the smoke det moved out
        cnc_cut_skill.unclamp()
        time.sleep(0.5)
        cnc_cut_skill.clamp()
    if step_2_remove_battery_cover_and_drop_to_bin:
        cnc_cut_skill.clamp()

        # Remove cover.
        panda_2_remove_fumonic_cover()
        # Drop battery cover
        p2.JMove(q = [0.5600515724870196, -0.7145430420825356, 0.5159374882045544, -2.4382984497500906, 0.8953325677580303, 2.192958761767145, 2.8659338815187416], t= 2.5)
        drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'battery_covered')

    if step_3_do_toolchange:
    # Cut battery contacts and Remove battery using vacuum gripper
        change_tool(robot = p1, desired_tool = 'tc_and_vacuum_demo.json')
        
    if step_4_do_homing_after_toolchange:
        print("DEBUG: panda 1 homing after toolchange")
        skill_homing.on_enter(p1)
        p1.JMoveFor(dq = [0,0,0,0,0,0,-np.pi], t = 2) # So robot is not close to joint limit

    if step_5_detect_battery_do_cut_and_remove_cover:
        print("DEBUG: cutting battery contacts")
        out = detect_battery_angle_and_perform_cnc_cut(eye_in_hand_camera_robot = p2,
                                                     battery_pickup_robot = p1,
                                                     cnc_client = cnc_cut_skill.cnc_cl,
                                                     tf_manager = tf_manager,
                                                     step_1_perform_homing = True,
                                                     step_2_look_at_table = True,
                                                     step_3_return_robot_to_home = True,
                                                     step_4_perform_cnc_contact_cutting = True,
                                                     step_5_pick_up_battery = True)
                                                     
        print("DEBUG: panda 1 dropping battery")
        drop_q_panda_1_battery = [0.3162027252573804, -0.9924010692024489, -0.6820927620519671, -2.8226129008440988, -0.6008581263025601, 1.9004807448519598, -0.9480955968556877]
        p1.error_recovery()
        p1.JMove(drop_q_panda_1_battery, 2)
        p1.gripper.open()
        #skill_drop.on_enter(robot = p1, drop_location = 'table_cnc', drop_object = 'battery', perform_initial_jmove = False)
        skill_homing.on_enter(p1)
    if step_6_do_toolchange_to_3jaw:
        print("DEBUG: panda 1 changing tool to 3jaw chuck")
        # Toolchange
        change_tool(robot = p1, desired_tool = 'tc_and_3jaw_chuck.json')
        # Drop battery
        skill_homing.on_enter(p1)
    if step_7_pickup_smokedet_and_drop:
        print("DEBUG: panda 1 picking smokedet and placing into waste bin")
        # Smoke detector dropping in table
        #change_tool(robot = p1, new_tool = 'three_jaw_chuck', old_tool = 'offset_vacuum_gripper')
        cnc_cut_skill.unclamp()
        panda_1_pick_up_smoke_detector_from_cnc_and_place_on_table(smoke_detector_type = 'fumonic')

def home_all_robots():
    skill_homing.on_enter(robot=p1)
    skill_homing.on_enter(robot=p2)

def hekatron_cnc_cut(smoke_detector_detection):
    vi_realsense.enable_camera(True)
    cnc_cut_skill.clamp()
    result = check.on_enter(robot = p2, location = None, object_to_check = smoke_detector_detection)
    cnc_cut_skill.on_enter(enable_spindle = True, **{'object':smoke_detector_detection})
    #cnc_cut_skill.on_enter_hekatron(battery_rectangle_rotation_deg = battery_rectangle_rotation_deg, enable_spindle = True)

def hekatron_remove_battery_cover():
    vi_realsense.enable_camera(True)
    cnc_cut_skill.clamp()
    result = check.on_enter(robot = p2, location = None, object_to_check = None, return_to_home = False, only_move_to_position_and_ignore_detecting_objects = True)
    time.sleep(1)
    all_detections = vi_realsense.get_detections() # Detect either battery_covered or 
    # Battery covered removal
    if check_if_detections_contain_label(all_detections, Label.battery_covered):
        battery_covered = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections,
                                                                                          desired_detection_class = Label.battery_covered)
        print("DEBUG: Removing battery cover")
        #cnc_cut_skill.clamp_skill.on_enter() # TODO myb uncomment
        #cnc_cut_skill.unclamp_skill.on_enter()
        skill_exception_handling_wrapper.on_enter(robot = p2,
                                          look_at_skill = skill_look_at_table,
                                          look_at_skill_kwargs = {'robot':p2, 'location': 'table_cnc', 'return_to_home':False},
                                          vision_interface = p2.camera_vision_interface,
                                          base_skill = pickup_top,
                                          base_skill_kwargs = {'robot': p2, 'gripper_close_command': 1.4},
                                          object_to_handle = Label.battery_covered,
                                          return_to_home = False)

        panda_2_drop_battery_cover()

def panda_1_pick_up_smoke_detector_from_table_vision_and_place_in_cnc():
    vi_basler.enable_camera(True)
    time.sleep(1) # Give vision a bit of time in case robot just moved out of way of camera
    all_detections = vi_basler.get_detections()
    smoke_detector = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections,
                                                                                     desired_detection_class= Label.smoke_detector)
    ### Pickup smoke detector and move to CNC
    result = pickup.on_enter(robot = p1, object_class = smoke_detector.label.name, object_tf_name = smoke_detector.tf_name,
                             pickup_location = 'table_vision', ignore_orientation = True)
    # Intermediate moves
    result = drop.on_enter(robot = p1, drop_object = smoke_detector, drop_location = cnc_module)
    result = skill_homing.on_enter(robot=p1)
    return smoke_detector

def panda_1_pick_up_smoke_detector_from_cnc_and_place_on_table(smoke_detector_type = 'smoke_detector'):
    assert smoke_detector_type in ['fumonic', 'smoke_detector']
    skill_homing.on_enter(p1)
    cnc_cut_skill.unclamp()
    pickup = PickupObject()
    pickup.on_enter(robot = p1, pickup_location = 'table_cnc', object_class = smoke_detector_type, move_time_to_pickup = 3,
                    safe_height_above_pickup_pose = 0.1)
    drop = DropObject()
    drop.on_enter(robot = p1, drop_location = 'table_drop_waste', drop_object = 'smoke_detector', perform_initial_jmove = False)
    #skill_homing.on_enter(p1, open_gripper = False)
    #p1.gripper.open()
    skill_homing.on_enter(p1, open_gripper = False)

def panda_2_wiggling_battery_removal():
    cnc_cut_skill.clamp()
    wiggling_skill.on_enter(robot = p2, vision_interface = vi_realsense)
    cnc_cut_skill.unclamp()
 
def panda_2_remove_fumonic_cover():
    skill_look_at_table.on_enter(robot = p2,
                                 location = 'table_cnc',
                                 location_subframe = 'chuck_fumonic',
                                 return_to_init_q = False,
                                 return_to_home = False)
    init_q = [0.8856514897264335,
     0.7054613703555257,
     0.779697128682774,
     -1.3411003158435697,
     -0.5528807663321496,
     1.9237545668284097,
     -2.263859606181131]
    p2.gripper.open()
    p2.error_recovery()
    p2.JMove(init_q, 3)
    p2.error_recovery()
    p2.CMoveFor([0,0, -0.01], 2)
    p2.error_recovery()
    p2.CMoveFor(dx = np.eye(3)@rot_y(-45, unit='deg'), t = 3, task_space = 'Tool')
    p2.error_recovery()
    p2.CMoveFor(dx = [0,0.04, 0], t=2)
    p2.error_recovery()
    p2.gripper.close()
    p2.CMoveFor(dx = np.eye(3)@rot_y(30, unit='deg'), t = 1.5, task_space = 'Tool')
    p2.error_recovery()
    p2.CMoveFor(dx = np.eye(3)@rot_y(-30, unit='deg'), t = 1.5, task_space = 'Tool')
    p2.error_recovery()
    p2.CMoveFor(dx = np.eye(3)@rot_y(30, unit='deg'), t = 1.5, task_space = 'Tool')
    p2.error_recovery()
    p2.CMoveFor(dx =[0, 0.03, 0], t = 1.5)
    p2.error_recovery()
    p2.CMoveFor(dx = [0,0, 0.03], t=1.5)
    p2.error_recovery()
    
def panda_2_drop_battery():
    drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'battery')

def panda_2_drop_battery_cover():
    drop.on_enter(robot = p2, drop_location = 'table_drop_waste', drop_object = 'battery_covered')


def main():
    # Enable cameras
    realsense_img_listener_raw = ROSImageListener(image_topic = '/realsense/color/image_raw')
    realsense_img_listener_detections = ROSImageListener(image_topic = '/vision/realsense/colour')

    ENABLE_VISION = True
    #ROBOT_TSAMP = 1/200.
    if ENABLE_VISION:
        vision_module = VisionModule()
        vi_basler = vision_module.get_callable_submodules()['basler']['vision_interface']
        vi_realsense = vision_module.get_callable_submodules()['realsense']['vision_interface']
        #vi_basler.enable_camera()
        #vi_realsense.enable_camera()

    # Initialize robots
    p1_module = Panda1Module()
    p1_module.enable_airblock()
    p1 = p1_module.get_callable_submodules()['robot']
    p1.SetNewEEConfig('tc_and_3jaw_chuck.json')
    p1.SetLoggerLevel(level='warn')
    p1.SetCollisionBehavior(F = np.array([45.0, 45.0, 45.0]), T = np.array([20.0, 20.0, 18.0]), tq = np.array([87, 87, 87, 87, 12, 12, 12])*0.95, restart = True)

    p2_module = Panda2Module(activate_eye_in_hand_camera = ENABLE_VISION)
    p2 = p2_module.get_callable_submodules()['robot']
    p2.SetNewEEConfig('tc_and_vsg.json') # TODO add to Panda2Module, but then change toolchanger Pins so p2.tc doesnt open p1.tc
    p2.SetLoggerLevel(level='warn')
    p2.SetCollisionBehavior(F = np.array([45.0, 45.0, 45.0]), T = np.array([20.0, 20.0, 18.0]), tq = np.array([87, 87, 87, 87, 12, 12, 12])*0.95, restart = True)

    # Initialize TF manager
    tf_manager = TFManager()
    tf2x = tf_manager.tf2x
    sendTf = tf_manager.SendTransform2tf

    # Initialize modules
    cnc_module = CNCModule()

    # Initialize skills
    skill_homing = RobotHoming()
    pickup = PickupObject()
    pickup_top = PickupDetectorTop()
    drop = DropObject(move_above_z = 0.08)
    check = CheckClampedObject(tf_manager = tf_manager)
    cnc_cut_skill = CNCCutSmokeDetector(init_ros_node = False)
    estimate_battery_pose_skill = EstimateBatteryContactPose()
    wiggling_skill = WigglingBatteryRemoval(tf_manager = tf_manager)
    skill_look_at_table = LookAtTable()
    skill_exception_handling_wrapper = ExceptionHandlingSkillWrapper()
    skill_jmove_above_table = JMoveAboveTable()
    skill_drop = DropObject()

    # Run the actual cycle
    # Move robot to initial position (be careful)
    skill_homing.on_enter(robot=p1)
    skill_homing.on_enter(robot=p2)
    check_vision_is_working(vision_interfaces=[vi_realsense])

    perform_hekatron_cycle(step_1_put_smokedet_into_cnc_and_cut = True,
                           step_2_remove_battery_cover = True,
                           step_3_remove_battery_by_wiggling = True,
                           step_4_pickup_smokedet_and_drop_into_bin = True)


if __name__ == '__main__':
    main()