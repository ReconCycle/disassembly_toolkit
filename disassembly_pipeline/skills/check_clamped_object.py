import time
import numpy as np
from context_action_framework.types import Label # Required to determine 'battery covered' class name. MYB hardcode battery_covered and avoid import?

from disassembly_pipeline.utils.tf_utils import TFManager#, tf_obj2x
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.skills.robot_homing import RobotHoming

from robotblockset_python.transformations import *
from robotblockset_python.robots import robot as Robot
from context_action_framework.types import Detection
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from typing import Union

# TODO move to config/pose_db
# Panda_2 home q
init_q = [-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,1.850346  ,  0.712032  ]
# panda_2 q close to CNC look pose
close_to_cnc_q = [1.3098202525070233,-0.5892040817753124,-0.12905068012466778,-2.260760692986392,-0.07190256026405543,1.6756157788565138,0.4244229040846642]
# panda_2 cartesian position to look at CNC chuck
pos_above_chuck_hekatron = [0.157, 0.38, 0.34886717]


class CheckClampedObject(BaseSkill):
    def __init__(self, tf_manager: TFManager = None):

        if tf_manager is None:
            tf_manager = TFManager()

        self.tf_manager = tf_manager
        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        return self.on_enter(**kwargs)

    def on_enter(self,
                 robot: Robot,
                 location: Union[str, BaseWorkcellModule],
                 object_to_check: Detection,
                 only_move_to_position_and_ignore_detecting_objects = False,
                 return_to_home = True) -> SkillExecutionResult:
        """
        Args:
            robot: robotblockset_python.robot.Robot
            location: BaseTable
            object_to_check: context_action_framework.types.Detection
        """

        r = robot
        if not hasattr(r, 'camera_vision_interface'):
            raise ValueError("To check objects, eye-in-hand or overhead camera is required. robot should have attribute r.camera_vision_interface of type context_action_framework.vision_interface.VisionInterface")

        
        vision_interface = r.camera_vision_interface
        vision_interface.enable_camera(True)
        modified_variables = {}

        if only_move_to_position_and_ignore_detecting_objects and (return_to_home == False):
            move_to_look_pose(robot, jmove_speed_factor = 0.6)
        else:
            battery_rotation = find_hekatron_rotation(robot = r,
                                                    vision_utils = vision_interface,
                                                    tf_manager = self.tf_manager,
                                                    return_to_home = return_to_home,
                                                    machine_base_frame = 'cnc_machine_home')

            # modify input objects
            if object_to_check is not None:
                object_to_check.battery_rotation = battery_rotation
            modified_variables = dict(object_to_check= {'battery_rotation': battery_rotation })
        #vision_interface.enable_camera(False)

        success = True
        result = SkillExecutionResult(success = success,
                                      textual_result= 'Object was looked at.',
                                      modified_variables_dict= modified_variables)
        return result

    def execute(self):
        0

    def on_exit(self):
        0

# Finding the rotation of the smoke detector
def find_hekatron_rotation(robot,
                           vision_utils,
                           tf_manager,
                           jmove_speed_factor = 0.6,
                           return_to_home = True,
                           machine_base_frame = 'cnc_machine_home'):
    """ Hardcoded for panda_2
    Robot movement cycle to go to init, move above CNC chuck where Hekatron should be, try to find battery,
    and return battery rotation.

    Args:
        robot: robotblockset_python.robot object
        vision_utils: context_action_framework.vision_interface.VisionInterface object. (For getting eye-in-hand camera detections.)
        sendtf_fn: disassembly_pipeline.utils.tf_utils.TFManager.SendTransform2TF function.

    """

    r = robot
    move_to_look_pose(robot = r, tf_manager = tf_manager, jmove_speed_factor = jmove_speed_factor)
    time.sleep(0.5)

    z_rot_deg = determine_battery_covered_rotation(vision_utils = vision_utils, tf_manager = tf_manager, machine_base_frame = machine_base_frame)

    if return_to_home:
        # Move robot back
        move_back(robot = r, jmove_speed_factor = jmove_speed_factor)

    return z_rot_deg

def move_to_look_pose(robot, tf_manager = None, jmove_speed_factor = 1):
    r = robot
    if r._control_strategy != 'JointPositionTrajectory':
        r.Switch_controller(start_controller = 'JointPositionTrajectory')

    r.error_recovery()

    #r.JMove(init_q, 1.5, qdot_max_factor = jmove_speed_factor, qddot_max_factor = jmove_speed_factor)

    r.JMove(close_to_cnc_q, 2.5, qdot_max_factor = jmove_speed_factor, qddot_max_factor = jmove_speed_factor)

    #p2.CMoveFor([0, 0.27, 0], t=3)

    # Target pose that overlooks the hekatron chuck
    target_EE_rot = np.eye(3)@rot_x(180 ,unit='deg')@rot_z(270, unit = 'deg')
    target_T = np.eye(4)
    target_T[0:3, 0:3] = target_EE_rot
    target_T[0:3, -1] = pos_above_chuck_hekatron
    #if tf_manager is not None:
    #    tf_manager.SendTransform2tf(p = target_T[0:3, -1], q = r2q(target_T[0:3]), parent_frame = r.Base_link_name, child_frame = 'TEST_x')

    r.CMove(target_T, t=1)

def determine_battery_covered_rotation(vision_utils, tf_manager, machine_base_frame = 'cnc_machine_home'):
    MAX_N_RETRIES = 1
    n_retry = 0
    TIMEOUT_S = 4 # seconds
    SUCCESS = 0
    while (SUCCESS==0) and (n_retry<MAX_N_RETRIES):
        try:
            battery_det = vision_utils.get_detections(desired_class = Label.battery_covered, timeout = TIMEOUT_S)[0]
            if battery_det is not None:
                SUCCESS = 1
        except Exception as e:
            n_retry+=1
            print("Exception", e)
            print("Failed finding battery")

    if SUCCESS:
        battery_tf = battery_det.tf_name

        machine_to_battery_x = tf_manager.tf2x(machine_base_frame, battery_tf)
        machine_to_battery_T = x2t(machine_to_battery_x)
        machine_to_battery_rpy_rad = r2rpy(machine_to_battery_T[0:3, 0:3])
        machine_to_battery_rpy_deg = machine_to_battery_rpy_rad * 180 / np.pi
        machine_to_battery_Z_rotation = machine_to_battery_rpy_deg[0]
        z_rot_deg = machine_to_battery_Z_rotation
        #rot = battery_det.tf_px.rotation
        #bat_q = [rot.w, rot.x, rot.y, rot.z]
        #camera_to_bat_rpy = q2rpy(bat_q)* 180/np.pi
        #z_rot_deg = camera_to_bat_rpy[0]

        print(f"Found battery, z_rot_deg={z_rot_deg}")
    else:
        raise ValueError("Did not detect battery in hekatron. check camera and vision_pipeline. Returning battery rotation 0deg")
        z_rot_deg = None

    return z_rot_deg

def move_back(robot, jmove_speed_factor = 1):

    r = robot
    r.JMove(close_to_cnc_q, 1.2, qdot_max_factor = jmove_speed_factor, qddot_max_factor = jmove_speed_factor)

    skill_homing = RobotHoming()
    skill_homing.on_enter(r)
    #r.JMove(init_q, 3, qdot_max_factor = jmove_speed_factor, qddot_max_factor = jmove_speed_factor)


if __name__ == '__main__':
    0
