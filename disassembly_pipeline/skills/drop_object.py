import numpy as np
import copy
from typing import Union
from disassembly_pipeline.utils.tf_utils import TFManager
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from robotblockset_python.transformations import *
from robotblockset_python.robots import robot as Robot
from context_action_framework.types import Detection
from disassembly_pipeline.skills.jmove_above_table import JMoveAboveTable

class DropObject(BaseSkill):
    def __init__(self, robot= None,
                 using_controller = 'JointPositionTrajectory',
                 move_above_z = 0.05,
                 drop_db_location = 'config/drop_object.json'):

        """Generic function to drop an object with whichever gripper."""

        self.robot = robot

        self.using_controller = using_controller
        self.move_above_z = move_above_z

        self.tflistener = TFManager()
        self.tf2x = self.tflistener.tf2x
        self.drop_frame_debug_name = 'drop_pose' # TF name of gripper drop pose that will be shown in ros

         # Open .json parameter file relative to parent package (disassembly_pipeline)
        # Open JSON params file
        self.drop_db = read_json_from_package(package_name = 'disassembly_pipeline', relative_json_path_within_package = drop_db_location)

        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        robot = kwargs['robot']
        drop_location = kwargs['location']
        drop_object = kwargs.get('object', None)

        result = self.on_enter(robot = robot, drop_location = drop_location, drop_object = drop_object)
        return result

    def on_enter(self,
                 robot: Robot,
                 drop_location: Union[BaseWorkcellModule, str],
                 drop_object: Union[Detection, str] = None,
                 perform_initial_jmove = True) -> SkillExecutionResult:

        """ 
        
        """

        KEEP_INITIAL_GRIPPER_ROTATION = False # if True, Only take into account positions of drop poses, ignore rotations
        MOVE_VELOCITY = 0.2 # [m/s]
        T_FROM_DROP_ABOVE_TO_DROP = 1.5 # We construct drop_T_above and drop_T (first move above drop pose, then move down). This defines time to move down.

        r = robot
        initial_robot_controller = copy.deepcopy(r._control_strategy)

        if isinstance(drop_location, str):
            drop_location_name = drop_location
        else:
            drop_location_name = drop_location.get_name()

        robot_base_frame = robot.Base_link_name # robot.get_base_link()
        robot_gripper_name = robot.gripper.Name # robot.get_gripper_name()

        if isinstance(drop_object, str):
            object_class = drop_object
        else:
            object_class = drop_object.get_class() if hasattr(drop_object, 'get_class') else drop_object.label_precise_name

        #print(f"""Obj class: {object_class}, robot_gripper_name: {robot_gripper_name} """)

        ### Get drop frame in table_base_frame

        # Get drop frame for a specific table/location
        drop_tf = self.drop_db[drop_location_name]

        drop_location_base_frame = self.drop_db[drop_location_name]['base_frame']

        # Check if we have object-specific drop frame
        if object_class in drop_tf.keys():
            drop_tf = drop_tf[object_class]
        else:
            print(f"Did not find drop_tf for drop location {drop_location_name} and object class {object_class}")
            drop_tf = drop_tf['default']

        # Check if we have gripper-specific drop frame
        if robot_gripper_name in drop_tf.keys():
            drop_tf = drop_tf[robot_gripper_name]
        else:
            print(f"Did not find drop_tf for drop location {drop_location_name}, object class {object_class} and gripper {robot_gripper_name}")
            drop_tf = drop_tf['default']

        # Try to get frame name or base_to_drop_T
        drop_frame = None
        if 'frame' in drop_tf.keys():
            drop_frame = drop_tf['frame']
        elif 'dT' in drop_tf.keys():
            drop_dT = drop_tf['dT']
        else:
            raise ValueError(f"Did not find either drop frame or drop dT for location {drop_location_name}, gripper {robot_gripper_name}, and object class {object_class}")

        ### Generate drop_T in robot base frame
        drop_T_in_robot_frame = None
        if drop_frame is not None:
            # Directly get tf from robot base to drop frame
            drop_T_in_robot_frame = x2t(self.tf2x(robot_base_frame, drop_frame))
        else:
            location_T_in_robot_base_frame = x2t(self.tf2x(robot_base_frame, drop_location_base_frame))
            drop_T_in_robot_frame = location_T_in_robot_base_frame@drop_dT

        #print(f"DropObject: drop position in robot base frame: {drop_T_in_robot_frame[0:3, -1]}")

        # Finished calculating drop pose
        drop_T = drop_T_in_robot_frame

        # Keep the robot EE orientation, for now
        r.GetState()
        if KEEP_INITIAL_GRIPPER_ROTATION:
            drop_T[0:3, 0:3] = x2t(r.x)[0:3, 0:3]

        drop_T_above = copy.deepcopy(drop_T)
        drop_T_above[2, -1] += self.move_above_z

        # Fix for when we are using more than 2 controllers.
        if r._control_strategy != self.using_controller:
            r.Switch_controller(start_controller =  self.using_controller)

        r.error_recovery()

        if self.using_controller == 'CartesianImpedance':
            # Set low cart stiffness for moves
            r.SetCartesianStiff_helper(m=0.9, n=0.75)

        if perform_initial_jmove and (drop_location is not None):
            jmove_skill = JMoveAboveTable()
            jmove_skill.on_enter(robot=r, location=drop_location)

        #t_move = r.estimate_time()
        # Estimate move time
        r.GetState()
        cur_x = np.array(r.x[0:3])
        new_x = np.array(t2x(drop_T)[0:3])
        dist = np.sqrt(np.sum((new_x-cur_x)**2, axis=0))
        t = dist/MOVE_VELOCITY

        drop_x = t2x(drop_T)
        self.tflistener.SendTransform2tf(p=drop_x[0:3],q=drop_x[3:], parent_frame = robot_base_frame,child_frame = self.drop_frame_debug_name)
        r.CMove(x = t2x(drop_T_above), t = t, v_max_factor = 0.4, a_max_factor = 0.4)
        # r.CPath([t2x(drop_T_above)], t = t)
        r.CMove(x = drop_x, t = T_FROM_DROP_ABOVE_TO_DROP, v_max_factor = 0.4, a_max_factor = 0.4)
        r.gripper.open(1, sleep=True, wait_for_result=False)

        r.CMove(x = t2x(drop_T_above), t = T_FROM_DROP_ABOVE_TO_DROP, v_max_factor = 0.4, a_max_factor = 0.4)

        # Update object location
        if not isinstance(drop_object, str):
            drop_object.table_name = drop_location_name
            drop_object.parent_frame = drop_location_base_frame
            drop_object.tf_name = None

        #do_concurrently([[r.CMove, {'x': x_above_pickup, 't': 6}],[r.gripper.open, {}]], wait_until_task_completion = True)

        #r.CMove(x=x_pickup, t = 2, v_max_factor = 0.4, a_max_factor = 0.4)
        #r.gripper.close(1, stiffness = 10000, motion_duration = 0.1, sleep = True) # Sleep=True means it will sleep until gripper is open(0.6 seconds)
        #time.sleep(1)
        #r.CMove(x = x_above_pickup, t = 2, v_max_factor = 0.4, a_max_factor = 0.4)

        # Switch to original controller
        if r._control_strategy != initial_robot_controller:
            r.Switch_controller(start_controller = initial_robot_controller)

        success = True
        result = SkillExecutionResult(success = success,
                                    textual_result = f"Object was dropped.",
                                    modified_variables_dict = {'object': {'tf_name': None,
                                                               'parent_frame': drop_location_base_frame,
                                                               'table_name': drop_location_name}})

        return result

    def execute(self):
        0

    def on_exit(self):
        0
