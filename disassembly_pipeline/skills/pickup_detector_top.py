import numpy as np
import json
import os
import importlib.util
import time
import copy
import rospy

from disassembly_pipeline.utils.tf_utils import TFManager#, tf_obj2x
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package

from robotblockset_python.transformations import *
from robotblockset_python.robots import robot as Robot

from context_action_framework.types import Detection
import numpy as np
np.set_printoptions(suppress = True)


class GetRelativePickupOrDropPose:
    def __init__(self, 
                 robot: Robot = None,
                 robot_ee_frame_name: str = 'panda_2_EE',
                 detection: Detection = None,
                 tf_manager: TFManager = None):
        """ Record a relative pickup pose that is object-class-specific.
        
        The input is a context_action_framework Detection. The TF is given relative to (robot eye-in-hand camera),
        so it's first converted to robot base frame and remember. 
        Then the robot is moved to desired grasp pose, and the relative position between the object and gripper is recorded.
        The resulting dT is put into config/object_pickup_parameters.json
        
        Example call:
        from disassembly_pipeline.skills.pickup_object import GetRelativePickupOrDropPose
        from context_action_framework.vision_interface import VisionInterface

        vi = VisionInterface(vision_topic = '/vision/realsense/detections')
        detections = vi.get_detections()
        example_detection = detections[0]
        pickup_pose_generator = GetRelativePickupOrDropPose(tf_manager = tf_manager,
                                                            robot = p2,
                                                            detection = example_detection,
                                                            robot_ee_frame_name= 'panda_2_EE')
        out_dict = pickup_pose_generator.on_enter()
        """

        self._robot = robot
        self._robot_ee_frame = robot_ee_frame_name
        self._detection = detection
        self._tf_manager = tf_manager

        if (robot is None) or (detection is None) or (tf_manager is None):
            raise ValueError("None of the input args can be None!")

    def on_enter(self):

        # First get object TF in robot base frame
        object_tf_in_robot_base_frame = self._convert_detection_to_robot_base_frame()

        # Ask user to move robot to grasp pose
        user_confirmed = input("Move the robot's gripper to the desired grasp pose next to the object, and press enter.")

        # Calculate relative grasp pose
        dT_grasp = self._calculate_relative_robot_grasp_pose(object_tf_in_robot_base_frame = object_tf_in_robot_base_frame)

        out_dict = dict(object_class = self._detection.label.name,
                        robot_gripper_name = self._robot.gripper.Name,
                        dT_grasp = dT_grasp)
        return out_dict

    def _convert_detection_to_robot_base_frame(self):

        # First get object TF in robot base frame
        robot_base_frame = self._robot.Base_link_name
        robot_ee_frame = self._robot_ee_frame

        object_class = self._detection.label.name
        object_tf_parent_frame = self._detection.parent_frame
        object_tf_in_parent_frame = self._detection.tf
        object_tf_name = self._detection.tf_name

        object_tf_in_robot_base_frame = self._tf_manager.tf2x(parent_frame = robot_base_frame, child_frame = object_tf_name)
        object_tf_in_robot_base_frame = x2t(object_tf_in_robot_base_frame)
        return object_tf_in_robot_base_frame

    def _calculate_relative_robot_grasp_pose(self, object_tf_in_robot_base_frame):
        robot_ee_in_base_frame = self._tf_manager.tf2x(parent_frame = self._robot.Base_link_name, child_frame = self._robot_ee_frame)
        robot_ee_in_base_frame = x2t(robot_ee_in_base_frame)

        dT_grasp = np.linalg.solve(object_tf_in_robot_base_frame, robot_ee_in_base_frame)
        return dT_grasp

def object_pose_in_base_cs_to_robot_cs(object_T, object_T_base_frame, robot_base_frame, tf2x):
    """


    Args:
    object_T : T of object to pick up
    object_T_base_frame: Frame in which object_T is given
    robot_base_frame: Base frame of robot in which the robot_pickup_T will be returned
    tf2x: tf2x function from tf_utils (ros lookup_transform).
    """

    # Get tf between robot frame and object frame
    # use tfmanager
    x_rob_to_obj_frame = tf2x(robot_base_frame, object_T_base_frame)
    T_rob_to_obj_frame = x2t(x_rob_to_obj_frame)

    robot_pickup_T = T_rob_to_obj_frame@object_T

    return robot_pickup_T


class PickupDetectorTop(BaseSkill):
    def __init__(self,
                 robot: Robot = None,
                 safe_init = False,
                 gripper_sleep = False,
                 using_controller = 'JointPositionTrajectory',
                 move_above_z = 0.15,
                 offset = [0,0,0],
                 v_max_factor = 1,
                 a_max_factor = 1,
                 pickup_parameters_filename = "config/pickup_object.json"):
        """Generic function to pick up an object with whichever gripper.

	Example call:
        >>> pickup_obj_skill = PickupDetectorTop()
        >>> pickup_obj_skill.on_enter(object_class = 'battery',
                                      robot = panda_2_robotblockset_object,
                                      object_tf_name = 'battery_0_table_vision')
        OR
        >>> pickup_obj_skill.on_enter(object_class = 'battery',
                                      robot = panda_2_robotblockset_object,
                                      object_x_in_robot_cs = [0.3, 0.3, 0.3, 0, 0, 1, 0])"""

        self.robot = robot
        self.safe_init = safe_init
        self.gripper_sleep = gripper_sleep
        self.using_controller = using_controller
        self.move_above_z = move_above_z
        self.offset = offset
        self.parameter_filename = pickup_parameters_filename

        self.v_max_factor = v_max_factor
        self.a_max_factor = a_max_factor

        # Open JSON params file
        self.object_grasp_params = read_json_from_package(package_name = 'disassembly_pipeline', relative_json_path_within_package = pickup_parameters_filename)

        # Will fail if ROS node is not initialized
        if not rospy.get_node_uri():
            rospy.init_node("actions_node", anonymous = True)
        self.tflistener = TFManager()

        self.tf2x = self.tflistener.tf2x
        self.pick_up_frame_debug_name = 'pick_up_pose'  # TF name of gripper pick-up pose that will be shown in ros

        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:

        robot = kwargs['robot']
        obj = kwargs['object']
        location_from = kwargs['location_from']

        object_class = obj.label.name
        object_tf_name = obj.tf_name

        result = self.on_enter(object_class = object_class, robot = robot, object_tf_name = object_tf_name, pickup_location = location_from)
        return result

    def on_enter(self, object_class: str = None,
                       robot: Robot = None,
                       object_x_in_robot_cs: str = None,
                       object_tf_name: str = None,
                       detection: Detection = None,
                       pickup_location = None,
                       move_time_to_pickup = 2,
                       debug = 0,
                       ignore_orientation: bool = False,
                       gripper_close_command = 1) -> SkillExecutionResult:
        """
        Takes as input either object_x_in_robot_cs, or object_tf_name. The reason is that some other function should bring the robot in vicinity of pick up pose.
        In that case, the object might become occluded and we could not find the robot_base to object transform.
        Args:
        ----------------
        object_class: str: object to pick up class (i.e hca_kalo, firealarm, ...)
        robot: robotblockset_python object

        only ONE of these two parameters needs to be provided:
        object_x_in_robot_cs: 7dim(position + quaternion) of pickup object in robot base coordinate system. In this case, we don't look up transforms.
        object_tf_name: string tf name that we will look up to find TF between robot base frame and object.

        Example call"""

        MOVE_VELOCITY = 0.1 # [m / s]

        # Ignore orientation for round circular objects
        if object_class in ['firealarm', 'smoke_detector', 'fumonic']: ignore_orientation = True


        # The assumption is that it will a top-down movement and then a grip.

        # Check if we are approximately above the object, if not, move there.

        # Get possible grasp poses

        # Find optimal grasp pose:
        # a) check if reachable with robot (invkin solvable?)
        # b) check collision planning

        # Get the dT parameter. T_obj@dT = T_grasp_in_robot_coords.

        # __________________________________________-

        if detection is not None:
            object_class = detection.label.name
            object_tf_name = detection.tf_name

        if robot is None:
            robot = self.robot

        r = robot
        initial_robot_controller = copy.deepcopy(r._control_strategy)

        robot_base_frame = r.Base_link_name
        gripper_name = r.gripper.Name

        # Find grasp parameters
        try:
            dT = self.object_grasp_params[object_class][gripper_name]['dT_grip']
        except:
            rospy.loginfo("disassembly pipeline didn't find grip parameters for tool {} and object class {}".format(gripper_name, object_class))
            return SkillExecutionResult(success = False,
                                    textual_result = f"Object was not picked up, grasp parameters unavailable.",
                                    changes_dict = {})

        if (object_x_in_robot_cs is not None) and (object_tf_name is not None):
            print("Warning: object_x_in_robot_cs and object_tf_name are both set. Using object_x_in_robot_cs.")

        # 1. Try to pick up based on fixed pickup frame object_x_in_robot_cs
        if object_x_in_robot_cs is not None:
            x_pickup = object_x_in_robot_cs  # by default set to object_tf_robot_cs. Overwrite in if clause, needed.
        elif object_tf_name is not None:
            # 2. Else Try to pick up based on object_tf_name.
            try:
                x_pickup = self.tf2x(parent_frame = robot_base_frame, child_frame = object_tf_name)
            except KeyboardInterrupt:
                return 0
            except Exception as e:
                print(e)
        # 3. If all fails (TF not found), try to get fixed pickup location (e.g. from within CNC machine.)
        else:
            pickup_location_name = pickup_location.get_name()
            base_frame = self.object_grasp_params[pickup_location_name]['base_frame']
            T_base_frame_to_gripper_frame = self.object_grasp_params[pickup_location_name][object_class][gripper_name]['dT']
            T_pickup = object_pose_in_base_cs_to_robot_cs(T_base_frame_to_gripper_frame, base_frame, robot_base_frame, self.tf2x)
            x_pickup = t2x(T_pickup)

        # Generate pick-up pose
        T_pickup = x2t(x_pickup)
        T_pickup = T_pickup@dT
        print(dT)
        print(T_pickup)

        x_pickup = t2x(T_pickup)

        if ignore_orientation:
            r.GetState()
            cur_r_x = r.x
            x_pickup[3:] = cur_r_x[3:]

        # Perform robot moves if not in debug mode. Otherwise just show the pick up pose in ROS.
        if debug:
            self.tflistener.SendTransform2tf(p=x_pickup[0:3],q=x_pickup[3:], 
                                             parent_frame = robot_base_frame,
                                             child_frame = self.pick_up_frame_debug_name)
            return 'continue'
    
        if self.safe_init == True: self.robot_go_to_init_and_open_gripper(r)

        # Fix for when we are using more than 2 controllers.
        if r._control_strategy != self.using_controller:
            r.Switch_controller(start_controller =  self.using_controller)

        r.error_recovery()

        if self.using_controller == 'CartesianImpedance':
            # Set low cart stiffness for moves
            r.SetCartesianStiff_helper(m=0.9, n=0.75)

        if self.offset is not None:
            if len(self.offset) != 3:
                raise ValueError("Offset should be [dx, dy, dz]!")
            x_pickup = np.array(x_pickup)
            x_pickup[0:3] += self.offset
            x_pickup = x_pickup.tolist()

        # rot_pickup = x2t(x_pickup)[0:3, 0:3]@rot_z(-90, unit='deg')
        # x_pickup[3:] = r2q(rot_pickup)
        x_above_pickup = copy.deepcopy(x_pickup)
        x_above_pickup[2] += self.move_above_z

        # Calculate time
        r.GetState()
        cur_x = np.array(r.x[0:3])
        new_x = np.array(x_above_pickup[0:3])
        dist = np.sqrt(np.sum((new_x-cur_x)**2, axis=0))
        t = dist/MOVE_VELOCITY

        r.CMove(x = x_above_pickup, t = t, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)
        r.gripper.open(1, sleep=self.gripper_sleep, wait_for_result=False)
        #do_concurrently([[r.CMove, {'x': x_above_pickup, 't': 6}],[r.gripper.open, {}]], wait_until_task_completion = True)
        
        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')

        if self.using_controller == 'CartesianImpedance':
            # Set low cart stiffness for moves
            # r.SetCartesianStiff_helper(m = 1.5, n= 1.5)
            r.SetCartesianStiff_helper(m=0.6, n=0.75)

        ###################### The code until here is generic pickup skill ########################
        ###################### Add specific motion here ###########################################
        
        r.CMove(x = x_pickup, t = move_time_to_pickup, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)

        r.CMoveFor(dx=[0,0,-0.007], t=1, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)
        r.gripper.close(gripper_close_command)

        # Add some stuff here
        r.GetState()
        current_pose = r.x
        current_pose = x2t(current_pose)

        dR_robot_base_to_EE = np.linalg.solve(current_pose[0:3, 0:3], np.eye(3))   
        FT_pickup = np.array([-3, 0, 0])
        FT_pickup = dR_robot_base_to_EE@FT_pickup
        FT_pickup_in_world_frame = np.zeros(6)
        FT_pickup_in_world_frame[0:3] = FT_pickup
        r.error_recovery()
        print("debug")
            
        x_slightly_above_pickup = copy.deepcopy(x_pickup)
        x_slightly_above_pickup[2] += 0.01
        r.CMove(x = x_slightly_above_pickup, t = 2, 
                v_max_factor = self.v_max_factor, 
                a_max_factor = self.a_max_factor,
                FT = FT_pickup_in_world_frame)

        r.CMove(x = x_above_pickup, t = move_time_to_pickup, 
                v_max_factor = self.v_max_factor, 
                a_max_factor = self.a_max_factor)

        ###################### Add specific motion above ##########################################

        if r._control_strategy != initial_robot_controller:
            r.Switch_controller(start_controller = initial_robot_controller)

        success = True # Todo add exception checking

        result = SkillExecutionResult(success = success,
                                    textual_result = f"Object was picked up.",
                                    modified_variables_dict = {})
        return result

    def execute(self):
        0

    def on_exit(self):
        0


if __name__ == '__main__':
    0
