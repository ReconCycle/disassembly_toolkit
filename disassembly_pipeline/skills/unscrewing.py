import numpy as np
import json
import os
import time
import copy
import rospy

from disassembly_pipeline.utils.tf_utils import TFManager#, tf_obj2x
from .base_skill import BaseSkill
from disassembly_pipeline.utils.pddl_utils import get_all_objects_of_class

from robotblockset_python.transformations import *

from unified_planning.shortcuts import *

class Unscrewing(BaseSkill):
    def __init__(self, robot= None,
                 name = "unscrew(robot, screw)",
                 using_controller = 'JointPositionTrajectory',
                 move_above_z = 0.05,
                 offset = [0,0,0],
                 v_max_factor = 1,
                 a_max_factor = 1,
                 description = "Use robot to unscrew a screw."):
        """Generic function to pick up an object with whichever gripper.

	Example call:
        >>> unscrewing_skill = Unscrewing()
        >>> unscrewing_skill.on_enter(robot = panda_2_robotblockset_object,
                                      )
        OR
        >>> pickup_obj_skill.on_enter(object_class = 'battery',
                                      robot = panda_2_robotblockset_object,
                                      object_tf_in_robot_cs = [0.3, 0.3, 0.3, 0, 0, 1, 0])"""

        self._name = name
        self._using_controller = using_controller
        self._move_above_z = move_above_z
        self._offset = offset
        self._description = f"""{self._name}: {description}"""
        self._v_max_factor = v_max_factor
        self._a_max_factor = a_max_factor

        try:
            self._tflistener = TFManager()
        except rospy.exceptions.ROSInitException as e:
            # Will fail if ROS node is not initialized
            rospy.init_node("unscrewing_skill", anonymous = True)
            self._tflistener = TFManager()
        self._tf2x = self._tflistener.tf2x
        self._debug_frame_name = 'unscrewing_pose'

        super().__init__(name = name, description = description, action_arg_names = action_arg_names)

    def get_valid_args_regex(self, pddl_problem):
        self._valid_pddl_classes = ['screw']
        valid_pddl_objects = get_all_objects_of_class(pddl_problem, self._valid_pddl_classes)

        valid_regex = ''

        if len(valid_pddl_objects)>0:
            valid_regex = '|'.join(valid_pddl_objects)
        return valid_regex

    def on_enter_pddl(self, **kwargs):

        robot = kwargs['robot']
        obj = kwargs['physical_object']

        #object_class = obj.general_class
        #object_tf_name = obj.tf_name

        #self.on_enter(robot, object_class, object_tf_name)

    def on_enter(self, object_class,
                       robot = None,
                       object_tf_in_robot_cs = None,
                       object_tf_name = None,
                       move_time_to_above = 2,
                       move_time_to_pickup=1,
                       debug = 0,
                       ignore_orientation = False):
        """
        Takes as input either object_tf_in_robot_cs, or object_tf_name. The reason is that some other function should bring the robot in vicinity of pick up pose.
        In that case, the object might become occluded and we could not find the robot_base to object transform.
        Args
        ----------------
        robot: robotblockset_python object
        object_class: string object to pick up class (i.e hca_kalo, firealarm, ...)

        only ONE of these two parameters needs to be provided:
        object_tf_in_robot_cs: 7dim(position + quaternion) of pickup object in robot base coordinate system. In this case, we don't look up transforms.
        object_tf_name: string tf name that we will look up to find TF between robot base frame and object."""

        # The assumption is that it will a top-down movement and then a grip.

        # Check if we are approximately above the object, if not, move there.

        # Get possible grasp poses

        # Find optimal grasp pose:
        # a) check if reachable with robot (invkin solvable?)
        # b) check collision planning

        # Get the dT parameter. T_obj@dT = T_grasp_in_robot_coords.

        # __________________________________________-
        if robot is None:
            robot = self.robot

        r = robot

        robot_link0_tfname = r.Base_link_name

        # Find grasp parameters
        try:
            dT = self.parameters[object_class][r.gripper.Name]['dT_grip']
        except:
            rospy.loginfo("disassembly pipeline didn't find grip parameters for tool {} and object class {}".format(r.gripper.Name, object_class))
            return 0

        if (object_tf_in_robot_cs is not None) and (object_tf_name is not None):
            raise ValueError("Either the object_tf_in_robot_cs or object_tf_name should be provided, not both.")

        x_pickup = object_tf_in_robot_cs # by default set to object_tf_robot_cs. Overwrite in if clause, needed.
        if x_pickup is None:
            success = 0
            while success==0:
                try:
                    x_pickup = self.tf2x(parent_frame = robot_link0_tfname, child_frame = object_tf_name)
                    success=1
                except KeyboardInterrupt:
                    return 0
                except Exception as e:
                    print(e)
                    time.sleep(0.1)
                    pass

        # Generate pick-up pose
        T_pickup = x2t(x_pickup)
        T_pickup = T_pickup@dT

        x_pickup = t2x(T_pickup)

        if ignore_orientation:
            r.GetState()
            cur_r_x = r.x
            x_pickup[3:] = cur_r_x[3:]

        self.tflistener.SendTransform2tf(p=x_pickup[0:3],q=x_pickup[3:], parent_frame = robot_link0_tfname,child_frame = self.pick_up_frame_debug_name )

        # Perform robot moves if not in debug mode. Otherwise just show the pick up pose in ROS.
        if not debug:
            if self.safe_init == True: self.robot_go_to_init_and_open_gripper(r)

            # Fix for when we are using more than 2 controllers.
            if r._control_strategy != self.using_controller:
                r.Switch_controller(start_controller =  self.using_controller)

            r.error_recovery()

            if self.using_controller == 'CartesianImpedance':
                # Set low cart stiffness for moves
                r.SetCartesianStiff_helper(m=0.9, n=0.75)

            if self.offset is not None:
                if len(self.offsset) != 3:
                    raise ValueError("Offset should be [dx, dy, dz]!")
                x_pickup = np.array(x_pickup)
                x_pickup[0:3] += self.offset
                x_pickup = x_pickup.tolist()

            x_above_pickup = copy.deepcopy(x_pickup)
            x_above_pickup[2] += self.move_above_z

            r.CMove(x = x_above_pickup, t = move_time_to_above, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)
            r.gripper.open(1, sleep=self.gripper_sleep, wait_for_result=False)
            #do_concurrently([[r.CMove, {'x': x_above_pickup, 't': 6}],[r.gripper.open, {}]], wait_until_task_completion = True)

            if self.using_controller == 'CartesianImpedance':
                # Set low cart stiffness for moves
                r.SetCartesianStiff_helper(m=0.6, n=0.75)

            r.CMove(x=x_pickup, t = move_time_to_pickup, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)
            r.gripper.close(1, sleep = self.gripper_sleep) # Sleep=True means it will sleep until gripper is open(0.6 seconds)
            r.CMove(x = x_above_pickup, t = move_time_to_pickup, v_max_factor = self.v_max_factor, a_max_factor = self.a_max_factor)

        return 'continue'

    def execute(self):
        0
    def on_exit(self):
        0

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Check gripper type is screwdriver

        # Other needed fluents
        robot_type = UserType('robot')
        location_type = UserType('location')
        physical_object_type = UserType('physical_object')

        unscrew = InstantaneousAction(self._name, robot = robot_type, l_from=location_type, physical_object = physical_object_type)

        robot = pick.parameter('robot')
        l_from = pick.parameter('l_from')
        obj = pick.parameter('physical_object')

        pick.add_precondition(ontable(obj, l_from))
        pick.add_precondition(clear(obj))
        pick.add_precondition(gripper_empty(robot))

        unscrew.add_effect(exists(obj), False)

        pddl_problem.add_action(pick)

        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return pddl_problem, pddl_to_world_obj_links

if __name__ == '__main__':
    0
