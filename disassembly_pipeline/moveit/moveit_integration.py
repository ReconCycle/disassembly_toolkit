from geometry_msgs.msg import Pose, PoseStamped

#from __future__ import print_function
from six.moves import input

from time import sleep
import sys
import copy
import numpy as np
import rospy

import moveit_commander

import moveit_msgs.msg
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.conversions import pose_to_list

import geometry_msgs.msg

from robotblockset_python.transformations import x2t, t2x

from std_msgs.msg import String

from .moveit_planning_scene_generator import create_moveit_planning_scene
from disassembly_pipeline.utils.tf_utils import TFManager
#from disassembly_pipeline.utils.robot_quick_init import DummyRobot

class ReconCycleMoveitManager:
    def __init__(self, robot_objects:list):
        """ This class should be interface to all higher-up classes (such as disassembly pipeline) so as to minimize moveit code within those classes.

        Contains robot commanders and planning scenes. """

        self.robot_objects = robot_objects
        self.planning_scene_interfaces = {}
        self.robot_commanders = {}
        self.move_groups = {}

        print("ReconcycleMoveitManager Debug PT A")

        self.tf_listener = TFManager()
        self.tf2x = self.tf_listener.tf2x

        print("ReconcycleMoveitManager Debug PT B")

    def init_moveit(self):
        print("Starting init_moveit")
        for robot in self.robot_objects:
            robot_commander = None
            move_group = None
            pl_scn_interface = None
            if 1:
                try:
                    print("Recon.MoveitMan. creating planning scene robot {}".format(robot.Name))
                    #pl_scn_interface = create_moveit_planning_scene(ns='/{}'.format(robot.Name), tf2x = self.tf2x)
                    pl_scn_interface = create_moveit_planning_scene(ns='',
                                                                    robot_name = robot.Name,
                                                                    tf2x = self.tf2x)

                    print("Recon.MoveitMan. initing robot {}".format(robot.Name))
                    print(f"Robot name: {robot.Name}")
                    print(f"Scene interface: {pl_scn_interface}")
                    robot_commander, move_group = setup_moveit_interface(ns = '',
                                                                         planning_scene_interface = pl_scn_interface)
                except Exception as e:print("Exception:", e)
            robot.robot_commander = robot_commander
            robot.move_group = move_group
            robot.planning_scene_interface = pl_scn_interface
                
            self.robot_commanders[robot.Name] = robot_commander
            self.move_groups[robot.Name] = move_group
            self.planning_scene_interfaces[robot.Name] = pl_scn_interface

        return 0
    def clear_moveit_planning_scenes():
        for robotname in self.planning_scene_interfaces.keys():
            pl_scn = self.planning_scene_interfaces[robotname]
            pl_scn.clear()
            sleep(0.2)

def setup_moveit_interface(ns = '',
                           robot_description = '/robot_description',
                           name = 'panda_2_arm',
                           planning_scene_interface = None):
    moveit_commander.roscpp_initialize(args={})
    try:rospy.init_node('moveit_node_{}'.format(ns))
    except Exception as e:print("Exception during moveit node init: {}".format(e))

    #rd = "/{}/robot_description".format(ns)
    
    robot_commander = moveit_commander.RobotCommander(robot_description = robot_description, ns = ns)

    #scene = moveit_commander.PlanningSceneInterface()
    scene = planning_scene_interface
    
    move_group = moveit_commander.MoveGroupCommander(name = name, robot_description = robot_description, ns = ns)

    display_trajectory_publisher = rospy.Publisher(
        "{}/move_group/display_planned_path".format(ns),
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot_commander.get_group_names()
    print("============ Available Planning Groups:", robot_commander.get_group_names())

    """
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot_commander.get_current_state())
    print("")
    
    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

    move_group.execute(plan, wait=True)
    """
    return robot_commander, move_group

def move_with_moveit(robot_obj, move_group, x):
    """ 
    Moveit is actually controlling/moving panda_n_link8. The input x is the desired end-effector position.
    We take the inverse transform, to find where link8 should be so that panda_n_EE is in "x" 
    Args:
    robot_obj : robotblockset_python object
    move_group : moveit move group for this robot
    x - target x pose to move to."""

    initial_robot_controller = robot_obj._control_strategy
    if robot_obj._control_strategy != 'JointPositionTrajectory':
        robot_obj.Switch_controller(start_controller = 'JointPositionTrajectory')

    robot_obj.error_recovery()
    robot_obj.GetState()
    
    flange_to_EE = np.reshape(robot_obj.state.F_T_EE, (4,4), order = 'F')
    inv_flange_to_EE = np.linalg.inv(flange_to_EE)

    desired_T = x2t(x)@inv_flange_to_EE

    x = t2x(desired_T)

    move_group.stop()

    pose_goal = Pose()

    pose_goal.position.x = x[0]
    pose_goal.position.y = x[1]
    pose_goal.position.z = x[2] 

    pose_goal.orientation.x = x[4]
    pose_goal.orientation.y = x[5]
    pose_goal.orientation.z = x[6]
    pose_goal.orientation.w = x[3]

    move_group.set_pose_target(pose_goal)
    rospy.loginfo("{} moving to {}, {}, {}, {}, {}, {}, {}".format(robot_obj.Name, x[0], x[1], x[2], x[3], x[4], x[5], x[6]))
    success = move_group.go(wait=True)

    robot_obj.GetState()
    robot_obj.ResetCurrentTarget() # TODO remove this reset and just set command_int.q and x.

    # Switch to original controller
    if robot_obj._control_strategy != initial_robot_controller:
        robot_obj.Switch_controller(start_controller = initial_robot_controller)
    
    return 0
