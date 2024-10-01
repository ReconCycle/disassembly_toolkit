from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.ros.grippers_ros import VariableStiffnessGripper, SofthandGripper, ThreeJawChuck, ToolChanger
from colorama import Fore
import numpy as np

import rospy

def initialize_robot(robot_name:str,
                     tool_name:str,
                     start_controller = 'position_joint_trajectory_controller',
                     collision_thresholds = {"F": 50, "T": 20, "tq": 30},
                     gripper_init_kwargs = {},
                     toolchanger = None,
                     toolchanger_init_kwargs = {}):
    """ Function to initialize a robot and a robot tool.

    Args:
        robot_name : str
            Robot name, i.e. robot controller namespace, such as "panda_1".
        tool_name: str
            One of elements in ee_config_dict, will be looked for in disassembly_pipeline/robot_ee_settings
        start_controller : str
            The controller to start, either "position_joint_trajectory" or "cartesian_impedance".
        collision_thresholds : (dict["F": 50, "T": 20, "tq":30])
            maximum EE force, torque, and joint torques, to consider a collision as having happened.
        gripper_init_kwargs: dict
            any args to pass to the gripper object during initialization.
        toolchanger: bool
            if not None or False, initialize a robotblockset_python.ros.grippers_ros ToolChanger object
        toolchanger_init_kwargs : dict
            kwargs to pass during ToolChanger initialization.
    Returns:
        robot: robotblockset_python.Robot object
            robot object with robot.gripper and robot.tc (toolchanger) if they were initialized

    Example call:
    >>> panda_1 = initialize_robot(robot_name = 'panda_1',
                     tool_name = 'ThreeJawChuck',
                     start_controller = 'position_joint_trajectory_controller',
                     collision_thresholds = {"F": 70, "T": 20, "tq": 30},
                     gripper_init_kwargs = {},
                     toolchanger = True,
                     toolchanger_init_kwargs = {})

    """
    # TODO don't have 3 useless dicts that confuse users.
    # CHANGED: Now using globals() fn and pre-imported gripper classes. Gripper class name must be same as param tool_name.
    #tool_str_to_tool_objects = {'VariableStiffnessGripper': VariableStiffnessGripper,
    #                            'SofthandGripper': SofthandGripper,
    #                            'ThreeJawChuck': ThreeJawChuck}

    # TODO: Move this to gripper object or similar. 
    # Gripper objects are in robotblockset. EE configs are specific to reconcycle (changing adapters, toolchanger or not, etc...)
    ee_config_dict = {"ThreeJawChuck": "tc_and_3jaw_chuck.json",
                      "Adapter" : "tc_and_adapter.json",
                      "VariableStiffnessGripper" : "tc_and_adapter.json"} # Downstream, pass config file to robot.SetNewEEConfig
                    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Temporary bullshit

    robot = panda_ros(robot_name, init_node=True, init_frankadesk_gripper_TCP=True, start_controller=start_controller)
    robot.Stop_controller()
    robot.SetJointImpedanceFranka(np.array([9000]*7), restart=False)
    robot.SetCollisionBehavior(**collision_thresholds, restart = False) # Restart required because controllers must be stopped before changing thresholds.
    robot.Start_controller()
    robot.error_recovery()
    robot.GetState()
    robot.ResetCurrentTarget()
    
    if tool_name is not None:
        #assert tool_name in tool_str_to_tool_objects.keys(),(f"{tool_name}, should be one of: {tool_str_to_tool_objects.keys()}") 
        #robot.gripper = tool_str_to_tool_objects[tool_name](**gripper_init_kwargs)

        # Assume gripper class is already imported (import everything from robotblockset.ros.grippers_ros)
        if tool_name not in globals().keys(): raise ValueError(f"Ensure gripper {tool_name} is imported in robot_quick_init.py")
        gripper_class = globals()[tool_name]
        robot.gripper = gripper_class(**gripper_init_kwargs)

        rospy.set_param('{}/current_tool'.format(robot_name), tool_name)

    if (toolchanger not in [None, False]) and (tool_name is not None):
        
        # Only set EE config if toolchanger is being used. Otherwise the default config in FrankaDesk is used.
        robot.SetNewEEConfig(ee_config_dict[tool_name])

        tc = ToolChanger(**toolchanger_init_kwargs)
        robot.tc = tc

    return robot