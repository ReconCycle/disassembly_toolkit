from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
import json
from colorama import Fore
from disassembly_pipeline.utils.disassembly_manager import DisassemblyManager
from robot_quick_init import DummyRobot
import numpy as np

def updateToolPose(robot:DummyRobot, tool_name:str, pose_type:str, rotation:int, manager:DisassemblyManager, config:str):
    #TODO: Implement absolute positions
    #TODO: Update docstrings
    #TODO: Implement ee configs
    """
    This function stores the current position of P1 with the current TCP as a new toolpost position
    
    **CURRENTLY ONLY WORKS WITH P1 SINCE IT'S THE ONLY ONE WITH A TOOLCHANGER**
    
    Args:
    -----
        - robot(DummyRobot) : The robot class from which the function acquires the current TCP position
        - tool_name(str) : The name of the tool, this is going to be the name of the new entry
        - pose_type(str) : Each toolpost has 2 positions, one to be picked up and one to be dropped
            - 'grab' -> this position should be set with the TCP set to toolchanger only, since the robot won't be carrying a tool when mounting a new one
            - 'drop' -> this position should be set with the TCP set to the respective tool
        - rotation(int) : Specifies the orientation of the toolpost in respect to the robot base
        - manager(DisassemblyManager) : Allows the function to interface with the workcell
        - config(str) : The name of the .json config file for the respective tool's end-effector

    Examples:
    ---------
        >>> updateToolPose(
            robot=p1,
            tool_name='dual_vacuum_gripper',
            pose_type='grab',
            rotation=-90,
            manager=cyc_manager
        )

        >>> updateToolPose(
            robot=p1,
            tool_name='4jaw_chuck',
            pose_type='drop',
            rotation=90,
            manager=cyc_manager
        )
    """
    rotations = [90, -90, 180, -180]
    types = ['grab', 'drop']
    assert rotation in rotations, "Invalid rotation, should be: {}".format(rotations)
    assert pose_type in types, "Invalid pose type, should be: {}".format(types)
    
    with open("test_tools.json", "r+") as original_file:
        data = json.load(original_file)
        print(f"{Fore.RED} Original: {data}")

    print(f"{Fore.CYAN}testest")
    if pose_type=='grab':
        assert str(robot.TCP[2][-1])[:5] == '0.035', "You already have a tool mounted!"
    else:
        assert str(robot.TCP[2][-1])[:5] != '0.035', "You don't have a tool mounted!"

    rot = rot_z(rotation)

    print(f"{Fore.CYAN}testest")
    if tool_name in data["tools"]:
        print(f"{Fore.CYAN}testest")
        robot.GetState()
            
        data["tools"][tool_name].update(
            {pose_type:
                {
                "pose_q":list(robot._actual_int.q),
                "pose_x":list(robot._actual_int.x)
                }
            }
        )
    else:
        robot.GetState()
        data["tools"].update(
            {tool_name:
                {pose_type:
                    {
                    "pose_q":list(robot._actual_int.q),
                    "pose_x":list(robot._actual_int.x)
                    },
                "rotmat" : [rot[0][0], rot[0][1], rot[1][0], rot[1][1]],
                "absolute" : list(manager.tf2x(parent_frame='world', child_frame='panda_1_EE')),
                "ee_config" : config
                }
            }
        )

    with open("test_tools.json", "w") as outfile:
        json.dump(data, outfile, indent=4)
    print(outfile)
    outfile.close()

def updateRobotPose(robot_name:str, robot, position_name:str, subposition_tag:str, ee_config:str):
    """
    This function stores the current position of the specified robot
    
    Args:
    -----
        - robot_name(str) : Robot index in the database under which we want the position to be stored
        - robot(DummyRobot) : The robot class from which the function acquires the current position
        - position_name(str) : Name of the position we eant to store
        - subposition_tag(str): The subindex of the position, for example:
            
            'above' -> If the main position is for example 'vise' this is going to be above the vise
            
            'inside' -> If the main position is for example 'vise' this position is going to be inside the vise

    Examples:
    ---------
        >>> updateRobotPose(
            robot_name='p1',
            robot=p1,
            position_name='chuck_hekatron',
            subposition_tag='above'
        )

        >>> updateRobotPose(
            robot_name='p1',
            robot=p1,
            position_name='chuck_hekatron',
            subposition_tag='inside'
        )
    """
    robot_names = ["p1", "p2"]
    assert robot_name in robot_names, "Invalid robot name, should be: {}".format(robot_names)
    
    with open("/ros_ws/src/disassembly_pipeline/disassembly_pipeline/poses/pose_database.json", "r+") as original_file:
        data = json.load(original_file)
        print(f"{Fore.LIGHTGREEN_EX} Original: {data}")

    if robot_name == "p1":
        if subposition_tag != None:
            if position_name in data[robot_name]:
                robot.GetState()
                data["p1"][position_name].update(
                    {subposition_tag:
                        {
                        "pose_q":list(robot._actual_int.q),
                        "pose_x":list(robot._actual_int.x)
                        }
                    }
                )
            else:
                robot.GetState()
                data["p1"].update(
                    {position_name:
                        {subposition_tag:
                            {
                            "pose_q":list(robot._actual_int.q),
                            "pose_x":list(robot._actual_int.x)
                            }
                        }
                    }
                )
        else:
            data["p1"].update(
                {position_name:
                    {
                    "pose_q":list(robot._actual_int.q),
                    "pose_x":list(robot._actual_int.x)
                    }
                }
            )
    elif robot_name == "p2":
        if subposition_tag != None:
            if position_name in data[robot_name]:
                robot.GetState()
                data["p2"][position_name].update(
                    {subposition_tag:
                        {
                        "pose_q":list(robot._actual_int.q),
                        "pose_x":list(robot._actual_int.x)
                        }
                    }
                )
            else:
                robot.GetState()
                data["p2"].update(
                    {position_name:
                        {subposition_tag:
                            {
                            "pose_q":list(robot._actual_int.q),
                            "pose_x":list(robot._actual_int.x)
                            }
                        }
                    }
                )
        else:
            data["p2"].update(
                {position_name:
                    {
                    "pose_q":list(robot._actual_int.q),
                    "pose_x":list(robot._actual_int.x)
                    }
                }
            )
        print(f"{Fore.LIGHTMAGENTA_EX} New: {data}")
    with open("/ros_ws/src/disassembly_pipeline/disassembly_pipeline/poses/pose_database.json", "w") as outfile:
        json.dump(data, outfile, indent=4)
    print(outfile)
    outfile.close()


    

    