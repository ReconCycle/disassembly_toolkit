import copy
import rospy
from robotblockset_python.ros.grippers_ros import *
from disassembly_pipeline.skills.robot_homing import RobotHoming


def change_tool(robot, desired_tool:str = 'tc_and_3jaw_chuck.json'):
    
    tp_long_vac_preposition_q = [-1.6974549932563514, 0.5622164697527973, -0.1705058157554739, -1.886564042304372, 0.13208182244350858, 2.445774705713231, 1.6937645266388321]
    tp_3jaw_preposition_q = [-1.2575119912749841, 0.5589472784745064, -0.029530055463149456, -1.8817095682579172, -0.03882824401369601, 2.4379272724122636, 0.3010791150103023]
    
    above_tp_long_vac = [-0.18224908, -0.45714616,  0.1298632 ,  0.00288038,  0.1402869 , -0.99010521, -0.00172288]
    above_tp_3jaw  = [ 0.17626013, -0.46180917,  0.12726233,  0.00320412, -0.92497183, 0.37983635,  0.01188216]

    shaft_in_tp_3jaw = [ 0.17615456, -0.46138909,  0.02729533,  0.00385623, -0.92500563, 0.37971298,  0.01295284]
    shaft_in_tp_long_vac = [-0.18206466, -0.45777135,  0.02991378,  0.0025545 ,  0.14032681, -0.99009766, -0.00291311]

    between_tps_q = [-1.428628551575325, 0.49500293770140064, -0.15874068058776955, -2.0057346611784035, 0.08766030245186336, 2.5151915648773375, 1.9954320150348874]
    
    current_tool = rospy.get_param(f'{robot.Name}/current_tool')
    allowed_tools = ['tc_and_vacuum_demo.json', 'tc_and_vacuum_demo.json', 'tc_and_3jaw_chuck.json', 'tc_and_adapter.json']

    current_controller = copy.deepcopy(robot._control_strategy)
    if current_controller != 'JointPositionTrajectory':
        robot.Switch_controller(start_controller = 'JointPositionTrajectory')
    if robot.Name != 'panda_1':
        raise Exception(f"Invalid robot name: {robot.Name}")
    
    ee_dict = {"tc_and_3jaw_chuck.json" : ThreeJawChuck(),
               "tc_and_vacuum_demo.json": OffsetVacuumGripper(),
               "tc_and_vacuum_demo.json" : OffsetVacuumGripper()}

    # TODO add increase cartesian force limits because force drops a lot when changing from heavy 3 jaw chuck
    skill_homing = RobotHoming()
    skill_homing.on_enter(robot = robot)
    

    if current_tool == 'tc_and_vacuum_demo.json':
        robot.error_recovery()
        robot.JMove(tp_long_vac_preposition_q, t = 2)
        print("Finished JMove to preposition q")
        robot.CMoveFor([0, 0, -0.15], t=2)
        print("Finished CMoveFor to toolpost level")
        robot.CMoveFor([0, 0.155, 0], t=2)
        print("Finished CMoveFor inside toolpost")
        robot.CMoveFor([0, 0, -0.008], t=1)
        print("Finished CMoveFor before drop")
        robot.gripper.reset()
        robot.tc.open()
        print("Finished releasing tool")
        robot.CMoveFor([0, 0, 0.03], t=1)
        print("Finished raising up from toolpost")
        robot.SetNewEEConfig('tc_and_adapter.json')
        check_for_error(robot=robot)
        print("Finished setting new EE config: tc_and_adapter.json")
        print(f"New TCP: {robot.TCP}")
        robot.JMove(tp_long_vac_preposition_q, t = 2)
        robot.JMove(between_tps_q, t = 2)

    elif current_tool == 'tc_and_3jaw_chuck.json':
        robot.error_recovery()
        robot.JMove(tp_3jaw_preposition_q, t = 2)
        print("Finished JMove to preposition q")
        robot.CMoveFor([0, 0, -0.15], t=2)
        robot.CMoveFor([0, 0.15, 0], t=2)
        robot.CMoveFor([0, 0, -0.008], t=1)
        print("Finished CMoveFor before drop")
        robot.gripper.reset()
        robot.tc.open()
        print("Finished releasing tool")
        robot.CMoveFor([0, 0, 0.03], t=1)
        print("Finished raising up from toolpost")
        robot.SetNewEEConfig('tc_and_adapter.json')
        check_for_error(robot=robot)
        print("Finished setting new EE config: tc_and_adapter.json")
        print(f"New TCP: {robot.TCP}")
        robot.JMove(tp_3jaw_preposition_q, t = 2)
        robot.JMove(between_tps_q, t = 2)
    elif current_tool == 'tc_and_adapter.json':
        pass
    else:
        raise Exception(f"Invalid tool mounted, must be one of: {allowed_tools}")

    if desired_tool == 'tc_and_vacuum_demo.json':
        robot.error_recovery()
        robot.tc.open()
        robot.JMove(tp_long_vac_preposition_q, t = 2)
        print("Finished JMove to preposition q")
        robot.CMove(above_tp_long_vac, t=2)
        robot.CMove(shaft_in_tp_long_vac, t = 2)
        robot.CMoveFor([0, 0, -0.014], t=1)
        robot.tc.close()
        check_for_error(robot=robot)
        robot.SetNewEEConfig(desired_tool)
        check_for_error(robot=robot)
        print(f"Finished setting new EE config: {desired_tool}")
        print(f"New TCP: {robot.TCP}")
        robot.gripper=ee_dict[desired_tool]
        print(f"New gripper: {robot.gripper}")
        robot.CMoveFor([0, 0, 0.01], t=1)
        robot.CMoveFor([0, -0.15, 0], t=2)
        robot.CMoveFor([0, 0, 0.15], t=2)
        robot.JMove(between_tps_q, t = 1)
        
    elif desired_tool == 'tc_and_3jaw_chuck.json':
        robot.error_recovery()
        robot.tc.open()
        robot.JMove(tp_3jaw_preposition_q, t = 2)
        print("Finished JMove to preposition q")
        robot.CMove(above_tp_3jaw, t=2)
        robot.CMove(shaft_in_tp_3jaw, t = 2)
        robot.CMoveFor([0, 0, -0.014], t=1)
        robot.tc.close()
        check_for_error(robot=robot)
        robot.SetNewEEConfig(desired_tool)
        check_for_error(robot=robot)
        print(f"Finished setting new EE config: {desired_tool}")
        print(f"New TCP: {robot.TCP}")
        robot.gripper=ee_dict[desired_tool]
        print(f"New gripper: {robot.gripper}")
        robot.CMoveFor([0, 0, 0.01], t=1)
        robot.CMoveFor([0, -0.15, 0], t=2)
        robot.CMoveFor([0, 0, 0.15], t=2)
        robot.JMove(between_tps_q, t = 1)
    elif desired_tool == 'tc_and_adapter.json':
        pass
    else:
        raise Exception(f"Invalid tool mounted, must be one of: {allowed_tools}")

    # Switch back to initial controller
    if current_controller != 'JointPositionTrajectory':
        robot.Switch_controller(start_controller = 'JointPositionTrajectory')
        
        
        
def check_for_error(robot):
    robot.GetState()
    if robot.state.robot_mode != 2:
        robot.error_recovery()