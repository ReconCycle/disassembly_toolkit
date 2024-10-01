import numpy as np
import json
import os
import sys
import matplotlib.pyplot as plt

# Set print options for numpy arrays
np.set_printoptions(precision=6,linewidth=110, suppress = True)

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.grippers import ToolChanger, ThreeJawChuck, dualVacuumGripper
from colorama import Fore
import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import rospy

import warnings
warnings.filterwarnings('ignore')

from disassembly_pipeline.utils.Levering_block_v3 import move_until_contact
from disassembly_pipeline.utils.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.utils.tf_utils import GenericTransformListener
from disassembly_pipeline.utils.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.utils.change_tool import DropCurrentAndPickUpNewTool, DetectAndGrabSmokeDetector
from disassembly_pipeline.utils.multithreading import do_concurrently
from disassembly_pipeline.skills.disassembly_skills import Move
from disassembly_pipeline.utils.disassembly_manager import DisassemblyManager, DummyRobot
from disassembly_pipeline.cnc_manager.src.action_client_cnc import CNCActionClient
##############################################################################################################################################################################a

class SmokedetectorRecycling():
    def __init__(self, robot:str):
        poseFile = open('../pose_database.json')
        self.poseDB = json.load(poseFile)
        self.chuck = ThreeJawChuck()
        
        allowedRobots = ['panda_1', 'panda_2']
        assert robot in allowedRobots, "Invalid robot name, should be: {}".format(allowedRobots)

        if robot == 'panda_1':
            self.p1 = panda_ros(robot, init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
            self.p1.Stop_controller()
            self.p1.SetNewEEConfig(tool_name='tc_and_adapter.json', restart=False)
            self.p1.SetJointImpedanceFranka(np.array([10000]*7), restart=False)
            self.p1.SetCollisionBehavior(F=70, T=20, tq=30, restart=False)
            self.p1.Start_controller()
            self.p1.error_recovery()
            self.toolChanger = ToolChanger()
            self.p1.GetState()
            self.p2 = DummyRobot(name='panda_2')
        else:
            self.p2 = panda_ros(robot, init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'position_joint_trajectory_controller')
            self.p2.SetNewEEConfig('tc_and_adapter.json')
            self.p2.SetCollisionBehavior(F=70, T= 20, tq = 30)
            self.p2.error_recovery()
            self.p2.GetState()
            self.p1 = DummyRobot(name='panda_1')

        self.dissassembyManager = DisassemblyManager(p1 = self.p1, p2 = self.p2)
        self.pneumaticsManager = self.dissassembyManager.pm
        self.pneumaticsManager.prepare_pneumatics()

        self.cncClient = CNCActionClient(wait_for_server=True, init_ros_node=False)
        self.cncClient.call_server_prespecified(gcode_operation_name='homing')

    def pick_up_chuck_and_place_detector_into_cnc(self):
        print("P1")
        self.p1.error_recovery()
        print("P2")
        self.p1.JMove(self.poseDB['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
        rospy.loginfo(f"{Fore.RED}Entered function, tf is going on?!")
        print(f"{Fore.RED}Entered function, tf is going on?!")
        DropCurrentAndPickUpNewTool(self.p1, self.poseDB, self.toolChanger, current_tool=None, new_tool='3JAWCHCK')

        #Pick and place smoke detector
        self.chuck.open()
        self.p1.JMove(self.poseDB['demo_poses']['smoke_detector_pickup_q'], t = 2)
        DetectAndGrabSmokeDetector(self.p1, self.chuck,self.dissassembyManager, self.poseDB, smoke_detector=0)
        self.p1.JMove(self.poseDB['demo_poses']['cnc_q'], t=3)

        smokedet_cnc_place_x = self.poseDB['demo_poses']['place_smoke_detector_to_cnc_x']
        smokedet_cnc_place_offset = np.array([0, 0, 0.1, 0, 0, 0, 0])

        self.pneumaticsManager.handle_cnc_chuck('open')

        self.p1.CMove(np.array(smokedet_cnc_place_x) + smokedet_cnc_place_offset, t=2)
        self.p1.CMove(smokedet_cnc_place_x, t=1)
        self.chuck.open()
        self.p1.CMove(np.array(smokedet_cnc_place_x) + smokedet_cnc_place_offset, t=2)
        self.pneumaticsManager.handle_cnc_chuck('close')
        self.p1.JMove(self.poseDB['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)

    def cut_smoke_detector_with_cnc(self):
        commands = ['G01X-241Y-150Z30F1000',
                'M3S1000',
                'G01X-241Y-154Z9F200',
                'G17G02X-241Y-154I0J-38F500',
                'G01X-241Y-147Z9F200',
                'G17G02X-241Y-147I0J-45F500',
                'M5',
                'G01X-241Y-151Z20F1000']
        self.cncClient.call_server(gcode=commands)

        command = 'G01X-241Y-151Z20F1000'
        self.cncClient.call_server(gcode=command)

    def change_to_vacuum_gripper_pick_up_battery(self):
        self.p1.JMove(self.poseDB['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
        vac_gripper = dualVacuumGripper()
        DropCurrentAndPickUpNewTool(self.p1, self.poseDB, self.toolChanger, current_tool='3JAWCHCK', new_tool='2VACGRPR')
        self.p1.JMove(self.poseDB['demo_poses']['cnc_q'], t=2)
        battery_cnc_x = self.poseDB['demo_poses']['battery_in_cnc_x']
        battery_cnc_offset = np.array([0, 0, 0.05, 0, 0, 0, 0])
        self.p1.CMove(np.array(battery_cnc_x) + battery_cnc_offset, t=1)
        vac_gripper = dualVacuumGripper()
        self.p1.CMove([ 0.429512,  0.042785,  0.193123,  0.013116, -0.6171  ,  0.786657, -0.013626], t=1)
        vac_gripper.grasp()
        time.sleep(1)
        self.p1.CMove(np.array(battery_cnc_x) + battery_cnc_offset, t=1)
        self.p1.JMove(self.poseDB['demo_poses']['battery_drop_q'], t=3)
        vac_gripper.open()
        self.p1.JMove(self.poseDB['q_init']['p1'], t = 2, qdot_max_factor = 0.4, qddot_max_factor = 0.4)
        DropCurrentAndPickUpNewTool(self.p1, self.poseDB, self.toolChanger, current_tool='2VACGRPR', new_tool=None)

if __name__ == '__main__':
    sdr = SmokedetectorRecycling(robot='panda_1')
    rospy.loginfo("Got A")
    sdr.pick_up_chuck_and_place_detector_into_cnc()
    rospy.loginfo("Got B")
    sdr.cut_smoke_detector_with_cnc()
    rospy.loginfo("Got C")
    sdr.change_to_vacuum_gripper_pick_up_battery()