#!/usr/bin/env python
import numpy as np
import time
import copy
import rostopic, rosgraph
import rospy
from rospy import Time
import tf2_ros as tf2
import warnings
from franka_msgs.msg import FrankaState as fs

import json

warnings.filterwarnings('ignore')

from robotblockset_python.transformations import *
from robotblockset_python.robots import robot as Robot
from panda_ros import panda_ros as PandaROS

import disassembly_pipeline # so we can access __path__
from .Levering_block_v2 import LeverBlock
from .cell_init_utils import CellManager, userdata
from .yaml_parse_utils import parse_yaml_to_get_q
from .pneumatics_utils import PneumaticsManager
from .vision_utils import set_realsense_height
from .tf_utils import tf_obj2x
from .vision_utils import VisionUtils, set_realsense_height, get_realsense_height
from .safety_utils import get_visiontable_pickup_safety, get_position_within_four_bboxes
from ..skills.disassembly_skills import Move
from .multithreading import do_concurrently

from typing import Optional
#from ..environment.environment import Environment
#from ..environment.robot_to_env_interface import Franka_robot_env_wrapper
#from ..environment.camera_to_env_interface import Camera_wrapper

#from .robot_quick_init import initialize_robot

#from context_action_framework.types import *
#from context_action_framework.msg import Detection, Detections

from ..skills.robot_homing import robot_go_to_init_and_open_gripper

import os
#print(os.listdir())
pose_database_file = open('/ros_ws/src/disassembly_pipeline/disassembly_pipeline/poses/pose_database.json')
#pose_database_file = open(disassembly_pipeline.__path__[0] + '/' + 'pose_database.json') # Todo __path__ has several variables, if order changes this [0] might fail.
pose_database = json.load(pose_database_file)
p1_q_init = pose_database['q_init']['p1']
p2_q_init = pose_database['q_init']['p2']

class DisassemblyManager:
    """ 
    Superclass, containing utilities for managing the  workcell when operational
    When initialize_using_cellmanager is true, the states will be initialised internally and do not need to be passed

    Args :
    ------
        - p1 (PandaROS) : Panda 1 robot object
        - p2 (PandaROS) : Panda 2 robot object
        - user_data (userdata) - userdata object
        - jointstate (JointState) : Passes through for other functions to use
        - user_data (UserData) : Passes through for other functions to use
        - cnc_airblock_state (ActivateRaspiDigitalOutput) : Stores the state of the CNC airblock
        - cnc_chuck_state (ActivateRaspiDigitalOutput) : Stores the state of the CNC chucks
        - basler_state (ActivateBoolService) : Stores the state of the basler portion of vision utils
        - realsense_state (ActivateBoolService) : Stores the state of the realsense porion of vision utils
        - panda_1_airblock_state (ActivateRaspiDigitalOutput) : Stores the state of panda_1 airblock
        - block2_state (ActivateRaspiDigitalOutput) : Stores the states of the vise airblock (takes multiple ports)
        - cutter_state (ActivateRaspiDigitalOutput) : Stores the state of the cutter module
        - cutter_airblower_st (ActivateRaspiDigitalOutput) : Stores the state of the cutter evacuation nozzle
        - tf_manager (TFManager) : Initialised transform listener object 
    """
    def __init__(self,
                p1: Optional[PandaROS] = None,
                 p2: Optional[PandaROS] = None,
                 user_data:userdata = None,
                 basler_state = None,
                 realsense_state = None,
                 block_2_state = None,
                 mainjaws_state = None,
                 sidejaws_state = None,
                 slider_state = None,
                 clamptray_state = None,
                 rotate_vise_state = None,
                 cutter_state = None,
                 cutter_airblast_state = None,
                 tf_manager = None,
                 use_moveit = False,
                 initialize_using_cellmanager = True,
                 initialize_modules = ['cnc', 'panda_1', 'vision'],
                 parameters_filename = "/ros_ws/src/disassembly_pipeline/disassembly_pipeline/skills/object_grasp_parameters.json"
                 ):

        # Check the modules we want to initialize are valid
        valid_modules = ['vision', 'cutter', 'vise', 'cnc', 'panda_1']
        for element in initialize_modules:
            assert element in valid_modules, (f"Invalid module {element}, module should be one, specified in: {valid_modules}")

        if initialize_using_cellmanager:
            self.cm = CellManager()
            try:
                rospy.rostime.get_rostime() # This will fail if node is not initialized (if both p1 and p2 are DummyRobot, for example)

            except:
                rospy.init_node("disassembly_manager", anonymous = True)

            jointstate, user_data, cnc_airblock_st, cnc_chuck_st, basler_state, realsense_state, activate_panda1_airblock_st, block_2_state, \
                mainjaws_state, sidejaws_state, slider_state, clamptray_state, rotate_vise_state, cutter_state, cutter_airblast_state, tf_manager = self.cm.init_cell(initialize_modules)

        self.parameters_filename = parameters_filename


        self.robots = [p1, p2]

        self.userdata = user_data

        # CNC stuff
        self.cnc_airblock_state = cnc_airblock_st
        self.cnc_chuck_state = cnc_chuck_st

        # Vision stuff
        self.basler_state = basler_state
        self.realsense_state= realsense_state
        # Airblocks stuff
        self.panda_1_airblock_st = activate_panda1_airblock_st
        self.block_2_state = block_2_state 
        # Vise stuff
        self.mainjaws_st = mainjaws_state
        self.sidejaws_state = sidejaws_state
        self.slider_state = slider_state
        self.clamptray_state = clamptray_state
        self.rotate_vise_state = rotate_vise_state
        # Cutter stuff
        self.cutter_state = cutter_state
        self.cutter_airblast_state = cutter_airblast_state
        self.tf_manager = tf_manager
        # Ros TF stuff
        self.tf2x = self.tf_manager.tf2x # Shortcut for faster use.
        self.sendTf = self.tf_manager.SendTransform2tf

        # Initialize PneumaticsManager class
        self.pneum_mngr = PneumaticsManager(userdata = self.userdata,
                                    cnc_airblock_state = self.cnc_airblock_state, \
                                    cnc_chuck_state = self.cnc_chuck_state, panda_1_airblock = self.panda_1_airblock_st, \
                                    activate_block2_st = self.block_2_state, \
                                    mainjaws_st = self.mainjaws_st, sidejaws_st = self.sidejaws_state, \
                                    slider_st = self.slider_state, clamptray_st = self.clamptray_state, \
                                    rotate_holder_st = self.rotate_vise_state, \
                                    cutter_st = self.cutter_state, cutter_airblower_st = self.cutter_airblast_state
                                    )
        
        # initialise vision utils
        # For basler
        self.vis_utils_basler = VisionUtils(camera_name = 'basler', vision_topic = '/vision/basler/detections', table_name = 'table_vision')
        # For realsense
        self.vis_utils_realsense = VisionUtils(camera_name = 'realsense', vision_topic = '/vision/realsense/detections') 

        self.last_print_t = time.time() # Keeping track of last print time, dont flood.
        self.min_print_dt = 0.1

        self.p2_leverblock = None # We will save the leverblock after initializing it.

        # Init positions for robots
        self.robot_init_qs = {}
        if p1:
            self.p1 = p1
            self.robot_init_qs[self.p1.Name] = [p1_q_init]
        if p2:
            self.p2 = p2
            self.robot_init_qs[self.p2.Name] = [p2_q_init]

        # Stuff for recording the robot movements/states
        self.recording_n = 0
        #self.recording_length_s =
        #self.recorded_states_list = np.array([self.p1.state for i in range(4)])
        self.recorded_states_list = []

        # Disassembly parameters
        with open(self.parameters_filename) as f: self.parameters = json.load(f) # Parameters for gripping an dropping different objects. 

        self.disassembly_params = {'use_cmove_instead_of_cpath':False,
                    'use_moveit':use_moveit,
                    'cycle_speed':'slow',
                    'realsense_height':0.34,
                    'levering_direction_deg': 90} # The direction in which to lever (Rotation around robot base C.S z-axis 

        self.max_speed_acceleration_factors = { 'slow': 0.5 ,
                                                'fast': 0.8,
                                                'warp': 0.9,
                                                'ludicrous':1} #robots tend to shake

        # Handle speed settings for cycle
        self.speed_factors = {'slow':1, 
                              'fast':1.4, 
                              'warp':1.85, 
                              'ludicrous': 2}
        
        self.set_cycle_speed(self.disassembly_params['cycle_speed'])

        # Handle realsense camera settings
        # Param name for setting the camera (Realsense) to surface height for 3D world coordinates calculation
        self.realsense_height_param_name = '/vision/realsense_height'
        # Default realsense height is 0.34, functions should change it later.
        set_realsense_height(self.realsense_height_param_name, Z = self.disassembly_params['realsense_height'])

        if self.disassembly_params['use_moveit']:
            #from ..moveit.moveit_integration import move_with_moveit
            from disassembly_pipeline.moveit.moveit_integration import ReconCycleMoveitManager

            self.MoveitManager = ReconCycleMoveitManager(robot_objects = [self.p1, self.p2])

            self.MoveitManager.init_moveit()

            # Also add to robot objects so downstream PDDL operators/skills can use it.
            # Add moveit handles to the robot object
            for robot in self.robots:
                robot.robot_commander = self.MoveitManager.robot_commanders[robot.Name]
                robot.move_group = self.MoveitManager.move_groups[robot.Name]
                robot.planning_scene_interface = self.MoveitManager.planning_scene_interfaces[robot.Name]
        
    def set_cycle_speed(self, speed:str = 'slow'):
        valid_speeds = ['slow', 'fast', 'warp', 'ludicrous']
        assert speed in valid_speeds, (f"Invalid speed {speed}, should be one, listed in: {valid_speeds}")
        self.speed_factor = self.speed_factors[speed]
    
    def perform_cycle(self):
        #TODO: Ko bo pddl zares implementiran, brisi!!!
        USE_ENVIRONMENT = False

        # Initialize environment
        if USE_ENVIRONMENT:
            env = Environment(ros = True)
            self.env = env

            p1_wrapper = Franka_robot_env_wrapper(self.p1)
            p2_wrapper = Franka_robot_env_wrapper(self.p2)

            env.add_env_object(obj = p1_wrapper, obj_type = 'robot')
            env.add_env_object(obj = p2_wrapper, obj_type = 'robot')

            cam_basler = Camera_wrapper(name = 'basler', camera_topic = '/vision/basler/colour')
            cam_realsense = Camera_wrapper(name = 'realsense', camera_topic = '/vision/realsense/colour')

            env.add_env_object(obj = cam_basler, obj_type = 'camera')
            env.add_env_object(obj = cam_realsense, obj_type = 'camera')

            env.add_env_object(obj = self.vis_utils_basler, obj_type = 'vision_info_feeder')
            env.add_env_object(obj = self.vis_utils_realsense, obj_type = 'vision_info_feeder')

            # Update environment state (find visible smoke detectors, HCAs, etc.)
            detected_env_objects = env.get_all_detected_objects()
        
        # Select one of the object to disassemble.
        #selection_fn = self.select_by_priority
        #selection_fn_kwargs = {'priority': ['hca_back', 'smoke_detector', 'battery']}
        #object_to_disasemble = selection_fn(detected_env_objects, selection_fn_kwargs) 

        # Select action and perform it

        0
    
    #TODO: Spremeni parameter rotation_deg, da bo sprejel transformacijsko matriko, ne pa samo integerja rotacije, naredi da bo intuitivno!!!
    #TODO: Treba je dorect kako bomo handlali hitrost robotov
    def init_leverblock(self, robot:PandaROS, hca_type:str = 'kalo', direction_deg:int = 90, speed_factor:float = 1) -> LeverBlock:
        """
        Initializes levering parameters for HCA disassembly

        Args:
        -----
            - robot (PandaROS) : Robot object
            - hca_type (str) : Type of HCA we want to diassemble
            - direction_deg (int) : The orientation of the levering fulcrum, in respect to the robot's Z axis
            - speed_factor (float) : Specifies the robot speed, 1 is full specified speed, other factors slow it down or speed it up

        Returns:
        --------
            - leverblock (LeverBlock) : Class containing all parameters needed for levering operations
        """
        valid_hcas = ['kalo', 'qundis']
        assert hca_type in valid_hcas, f"Invalid HCA type: {hca_type}, should be one of: {valid_hcas}"
        assert direction_deg is not None, f"Invalid levering direction, shoulde be other than None"

        #################### parameters
        #TODO: Pojasni parametre (Boris)
        if hca_type == 'kalo':
            # Written in robot coord system. Should be written relative to vise. FIX.
            start_cart_pos = [0.68, -0.105, 0.18] # Z min is 0.16

            R_limits = (-0.35, -100)
            #self.PDMP_M_THR_CONDITION = 1
        else:
            #start_cart_pos = [0.6695, -0.085, 0.17]
            #start_cart_pos = [0.68, -0.09, 0.17]
            start_cart_pos = [0.678, -0.09, 0.17]

            R_limits = (-0.7, -100) # Tune this 0.8. In general the short one will be levered much faster.
            #self.PDMP_M_THR_CONDITION = 1
            
        rospy.loginfo("Leverblock cart start pos:")
        rospy.loginfo(start_cart_pos)

        #Initialise LeverBlock class
        leverblock = LeverBlock(robot = robot,
                                end_effector = 'softclaw',
                                to_module = None, 
                                neutral_rot_vertical = 1,
                                initial_cart_pos = start_cart_pos, 
                                direction = direction_deg, 
                                bbox = None,
                                allowed_tool_angles = (None,None), 
                                speed_factor= speed_factor)
            
        ####### LEVERBLOCK PARAMETERS
        leverblock.PDMP_M_THR_CONDITION = 1.5
        leverblock.allowed_tool_angles = R_limits # Checking p2.R[2,2] so the Z component.
        
        return leverblock
            
    def prepare_vision(self, activate_basler:bool = None,
                        activate_realsense:bool = None,
                        set_publish_basler:bool = None,
                        set_publish_realsense_labeled:bool = None,
                        set_publish_realsense_other:bool = None):
        
        if activate_basler is not None:
            assert activate_basler in [True, False], "'activate_basler' argument should be a boolean type"
            self.userdata.value = activate_basler
            self.basler_state.on_enter(self.userdata);time.sleep(0.2)
            
        if activate_realsense is not None:
            assert activate_realsense in [True, False], "'activate_realsense' argument should be a boolean type"
            self.userdata.value = activate_realsense
            self.realsense_state.on_enter(self.userdata);time.sleep(0.2)

        if set_publish_basler is not None:
            assert set_publish_basler in [True, False], "'set_publish_basler' argument should be a boolean type"
            rospy.set_param('/vision/basler/publish_labeled_img', set_publish_basler)

        if set_publish_realsense_labeled is not None:
            assert set_publish_realsense_labeled in [True, False], "'set_publish_realsense_llabeled' argument should be a boolean type"
            rospy.set_param('/vision/realsense/publish_labeled_img', set_publish_realsense_labeled)

        if set_publish_realsense_other is not None:
            assert set_publish_realsense_other in [True, False], "'set_publish_realsense_other' argument should be a boolean type"
            rospy.set_param('/vision/realsense/publish_depth_img', set_publish_realsense_other)
            rospy.set_param('/vision/realsense/publish_cluster_img', set_publish_realsense_other)


            
        return 0
        
    

