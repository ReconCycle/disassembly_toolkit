#!/usr/bin/env python
from .tf_utils import TFManager

from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
from reconcycle_flexbe_states.MoveSoftClaw import MoveSoftClaw
#from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.activate_raspi_output import ActivateRaspiDigitalOutput
from reconcycle_flexbe_states.ReadNextVisionAction import ReadNextVisionAction
from reconcycle_flexbe_states.activate_bool_service import ActivateBoolService

#from reconcycle_flexbe_states.write_to_mongodb import WriteToMongo
#from reconcycle_flexbe_states.write_to_mongodb_pose import WriteToMongoPOSE
#from reconcycle_flexbe_states.write_to_mongodb_tf import WriteToMongoTF
#from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
#from reconcycle_flexbe_states.read_from_mongodb_POSE import ReadFromMongoPOSE

from reconcycle_flexbe_states.get_next_action import GetNextAction
import sys
sys.path.append('/ros_ws/src/context_action_framework')
from context_action_framework.types import Action, Detection, Gap, Label, Module, Robot, EndEffector, Camera

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.ros.grippers_ros import VariableStiffnessGripper, SofthandGripper

import rospy
from sensor_msgs.msg import JointState

class userdata:
    def __init__(self):
        # Read tf cart lin
        self.offset = [0,0,0]
        self.rotation = [0,0,0]
        self.t2_data = None
        # Softhand
        self.goal_hand_pos = [0]
        #Softclaw
        self.goal_claw_pos = [0]
        self.goal_claw_vel = [0]
        self.goal_claw_acc = [0]
        self.goal_claw_effort = [0]
        # Slider, mainjaws, sidejaws, holder_rotation, clamping_tray(below)
        self.value = False

class CellManager:
    def __init__(self):
        pass
    def init_cell(self, initialize_modules:list = None):
        """
        Cell initialisation toolkit

        Initializes hardware dependancies for specific objects in the workcell

        Args:
        -----
            - initialize_modules (list) : Modules to initialise

        Returns:
        --------
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
            - tf_listener (GenericTransformListener) : Initialised transform listener object 
        """
        #TODO: Panda 2???
        valid_modules = ['vision', 'cutter', 'vise', 'cnc', 'panda_1', 'panda_2']
        for element in initialize_modules:
            assert element in valid_modules, (f"Invalid module {element}, module should be one of the ones, specified in: {initialize_modules}")

        self.jointstate  = JointState()
        self.user_data = userdata()

        #Init userdata
        # Init flexbe states
        # Enabling vision pipeline
        init_panda_1 = None
        init_vision = None
        init_vise = None
        init_cutter = None
        init_cnc = None

        if 'panda_1' in initialize_modules:
            init_panda_1 = True
        self.panda_1_airblock_state = self.initialize_panda_1(init_panda_1=init_panda_1)
        
        if 'vision' in initialize_modules:
            init_vision = True
        self.basler_state, self.realsense_state = self.initialize_vision(init_vision)

        if 'vise' in initialize_modules:
            init_vise = True
        self.mainjaws_state, self.sidejaws_state, self.slider_state, self.clamptray_state, self.rotate_holder_state, self.block2_state = self.initialize_vise(init_vise=init_vise)

        if 'cutter' in initialize_modules:
            init_cutter = True
        self.cutter_state, self.cutter_airblower_state = self.initialize_cutter(init_cutter= init_cutter)

        if 'cnc' in initialize_modules:
            init_cnc = True
        self.cnc_airblock_state, self.cnc_chuck_state = self.initialize_cnc(init_cnc = init_cnc)
        
        self.tf_listener = self.initialize_misc()

        #ReadTFCartLin(target_frame = "hca_back_vision_table_zero", source_frame = "panda_1/panda_1_link0")
        #self.softhand_st = MoveSoftHand(motion_duration = 2, motion_timestep = 0.1)
        #self.softclaw_st = MoveSoftClaw(motion_duration = 2)

        #self.db_write_j = WriteToMongo() # ud.entry_data = q
        #self.db_write_pose = WriteToMongoPOSE() # ud
        #self.db_write_tf = WriteToMongoTF()
        #self.db_read_j = ReadFromMongo()
        #ud.entry_name = entry_name
        #j = db_read_j.on_enter(ud)

        #db_read_pose = ReadFromMongoPOSE(robot_ns = p1.Name)
        #ud.entry_name = entry_name
        #pose = db_read_pose.on_enter(ud)

        return self.jointstate, self.user_data, self.cnc_airblock_state, self.cnc_chuck_state, self.basler_state, self.realsense_state, self.panda_1_airblock_state, \
            self.block2_state, self.mainjaws_state, self.sidejaws_state, self.slider_state, self.clamptray_state, self.rotate_holder_state, self.cutter_state, self.cutter_airblower_state, self.tf_listener

    def initialize_panda_1(self, init_panda_1 = None):
        if init_panda_1 is not None:
            panda_1_airblock_state = ActivateRaspiDigitalOutput(service_name='airblock2_air_ON')
        else:
            panda_1_airblock_state = None
        return panda_1_airblock_state
    
    def initialize_cnc(self, init_cnc = None):
        if init_cnc is not None:
            cnc_airblock_state = ActivateRaspiDigitalOutput(service_name = 'cnc_airblock_activate')
            cnc_chuck_state = ActivateRaspiDigitalOutput(service_name = 'cnc_chuck_open')
        else:
            cnc_airblock_state = None
            cnc_chuck_state = None
        return cnc_airblock_state, cnc_chuck_state
    
    def initialize_vise(self, init_vise = None):
        if init_vise is not None:
            mainjaws_state = ActivateRaspiDigitalOutput(service_name = 'mainjaws_activate')
            sidejaws_state = ActivateRaspiDigitalOutput(service_name = 'sidejaws_activate')
            slider_state = ActivateRaspiDigitalOutput(service_name = 'move_slide')
            clamptray_state = ActivateRaspiDigitalOutput(service_name = 'clamping_tray')
            rotate_holder_state = ActivateRaspiDigitalOutput(service_name = 'obr_rotate')
            block2_state = ActivateRaspiDigitalOutput(service_name = 'obr_block2_ON')
        else:
            mainjaws_state = None
            sidejaws_state = None
            slider_state = None
            clamptray_state = None
            rotate_holder_state = None
            block2_state = None

        return mainjaws_state, sidejaws_state, slider_state, clamptray_state, rotate_holder_state, block2_state

    def initialize_cutter(self, init_cutter = None):
        if init_cutter is not None:
            cutter_state = ActivateRaspiDigitalOutput(service_name = 'cutter_activate')
            cutter_airblower_state = ActivateRaspiDigitalOutput(service_name = 'cutter_clean')
        else:
            cutter_state = None
            cutter_airblower_state = None
        return cutter_state, cutter_airblower_state

    def initialize_vision(self, init_vision = None):
        """ Initializes states/classes which enable turning vision system on and off."""

        if init_vision is not None:
            basler_state = ActivateBoolService(service_name ='/vision/basler/continuous')
            realsense_state = ActivateBoolService(service_name ='/vision/realsense/continuous')
        else:
            basler_state = None
            realsense_state = None

        return basler_state, realsense_state

    def initialize_misc(self):
        """ Initializes other misc parts:
        including:

        GenericTransformListener()"""

        tf_listener = TFManager()

        return tf_listener

if __name__ == '__main__':

    cellmanager = CellManager()
    jointstate, user_data, basler_state, realsense_state, block2_state, mainjaws_state, sidejaws_state, slider_state, clamptray_state, rotate_holder_state, cutter_state, cutter_airblower_state, softhand_state, softclaw_state, tfs_stateate = cellmanager.init_cell()
    # Trying to write to MongoDB (Doesnt work currently)
    #ud.entry_name = 'p2_realsense_q'
    #ud.entry_data = p2_realsense_q
    #db_write_j.execute(ud)
