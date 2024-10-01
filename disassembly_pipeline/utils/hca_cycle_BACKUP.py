import numpy as np
import time
import copy
import rostopic, rosgraph
import rospy
from rospy import Time
import tf2_ros as tf2
import warnings
from franka_msgs.msg import FrankaState as fs

warnings.filterwarnings('ignore')

from robotblockset_python.transformations import *

from .Levering_block_v2 import LeverBlock
from .cell_init_utils import CellManager
from .yaml_parse_utils import parse_yaml_to_get_q
from .pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray
from .vision_utils import set_realsense_height
from .tf_utils import tfobj_to_x
from .vision_utils import VisionUtils, set_realsense_height, get_realsense_height
from .safety_utils import get_visiontable_pickup_safety, get_position_within_four_bboxes
from .disassembly_blocks import MoveBlock
from .multithreading import do_concurrently


import sys
sys.path.append('/ros_ws/src/context_action_framework')

#from context_action_framework.types import *
#from context_action_framework.msg import Detection, Detections

from .saved_positions import p1_q1_init, p1_q1_init_downlow # , p1_q1_above_sl # p1_q1_above_table
from .saved_positions import p1_q_vice_grip_hca, p1_q_above_vice_grip_hca
from .saved_positions import p2_q1_init


class Disassembly_cycle:
    def __init__(self, p1, p2, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st,
                       sidejaws_st, slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st,
                       tfread_st, get_next_action_state, use_moveit = False):
        """ Args :
        p1 - robot panda_1 object
        p2 - robot panda_2 object
        ud - userdata object"""
        
        self.p1 = p1
        self.p2 = p2
        self.ud = ud
        self.activate_basler_st = activate_basler_st
        self.activate_realsense_st=activate_realsense_st
        self.activate_block2_st = activate_block2_st
        self.mainjaws_st = mainjaws_st
        self.sidejaws_st = sidejaws_st
        self.slider_st = slider_st
        self.clamptray_st = clamptray_st
        self.rotate_holder_st = rotate_holder_st
        self.cutter_st = cutter_st
        self.cutter_airblower_st = cutter_airblower_st
        self.tfread_st = tfread_st
        
        self.tf2x = self.tfread_st.tf2x # Shortcut for faster use.
        self.sendTf = self.tfread_st.SendTransform2tf
        
        self.get_next_action_st = get_next_action_state
        
        # Get param for realsense height, then set it 
        self.VU = VisionUtils('/vision/basler/detections')
        self.VU_rs = VisionUtils(vision_topic = '/vision/realsense/detections') # For realsense

        self.last_print_t = time.time() # Keeping track of last print time, dont flood.
        self.min_print_dt = 0.1

        self.p2_leverblock = None # We will save the leverblock after initializing it.
        # Init positions for robots
        self.robot_init_qs = {}
        self.robot_init_qs[self.p1.Name] = [p1_q1_init, p1_q1_init_downlow]
        self.robot_init_qs[self.p2.Name] = [p2_q1_init]
                        
        # Stuff for recording the robot movements/states
        self.recording_n = 0
        #self.recording_length_s = 
        #self.recorded_states_list = np.array([self.p1.state for i in range(4)])
        self.recorded_states_list = []
        
        # Disassembly parameters
        self.d_p = {'use_cmove_instead_of_cpath':False,
                    'use_moveit':use_moveit,
                    'cycle_speed':'slow',
                    'realsense_height':0.34,
                    'levering_direction_deg': 90} # The direction in which to lever (Rotation around robot base C.S z-axis 
        
        self.max_speed_acceleration_factors = {'slow': 0 ,
                                              'fast': 0,
                                            'warp': 0, 
                                            'ludicrous':0}
        
        # Handle speed settings for cycle
        self.speed_factors = {'slow':1, 'fast':1.4, 'warp':1.85, 'ludicrous': 2}
        self.set_cycle_speed(self.d_p['cycle_speed'])
        
        # Handle realsense camera settings
        # Param name for setting the camera (Realsense) to surface height for 3D world coordinates calculation
        self.realsense_height_param_name = '/vision/realsense_height'
        # Default realsense height is 0.34, functions should change it later.
        set_realsense_height(self.realsense_height_param_name, Z = self.d_p['realsense_height'])
        
        if self.d_p['use_moveit']:
            self.init_moveit()
            
    def init_moveit(self):
        #raise Exception("Not working. Seems like the reconcycle_description package should be installed inside the docker where this runs.")
        self.setup_moveit_planning_scene()
        
    def setup_moveit_planning_scene(self):
        # Moveit Stuff
        from disassembly_pipeline.moveit_planning_scene_generator import create_moveit_planning_scene
        from disassembly_pipeline.moveit_integration import setup_moveit_interface

        try:self.p1_pl_scn_interface = create_moveit_planning_scene(ns='/{}'.format(self.p1.Name))
        except Exception as e:print(e)
            
        # Moveit Stuff
        try:self.p2_pl_scn_interface = create_moveit_planning_scene(ns='/{}'.format(self.p2.Name))
        except Exception as e:print(e)
        


        self.p1_robot_commander, self.p1_move_group = setup_moveit_interface(ns = self.p1.Name, planning_scene_interface = self.p1_pl_scn_interface)
        self.p2_robot_commander, self.p2_move_group = setup_moveit_interface(ns = self.p2.Name, planning_scene_interface = self.p2_pl_scn_interface)
        
        return 0
        
    def clear_moveit_planning_scene(self):
        self.p1_pl_scn_interface.clear();time.sleep(0.1)
        self.p2_pl_scn_interface.clear();time.sleep(0.1)
        
    def set_cycle_speed(self, speed = 'slow'):
        assert speed in ['slow', 'fast', 'warp', 'ludicrous']
        self.sf = self.speed_factors[speed]
        
    def robot_flip_object(self, robot, init_safe = True):
        """ If we only detect hca_front, we might want to flip it using the softclaw """
        
        r = robot
        
        assert r.gripper.Name in ['softclaw', 'vsgripper'] 
        
        # Flip parameters
        # The parameters are a function of (gripper_type, object_type), where object type is kalo HCA, qundis HCA, smoke detector A, etc.
        
        self.flip_p = {}
        
        ### PARAMETERS
        rel_tool_move = np.eye(4)
        rel_tool_move[0:3, -1] = [-0.035, 0.03, 0] # For softclaw
        
        tool_rot_z = 15 # Rotate around Z a bit so the moveable gripper part doesnt hit the table
        tool_rot_y = 55 # 
        
        ### END PARAMETERS
        
        assert r.gripper.Name == 'vsgripper' # WE can only flip it with the softclaw or similar type gripper.
        r.gripper.open(1, sleep=False)
        
        self.prepare_vision(activate_basler=True)
        
        if init_safe:
            self.robot_go_to_init_and_open_gripper(r)
        
        if r._control_strategy != 'CartesianImpedance': 
            r.Switch_controller(start_controller =  'CartesianImpedance')
        
        r.error_recovery()
        
        r.SetCartesianStiff_helper(m=1.2,n=0.65)
        # First condition : THe HCa must be rotated so that the robot won't hit the vise or the basler camera frame.
        # THat is, the hca X must be at least approximately alligned with the robot X.
        
        
        out = self.VU.get_particular_class_from_detections(detections = [], desired_class = 'hca_front')
        if len(out) > 0:
            out = out[0] # Let's take the first detection
        else:
            rospy.loginfo("No HCA_front detected ! Exiting robot_flip_hca.")
            return 0
        
        # Get tf from 
        child_frame = 'hca_front_%d_vision_table_zero'%out.id
        x_hca_front = self.tf2x(parent_frame = r.Base_link_name, child_frame = child_frame)
        R_hca_front = q2r(x_hca_front[3:])
        
        ang = 180 * np.arccos(R_hca_front[0,0]) / np.pi
        rospy.loginfo("Angle HCA X to robot X : {}".format(ang))
        
        good_pos_for_flip = 0
        
        d = 40
        if (-d < ang< d) or (180-d  < ang < 180 + d):
            rospy.loginfo("Good position for flipping")
            good_pos_for_flip = 1
        else:
            rospy.loginfo("Bad pos for flip. Hca must first be rotated by N degs")
            
        R_hca_front = R_hca_front@rot_x(180, unit='deg')
        x_hca_front[3:] = r2q(R_hca_front)
        
        if not good_pos_for_flip:
            # Go with the gripper above this. then rotate it by "ang". Then the X or the HCA will be aligned
            T_hca = x2t(x_hca_front)@rel_tool_move
            T_tmp = copy.deepcopy(T_hca)

            T_tmp[2, -1] += 0.05
            
            T_hca[2,-1] +=0.02
            
            # Move above
            r.CMove(T_tmp, t=4)
            
            r.CMove(T_hca, t = 1.5)
            
            r.gripper.close(0.9, sleep=True)

            # Rotate the previous matrix by ANG
            R_hca_front = T_hca[0:3, 0:3]@rot_z(ang, unit ='deg')
            T_hca[0:3,0:3] = R_hca_front
            r.SetCartesianStiff_helper(m=1,n=0.8)
            r.CMove(T_hca, t = 3)
            r.gripper.open(sleep=True)
            r.SetCartesianStiff_helper(m=1,n=0.55)
            
            #T_hca[2, -1] += 0.05
            #r.CMove(T_hca, t =1.5)
        else:
            T_hca = x2t(x_hca_front)
            
        # Now we must check that the X is pointing same dir as robot x
        if R_hca_front[0,0] < 0 :
            R_hca_front = R_hca_front@rot_z(180, unit = 'deg')
            T_hca[0:3, 0:3] = R_hca_front
            #rel_tool_move[0:3,-1] = -2*rel_tool_move[0:3,-1]
            
        if good_pos_for_flip:
            T_hca = T_hca@rel_tool_move
        # Now the HCA front is rotated so the robot won't (hopefully) crash when flipping it.
        R_hca = T_hca[0:3,0:3]
        R_hca = R_hca@rot_y(tool_rot_y, unit='deg')@rot_z(tool_rot_z, unit='deg')
        T_hca[0:3, 0:3] = R_hca
        
        T_above = copy.deepcopy(T_hca)
        T_above[0:3, 0:3] = R_hca
        T_above[2, -1] +=0.03
        r.CMove(T_above, t=4)
        
        r.SetCartesianStiff_helper(m=1.1,n=0.65)
        
        T_hca[2,-1] += 0.000
        r.CMove(T_hca, t=3)
        r.gripper.close(1, sleep=True)
        r.CMoveFor(dx=[0,0,0.06], t=4)

        R_hca = T_hca[0:3, 0:3]@rot_z(+150, unit='deg')
        T_hca[0:3,-1] = r.x[0:3]
        T_hca[0:3, 0:3] = R_hca
        r.CMove(T_hca,t=5)
        
        r.gripper.open(sleep=True)
        r.CMoveFor(dx =[0,0,0.1], t = 4)
            
        rospy.loginfo("{}".format(x_hca_front))
        return 0 
        
    def robot_pick_up_object(self, robot, obj_tf_name, t = 10, safe_init = True, move_above_z = 0.05, using_controller = 'CartesianImpedance'):
        """Generic function to pick up an object with whichever gripper. DOES NOT WORK YET - well it just about works.""" 
        
        r = robot
        
        assert r.gripper.Name in ['softhand', 'vsgripper']

        
        if safe_init==True:self.robot_go_to_init_and_open_gripper(r)
        
        # Fix for when we are using more than 2 controllers.
        if r._control_strategy != using_controller:
            r.Switch_controller(start_controller =  using_controller)
            
        # Set low cart stiffness for moves
        r.SetCartesianStiff_helper(m=0.9, n=0.75)
        
        # Get tf from robot to object
        robot_link0_tfname = r.Base_link_name#r.Name + '/' + r.Name+ '_link0'
        
        success = 0
        while success==0:
            try:
                x_pickup = self.tf2x(parent_frame = robot_link0_tfname, child_frame = obj_tf_name)
                success=1
            except KeyboardInterrupt:
                return 0
            except:
                time.sleep(0.1)
                pass
        # Handle rotations ! 
        # Worst case scenario is that Z is pointing up, so we should rotate around X
        Tx = x2t(x_pickup)
        Rx = Tx[0:3, 0:3]@rot_x(180, unit ='deg')
        Tx[0:3, 0:3] = Rx
        x_pickup = t2x(Tx)
        
        #rospy.loginfo(x_pickup)
        #return 0
        
        if r.gripper.Name == 'softhand':
            # We have to rotate the hand by at least 30 degs for a grip.
            0
        elif r.gripper.Name == 'vsgripper':
            0
        
        x_above_pickup = copy.deepcopy(x_pickup)
        x_above_pickup[2] +=move_above_z
        
        #t_move = r.estimate_time()
        r.CMove(x = x_above_pickup, t = 6)
        r.gripper.open(sleep=False, wait_for_result=False)
        #do_concurrently([[r.CMove, {'x': x_above_pickup, 't': 6}],[r.gripper.open, {}]], wait_until_task_completion = True)
        
        r.CMove(x=x_pickup, t = 2)
        r.gripper.close(sleep=True) # Sleep=True means it will sleep until gripper is open(0.6 seconds)
        r.CMove(x = x_above_pickup, t = 2)
        
        return 0
    
    def get_next_action(self):
        """ Get next action from the action-predictor."""
        
        out = self.get_next_action_st.on_enter(self.ud)
        action_type = self.ud.action_type_prev
        action_details = self.ud.action_details_prev
        action_block = self.ud.action_block_prev
        a = Action(self.ud.action_type_prev).name
        
        return action_type, action_details, action_block, a
                    
    def perform_cycle_using_action_predictor(self):
        """ This will actually call the action predictor to get the next action, then do it """
        
        # Open grippers and prepare pneumatics
        do_concurrently([[self.prepare_vision, {'activate_basler':True, 'activate_realsense':False}] ,[self.prepare_pneumatics, {}]], wait_until_task_completion = True)
        do_concurrently([[self.robot_go_to_init_and_open_gripper, {'robot_obj':self.p1}], [self.robot_go_to_init_and_open_gripper, {'robot_obj':self.p2}]], wait_until_task_completion=True)
        
        a = 'start'
        # Run loop until we get end from the action predictor.
        while (a != 'end'):
            action_type, action_details, action_block, a = self.get_next_action()
            
            rospy.loginfo("Disas.Cy. exec ACTION: {}".format(a))
            rospy.loginfo("{}".format(action_block))
            return 0
            
            if a == 'none':0
            elif a == 'start':0
            elif a == 'end':0
            elif a == 'cut':0
            elif a == 'lever':
                self.p2_perform_levering()
            elif a == 'move':0
            elif a == 'push':
                # Perform pin pushing
                0
                self.p2_perform_pinpush()
            elif a == 'turn_over':
                # Grab the HCA or whatever with softhand/softclaw and turn it over
                0 
            elif a == 'vision':
                if action_block == '':
                    # Empty action block. activate the vision so it can predict something.
                    self.prepare_vision(activate_basler = True, activate_realsense = False)
                else:
                    camera_name = Camera(action_block.camera).name
                    if camera_name == 'basler':
                        # Get vision info from camera
                        0 
                   
                    elif camera_name == 'realsense':
                        # Get the robot to realsense position above cutter
                        self.prepare_vision(activate_basler = False, activate_realsense = True)
                  
            time.sleep(5) # For safety :)
            
    def perform_cycle(self, force_pinpush = True, allow_pinpush= False, double_tap_vise = False, rotate_vise = False, do_gap_detect = False, adaptive_levering = False, activate_cutter = False):
        """Perform the disassembly cycle with a fixed order of operations."""
        # Params : 

        # Open grippers and prepare pneumatics
        do_concurrently([[self.robot_go_to_init_and_open_gripper, {'robot_obj': self.p1}],
                        [self.robot_go_to_init_and_open_gripper, {'robot_obj': self.p2}],
                        [self.prepare_vision, {'activate_basler':False, 'activate_realsense':False}],
                        [self.prepare_pneumatics, {}]], wait_until_task_completion=True)

        task_start_time =time.time()
        ### PICKING UP THE HCA - PANDA 1
        # Get HCA type from vision !# Test moveblock to put for example realsense frame to some position
        mbl_preposition_p2 = MoveBlock(from_module = None, from_tf = 'panda_2/panda_EE', to_module = None, to_tf = 'vise/via1', obb_3d = None,
                                        robot = self.p2, end_effector = None, tfreader = self.tf2x)
        
        do_concurrently([[mbl_preposition_p2.on_enter, {}]], wait_until_task_completion = False)
        
        hca_type, has_pin = self.p1_pickup_hca_and_put_into_vice(init_safe = False, double_tap_vise = double_tap_vise, return_to_via = True)
        rospy.loginfo("Hca type: {}, has pin: {}".format(hca_type, has_pin)) 
        
        #if (force_pinpush) or (has_pin and allow_pinpush):
        #    self.p2_perform_pinpush(hca_type = hca_type, rotate_vise = rotate_vise, init_safe = False)
        
        #############################################  Performing gap detection with p2
        if do_gap_detect:
            self.p2_gap_detection(init_safe = False, wait = True, return_to_init = False)
        
        if (force_pinpush) or (has_pin and allow_pinpush):
            self.p2_perform_pinpush(hca_type = hca_type, rotate_vise = rotate_vise, init_safe = False)
            
        #############################################  Performing levering with 
        self.p2_perform_levering(hca_type=hca_type, direction_deg = self.d_p['levering_direction_deg'], adaptive = adaptive_levering)

        ############################################ Handling the PCB with p2 
        self.p2_handle_pcb_to_cutter(hca_type = hca_type, init_safe = False, activate_cutter = activate_cutter)
        
        ############################################ Handling the HCA frame with p1
        
        # Open the vice and handle HCA frame to bin (either p1 or p2)

        do_concurrently([[self.p2_pick_up_battery, {'get_into_camera_position' : True, 'safe_to_camera' : False, 'pick_up_battery' : True}],
                         [self.p1_handle_hca_frame_to_bin, {}]], wait_until_task_completion=True)
        
        task_end_time = time.time()
        task_duration = task_end_time - task_start_time
        rospy.loginfo("Disassembly took {} s".format(task_duration))
        return task_duration
    
    def p2_perform_levering(self, hca_type = None, direction_deg = None, adaptive = False):
        """ 
        TODO - fix direction_deg to not be hardcoded.
                 - Get start_cart_pos from input arguments."""
        assert hca_type in ['kalo', 'qundis']
        rospy.loginfo("START OF LEVERING")
        
        robot = self.p2
        
        ### PARAMETERS
        init_world_raise_after_levering = 0.01 # After levering raise a bit in world space to increase success change
        tool_retract_after_levering = -0.08 # Then retract out by this much in tool space
        end_world_raise_after_levering = 0.05 # At the end raise in world coords by this much to clear the HCA frame and vise.
        ### END PARAMETERS
        
        if robot._control_strategy != 'CartesianImpedance':
            robot.GetState()
            robot.Switch_controller(start_controller =  'CartesianImpedance')
            time.sleep(0.1)
            robot.ResetCurrentTarget()
            robot.SetCartesianStiff_helper(m=1.5, n=0.75)
            robot.SetCartImpContNullspace(q = robot.q, k=[0,0,0,0,0,0,0])
            
        #if robot._control_strategy != 'JointPositionTrajectory':
        #    robot.Switch_controller(start_controller =  'JointPositionTrajectory')

        robot.error_recovery()        

        #robot.SetCartesianStiff_helper(m=1.2, n=0.75)
        
        t = time.time()
        leverblock = self.init_leverblock(robot = robot, hca_type=hca_type, direction_deg = direction_deg, speed_factor = self.sf)
        #rospy.loginfo("Leverblock init took {} s".format(time.time()-t))
        
        leverblock.IF_go_to_init = 1
        leverblock.IF_perform_Zminus_search = 1
        leverblock.IF_perform_Xplus_search = 1
        leverblock.IF_perform_fulcrum_detect_rotation = 1
        leverblock.IF_perform_PDMP = 1
        
        # Open the sidejaws so less force is needed
        # Assert mainjaws are closed - mainjaws close when value is False...
        
        self.ud.value = False
        self.mainjaws_st.on_enter(self.ud); time.sleep(1)
        self.sidejaws_st.on_enter(self.ud); time.sleep(0.2)

        robot.gripper.open(sleep=False, wait_for_result=False)
                
        # Actually perform the levering
        #robot.SetCollisionBehavior(F=20, T = 3, tq = 3)
        robot.reset_recorded_states()

        robot.SetCaptureCallback(robot.record_states)
        leverblock.on_enter(hca_type = hca_type, adaptive=adaptive)
        robot.SetCaptureCallback(None)
        robot.StopCapture()

        # After levering, we expect the PCB to be out and within the gripper.
        # DO a small cmovefor in tool space to grip the HCA better
        #robot.CMoveFor(dx=[0,0,0.01], t=1/self.sf, task_space = 'Tool')
        
        # Close softclaw and lift up the robot
        
        #if hca_type=='short': robot.CMoveFor(dx=[0,0,-0.01], t=1/self.sf, task_space = 'Tool')
        
        #if hca_type=='long': robot.CMoveFor(dx=[0,0,0.02], t=1/self.sf, task_space = 'Tool') # For the long HCA try to move in a bit to grip it better. Dont need this for short HCA.

        robot.gripper.close(1, sleep=True)
        
        robot.GetState();cur_x = robot.x
        robot.SetCartesianStiff_helper(m=1.0, n=0.75)
        
        robot.CMoveFor(dx=[0,0,init_world_raise_after_levering], t=1/self.sf, task_space='World')
        if hca_type == 'qundis': robot.CMoveFor(dx=[0,0,tool_retract_after_levering], t = 2/self.sf, task_space = 'Tool')   # Pull PCB out in tool space
        robot.CMoveFor(dx=[0,0,end_world_raise_after_levering], t = 1.5/self.sf, task_space = 'World') # Move up in world space.
        
        # Constant stiffness on the way out
        robot.SetCartesianStiff_helper(m=1, n=0.75)
        
        self.p2_leverblock = leverblock
        return 0

    def p1_pickup_hca_and_put_into_vice(self, init_safe = True, double_tap_vise = False, ignore_bad_rotation = False, stop_after_raising = False, return_to_via = False, hca_x= None):
        
        """p1_q1_init is hardcoded but that's standard start pos for the robot.
        FIX - p1_q_above_table is hardcoded
        Args:
        init_safe : Go to joint imp controller, error recover and move to p1_q1_init
        
        ignore_bad_rotation : If true, then the PLACE POSITION in vise will always be the same.
        hca_x: if not None, then we will ignore vision and just move there"""
        
        self.VU.clear_detections()
        
        robot = self.p1 # Robot generic object so we don't write p1 in the code
        pf = robot.Base_link_name # Parent frame.
        
        rospy.loginfo("START OF P1 HCA TO VISE")
        if hca_x is not None:
            assert hca_x.shape[0] == 7
        ###########################
        # Parameters
        # PICKUP HCA PARAMS
        additional_hand_rot = -25 # -35
        
        #Short HCA
        rel_move_shorthca = np.eye(4) # Relative move in relation to HCA TF
        rel_move_shorthca[0,-1] +=0.01#0.02
        rel_move_shorthca[1,-1] +=0.05 # 0.03
        #rel_move_shorthca[2,-1] -= 0.05

        # Long hca
        rel_move_longhca = np.eye(4) # Relative move in relation to HCA TF
        rel_move_longhca[0,-1] +=0.015
        rel_move_longhca[1,-1] +=0.04
        #rel_move_longhca[2,-1] -=0.02
        
        #rel_dropmove_longhca = np.eye(4)
        #rel_dropmove_longhca[1, -1] -= 0.01
        #rel_dropmove_longhca = [0,-0.05,0]
        
        # DROP HCA INTO VISE PARAMS 
        rel_dropmove_shorthca = np.array([0,0,0])
        rel_dropmove_longhca = np.array([0,0.0,0]) # This is relative to robot/robot_link0 frame.
        
        drop_rotation_x = -10 # Rotate the hand a bit when dropping

        # Initial positioning above HCA height ( when we get result from vision, we move ABOVE IT (defined here) and then down
        #init_height_above_hca = 0.1
        init_height_above_hca= 0.2
        drop_down_by_z = (init_height_above_hca - 0.1) + 0.075 # From this upper value we drop down by 0.75. Basically always go to the same height when picking up HCA

        ############################
        
        if init_safe: 
            self.robot_go_to_init_and_open_gripper(robot)
            
        # Assert we are in the correct controller   - USING JOINT IMPEDANCE CONTROLLER ! 
        """
        if robot._control_strategy != 'JointImpedance':  #FIXME: Miha spreminl na JI iz CI
            robot.Switch_controller(start_controller =  'JointImpedance')
            robot.Start_controller()
            
        assert robot._control_strategy == 'JointImpedance' 
        """
        do_concurrently([[self.prepare_vision, {'activate_basler':True, 'activate_realsense': None}]], wait_until_task_completion = False)
            
        if robot._control_strategy != 'JointPositionTrajectory': 
            robot.ResetCurrentTarget()
            robot.Switch_controller(start_controller =  'JointPositionTrajectory')
            
        #assert robot._control_strategy == 'CartesianImpedance' 
        
        robot.error_recovery()
        
        robot.SetCartesianStiff_helper(m=0.9, n=0.85)  # Keep this in case we start using cart imp controller later
                
        # Set the nullspace
        neutral_q = (robot.q_max + robot.q_min) / 2
        neutral_q[3] = -2.553869
        robot.SetCartImpContNullspace(q =neutral_q,  k=[0, 0, 0, 4, 0, 0, 0]) # Only enforce for last joint, for now.
        robot.SetCartesianStiff_helper(m = 1.1, n=0.65)
        
        
        # Drop position - get from tf
        x_drop_sh = self.tf2x(parent_frame = pf, child_frame = 'vise/hcaDropHand')
         # Mini fix for greater success. Add changes to TF later
        #x_drop_sh[0] -= 0.005
        x_drop_sh[0] -= 0.025
        x_drop_sh[1] += 0.015
        x_drop_sh[2] +=0.01
        # Via position - get from tf
        x_midpoint = self.tf2x(parent_frame = pf, child_frame = 'vise/via2')
        x_midpoint[2] +=0.1
        x_return_via = self.tf2x(parent_frame = pf, child_frame = 'vise/via3')
        x_return_via[0] -= 0.05 # Move it further out to better clear panda_2
        x_return_via[2] -=0.03 # Lower down a bit.

        do_concurrently([[handle_pneumatics_jaws, {'ud':self.ud, 'mainjaws_st': self.mainjaws_st, 'sidejaws_st':self.sidejaws_st, 'command': 'open'}]], wait_until_task_completion = False)
        time.sleep(0.5)
        
        if hca_x is not None:
            T_hca = x2t(hca_x)
            incorrect_rotation = False
            has_pin = False
            hca_type = 'kalo'
        else:
            T_hca, incorrect_rotation, has_pin, hca_type = self.VU.get_hca_pickup_with_singular_orientation(robot = robot, fn_tf2x = self.tf2x, max_wait_time_s = 5)
            
            if T_hca is None: 
                rospy.loginfo("There is no HCA on the table (or the vision system is not working)")
                return 0
            
            else:
                rospy.loginfo("p1_pickup_hca got vision result")
        # Turn off vision
        do_concurrently([[self.prepare_vision, {'activate_basler':False, 'activate_realsense': None}]], wait_until_task_completion = False)
                                 
        # If the HCA is rotated the wrong way, drop it from a different position (so the levering gap will always be on the same side of the vise)
        if (incorrect_rotation) and (ignore_bad_rotation == False):
            x_drop_sh = self.tf2x(parent_frame = pf, child_frame = 'vise/hcaDropHand2') # the 2 at the end of hcaDropHand2 - alternative drop position.
            x_drop_sh[2] -= 0.05
            # The via position must also be rotated
            R_midpoint = q2r(x_midpoint[3:])@rot_z(-90, unit='deg')
            x_midpoint[3:] = r2q(R_midpoint)
            
        #rospy.loginfo("p1_hca_pickup: Incorrect rotation: {}".format(incorrect_rotation))
        #rospy.loginfo("p1_hca_pickup: Hca type: {}".format(hca_type))

        #self.sendTf(p = T_hca[0:3, -1], q = r2q(T_hca[0:3,0:3]), parent_frame = robot.Base_link_name, child_frame = 'TEST1')
        
        rx = T_hca[0:3, -1] 
        
        # Rotate the position by degs specified
        hca_R = T_hca[0:3, 0:3]@rot_z(-additional_hand_rot, unit = 'deg')
                        
        x_all = np.concatenate((rx,r2q(hca_R)))
        
        ### HANDLING OF DIFFERENT HCA TYPES (Pickup and drop positions
        if hca_type == 'kalo':rel_move = rel_move_longhca
        else: rel_move = rel_move_shorthca
        
        x_moved = x2t(x_all)@rel_move
        x_all = t2x(x_moved)
        x_all[2] +=init_height_above_hca # Move above pickup position

        if hca_type == 'kalo':
            x_drop_sh[0:3] += rel_dropmove_longhca
        elif hca_type == 'qundis':
            x_drop_sh[0:3] +=rel_dropmove_shorthca
        ### END
        
        above_x_drop_sh = copy.deepcopy(x_drop_sh)
        above_x_drop_sh[2] +=0.03
        
        # Change the drop position by rotating the hand a bit
        d_R = rot_x(drop_rotation_x, unit='deg')
        T_drop = x2t(x_drop_sh)
        T_drop[0:3, 0:3] = T_drop[0:3, 0:3]@d_R
        x_drop_sh = t2x(T_drop)
        
        x_drop_sh[1] += 0.005
        x_drop_sh[2] -= 0.01
        
        robot.ResetCurrentTarget()
        
        do_concurrently([[move_slider, {'ud': self.ud, 'slider_st': self.slider_st, 'position': 'back'}] ], wait_until_task_completion = False)
        do_concurrently([[robot.CMove, {'x' : x_all, 't' : 5 / self.sf, 'v_max_factor': 0.4, 'a_max_factor':0.4}]], wait_until_task_completion = True)

        #return 0
        x_all[2] -= drop_down_by_z
        robot.CMove(x = x_all, t = 3.3 / self.sf, task_space ='World')
        
        # Pick up HCA - close softhand
        robot.gripper.close(0.95, sleep=True)
        #robot.ResetCurrentTarget()

        # X_all is init place
        robot.GetState();cur_x = copy.deepcopy(robot._command_int.x)
        path = np.stack((cur_x, x_midpoint, above_x_drop_sh, x_drop_sh)) 
        

        #tpath = (3 + 2.5 + 3) / self.sf
        tpath = 7 / self.sf
        tpath = robot.tsamp * round( tpath / robot.tsamp )         # Due to code in CPath, tpath must be divisible by tsamp.
        
        rospy.loginfo("Tpath: {}".format(tpath))
        if self.d_p['use_cmove_instead_of_cpath']:
            robot.CMove(x_midpoint, t=4/self.sf, task_space='World')
            # Move above drop position
            robot.CMove(x = above_x_drop_sh, t = 2.5/self.sf)
            # Move to drop position
            robot.CMove(x = x_drop_sh, t = 1.5/self.sf)
        else:
            robot.CPath_new(path=path, t=tpath)
        
        # Release the HCA into the holder
        robot.gripper.open(0.7, sleep=True)
        # Move p1 to between position and then to init
        robot.CMoveFor(dx=[0,0,0.05], t= 1.5/self.sf,task_space = 'World')
        
        if double_tap_vise:
            rospy.loginfo("Double tapping vise")
            # Close and open the vise (twice) for better chance of grasp.
            handle_pneumatics_jaws(ud =self.ud, mainjaws_st= self.mainjaws_st, sidejaws_st = self.sidejaws_st, command=  'close')
            time.sleep(1.5)
            handle_pneumatics_jaws(ud =self.ud, mainjaws_st= self.mainjaws_st, sidejaws_st = self.sidejaws_st, command=  'open')
            time.sleep(1.5)
            
        # Reset nullspace 
        robot.SetCartImpContNullspace(q =neutral_q,  k=[0, 0, 0, 0, 0, 0, 0]) # Only enforce for last joint, for now.
        
        if stop_after_raising:
            return hca_type, has_pin
        
        if return_to_via ==False:
            # A safe return to init position
            if robot._control_strategy != 'JointPositionTrajectory':
                robot.ResetCurrentTarget()
                robot.Switch_controller(start_controller =  'JointPositionTrajectory')
            do_concurrently([[handle_pneumatics_jaws, {'ud':self.ud, 'mainjaws_st': self.mainjaws_st, 'sidejaws_st': self.sidejaws_st, 'command': 'close'}],[move_slider, {'ud': self.ud, 'slider_st': self.slider_st, 'position' : 'back'}],[robot.JMove, {'q': p1_q1_init, 't': 4/self.sf}]], wait_until_task_completion = True)
        else:
            # Fast return to via close to vise, for later gripping the HCA frame
            do_concurrently([[handle_pneumatics_jaws, {'ud':self.ud, 'mainjaws_st': self.mainjaws_st, 'sidejaws_st': self.sidejaws_st, 'command': 'close'}],
                             [move_slider, {'ud': self.ud, 'slider_st': self.slider_st, 'position' : 'back'}],
                             [robot.CMove, {'x': x_return_via, 't': 4.5/self.sf}],
                             [robot.gripper.close, {'command':0.5}]], wait_until_task_completion = False)
        
        return hca_type, has_pin
    
    def p1_handle_hca_frame_to_bin(self, return_to_init = True, drop_back_on_table = False):
        """ UNUSED ! Using Panda 1 , move the softhand above the vice, then down and grip the HCA frame that is still inside. Then drop it into the hardcoded bin.
            TODO - USE relative TFs instead of joint position."""
        
        rospy.loginfo("START OF P1 HCA FRAME TO BIN")
        
        ### PARAMETERS
        robot= self.p1
        
        y_rotation_inside_vise_grip = -11
        y_move_to_get_out_of_contact = 0.03
        ### END PARAMETERS
        robot.gripper.open(sleep=True)
        robot.ResetCurrentTarget()
        # Force using the same controller :
        if robot._control_strategy != 'CartesianImpedance':
            robot.Switch_controller(start_controller =  'CartesianImpedance')
        
        robot.error_recovery() # force = True
        robot.SetCartesianStiff_helper(m=1.2, n=0.9)

        # Concurrently : - Ensure the vice is open,    - Open softhand, - Move P1 to position above vice
        #do_concurrently([[self.prepare_pneumatics, {}], [robot.gripper.open, {'command': 1}], [robot.JMove, {'q': p1_q_above_vice_grip_hca, 't': 5/self.sf, 'qdot_max_factor':0.2,'qddot_max_factor':0.2}] ], wait_until_task_completion = True)
        
        self.prepare_pneumatics()
        
        robot.gripper.open(0.5, sleep=True)
        robot.JMove(q = p1_q_above_vice_grip_hca, t = 5/self.sf, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
        
        # Move slightly more down and inside the vise. Half-close the softhand.
        #do_concurrently([[robot.CMoveFor, {'dx': [-0.01, -0.02, 0], 't': 1.5/self.sf, 'task_space': 'World'}],
        #                 [robot.gripper.close, {'command':0.55}]],
        #                 wait_until_task_completion= True)
        robot.CMoveFor(dx =  [-0.01, -0.02, 0], t = 1.5/self.sf, task_space = 'World')

        # Get into grip position within vise.
        robot.CMoveFor(dx= [0, 0.0, -0.06], t = 2/self.sf, task_space = 'World')
        
        # Rotate slightly to better grasp the HCA frame with the THUMB.
        r = rot_y(y_rotation_inside_vise_grip, unit ='deg', out='R')
        robot.CMoveFor(r, t=1.5/self.sf)
        #return
        # Close hand and grip eet.
        robot.gripper.close(sleep=True);time.sleep(0.5)
        
        # Move slightly back to avoid the upper gripper part which is curved    ##### FIX TO MAKE RELATIVE !
        robot.CMoveFor(dx= [0, y_move_to_get_out_of_contact, 0], t = 1.2/self.sf, task_space = 'World')
        
        # Move back above
        robot.JMove(p1_q_above_vice_grip_hca, t = 3/self.sf)
        robot.CMoveFor(dx = [0, 0, 0.05], t=1.5/self.sf, task_space = 'World')
        
        if drop_back_on_table:
            0
            
        else:
            # Move above bin
            r1 = rot_z(-40, unit='deg', out = 'R')
            r2 = rot_x(10, unit = 'deg', out = 'R')
            r3 = np.matmul(r2,r1)
            r4 = rot_y(-20, unit ='deg', out='R')
            r = np.matmul(r3, r4)
            
            robot.CMoveFor(dx = r, t=2.5/self.sf)
            robot.CMoveFor(dx = [0.18, 0, -0.02], t=3/self.sf, task_space = 'World')
            
            # Open softhand
            robot.gripper.open(0.8, sleep=True);time.sleep(0.2)
            # Move Back 
            robot.CMoveFor(dx=[-0.18,0, 0.02], t=3/self.sf)
        
        if return_to_init:
            self.robot_go_to_init_and_open_gripper(robot)

        return 0 

    def p2_handle_hca_frame_to_bin(self, init_safe = False, rotate_hca_inside_vise = False):
        """Handling the HCA frame with p2    WITH PANDA 2 !! and the QBSOFTCLAW.
        Can also be used to rotate the HCA after it is inside the vise.
        
        Args:
        init_safe : Move to a known init pose in joint space (ensure always the same init robot position)
        rotate_hca_inside_vise: after picking up the hca it is rotated 180 degs and put back into the vise"""
        
        rospy.loginfo("START OF P2 HCA FRAME TO BIN")

        ### PARAMETERS
        robot = self.p2
        
        ### END PARAMS
        
        
        # Get this tf from panda2_link0 to pickup position on vise
        x_pickup = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame='vise/hcaPickupClaw')

        x_drop = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame='vise/hcaDropToBin')
        x_above_pickup = copy.deepcopy(x_pickup)
        x_above_pickup[2] += 0.1
        
        #self.ud.goal_claw_pos = [-0.9]; self.softclaw_st.on_enter(self.ud)
        
        # Open vise 
        handle_pneumatics_jaws(ud=self.ud,mainjaws_st= self.mainjaws_st, sidejaws_st = self.sidejaws_st, command='open')
        
        if init_safe:
            self.robot_go_to_init_and_open_gripper(robot)
        else:
            # Open softclaw
            robot.gripper.open(sleep=True)
    
        if robot._control_strategy != 'JointPositionTrajectory':
            robot.ResetCurrentTarget()
            robot.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        robot.error_recovery()
        robot.SetCartesianStiff_helper(m=0.8, n=0.55)
        robot.CMoveFor(dx=[0,0,0], t=0.02)
        
        # Move 10 cm above this x_init
        robot.CMove(x_above_pickup, t = 3/self.sf)
        
        robot.SetCartesianStiff_helper(m=1, n=0.75)

        # Move to pickup
        x_pickup[2]+=0.005
        robot.CMove(x_pickup, t = 3/self.sf)

        # Grasp with softclaw
        robot.gripper.close(1, sleep=True)
        #self.ud.goal_claw_pos = [0]
        #self.softclaw_st.on_enter(self.ud)
        #time.sleep(0.5)
          
        # move slightly out of the vise to avoid the upper sliding gripper.
        robot.CMove([0.008,0,0],t = 0.5/self.sf,task_space='Tool')

        robot.CMove(x_above_pickup, t = 2/self.sf)
        
        if rotate_hca_inside_vise == True:
            # Get the drop transform
            x_drop_above = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame = 'vise/softclaw_rotated')
            
            x_drop = copy.deepcopy(x_drop_above)
            x_drop[2] -= 0.03
            
            robot.CMove(x_drop_above, t = 2.3/self.sf)
            robot.CMove(x_drop, t = 1/self.sf)
            robot.gripper.open(sleep=True)
            robot.CMove(x_drop_above, t=1/self.sf)
            
        else:
            # Move to drop position
            x_drop[2] +=0.03
            robot.CMove(x = x_drop[0:3], t = 2/self.sf)

            # Open softclaw
            robot.gripper.open(sleep=True)
        return 0
    
    def p2_move_to_realsense_cutter_pos(self, init_safe = True):
        """ Move the p2 to the pre-recorded realsense position from which it can see the battery inside the cutter plate.
        Args:
        init_safe : if True, robot will move to p2_q1_init (standard initial robot pose with almost zero joint rotations). Slower but safer and more repeatable.
        
        TODO - remove JMove to p2_q_above_cutter"""
        rospy.loginfo("START OF P2 MOVE TO REALSENSE POS")
        robot = self.p2
        
        #p2_q_above_cutter_plate = (-1.239688, 0.374359, -0.167726, -1.697185, 0.076327, 2.081039, -2.180895) # Jmove assures the same robot config all the time
        #p2_q_above_cutter_plate = (-1.783957, 0.394097, 0.459416, -1.711819, -0.197851, 2.090388, -2.028577)
        
        #p2_q_above_cutter_plate = (0.219559, 0.730101, -1.802353, -2.011666, 0.720533, 1.785227, 0.565119) # The alternative position where the camera is further away from the robot.
        #p2_q_above_cutter_plate = (0.169603, 0.764529, -1.711025, -2.202251, 0.797649, 1.929631, 0.412667)
        #p2_q_above_cutter_plate = (0.158586, 0.804870, -1.706036, -2.198525, 0.828139, 1.913337, 0.396890)
        
        #p2_q_above_cutter_plate = (0.021589, 0.853099, -1.651550, -2.074406, 0.901931, 1.845068, 0.406469)
        
        #  p2_q_above_cutter_plate = (-0.942758, 0.114922, -0.348879, -1.975905, -0.017976, 2.109908, 1.086374)
        #p2_q_above_cutter_plate =(-0.922987, -0.000271, -0.337441, -2.116944, -0.065901, 2.132774, 1.139987)
        p2_q_above_cutter_plate = (-0.991934, 0.044061, -0.399815, -2.146095, 0.028281, 2.184489, 0.943811)
        
        x_1 = self.tf2x(parent_frame = robot.Name + '/' + robot.Name + '_link0',
                        child_frame = 'cutter/realsense2')
        #
        x_2 = self.tf2x(parent_frame = robot.Name + '/' +  'panda_EE',
                            child_frame = 'panda_2/realsense')
        # 
        T_end = x2t(x_1)@np.linalg.inv(x2t(x_2))
        x_above_cutter_plate = t2x(T_end)
                
        # Go to joint controller and get to camera position, to make sure to work in correct joint configuration
        #if robot._control_strategy!="JointImpedance":
        #    robot.Switch_controller(start_controller =  'JointImpedance')
        robot.ResetCurrentTarget()

        if robot._control_strategy!="JointPositionTrajectory":
            robot.Switch_controller(start_controller =  'JointPositionTrajectory')
        robot.error_recovery()
        
        if init_safe:
            robot.ResetCurrentTarget()
            cur_q = robot.q
            dq = abs(robot.JointDistance(p2_q1_init))

            min_t = 8 * dq / robot.qdot_max
            min_t = np.max(min_t)
            t_move = np.max((min_t, 5/self.sf))
            
            robot.JMove(p2_q1_init,t_move)
        
        # THis JMove should later be removed and the TIME of the lower CMove should be increased. However currently there's a bit rotational error. DOnt wanna increase rot stiff too much due to shaking.
        robot.ResetCurrentTarget()
        robot.JMove(p2_q_above_cutter_plate,t = 5 /self.sf, max_vel = 0.7, max_acc = 0.7)
        
        # Switch to cart controller
        #robot.Switch_controller(start_controller =  'CartesianImpedance')
        #p2.Stiff()
        #robot.SetCartesianStiff_helper(m=1,n=0.75)

        # Get robot to position above cutter
        #robot.CMove(x_above_cutter_plate,1.5/self.sf)
        
        #robot.SetCartesianStiff_helper(m=1,n=1.2)
        #robot.CMove(x_above_cutter_plate[3:], t= 0.4/self.sf)

        print("\x1b[1;34m","GOT INTO CAMERA POSITION","\x1b[0m")
        
        #robot.SetCartesianStiff_helper(m=1.1,n=0.95)

        return 0
    
    def p2_handle_pcb_to_cutter(self,hca_type = 'qundis', init_safe = False, activate_cutter=False, move_to_realsense_pos = True):
        """ USING p2 ! .Function to move PCB from LEVERING END POSITION into the CUTTER. This means that the PCB
        must already be inside the SoftClaw.

        TODO - move to positions described relatively to cutter"""
        rospy.loginfo("Starting p2_handle_pcb_to_cutter")
        
        assert hca_type in ['kalo', 'qundis']
        
        # Parameters :
        robot = self.p2
        
        ### PARAMS
        
        rot_before_dropping_pcb_into_cutter = -4
         
        rot_after_dropping_pcb_into_cutter = -5 # deg  # When we pull out we rot the gripper a bit
        
        rot_x_after_moving_out_of_cutter = -45
        rot_z_after_moving_out_of_cutter = 45
        p_diff_after_moving_out_of_cutter = [0,0,-0.05]
        
        ### END PARAMS
        
        # Positions

        start_x = [ 0.665403, -0.107808,  0.255504,  0.214467, -0.976269,  0.004607, -0.029681] # After levering
        #midpoint_x = [ 0.392427, -0.487466,  0.305212,  0.16012 ,  0.910683, -0.340604, -0.170316]
        midpoint_x = [ 0.45899 , -0.476439,  0.281459,  0.068311,  0.93677 ,  0.243386, -0.241988]
        
        q_mid_cutter = (-0.714028, 0.754701, -0.528818, -1.087188, -0.267694, 1.280913, 1.268164) # Position after moving out of cutter

        x_next_to_cutter = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame=  'cutter/place_sc')
        
        if hca_type == 'kalo':
            # For the long HCA, we position it a bit differently so both the batt and capacitor are cut out at once.
            rospy.loginfo("PCB2Cutter changing drop position by 6cm")
            x_next_to_cutter[1] += 0.06
        
        x_next_to_cutter[1] -=0.002 # A bit towards the center of the cutter
        #x_next_to_cutter[2] -=0.005 # Lower up a bit in the Z direction
        x_next_to_cutter[2] -=0.005 # Lower up a bit in the Z direction
        
        TX = x2t(x_next_to_cutter) 
        RX = TX[0:3,0:3]@rot_x(rot_before_dropping_pcb_into_cutter, out = 'R', unit='deg') # Add some tool rotation so the PCB slides down the "finger"
        TX[0:3, 0:3] = RX
        x_next_to_cutter = t2x(TX)
        
        if self.p2_leverblock is None:
            leverblock = self.init_leverblock(robot= robot) # we need the move_until_contact function.
        else:
            leverblock = self.p2_leverblock
        
        # Close the softclaw
        robot.gripper.close(1, sleep = False)
        
        # Force using CartesianImpedance controller
        #if not(robot._control_strategy=='CartesianImpedance'):
        #    robot.Switch_controller(start_controller =  'CartesianImpedance')
        #    print("Switched the controller to CartesianImpedance")
        if robot._control_strategy != 'JointPositionTrajectory':
            robot.ResetCurrentTarget();time.sleep(0.01)
            robot.Switch_controller(start_controller =  'JointPositionTrajectory')
            print("Switched the controller to JointPositionTrajectory")
            
        robot.error_recovery()
        #robot.SetCartesianStiff_helper(m=1,n=0.65)
        
        #robot.ResetCurrentTarget()
        #robot.SetCartesianStiff_helper(m=1,n=0.75)
    
        #if init_safe:
        #robot.ResetCurrentTarget();time.sleep(0.01);robot.GetState()
        robot.CMove(x=start_x, t= 4.5/self.sf)
    
        # Move to midpoint_x
        # Move next to cutter.
        # Move into the cutter

        #robot.SetCartesianStiff_helper(m=1,n=0.55)        
        
        time.sleep(0.2)
        robot.GetState()
        #robot.ResetCurrentTarget()
        cur_x = copy.deepcopy(robot._command_int.x)
        # np.array(cur_x)+[0,0,0.02,0,0,0,0],

        path = np.stack((cur_x,
                         midpoint_x,
                         x_next_to_cutter,
                         np.array(x_next_to_cutter)+[-0.06,+0.03,0.0,0,0,0,0],
                         np.array(x_next_to_cutter)+[-0.09,+0.02,0.0,0,0,0,0],
                         np.array(x_next_to_cutter)+[-0.11,0,0.0,0,0,0,0]))
        
        tmove =  (2.5 + 2.5 + 2.5) / self.sf
        tmove = robot.tsamp * round( tmove / robot.tsamp )         # Due to code in CPath, tmove must be divisible by tsamp.
        rospy.loginfo("T: {}".format(tmove))
        
        if self.d_p['use_cmove_instead_of_cpath']:
            robot.CMove(x=midpoint_x, t= 3.5/self.sf)
            robot.CMove(x_next_to_cutter,3.5/self.sf)
            robot.CMove(np.array(x_next_to_cutter)+[-0.06,+0.03,0.0,0,0,0,0], t = 2.5 /self.sf)
            #robot.CMove(np.array(x_next_to_cutter)+[-0.09,+0.02,0.0,0,0,0,0], t = 1.5 /self.sf)
            robot.CMove(np.array(x_next_to_cutter)+[-0.11, 0.0,0.0,0,0,0,0],  t = 1.5 /self.sf)
        else:
            robot.CPath_new(path = path, t = tmove)
            #out = robot.CPath(path = path, t = tmove)
        
        time.sleep(0.5)
        robot.ResetCurrentTarget()
        
        pos_err = np.abs(robot.x[0:3] - (np.array(x_next_to_cutter[0:3])+[-0.11,0,0]))
        pos_err_sum = np.sum(pos_err)
        rospy.loginfo("POS ERROR when placing into cutter: {}".format(pos_err))
        
        if pos_err_sum > 0.01:
            rospy.loginfo("Something was stuck in cutter!")
            robot.ResetCurrentTarget()
            robot.error_recovery()
            robot.CMoveFor([0.1, 0, 0], 2.5)
        else:
            #robot.CMove(midpoint_x, 2.5/self.sf)
            #robot.CMove(x_next_to_cutter, 4.5/self.sf)
            #robot.CMove(np.array(x_next_to_cutter)+[-0.11,0,0.0,0,0,0,0], 2.5/self.sf)

            #from robot_module_msgs.msg import CartesianCommand

            #msg = rospy.wait_for_message(robot.cart_command_topic,CartesianCommand, 1)
            #print(msg)
            robot.ResetCurrentTarget()
            robot.Switch_controller(start_controller =  'CartesianImpedance')
            time.sleep(0.1)
            robot.ResetCurrentTarget()
            #robot.SetCartesianStiff_helper(m=1,n=0.65, hold_pose='on')
            robot.SetCartImpContNullspace(q = robot.q, k=[0,0,0,0,0,0,0])
            print("Switched the controller to CartesianImpedance")
            robot.SetCartesianStiff_helper(m=0.8,n=0.55)
            # Check contact in depth
            robot.ResetCurrentTarget()
                    
            leverblock.lv.move_until_contact(mv_direction = [-0.0002, 0, 0.0], rot_direction= None,dt = robot.tsamp,
                                    axes_to_monitor = [0,0,1,0,0,0],
                                    max_allowed_values = [13,13,6,2,2,2], use_abs = True,
                                    max_allowed_t = 10)
            
            robot.CMoveFor(dx=[0,0,-0.002],t=0.6, task_space = 'Tool')

            # Check side contact
            #robot.ResetCurrentTarget()
            #leverblock.lv.move_until_contact(mv_direction = [0, -0.0002, 0], rot_direction= None,dt = 0.01,
            #                        axes_to_monitor = [1,0,0,0,0,0],
            #                        max_allowed_values = [5,4,6,2,2,2], use_abs = True,
            #                        max_allowed_t = 10)
            
            # Check down contact
            robot.ResetCurrentTarget()
            leverblock.lv.move_until_contact(mv_direction = [0.0, 0, -0.0001], rot_direction= None,dt = robot.tsamp,
                                    axes_to_monitor = [0,1,0,0,0,0],
                                    max_allowed_values = [5,4,6,2,2,2], use_abs = True,
                                    max_allowed_t = 10)
            
            robot.ResetCurrentTarget()
            
            # Open softclaw
            robot.gripper.open(0.93, sleep=True)
            
            # Move p2 back out of cutter
            robot.CMoveFor(rot_x(rot_after_dropping_pcb_into_cutter,out='R',unit='deg'),t = 1/self.sf,task_space='Tool')
            robot.CMoveFor([0.01,0,0.0],t = 1/self.sf)
            
            robot.CMoveFor([0.155,0,0.0],t= 2.5/self.sf)  # Retract out
            
            #rospy.loginfo("Robot q at retracted out: {}".format(robot.q))
        
            robot.ResetCurrentTarget();time.sleep(0.1)
            robot.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        robot.ResetCurrentTarget()
        robot.JMove(q_mid_cutter, t  = 4.5/self.sf)
        robot.ResetCurrentTarget()

        #robot.GetState(); T = robot.T
        #robot.SetCartesianStiff_helper(m=0.8,n=0.65)

        #T[:3,-1] += p_diff_after_moving_out_of_cutter
        #T[0:3,0:3] = T[0:3, 0:3]@rot_x(rot_x_after_moving_out_of_cutter, unit='deg')@rot_z(rot_z_after_moving_out_of_cutter, unit='deg')
        #robot.CMove(x=T, t = 4.5/self.sf)
        
        # Assert robot not in error mode.
        if activate_cutter:
            robot.GetState();
            # If we want to activate cutter, assert the error in robot position is less than 2 cm.
            #pos_error_combined = np.sum(np.abs(robot._actual_int.x[0:3] - robot._command_int.x[0:3]))
            pos_error_combined = np.sum(np.abs(np.array(robot._actual_int.q) - np.array(robot._command_int.q)))

            if pos_error_combined > 0.06:
                rospy.loginfo("Not activating cutter- pos. error: {}".format(pos_error_combined))
                assert pos_error_combined < 0.06 
        
        if (move_to_realsense_pos and activate_cutter):
            # Both move to realsense and activate cutter concurrently
            do_concurrently([[self.helper_cut, {'only_blow_air' : False}]], wait_until_task_completion = False) # we dont want to wait for the cutter at all since the robot will be slower all in all.
            do_concurrently([[self.p2_move_to_realsense_cutter_pos, {'init_safe':False}]], wait_until_task_completion = True)
            #do_concurrently([[self.p2_move_to_realsense_cutter_pos, {'init_safe':False}], [self.helper_cut, {}]], wait_until_task_completion = True) # This works but waits for cutter for too long.
        elif (not activate_cutter) and move_to_realsense_pos:
            # Only move to realsense
            self.p2_move_to_realsense_cutter_pos(init_safe = False)
        elif (activate_cutter) and (not move_to_realsense_pos) :
            # Only activate cutter
            self.helper_cut(only_blow_air = False)
        else:
            rospy.loginfo("Something strange happened in p2_handle_hca_frame_to_bin, CHECK !")
           
        return 0
    
    def helper_cut(self, only_blow_air = True):
        """ Move this to pneumatics utils later. A helper function for activating the cutter sequence"""
        if not only_blow_air:
            move_cutter(self.ud, self.cutter_st, position = 'down'); time.sleep(4)
            move_cutter(self.ud, self.cutter_st, position = 'up');   time.sleep(1.5)
        activate_cutter_airblower(self.ud, self.cutter_airblower_st, position = 'on'); time.sleep(1.5)
        activate_cutter_airblower(self.ud, self.cutter_airblower_st, position = 'off')
        return 0
    
    def get_battery_pickup(self):
        raise Exception("Unfinished code")
        
        robot = self.p2
        
        panda_2_frame = robot.Base_link_name
        
        ### PARAMETERS
        #gripper_battery_pickup_rotation = 15 # deg. WE dont go straight down but rotate the gripper around its X axis slightly. (Otherwise softclaw finger hits the ground plane)
        gripper_battery_pickup_rotation = 0 # deg. WE dont go straight down but rotate the gripper around its X axis slightly. (Otherwise softclaw finger hits the ground plane)
                
        pdiff=[0,0.02,-0.002] # Do not put the TCP directly at the center of battery but a bit to the side ( so battery is in the middle)

        safe_height_above_battery = 0.035 # Before pickup up the battery we move THIS MUCH above it.
        z_pickup_height = 0.005 # Height of battery
        
        min_n_of_same_detections = 1
        max_t_stable_wait = 6 # Max time to wait for stable detection in seconds

        max_allowed_positional_deviation_in_subsequent_detections = 0.005
        
        sleep_t = 4.5 # Sleep for making sure to get the latest detection

        bf, x_r2batt, issafe = self.VU_rs.get_battery(fn_tf2x = self.tf2x, robot_frame = robot.Base_link_name, timeout = 4)
        if x_r2batt is None:
            rospy.loginfo("Could not find battery within specified time")
            return 0
        
        
        rospy.loginfo("Picking up {}".format(bf))
        rospy.loginfo("Issafe status: {}".format(issafe))

                    
        #rospy.loginfo("PICK UP BATTERY GOT RESULT")
        # At first, Rotate around X because Z is by default pointing upwards, while tool Z is pointing downwards.
        a=q2r(x_r2batt[3:])@rot_x(180,unit='deg')
        
        # Handling of cases of different battery rotation and position
        robot.GetState()
        tool_rpy = r2rpy(q2r(robot.x[3:]), out = 'deg')
        
        do_additional_rotation = 0 # Check whether rotation needs to be fixed
        additional_tool_rot_z = 180 # How much we should change the rotation, IF do_additional_rotation==1. This will always be either 180 or -180.
        
        copy_issafe = copy.deepcopy(issafe)
        rospy.loginfo("copy issafe:{}".format(copy_issafe))
        # Center center
        if issafe==1:
            # Fix rotation
            if a[1,1] > 0:
                print("NAROBE-ROTACIJA-\n",a[1,1])
                do_additional_rotation = 1
                #a=a@rot_z(180,unit='deg')   
        
        # Left edge
        elif issafe ==2:
            # Left edge is unsafe for pickup.
            issafe = 0
            rospy.loginfo("Left edge is unsafe for battery pickup")
        
        # Top center
        elif issafe==3:
            # Replace < with > to fix it later  
            if a[1,1] < 0:
                print("NAROBE-ROTACIJA--\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        # Top right
        elif issafe==4:
            if (a[1,1] > 0) and (a[0,1] >0):
                0
            else:
                #if (a[1,1] < 0) or:
                print("NAROBE-ROTACIJA---!\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            rospy.loginfo("Corner is unsafe for battery pickup")
        
        # Center right
        elif issafe==5:
            if a[0,1] < 0:
                print("NAROBE-ROTACIJA---\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        # Bottom right
        elif issafe==6:
            if (a[1,1] < 0) and (a[0,1] > 0):
                0
            else:
            #rospy.loginfo("Got into issafe==6")
            #if a[1,1] < 0:
                print("NAROBE-ROTACIJA---!\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            rospy.loginfo("Corner is unsafe for battery pickup..")
        
        # Bottom center
        elif issafe==7:
            if a[1,1] > 0:
                print("NAROBE-ROTACIJA----\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        """
        robot.GetState()
        q1 = np.array(robot.q)
        q1[-1] = robot.q_home[-1]
        robot.JMove(q1,1)"""
        #########################################
        
        if do_additional_rotation:
            # Check if we should rotate in the opposite way (by -180 instead of 180)
            rospy.loginfo("Performing additional rotation")
            midpoint_a = a@rot_z(180, unit = 'deg') # force the robot to go into the correct direction

            a = a@rot_z(additional_tool_rot_z, unit = 'deg')
        else:
            midpoint_a = a
            rospy.loginfo("Not rotating further")
                
            
        q1 = r2q(a)
        q_midpoint = r2q(midpoint_a)     
        
        pos_new_midpoint = (robot.x[:3] + x_r2batt[:3])/2
        x_new_midpoint = np.concatenate([pos_new_midpoint,q_midpoint])
        x_new=np.concatenate([x_r2batt[:3],q1])
        
        T_new_midpoint = x2t(x_new_midpoint)
        T_new = x2t(x_new)
        
        # Relative move in tool Y so we don't hit the center of the battery. Relative tool rotation so the gripper part doesnt hit the ground.
        Tdiff =np.eye(4)
        Rdiff = rot_x(gripper_battery_pickup_rotation,unit='deg')
        Tdiff = rp2t(R=Rdiff,p=pdiff)
        T_new=T_new@Tdiff
        x_new=t2x(T_new)
        x_new[2] = 0.015236
        #x_new[2] += 0.005 # Raise 3 mm above position.
        
        # Safe position above battery
        x_above_new = copy.deepcopy(x_new)
        x_above_new[2] =safe_height_above_battery
        
        # Raise up by 10 cm   # HARDCODED MOVEMENT IN robot X DIRECTION ! FIX !
        x_mid = copy.deepcopy(x_new)
        #x_mid[0] +=0.03;
        x_mid[2]+=0.08
        
        #x_drop_battery[3:] = x_mid[3:] 
        x_new[2] += 0.05
        
        r_new = q2r(x_new[3:])@rot_z(90, unit='deg')
        x_new[3:] = r2q(r_new)
        self.sendTf(x_new[:3],x_new[3:],parent_frame=panda_2_frame,child_frame="BAT_PICKUP_TF_ABOVE")  # Debug - send transform to rviz
        0
    
    def p2_pick_up_battery(self, get_into_camera_position = True, safe_to_camera = False, pick_up_battery = True, return_to_init = True, sleep_t = 1.5, qdot_max_factor = 0.3, qddot_max_factor = 0.3):
        """TODO - make all movements relative to TFs. 
        - remove x_mid_p_drop_batttery
        - A CMoveFor is hardcoded based on current robot-cutter relation. FIX so its relative to cutter.
        
        Args:
        get_into_camera_position - if you actually want to move to camera position
        safe_to_camera - If true, robot will move to p2_q_init and then SAFELY get to camera position.If false, it will go directly there
        pick_up_battery - Actually try to pick it up"""
        rospy.loginfo("START OF P2 PICK UP BATTERY")
        
        robot = self.p2
        
        ### PARAMETERS
        #gripper_battery_pickup_rotation = 15 # deg. WE dont go straight down but rotate the gripper around its X axis slightly. (Otherwise softclaw finger hits the ground plane)
        gripper_battery_pickup_rotation = 15 # deg. WE dont go straight down but rotate the gripper around its X axis slightly. (Otherwise softclaw finger hits the ground plane)
                
        pdiff=[0,0.02,-0.002] # Do not put the TCP directly at the center of battery but a bit to the side ( so battery is in the middle)
        
        #q_mid_above_cutter = (-0.986154, -0.038233, -0.036524, -2.045438, -0.062160, 1.996306, 0.947415) # After picking up the battery, we will first move joints [0,1] to this q. Then move all the other joints.
        q_mid_above_cutter = (-0.8, -0.038233, -0.036524, -2.045438, -0.062160, 1.996306, 0.947415) # After picking up the battery, we will first move joints [0,1] to this q. Then move all the other joints.

        q_drop = (-1.046642, -0.614133, -0.130953, -2.766449, -0.125643, 2.195187, 0.881122)
        q_return = (0.196505, 0.605992, -1.600995, -1.631527, 0.600258, 1.609751, 0.965432)

        safe_height_above_battery = 0.035 # Before pickup up the battery we move THIS MUCH above it.
        z_pickup_height = 0.005 # Height of battery
        
        min_n_of_same_detections = 1
        max_t_stable_wait = 6 # Max time to wait for stable detection in seconds

        max_allowed_positional_deviation_in_subsequent_detections = 0.005
        
        sleep_t = sleep_t # Sleep for making sure to get the latest detection
                
        ### END PARAMETERS
        
        # Get transform from panda to camera and camera to battery.
        panda_2_frame = "panda_2/panda_2_link0"
        battery_frame_names  = ["battery_0_realsense", "battery_1_realsense", "battery_2_realsense", "battery_3_realsense"]
        drop_frame = 'cutter/drop_battery'
        
         # WE have a TF but getting there reliably is a problem. So two hardcoded moves for now.
        x_drop_battery = self.tf2x(parent_frame = panda_2_frame, child_frame = drop_frame)
        x_drop_battery[1] += 0.04
        
        #rospy.set_param("/vision/realsense/publish_labeled_img",True)
        do_concurrently([[self.prepare_vision, {'activate_basler':None, 'activate_realsense':True}]],wait_until_task_completion = False)

        if get_into_camera_position:
            do_concurrently([[self.p2_move_to_realsense_cutter_pos, {'init_safe':safe_to_camera}]],wait_until_task_completion = True)
            
        ### Assert we're in the correct controller
        #if robot._control_strategy !='CartesianImpedance':
        #    robot.Switch_controller(start_controller = 'CartesianImpedance')
        if robot._control_strategy !='JointPositionTrajectory':
        #    print("Pickup battery switching to JointPositionTrajectory")
            robot.ResetCurrentTarget()
            robot.Switch_controller(start_controller = 'JointPositionTrajectory')
        
        robot.SetCartesianStiff_helper(m = 1.2, n=0.8)
        # do_concurrently([[self.prepare_vision, {'activate_basler':False, 'activate_realsense':True}]],wait_until_task_completion = False) # Commented because move a bit up.

        # Wait for getting a safe battery position
        issafe = 0 # Check whether battery position is safe to grasp (on the platter)
        success =0 # Check that battery position is CONSTANT in several consecutive frames/detections.
        last_n_detections = np.zeros((min_n_of_same_detections,3)) # Saving the last n consecutive detections

        n_added = 0 # Check how many detections we've received (and only calc if n>5)
        
        # TODO - Check all battery detections and find ones which are good
        
        # Have to check:
        # If it exists:
        
        #batteries = self.VU_rs.get_particular_class_from_detections(detections = [], desired_class = 'battery', check_size = True) # Just a dummy call 

        rospy.loginfo("batt start")
        st_t = time.time()
        while (time.time() - st_t) < sleep_t:
            pass
        #time.sleep(sleep_t)
        
       
        ### START COPIED
        bf, x_r2batt, issafe = self.VU_rs.get_battery(fn_tf2x = self.tf2x, robot_frame = robot.Base_link_name, timeout = 4)
        if x_r2batt is None:
            rospy.loginfo("Could not find battery within specified time")
            return 0
        
        rospy.loginfo("Picking up {}".format(bf))
        rospy.loginfo("Issafe status: {}".format(issafe))

        #rospy.loginfo("PICK UP BATTERY GOT RESULT")
        # At first, Rotate around X because Z is by default pointing upwards, while tool Z is pointing downwards.
        a=q2r(x_r2batt[3:])@rot_x(180,unit='deg')
        
        # Handling of cases of different battery rotation and position
        robot.GetState()
        tool_rpy = r2rpy(q2r(robot.x[3:]), out = 'deg')
        
        do_additional_rotation = 0 # Check whether rotation needs to be fixed
        additional_tool_rot_z = 180 # How much we should change the rotation, IF do_additional_rotation==1. This will always be either 180 or -180.
        
        copy_issafe = copy.deepcopy(issafe)
        rospy.loginfo("copy issafe:{}".format(copy_issafe))
        # Center center
        if issafe==1:
            # Fix rotation
            if a[1,1] > 0:
                print("NAROBE-ROTACIJA-\n",a[1,1])
                do_additional_rotation = 1
                #a=a@rot_z(180,unit='deg')   
        
        # Left edge
        elif issafe ==2:
            # Left edge is unsafe for pickup.
            issafe = 0
            rospy.loginfo("Left edge is unsafe for battery pickup")
        
        # Top center
        elif issafe==3:
            # Replace < with > to fix it later  
            if a[1,1] < 0:
                print("NAROBE-ROTACIJA--\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        # Top right
        elif issafe==4:
            if (a[1,1] > 0) and (a[0,1] >0):
                0
            else:
                #if (a[1,1] < 0) or:
                print("NAROBE-ROTACIJA---!\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            rospy.loginfo("Corner is unsafe for battery pickup")
        
        # Center right
        elif issafe==5:
            if a[0,1] < 0:
                print("NAROBE-ROTACIJA---\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        # Bottom right
        elif issafe==6:
            if (a[1,1] < 0) and (a[0,1] > 0):
                0
            else:
            #rospy.loginfo("Got into issafe==6")
            #if a[1,1] < 0:
                print("NAROBE-ROTACIJA---!\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            rospy.loginfo("Corner is unsafe for battery pickup..")
        
        # Bottom center
        elif issafe==7:
            if a[1,1] > 0:
                print("NAROBE-ROTACIJA----\n",a[1,1])
                do_additional_rotation = 1 
            issafe = 1
            #issafe = 0
        
        """
        robot.GetState()
        q1 = np.array(robot.q)
        q1[-1] = robot.q_home[-1]
        robot.JMove(q1,1)"""
        #########################################
        
        if do_additional_rotation:
            # Check if we should rotate in the opposite way (by -180 instead of 180)
            rospy.loginfo("Performing additional rotation")
            midpoint_a = a@rot_z(180, unit = 'deg') # force the robot to go into the correct direction

            a = a@rot_z(additional_tool_rot_z, unit = 'deg')
        else:
            midpoint_a = a
            rospy.loginfo("Not rotating further")
                
            
        q1 = r2q(a)
        q_midpoint = r2q(midpoint_a)     
        
        pos_new_midpoint = (robot.x[:3] + x_r2batt[:3])/2
        x_new_midpoint = np.concatenate([pos_new_midpoint,q_midpoint])
        x_new=np.concatenate([x_r2batt[:3],q1])
        
        T_new_midpoint = x2t(x_new_midpoint)
        T_new = x2t(x_new)
        
        # Relative move in tool Y so we don't hit the center of the battery. Relative tool rotation so the gripper part doesnt hit the ground.
        Tdiff =np.eye(4)
        Rdiff = rot_x(gripper_battery_pickup_rotation,unit='deg')
        Tdiff = rp2t(R=Rdiff,p=pdiff)
        T_new=T_new@Tdiff
        x_new=t2x(T_new)
        x_new[2] = 0.01
        #x_new[2] += 0.005 # Raise 3 mm above position.
        
        # Safe position above battery
        x_above_new = copy.deepcopy(x_new)
        x_above_new[2] =safe_height_above_battery
        
        # Raise up by 10 cm   # HARDCODED MOVEMENT IN robot X DIRECTION ! FIX !
        x_mid = copy.deepcopy(x_new)
        #x_mid[0] +=0.03;
        x_mid[2]+=0.08
        
        x_drop_battery[3:] = x_mid[3:] 

        self.sendTf(x_new[:3],x_new[3:],parent_frame=panda_2_frame,child_frame="BAT_PICKUP_TF")  # Debug - send transform to rviz
        
        ######### END COPIED
        #return 0
        
        # Check if we are about to go into a weird rotation of the tool.
        we_in_joint_trouble = 0
        r_bat = q2r(x_new[3:])
        if (r_bat[0,0] <0) and (r_bat[0,1]>0):
            rospy.loginfo("we in some trouble")
            we_in_joint_trouble = 1
        rospy.loginfo("{}, {}".format(r_bat[0,0], r_bat[0,1]))
            
        if pick_up_battery and (issafe==1):
            
            if we_in_joint_trouble:
                qq_dif = (-2.059724, 0.155259, 1.064711, -2.287018, -0.411701, 2.346840, -2.866111)
                # Force the robot into a different joint config.
                robot.JMove(q=qq_dif, t=4.5/self.sf)
            
            # Set nullspace so we don't hit joint limits with the last joint
            neutral_q = (robot.q_max + robot.q_min) / 2
            robot.SetCartImpContNullspace(q =neutral_q,  k=[3, 0, 0, 0, 0, 0, 5]) # Only enforce for last joint, for now.
            robot.SetCartesianStiff_helper(m = 1.1, n=0.65)

            # Open softclaw
            robot.gripper.open(sleep=False)
            # Move above battery
            #robot.CMove(x_above_new,t = 2.5/self.sf)
            
            # Move to battery grasp position
            #robot.CMove(x_new,t = 1.5/self.sf)
            
            # CPath combined move
            if copy_issafe in [1]:
                # center center
                0
            if copy_issafe in [3]:
                # Top center
                #robot.ResetCurrentTarget()
                #robot.JMoveFor([0,0,0,0,0,0, + np.pi/2],2)
                0                
                
            elif copy_issafe in [5]:
                #robot.JMoveFor([0,0,0,0,0,0,-np.pi],2)
                if a[1,1] > 0:
                    # Y of bat pickup is aligned with robot Y.
                    0
                else:
                    0
                    #robot.JMoveFor([0,0,0,0,0,0,-np.pi],2)
                    #robot.ResetCurrentTarget()
                
                            #else:
            elif copy_issafe in [4,6]:
                print("Hit case A")
                # The second condition is that the 
                #if (q2r(x_new[3:])[0,0] >0) and (copy_issafe in [5,6]) :
                #if copy_issafe in [5]:
                if 1:                
                    rospy.loginfo("Rotating J7 for exception strategy.")
                    #robot.JMoveFor([0,0,0,0,0,0,-np.pi],2)
                    robot.ResetCurrentTarget()
            elif copy_issafe in [7]:
                robot.JMoveFor([0,0,0,0,0,0,-np.pi/2],2.5)
            else:
                0
 
            robot.GetState();cur_x= copy.deepcopy(robot._command_int.x)
            path = np.stack((cur_x, x_above_new, x_new))
            #path = np.stack((x_above_new, x_new))
            tpath = (2.5+3) / self.sf
            tpath = robot.tsamp * round( tpath / robot.tsamp )         # Due to code in CPath, tpath must be divisible by tsamp.
            
            if self.d_p['use_cmove_instead_of_cpath']:
                robot.CMove(x_above_new, t = 6/self.sf)
                #robot.SetCartesianStiff_helper(m = 1.35, n=0.55) # Increase the positional stiffness and move to battery pickup pos.
                robot.CMove(x_new, t=4/self.sf)
            else:
                
                robot.CPath_new(path=path, t = tpath)
                        
            # Close gripper
            robot.gripper.close(1, sleep = True);time.sleep(0.4)
            robot.CMoveFor([0,0,0.02],1.5)
            
            # Print the positional error
            robot.GetState(); pos_error = np.abs(robot._actual_int.x - robot._command_int.x)[0:3]
            rospy.loginfo("P2_pickup_bat: pos error: {}".format(pos_error))
            
            #robot.CMove(x_new, t =2 /self.sf)
            #robot.CMoveFor(dx=[0,0.0.05], t=1/self.sf)
            #robot.CMove(x_drop_battery[0:3],t =2 /self.sf)
            # Drop to the Battery bin
            robot.ResetCurrentTarget() # In case we hit something when picking up.
            robot.error_recovery()
            
            # HACK TO get into neutral last joint position
            #robot.CMoveFor(dx=[0,0,0.02], t= 1.5)
            
            ####  Exception handling - if last J is near limits, lift the robot up a bit and ro
            """robot.GetState();qq = np.array(robot.q)
            rospy.loginfo("Robot last q: {}".format(qq[-1]))
            if (qq[-1] > 1.9):
                robot.JMoveFor([0,-0.2,0,0,0,0,-0.15], 4/self.sf)
                rospy.loginfo("Rotating last joint-")

            elif (qq[-1] < -1.9):
                robot.JMoveFor([0,-0.2,0,0,0,0,+0.15], 4/self.sf) 
                rospy.loginfo("Rotating last joint+")
            """
            #### END exception handling
            print("GOT TO near END")
            tmp_q = np.array(copy.deepcopy(robot._command_int.q))
            tmp_q[0:2] = q_mid_above_cutter[0:2]
            #robot.JMove(tmp_q, 2, qddot_max_factor = 0.2, qdot_max_factor=0.2)
            
            robot.JMove(q_mid_above_cutter, 2, qddot_max_factor = qddot_max_factor , qdot_max_factor=qdot_max_factor)
            
            # Move to drop position
            robot.JMove(q_drop, 2, qddot_max_factor = qddot_max_factor, qdot_max_factor=qdot_max_factor)
            
            
            """ Code to use CPath. Now we will use JMoves for better safety
            robot.GetState()
            robot.ResetCurrentTarget()
            cur_x= copy.deepcopy(robot._command_int.x)
            # Keep the current orientation.
            x_mid[3:] = cur_x[3:]
            x_drop_battery[3:] = cur_x[3:]
            
            # CPath combined move
            #path = np.stack((x_mid, x_drop_battery))
            path = np.stack((cur_x, x_mid, x_drop_battery))

            tpath = (2 + 3) / self.sf
            tpath = robot.tsamp * round( tpath / robot.tsamp )         # Due to code in CPath, tpath must be divisible by tsamp.
          
            robot.CPath_new(path=path, t = tpath)
            """
            
            
            # Open gripper
            robot.gripper.open(sleep = True)
            
            # Move up a bit
            #robot.CMoveFor(dx=[0,0,0.07],t=2.5/self.sf)
            # Remove the nullspace constraint
            robot.SetCartImpContNullspace(q =neutral_q,  k=[4, 4, 0, 0, 0, 0, 0]) # Only enforce for last joint, for now.

        #rospy.set_param("/vision/realsense/publish_labeled_img",False)

        do_concurrently([[self.prepare_vision , {'activate_basler' : None, 'activate_realsense' : False}]], wait_until_task_completion=False) # Disable realsense to not overheat it
        robot.ResetCurrentTarget()

        if return_to_init:
            if robot._control_strategy != 'JointPositionTrajectory':
                robot.ResetCurrentTarget()
                robot.Switch_controller(start_controller = 'JointPositionTrajectory')
            #robot.JMove(q_return, t = 5.5 /self.sf)
            
            self.robot_go_to_init_and_open_gripper(robot_obj =robot)
        
        return 0
        
    def p2_perform_pinpush(self, init_safe = True, init_safe_method='cartesian', hca_type = None, rotate_vise = False):
        """ Perform pin pushing with p2.
        if init_safe : get into this position safely. Else, assume we are in this position already
        
        TO - DO -
        x_diff_long - Sets the robot rotation in regard to vise_pinpush_init. Must be changed if the
                    robot gripper is changed.
                    
        Robot movement for pushing is NOT in tool space, but in robot Y direction, which could be improved.
        The problem is that the robot can not achieve the configuration for straight pushing in tool space.
        IMPROVE.
        """
        
        rospy.loginfo("START OF P2 PINPUSH")
        robot = self.p2  # So we dont use self.p2 in code

        # Get the TF
        x_init = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame='vise/pinpush_init')
        
        assert hca_type in ['qundis', 'kalo']
        
        #### PARAMS FOR LONG AND SHORT 
        
        if hca_type =='qundis':
            # Changes only required for short HCA
            x_init[0] -=0.006 # 0.004
            #x_init[2] +=0.0035
            x_init[2] +=0.008
            # Move it a bit in the neg x dir
            #push_dist = 0.045   # How far we push from vise_pinpush_init 
            push_dist = 0.04   # How far we push from vise_pinpush_init 
            print("pinpush moving init")

        elif hca_type == 'kalo':
            push_dist = 0.03
        
        tool_rot_x = -50
        tool_rot_y = 20
        
        second_tool_rot_x = 50
        
        tool_rot_z = -35
        
        # We have to lift the robot a bit tho
        
        #q_init=(-0.918323, 0.393909, 0.592221, -1.555150, 0.340555, 1.822672, 0.134631)
        
        ### PARAMETERS :
        # Robot tool rotation in regards to the vise_pinpush_init ( because the tool can NOT reach such rotation at this distance )
        
        #xdiff_long = np.array([0.01, 0.011, 0.015 ,1, 0, 0, 0]) # For tool space
        
        # Small changes to the TF vise_pinpush_init
        xdiff_long = np.array([0.01, 0.01, -0.012 ,1, 0, 0, 0]) # For world space
        
        dist_safe_z = 0.1 # How much above the vise_pinpush_init we move for init
                    
        if init_safe:
            if init_safe_method=='joint': 
                self.robot_go_to_init_and_open_gripper(robot)
            elif init_safe_method=='cartesian':
                robot.CMove([ 0.3715, -0.0800,  0.3577,  0.0186,  0.9997,  0.0069,  0.0106],5.5/self.sf)
                robot.gripper.open()
            
        # Check that the vise is closed
        # Check that gripper is open
        do_concurrently([[robot.gripper.open, {}],
                         [handle_pneumatics_jaws, {'ud':self.ud, 'mainjaws_st':self.mainjaws_st, 'sidejaws_st':self.sidejaws_st, 'command':'close'}]],
                        wait_until_task_completion=False)
        
        #if robot._control_strategy != 'JointPositionTrajectory': #FIXME: Miha CI->JI
        #    robot.ResetCurrentTarget()
        #    robot.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        # Handle controller setup. Force using cart controller
        if robot._control_strategy != 'CartesianImpedance': #FIXME: Miha CI->JI
            robot.ResetCurrentTarget()
            robot.Switch_controller(start_controller =  'CartesianImpedance')
            time.sleep(0.1)
        
            robot.SetCartImpContNullspace(q = robot.q, k=[0,0,0,0,0,0,0])

        # Using cart controller:
        robot.SetCartesianStiff_helper(m=1, n=0.7) # For moves and no shaking set this. #FIXME: Miha CI->JI no stiffness changes
        #robot.SetCollisionBehavior(F=25, T = 3.5, tq = 7)

        # Constructing the robot positions
        T_init = x2t(x_init)
        r_init = T_init[0:3,0:3]@rot_z(tool_rot_z, unit='deg')
        T_init[0:3,0:3] = r_init
        
        # Copy this end position. Raise Z
        T2 =copy.deepcopy(T_init) # Safe Pose
        T2[2,-1] +=dist_safe_z
        # This in-between position will have a bit of rotation
        R2 = T2[0:3,0:3]@rot_x(tool_rot_x, unit='deg')@rot_y(tool_rot_y, unit='deg')@rot_z(-tool_rot_z, unit='deg')
        T2[0:3, 0:3] = R2
        
        #t_init = x2t(xdiff_long)@t_init

        # Now rotate around x by 50 deg and rot_z
        #r_init = t_init[0:3, 0:3]@rot_x(second_tool_rot_x, unit = 'deg')
        #t_init[0:3, 0:3] = r_init

        Tdiff = x2t(xdiff_long)
        
        #T_new = T_init@Tdiff # Ta is initial position. Tdiff is the move IN TOOL SPACE.
        T_new = Tdiff@T_init # Ta is initial position. Tdiff is the move IN WORLD SPACE.
        
        if hca_type == 'qundis':
            # TODO make relative.
            # move a bit back so the robot gets a bit more speed
            T_new[1, -1] += 0.02 
            
        #T_new = Ta # Hack after we shortened the toolchangers.

        #T_new[1,-1] += dist
        #T_new[2,-1]+=0.002
        
        # Safely move above pinpush start position
        robot.GetState(); cur_x = copy.deepcopy(robot._command_int.x) 
        path = np.stack((cur_x, t2x(T2), t2x(T_new))) 
        #path = np.stack((t2x(T2), t2x(T_new))) 
        tpath = (4 + 3 + 1) / self.sf
        tpath = robot.tsamp * round( tpath / robot.tsamp )         # Due to code in CPath, tpath must be divisible by tsamp.
        
        rospy.loginfo("Tpath: {}".format(tpath))
        #robot.CPath(path=path, t=tpath)
        
        #robot.CPath_new(path=path, t=tpath)
        robot.CMove(t2x(T2), 3/self.sf)
        robot.CMove(t2x(T_new), 3/self.sf)

        #robot.CMove(T2, t = 4.5/self.sf, task_space = 'World')
        
        # Maybe increase stiffness a bit more for better repeatability.
        robot.SetCartesianStiff_helper(m=1.2, n=0.975) #FIXME: Miha CI->JI no stiffness changes
        
        # Move to pinpush start position 
        #robot.CMove(T_new,t = 3/self.sf)
        
        success = 0
        n_attempts = 0
        max_allowed_attempts = 2
        
        while success ==0:
            # Move to pinpush start position 
            
            pushmove = [0,push_dist,0]
           
            #robot.CMoveFor(dx=pushmove, t = 0.5/self.sf, task_space = 'World') # Literally push the pin out
            robot.CMoveFor(dx=pushmove, t = 0.3, task_space = 'World') # Literally push the pin out
            
            after_pinpush_T = copy.deepcopy(T_new)
            after_pinpush_T[0:3, -1] += pushmove
            
            time.sleep(0.5)            
            
            robot.GetState()
            pos_error = np.linalg.norm((after_pinpush_T[0:3, -1] - robot.x[0:3]))
            
            if (robot.last_franka_state.robot_mode != fs.ROBOT_MODE_MOVE) or (pos_error>0.01):
                rospy.loginfo("{}".format(pos_error))
                rospy.loginfo("Pinpush FAILED!")
                n_attempts +=1
                robot.ResetCurrentTarget()
                robot.error_recovery()
                robot.ResetCurrentTarget()
                robot.CMove(T_new,t = 3/self.sf)
                
                if n_attempts >= max_allowed_attempts:
                    success=1
            else:
                rospy.loginfo("After pinpush, robot state is {}".format(robot.last_franka_state.robot_mode))
                rospy.loginfo("Estimated error: {} ".format(pos_error))
                success = 1
            
            # Upon failure detection : 
            T_new[0,-1] -=0.001 # Move 1 mm closer to the robot and push again. 
            
        robot.SetCartesianStiff_helper(m=1, n=0.85) # Reduce stiffness again.
        #########################
        robot.ResetCurrentTarget()
        
        robot.error_recovery()
        robot.GetState()
        
        #x_via_1 = np.array(robot.x)+[0,-push_dist*0.333,0,0,0,0, 0]
        #x_via_2 = np.array(robot.x)+[0,-push_dist*0.667,0,0,0,0, 0]
        #x_new = np.array(robot.x)+[0,-push_dist,0,0,0,0, 0]
        
        #path = np.stack((robot.x, x_via_1, x_via_2, x_new)) 
        #tpath = 3/self.sf
        #robot.CPath_new(path=path, t=tpath)
        
        robot.CMoveFor(dx=[0,-0.04,0], t = 1.8/self.sf, task_space = 'World') # Retract # This works
        robot.error_recovery()

        robot.CMoveFor(dx=[0,0,0.05], t = 1.8/self.sf, task_space = 'World') # Retract
    
        robot.CMove(T2,t = 2.5/self.sf) # Safe pose
        
        # Check that we are reasonably close to commanded position
        robot.GetState(); combined_error = np.sum(np.abs(robot.p_err))
        
        allow_vise_rotation = False
        if combined_error < 0.05 : 
            allow_vise_rotation = True
        else:
            # If we get into error, we need to recover and retract again.
            rospy.loginfo("Not rotating vise because robot has large positional error - {}".format(combined_error))
            robot.error_recovery(enforce = True)
            robot.SetCartesianStiff_helper(m=1, n=0.85)

            robot.CMoveFor(dx=[0,-0.04,0], t = 2/self.sf, task_space = 'World') # Retract
            robot.CMove(T2,2.5/self.sf) # Safe pose
            
            # Check again that we are reasonably close to commanded position
            robot.GetState(); combined_error = np.sum(np.abs(robot.p_err))
            if combined_error < 0.05 : 
                allow_vise_rotation = True

        # Rotate vise so the pin falls out in case it's inside the HCA
        if (rotate_vise == True) and (allow_vise_rotation):
            move_slider(ud=self.ud, slider_st=self.slider_st, position='back')

            rotate_vice(ud = self.ud, rotate_vice_st= self.rotate_holder_st, position= 'down')
            time.sleep(4)
            rotate_vice(ud = self.ud, rotate_vice_st= self.rotate_holder_st, position= 'up')
            time.sleep(4)
        
        # Reset collision thresholds to normal
        #robot.SetCollisionBehavior(F=20, T = 2, tq = 5)
        
        return 0
    
    def p2_gap_detection(self, init_safe = True, wait = False, return_to_init = False):
    
        r = self.p2
        
        assert r.Name == 'panda_2' # Only panda 2 has realsense
        # Get tf for vision
        rs_to_ee = self.tf2x(parent_frame = 'panda_2/realsense', child_frame='panda_2/panda_EE')

        link0_to_realsense_vise_pos = self.tf2x(parent_frame = 'panda_2/panda_2_link0', child_frame='vise/realsense')
        
        T1 = x2t(rs_to_ee)
        T2 = x2t(link0_to_realsense_vise_pos)
        
        T_target = T2@T1
        
        x_init = t2x(T_target)
        # Move the init pos 10 cm lower
        x_init[2] -= 0.05
        
        if init_safe:
            if r._control_strategy != 'JointPositionTrajectory':
                r.Switch_controller(start_controller = 'JointPositionTrajectory')
                
            r.ResetCurrentTarget()
            r.error_recovery()
            # Go to p2 init
            r.JMove(p2_q1_init, t = 3, max_vel = 0.5, max_acc = 0.5)
            
        r.CMove(x_init,t =3)
        
        """
        # Get old realsense height so we can reset it later
        old_realsense_Z = get_realsense_height()
        
        # Get height from current realsense pos to vise_lowerplane
        x_init = self.tf2x(parent_frame = 'panda_2/realsense', child_frame='vise/lowerplane')
        new_realsense_Z = x_init[2]
        print("Setting realsense height to Z: %.3f m"%new_realsense_Z)
        
        set_realsense_height(Z=float(new_realsense_Z))
        
        # enable vision
        rospy.set_param("/vision/realsense/publish_depth_img", True)
        rospy.set_param("/vision/realsense/publish_cluster_img", True)

        """
        self.prepare_vision(activate_basler = False, activate_realsense= True)
        
        if wait:
            time.sleep(3)
            
        # Get the GAPP from VISIONN and draw approx position with ROS TF.

        # Reset realsense height back to old, as that is the default (for example if we do battery pickup later.)
        
        #set_realsense_height(Z = old_realsense_Z)
        
        if return_to_init:
            self.prepare_vision(activate_basler = False, activate_realsense= False)
            r.JMove(p2_q1_init, t = 3, max_vel = 0.5, max_acc = 0.5)
        else:
            
            r.CMoveFor([0,-0.1, -0.1], 4.5/self.sf)
            r.CMoveFor(rot_z(-90,unit='deg'), 2/self.sf, task_space='Tool')
        return 0
    
    def init_leverblock(self, robot, hca_type = 'kalo', direction_deg = 90, speed_factor = 1):
        """ Function to init the leverblock. here parameters to speed of movement can be changed."""
        
        assert hca_type in ['kalo', 'qundis']
        assert direction_deg is not None     

        #################### parameters
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
        leverblock = LeverBlock(robot = robot, end_effector = 'softclaw', to_module = None, neutral_rot_vertical = 1,
                                initial_cart_pos = start_cart_pos, direction = direction_deg, bbox=None,
                            allowed_tool_angles = (None,None), speed_factor= speed_factor)
            
        ####### LEVERBLOCK PARAMETERS
        
        leverblock.PDMP_M_THR_CONDITION = 1.5
        
        leverblock.allowed_tool_angles = R_limits # Checking p2.R[2,2] so the Z component.
        #leverblock.allowed_tool_angles = (None, None)
        
        return leverblock
            
    def prepare_vision(self, activate_basler= None, activate_realsense = None):
        assert ((activate_basler is not None) or (activate_realsense is not None)) # Some setting must be specified for at least one camera
        
        if activate_basler is not None:
        
            # Activate basler 
            self.ud.value = activate_basler
            self.activate_basler_st.on_enter(self.ud);time.sleep(0.2)
            
        if activate_realsense is not None:
            self.ud.value = activate_realsense
            self.activate_realsense_st.on_enter(self.ud);time.sleep(0.2)
            
        return 0
        
    def prepare_pneumatics(self):
        #Turn on air to block 2
        self.ud.value = True; self.activate_block2_st.on_enter(self.ud)
        
        # Open vice
        handle_pneumatics_jaws(ud= self.ud, mainjaws_st =self.mainjaws_st, sidejaws_st=self.sidejaws_st, command = 'open');time.sleep(0.1)
        # Move slider back
        #move_slider(self.ud, self.slider_st, position = 'back'); time.sleep(0.5)
        # Rotate vice
        #rotate_vice(self.ud, self.rotate_holder_st, position = 'up')
        return 0
    
    def robot_go_to_init_and_open_gripper(self, robot_obj):
        """Generic function for both robots. Go to q init and open the gripper """
        if robot_obj._control_strategy != 'JointPositionTrajectory':
            robot_obj.ResetCurrentTarget()
            robot_obj.Switch_controller(start_controller =  'JointPositionTrajectory')
            
        robot_obj.error_recovery()
        #robot_obj.ResetCurrentTarget();time.sleep(0.1)
        init_q_pos = self.robot_init_qs[robot_obj.Name]#
        
        robot_obj.gripper.open(1, sleep=False)
        robot_obj.JMove(init_q_pos[0], t= 1, qdot_max_factor = 0.2, qddot_max_factor = 0.2) # The time will be overridden based on distance and max vel/acc
        
        #do_concurrently([[robot_obj.JMove, {'q':init_q_pos[0], 't':4/self.sf, 'max_vel':0.8, 'max_acc':0.8}], [robot_obj.gripper.open, {}]], wait_until_task_completion = True)
        #robot_obj.WaitUntilStopped()
        # If we have more init Joint positions
        robot_obj.GetState()
        #robot_obj.ResetCurrentTarget()
        if len(init_q_pos)>1:
            for i in range(1,len(init_q_pos)):
                robot_obj.JMove(init_q_pos[i], t=1, qdot_max_factor = 0.5, qddot_max_factor = 0.5)
        return 0
