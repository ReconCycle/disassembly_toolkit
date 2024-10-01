import numpy as np
from disassembly_pipeline.skills.base_skill import BaseSkill

from disassembly_pipeline.utils.Levering_block_v3 import move_until_contact
from disassembly_pipeline.utils.tf_utils import TFManager
from robotblockset_python.ros.grippers_ros import VariableStiffnessGripper

from robotblockset_python.transformations import *

from unified_planning.shortcuts import *

import copy

class KaloRemotusOpening(BaseSkill):
    def __init__(self):
        """ Opening the KaloRemotus smokedetector with the QB VSA gripper, while it is clamped inside a 
        second QB VSA gripper on the table. (as per QB robotics example opening video)."""

        self.Name = 'kaloremotus_qbvsa_opening'

        # This fn relies on a table-mounted VSG. We will try to find this object/key (param_table_gripper_name) in kwargs.keys(), if not we will initialize it.
        self.param_robot_name = 'robot'
        self.param_table_gripper_name = 'table_gripper'
        self.param_tf_listener_name = 'tf_listener' # We will also try to find this in kwargs.keys(), if not, we initialize it.

        self.param_smokedetector_x_name = 'kaloremotus_center_x'


    def on_enter(self, **kwargs):
        """ Function to call when first starting the skill."""

        r = kwargs[self.param_robot_name] # Robotblockset_python object

        if self.param_table_gripper_name not in kwargs.keys():
            TABLE = VariableStiffnessGripper(**{'__ns':'/qbmove1'})
        else:
            TABLE = kwargs[self.param_table_gripper_name]

        if self.param_tf_listener_name not in kwargs.keys():
            tflist = TFManager()
        else:
            tflist = kwargs[self.param_tf_listener_name]

        if self.param_smokedetector_x_name not in kwargs.keys():
            # This we get from vision
            smokedet_x = [0.27, -0.53, 0.175]
        else:
            smokedet_x = kwargs[self.param_smokedetector_x_name]

        ROT_GRIPPER_X = 5

        smokedet_R = np.eye(3)@rot_y(90+ROT_GRIPPER_X, unit='deg')@rot_z(-90, unit='deg')
        smokdet_q = r2q(smokedet_R)

        T_smokedet=np.zeros((4,4))
        T_smokedet[0:3, -1] = smokedet_x
        T_smokedet[0:3, 0:3] = smokedet_R

        dT_smoke_center_to_initial_gap_pose = np.eye(4)
        dT_smoke_center_to_initial_gap_pose[0:3, -1] = [0, -0.055, -0.06]

        tflist.SendTransform2tf(p = T_smokedet[0:3,-1], q = r2q(T_smokedet[0:3, 0:3]), parent_frame = r.Base_link_name, child_frame = 'VIRT_SMOKEDET_POSE')

        T_initial_gap_pose = T_smokedet@dT_smoke_center_to_initial_gap_pose

        tflist.SendTransform2tf(p = T_initial_gap_pose[0:3, -1], q = r2q(T_initial_gap_pose[0:3, 0:3]), parent_frame = r.Base_link_name, child_frame = 'VIRT_SMOKEDET_POSE_2')

        r.error_recovery()

        qbk = QBKaloDisassembly(robot=r, table_gripper=TABLE, smokedet_T = T_smokedet)
        #qbk.perform_full_sequence()

        # PLAN A 
        #qbk.step_1_move_to_init()
        #qbk.step_2_cartesian()
        #qbk.step_3_detect_gap()
        #qbk.step_4_rotate_while_pushing_down()
        #qbk.step_5_rotate_vertical()
        #qbk.step_6_wiggle_and_rotate()
        #qbk.step_7_wiggle_and_push_down()
        #qbk.step_8_almost_vertical_and_push_down()

        # Plan B
        qbk.step_1_move_to_init()
        qbk.step_2_cartesian()
        qbk.step_3_detect_gap()
        qbk.step_4_rotate_while_pushing_down()
        qbk.step_6_B()

        TABLE.close(0.9)

        return 0

    def execute(self, **kwargs):
        """ Function that GETS CALLED periodically(possibly with high freq) during skill execution. """
        0
    def on_exit(self, **kwargs):
        """ Function that gets called after the skill finishes."""
        0

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Add robot_homing operator
        #robot_type = UserType('robot')

        #homing_operator = InstantaneousAction(self.Name, robot = robot_type)
        #robot = homing_operator.parameter('robot')

        #is_homed = problem.fluent(self.is_homed_fluent_name)
        #homing_operator.add_effect(is_homed(robot), True)

        #problem.add_action(homing_operator)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links

class QBKaloDisassembly:
    def __init__(self, robot, table_gripper, generic_tf_listener = None,
                smokedet_T = None, debug = 1):
        """ Convenience class containing all functions used in opening of the KaloRemotus smoke detector.
        Args:
        robot - robotblockset_python object
        table_gripper - VariableStiffnessGripper object of the gripper mounted on the table
        generic_tf_listener - GenericTransformListener object
        
        smokedet_tf - transform from ROBOT BASE FRAME to the SMOKE DETECTOR CENTER. z axis should point 
        perpendicularly to the smoke detector diameter, in the direction of the removable top shell.
        y-axis should point in negative z in regard to world frame. This is the expected input we get from vision.
        
        debug - if 1, publish transforms and print out more statements.
        
        """
        
        self.r = robot
        
        self.table_gripper = table_gripper
        
        self.tf_listener = generic_tf_listener
        if self.tf_listener == None:
            self.tf_listener = TFManager()
            
        self.smokedet_T = smokedet_T
        
        # Relative transform between the smoke detector center(look at this class args on definition) to initial gap pose
        dT_smoke_center_to_initial_gap_pose = np.eye(4)
        dT_smoke_center_to_initial_gap_pose[0:3, -1] = [0, -0.05, -0.06] # Always same for kaloremotus
        
        self.x_initial_gap_pose = self.smokedet_T@dT_smoke_center_to_initial_gap_pose
        
        # vertical gripper pose close to kalo
        self.kalo_home_q = (-0.7304958693228268,0.5720781059832135,-1.3968639765773132,-2.0155653766854926,
         1.2734205071793665,1.1957041064974967,-0.13477559892585156)
        
        self.kalo_close_q = (-1.1254009219327141,0.5116530463557648,-0.7302008016508498,-2.108335182162008,
                     1.5781495524115032,1.1863018726063899, -0.17213026520957872)
        
        
        
        self.nullspace_q = (-1.5374918436209362,0.19467146751546024,-0.5367565084364895,-1.9244464762172722,
                             0.9868535665847984,1.4535037213824002,0.09000419707844746)
        
        self.nullspace_k = np.ones(7)*1
        
        # TABLE GRIPPER PARAMETERS
        
        self.table_gripper_strong_grip_command = 1.6
        self.table_gripper_soft_grip_command = 1.2
        
        if debug:
            if self.smokedet_T is not None: 
                # Publish TFs for debug
                0
                #self.tf_listener.SendTransform2tf(p = self.smokedet_T[0:3, -1], )
                #self.tf_listener.SendTransform2tf(p = )


    def step_1_move_to_init(self):
        """ Perform joint moves to starting pose """
        
        r = self.r

        if r._control_strategy != 'JointPositionTrajectory':
            r.Switch_controller(start_controller = 'JointPositionTrajectory')

        r.error_recovery()
        r.JMove(self.kalo_home_q, 3, qdot_max_factor = 0.2, qddot_max_factor = 0.2)

        #r.JMove(self.kalo_close_q, 3, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
    
    def step_2_cartesian(self):
        
        """Switch to cartesianImpedance, move to known pose next to kalo. Switch to low stiffness in z"""
        
        r = self.r
        
        # Change to CartesianImpedance Controller and move to known pose next to smoke detector with HIGH STIFFNESS

        # When in position, start cartesian controller. Set low stiffness in z-axis
        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')
            
        r.error_recovery()
        
        r.ResetCurrentTarget()
        r.SetCartesianStiff_helper(Kp=[3000, 3000, 3000])
        
        # Set nullspace stiffness
        r.SetCartImpContNullspace(q = self.nullspace_q, k = self.nullspace_k)

        r.CMove(self.x_initial_gap_pose, 2) # Move closer with high stiffness
        
        
        return 0
    
    def step_3_detect_gap(self):
        """ Try to use move_until_contact to move in robot x axis, with low z stiffness, to detect the gap"""
        
        # Make the gripper stiffer
        self.table_gripper.close(self.table_gripper_strong_grip_command)
        
        r = self.r
        
        if r._control_strategy != 'JointPositionTrajectory':
            r.Switch_controller(start_controller = 'CartesianImpedance')
    
        r.SetCartesianStiff_helper(Kp=[3000, 3000, 100])
        # Get zeroing FT
        r.GetState()
        zeroing_FT = copy.deepcopy(r.state.K_F_ext_hat_K)

        move_until_contact(r,
                           task_space = 'World',
                           mv_unit_direction = [1, 0, 0],
                           mv_velocity = 0.03,
                           rot_unit_direction= None,
                           rot_velocity = 0.01,
                           dt = None,
                           axes_to_monitor = [1,0,0,0,0,0],
                           max_allowed_values = [1.3,0,10,2,2,2],
                           use_abs = True,
                           max_allowed_t = 5,
                           allowed_tool_angles = (None,None),
                           zeroing_FT = zeroing_FT,
                           min_F_measurement_dt = 0.01)
        
        # Upon detecting gap, rotate gripper to fixed known angle.
        # Rotate to angle 45 degs
        ROT_DEG = 45

        r.GetState()
        cur_T = r.T
        cur_rpy = r2rpy(cur_T[0:3, 0:3])
        print(cur_rpy * 180 / np.pi)

        target_rpy = cur_rpy
        #target_rpy[2] += 10 * np.pi/180
        target_rpy[2] = (-180 + ROT_DEG) *np.pi/180

        RR = rpy2r(target_rpy)

        cur_T[0:3, 0:3] = RR

        r.error_recovery(reset_target = False)
        r.CMove(cur_T, 3)
        
        if self.tf_listener is not None:
            self.tf_listener.SendTransform2tf(p = cur_T[0:3, -1], q = r2q(cur_T[0:3, 0:3]), parent_frame = 'panda_2/panda_2_link0', child_frame = 'TESTY')
    
    def step_4_rotate_while_pushing_down(self):
        """ The robot should be approximately at the gap pose.
        This function will rotate around tool x axis while pushing down in tool z-axis""" 

        r = self.r
        
        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')
            
        r.SetCartesianStiff_helper(Kp=[3000, 3000, 3000], Kr = [50,50,50])

        r.GetState()
        r.error_recovery()

        r.ResetCurrentTarget()

        r.GetState()
        zeroing_FT = r.state.K_F_ext_hat_K

        move_until_contact(r,
                           task_space = 'Tool',
                           mv_unit_direction = [0, 0, 1],
                           mv_velocity = 0.03,
                           rot_unit_direction= [0,0,1],
                           rot_velocity = 7,
                           dt = None,
                           axes_to_monitor = [1,0,0,0,0,0],
                           max_allowed_values = [5,0,10,2,2,2],
                           use_abs = True,
                           max_allowed_t = 5,
                           allowed_tool_angles = (None,None),
                           zeroing_FT = zeroing_FT,
                           min_F_measurement_dt = 0.01,
                           sendTf = self.tf_listener.SendTransform2tf)
        
        # Robot gripper is supposed to be inside. We reset the target and run error recovery just in case
        r.error_recovery()
        r.ResetCurrentTarget()
        
        
        return 0
    
    def step_5_rotate_vertical(self):
        # Now the tool SHOULD BE within the smoke detector. Rotate until z-axis is vertical. Then, rotate by tool z-axis twice
        # Now rotate tool until z-axis is vertical
        
        r = self.r
        
        SMOKE_DETECTOR_R = np.eye(3)@rot_x(180, unit='deg')@rot_z(90, unit='deg') # HARDCODED! Get info from vision

        # Get relative rotation between 
        dR = np.linalg.solve(r.R, SMOKE_DETECTOR_R)

        qn = r2q(r.R@dR)

        if self.tf_listener is not None:
            self.tf_listener.SendTransform2tf(p = [0.27, -0.53, 0.175], q = qn, parent_frame = 'panda_2/panda_2_link0', child_frame = 'VIRT_SMOKEDET_POSE')
        
        r.SetCartesianStiff_helper(Kp=[3000, 3000, 3000], Kr = [30,30,30])

        r.GetState()
        T = copy.deepcopy(r.T)
        T[0:3, 0:3] = r.R@dR
        r.CMove(T, 1.5)
        
        # Now we make the table gripper softer
        
        self.table_gripper.close(1.3)
        
    def step_6_B(self):
        """ Using move_until_contact to push into the smoke detector while """
        
        r = self.r
        
        r.error_recovery()

        self.table_gripper.close(1.8)

        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')

        r.SetCartesianStiff_helper(Kp = [3000,3000,3000], Kr = [40,40,40])

        r.error_recovery()

        additional_break_fns = {break_on_vsg_position: {'vsg_gripper_obj': self.table_gripper, 'low_threshold':0.15, 'high_threshold': None}}
        additional_break_fns = {}

        r.GetState()
        zeroing_FT = copy.deepcopy(r.FT)

        move_until_contact(r,
                           task_space = 'Tool',
                           mv_unit_direction = [0, 0, 1],
                           mv_velocity = 0.015,
                           rot_unit_direction= [0,0,-1],
                           rot_velocity = 25,
                           dt = None,
                           axes_to_monitor = [1,0,0,0,0,0],
                           max_allowed_values = [10,0,10,2,2,2],
                           use_abs = True,
                           max_allowed_t = 25,
                           allowed_tool_angles = (None,None),
                           zeroing_FT = zeroing_FT,
                           min_F_measurement_dt = 0.01,
                           sendTf = self.tf_listener.SendTransform2tf,
                           additional_break_fns = additional_break_fns)
        
    def step_6_wiggle_and_rotate(self):
        
        r = self.r
        r.error_recovery()
        
        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')
        
        wiggle_while_rotating(robot = self.r, rot_tool_x = -20, rot_tool_z = 20)
        
    def step_7_wiggle_and_push_down(self, n_loops = 3):
        
        r = self.r
        r.error_recovery()
        
        if r._control_strategy != 'CartesianImpedance':
            r.Switch_controller(start_controller = 'CartesianImpedance')
        
        wiggle_and_push_down(robot = r, n_loops=n_loops)
        
        r.error_recovery()
    
    def step_8_almost_vertical_and_push_down(self, deg = 25, n_wiggles = 3):
        
        r = self.r
        
        r.error_recovery(reset_target = False)
        self.step_5_rotate_vertical()
        
        # Make table gripper weaker
        self.table_gripper.close(self.table_gripper_soft_grip_command)
        
        d_rpy = np.array([0, 0, deg])*np.pi/180
        r.CMoveFor(rpy2r(d_rpy), t = 1, task_space='Tool')
        
        #r.CMoveFor([0,0,0.05], t=2, task_space = 'Tool')
        self.step_7_wiggle_and_push_down(n_wiggles)
    
    def perform_full_sequence_A(self):
        """ Performs the KaloRemotus opening sequence consisting of
        - joint move to initial position
        - switch to cartesian controller and moving to initial cartesian position
        - using move_until_contact with low Z stiffness, to detect the gap in the smokedet
        - rotating around tool x, to get to the perfect angle for getting into the detector shell
        - rotating around - tool x and pushing into the smoke det
        - moving tool to vertical position
        - steps of wiggling, rotating tool Z and pushing down.
        """
        self.step_1_move_to_init()
        self.step_2_cartesian()
        self.step_3_detect_gap()
        self.step_4_rotate_while_pushing_down()
        self.step_5_rotate_vertical()
        self.step_6_wiggle_and_rotate()
        self.step_7_wiggle_and_push_down()
        self.step_8_almost_vertical_and_push_down()
        
    def perform_full_sequence_B(self):
        """ BETTER.
        Performs the KaloRemotus opening sequence, simpler, consisting of
        - joint move to initial position
        - switch to cartesian controller and moving to initial cartesian position
        - using move_until_contact with low Z stiffness, to detect the gap in the smokedet
        - rotating around tool x, to get to the perfect angle for getting into the detector shell
        - rotating around - tool x and pushing into the smoke det
        - moving tool to vertical position
        - steps of wiggling, rotating tool Z and pushing down.
        """
        self.step_1_move_to_init()
        self.step_2_cartesian()
        self.step_3_detect_gap()
        self.step_4_rotate_while_pushing_down()
        self.step_6_B()

        self.table_gripper.close(0.9)
        
def break_on_vsg_position(vsg_gripper_obj, low_threshold = None, high_threshold = None):
    """ 
    vsg_gripper - robotblockset_python.ros.grippers VariableStiffnessGripper
    
    check if gripper position is outside of some bounds:
    
    If gripper position is LOWER than low_threshold, or HIGHER than high_threshold, then return 1
    Otherwise return 0
     """
    assert not ((low_threshold is None) and (high_threshold is None))
    
    pos = vsg_gripper_obj.position
    
    if ((low_threshold is not None) and (pos < low_threshold)):
        return 1

    if ((high_threshold is not None) and (pos > high_threshold)):
        return 1
    
    # If no condition is hit, return 0
    return 0

def break_fn_z(robot, initial_T, dx_dy_dz_threshold = [0,0,0.05], use_absolute = True):
    """ Simple break function to break out of loops when robot has moved by more than dx_dy_dz threshold in robot 
    base frame"""
    
    r = robot
    cur_T = copy.deepcopy(r.T)
    
    dX = cur_T[0:3, -1] - initial_T[0:3, -1]
    
    if use_absolute: dX = np.abs(dX)
        
    if (dX[0]> dx_dy_dz_threshold[0]) or (dX[1]> dx_dy_dz_threshold[1]) or (dX[2]> dx_dy_dz_threshold[2]):
        return 1
    return 0

def wiggle_and_push_down(robot, t = 0.9, n_loops = 3, sendTf = None, move_tool_z = 0.02, rot_tool_z = 45, break_fn = None):
    """ The robot gripper should be inside the smoke detector.
    This function will both rotate it around the tool z-axis(wiggling) while also rotating around tool y.
    
    We take the initial pose as initial_T
    
    Then we construct 2 new poses, T1, T2.  """
    
    # TODO: Specify wiggles per cm.
    wiggles_per_cm = 2
    
    TOOL_FRAME_Z_MOVE_DOWN = move_tool_z
    TOOL_FRAME_Z_ROT = rot_tool_z # deg
    T_WIGGLE = t
    
    r = robot
    
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller = 'CartesianImpedance')
    
    r.SetCartesianStiff_helper(Kp=[3000, 3000, 5000], Kr = [40,40,40])
    
    r.ResetCurrentTarget()
    
    r.GetState()
    initial_T = copy.deepcopy(r.T)
        
    dT1 = np.eye(4)
    dT1[0:3, -1] = [0,0,TOOL_FRAME_Z_MOVE_DOWN]
    dT1[0:3,0:3] = rot_z(TOOL_FRAME_Z_ROT, unit='deg')
    
    dT2 = np.eye(4)
    dT2[0:3, -1] = [0,0,TOOL_FRAME_Z_MOVE_DOWN]
    dT2[0:3,0:3] = rot_z(-2*TOOL_FRAME_Z_ROT, unit='deg')
    
    for i in range(0,n_loops):
        r.GetState()
        cur_T = copy.deepcopy(r.T)
        
        r.CMove(cur_T@dT1, t=T_WIGGLE)
        #r.ResetCurrentTarget()
        r.GetState()
        cur_T = copy.deepcopy(r.T)
        r.CMove(cur_T@dT2, t=1.5*T_WIGGLE)
        
        if break_fn is not None:
            if break_fn(robot) ==1:
                break
        #r.ResetCurrentTarget()
    
    return 0

def wiggle_while_rotating(robot, rot_tool_x = -20, rot_tool_z = 20, t = 1, sendTf = None):
    """ The robot gripper should be inside the smoke detector.
    This function will both rotate it around the tool z-axis(wiggling) while also rotating around tool y.
    
    We take the initial pose as initial_T
    
    Then we construct 2 new poses, T1, T2.  """
    
    r = robot
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller = 'CartesianImpedance')
    
    r.ResetCurrentTarget()
    
    r.SetCartesianStiff_helper(Kp=[3000, 3000, 5000], Kr = [40,40,40])

    r.GetState()
    initial_T = copy.deepcopy(r.T)
        
    dR1 = np.eye(3)@rot_x(rot_tool_x, unit = 'deg')@rot_z(rot_tool_z, unit = 'deg')
    
    dR2 = np.eye(3)@rot_x(rot_tool_x, unit = 'deg')@rot_z(-2*rot_tool_z, unit = 'deg')
        
    r.CMoveFor(dR1, t = t, task_space = 'Tool') # lever out in one direction
    
    r.CMove(initial_T, t=t, task_space = 'World') # Move to initial pose
        
    r.CMoveFor(dR2, t = t, task_space = 'Tool') # Lever out in second direction
    
    r.CMove(initial_T, t=t, task_space = 'World') # Move to initial pose
    
    #r.CMoveFor(np.transpose(dR2), t = t, task_space='Tool')
    
    #r.CMoveFor(np.transpose(dR1), t = t, task_space='Tool')