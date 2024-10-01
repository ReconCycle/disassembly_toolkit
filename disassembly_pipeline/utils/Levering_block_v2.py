#!/usr/bin/env python3

from ..dmp.dmp_periodic import PeriodicDMP
from .dmp_plotting import FT_plotter
#from .levering_utils import EE_to_fulcrum_dist, chkros

import matplotlib.pyplot as plt
import rospy
import time
import os
import numpy as np
import copy
#import sys
#sys.path.append('.robotblockset_python')

from robotblockset_python.robots import robot
from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *

class PDMP_levering_controller(object):
    def __init__(self, robotblockset_robot_object, initial_angle_ampl = 10, r = 1, max_vel = 0.5, max_acc = 0.5, max_allowed_t = 10):
        """ Args:
            robotblockset_robot_object - an object of class Robotblockset_python
            # so we can control the robot in this class
        """
        # We can call for example
        #self.robot_object.CMoveFor for relative moves
        self.robot = robotblockset_robot_object
        self.initial_angle_ampl = initial_angle_ampl
        self.r = r
        
        self.max_vel = max_vel # Passed to JMove and SentJointTrapVelGoal.
        self.max_acc = max_acc
        self.max_allowed_t = max_allowed_t
                
        self.min_print_dt = 0.5 # s between prints
        self.last_print_t = time.time()
        
        self.min_F_msr_dt = 0.01 # How often we sample the fores
        self.last_F_sample_t = time.time()
    
        self.encoded_dmp = None
        
        self.ref_F_signal = 'K_F_ext_hat_K' # K means from stiffness frame (panda_K). 
        # can also use O_F_ext_hat_K
        
        self.forces_desc = ['F_x', 'F_y', 'F_z', 'M_x', 'M_y', 'M_z']
        self.forces_unit = ['N', 'N', 'N', 'Nm', 'Nm', 'Nm']
        
        self.tmp_F_arr =np.zeros(6)
        
        self.DETECTED_SUCCESS = 0
        
    def move_until_contact(self, mv_direction = [0.001, 0, 0], rot_direction= None,dt = 0.01,
                           axes_to_monitor = [0,0,1,1,0,0],
                           max_allowed_values = [13,13,13,2,2,2], use_abs = True,
                           max_allowed_t = 10, allowed_tool_angles = (None,None),
                           zeroing_FT = [0, 0,0,0,0,0], **kwargs):
        """Function to move in world coords until detecting contact based on either Forces or Moments 
        Input args:
                   max_allowed_t     max amount of time that movement will take, if no contact detected after we return 0"""
                   
        start_t = time.time()
        T = self.robot.T
        contact = 0
        F_arr = np.zeros((6,50000))
        F_n = 0
        forces, feeling_contact = self.estimate_if_contact(axes_to_monitor, use_abs = use_abs, max_allowed_values =max_allowed_values, zeroing_FT = zeroing_FT)
        
        # chkros() used to be in the while loop
        while (not feeling_contact):
            self.robot.GetState()
            
            cur_t = time.time()
            #if self.estimate_if_contact(axes_to_monitor, use_abs = use_abs, max_allowed_values =max_allowed_values)
            #self.robot.CMoveFor([0,0,-0.005], t=0.5, task_space = 'World')
            
            if mv_direction is None:
                cur_rpy = r2rpy(T[0:3,0:3], out= 'deg')
                cur_rpy += rot_direction
                
                T[0:3, 0:3] = rpy2r(cur_rpy, unit='deg', out='R')
        
            else:
                T[0:3, 3] += mv_direction # Decrease z height
            
            self.robot.CMove(x=T,t=dt, task_space = 'World')
            #self.robot.GoTo_T(T=T, v =np.zeros((1,6)), FT = np.zeros((1,6)), wait=dt, varargs = None)
            if (cur_t-start_t) > max_allowed_t:
                print("move_until_contact stopping due to max_allowed_t") 
                return F_arr
            
            
                    
            
            # Sample Forces and append them to array.
            
            if cur_t - self.last_F_sample_t > self.min_F_msr_dt:
                #o = chkros())
                if allowed_tool_angles[0] is not None:
                    self.robot.GetState()
                    Rz = self.robot.R[2,2]
                    if (Rz > allowed_tool_angles[0]) or (Rz < allowed_tool_angles[1]):
                        print("move until contact stopping due to reaching min angle:", Rz)
                        self.DETECTED_SUCCESS = 1
                        return F_arr
                
                forces, feeling_contact = self.estimate_if_contact(axes_to_monitor, use_abs = use_abs, max_allowed_values =max_allowed_values, zeroing_FT = zeroing_FT)
                F_arr[:,F_n] = forces
                F_n+=1
                #F_arr.append(forces)
                self.last_F_sample_t = cur_t
                
        return F_arr

    def estimate_if_contact(self, axes_to_monitor = [0,0,1, 1, 0, 0], max_allowed_values = [10, 10, 10, 1.5, 1.5, 1.5], use_abs = True, zeroing_FT = [0,0,0,0,0,0]):
        """ Function to check if robot end effector is in contact with stuff"""
        
        self.robot.GetState() # Refresh forces
        self.tmp_F_arr = np.array(getattr(self.robot.state, self.ref_F_signal))
        
        self.tmp_F_arr = self.tmp_F_arr-zeroing_FT
            
        if use_abs: self.tmp_F_arr = np.abs(self.tmp_F_arr)
        
        #print(forces[3:])
        for i in range(0,6):
            force_val = self.tmp_F_arr[i]
            
            if (axes_to_monitor[i]) and (force_val> max_allowed_values[i]):
                print("Stopping cuz %s = %.3f %s"%(self.forces_desc[i], force_val, self.forces_unit[i]))        
                return self.tmp_F_arr, 1 
            
        tt = time.time()
        if tt-self.last_print_t > self.min_print_dt:
            self.tmp_F_arr = np.array(self.tmp_F_arr, dtype = np.float16)
            print("Fs:", self.tmp_F_arr)
            self.last_print_t = tt
        # If no conditions are hit, smooth sailing, return 0
        return self.tmp_F_arr, 0
   
    def init_dmp(self):
        T= 10
        D_T = 0.001
        num_weights = 25

        time_vec = np.arange(0, T, D_T)
        traj = np.array([self.initial_angle_ampl*np.sin(2*np.pi*time_vec/time_vec[-1]),
                                ]).transpose()

        # Encode the DMP
        if self.encoded_dmp is None:
            self.encoded_dmp = PeriodicDMP(traj, time_vec, num_weights = num_weights, r = self.r)
       
        # Decode the DMP
        dec_traj, _, _ = self.encoded_dmp.decode()

        print(np.min(dec_traj), np.max(dec_traj))
        assert np.min(dec_traj)> -1.3*self.initial_angle_ampl
        assert np.max(dec_traj)< 1.3*self.initial_angle_ampl
        self.plot_dmp(traj, dec_traj)
        return dec_traj, D_T
 
    def plot_dmp(self, traj, dec_traj):
        fig = plt.figure()
        plt.grid()
        plt.plot(dec_traj[:,0])
        plt.plot(traj[:,0])
        plt.savefig("graphs/dmp.jpg", dpi = 300)
        return 0

class LeverBlock:
    def __init__(self, robot,end_effector, to_module, initial_cart_pos, direction, bbox, neutral_rot_vertical = 0, max_allowed_t = 30, allowed_tool_angles = (None,None), speed_factor=1):
        
        assert end_effector in ['softclaw', 'screwdriver']
        self.robot = robot 
        self.end_effector = end_effector
        self.to_module = to_module
        self.cart_pos = initial_cart_pos
        self.max_allowed_t = max_allowed_t
        self.sf = speed_factor # Speed factor to speed up moves
        
        assert type(allowed_tool_angles) == tuple
        self.allowed_tool_angles = allowed_tool_angles # If this is in, we will perform levering AS LONG AS the current tool angle is WITHIN these allowed tool angles.
        # If this is 'out', we will perform levering as long as the current tool angle is OUT of these allowed angles.
        if self.allowed_tool_angles[0] is not None:
            assert self.allowed_tool_angles[0] < self.allowed_tool_angles[1]
        
        #####
        self.plotter = FT_plotter(self.robot, dt = 0.05)
        self.F_contact_threshold = 5
        self.M_contact_threshold = 10 # 2.2
        
        self.PDMP_M_THR_CONDITION = 1
        
        thr = np.concatenate((np.ones(3)* self.F_contact_threshold, np.ones(3)*self.M_contact_threshold))
        #self.z_FT_limits = [5,5,5,4,4,4]
        #self.x_FT_limits = [5,5,5,2,2,2]
        #self.fulcrum_FT_limits = [12,8,8,1.5,1.5,1.5]
        self.z_FT_limits = thr
        self.x_FT_limits = thr
        self.fulcrum_FT_limits = thr
        
        self.max_vel = 0.5
        self.max_acc = 0.5
        self.default_dx = 0.0001* self.sf # How much we move in single iteration of trying to detect contact
        self.default_rot_drpy = 0.052*self.sf
        self.dx_default_dt = self.robot.tsamp
        self.drpy_default_dt = self.robot.tsamp
        
        self.pdmp_dt_factor = 10 # Decrease to increase speed of pdmp
        self.pdmp_n_factor = 35 # Increase to move faster
        
        self.last_print_t = time.time()
        self.min_print_dt = 1 # 1 s between prints or more
        
        self.rot_sign = 1 # Change this to change the direction of rotation
        # 
        
        if end_effector == 'softclaw':
            self.hardcoded_tool_ang = -90 # softclaw
            self.rot_sign = -1
        else:
            self.hardcoded_tool_ang = 0
        self.Z_angle_deg = direction
        self.Z_angle_rad = self.Z_angle_deg * np.pi / 180
        
        # If the levering point is in center of robot workspace, we can set the tool to be vertical. otherwise not.
        self.neutral_rot_vertical = neutral_rot_vertical
        if self.neutral_rot_vertical:
            self.neutral_rot = rpy2r([0+self.hardcoded_tool_ang+ self.Z_angle_deg, 0, 180], unit='deg', out='R')
        else:
            cur_rot = r2rpy(self.robot.T[0:3, 0:3], out = 'deg')
            cur_rot[0] = self.hardcoded_tool_ang + self.Z_angle_deg
            self.neutral_rot = rpy2r(cur_rot, unit='deg', out = 'R')
        
        self.lv = PDMP_levering_controller(self.robot, max_vel = self.max_vel, max_acc = self.max_acc)
        
        
        ## Create arrays
        self.F_arr = []
        
        # What to actually perform
        self.IF_go_to_init = 1
        self.IF_perform_Zminus_search = 1 
        self.IF_perform_Xplus_search = 1
        self.IF_perform_fulcrum_detect_rotation = 1
        self.IF_perform_PDMP = 1

        assert self.rot_sign in [-1, 1] # Must be -1 or +1
        
        self.recorded_Ts = [] # Record positions for later adaptation
        
    def success_condition_angle(self):
        """ Calc if tool angle is such that we estimate success """
        self.robot.GetState()
        Rz = self.robot.R[2,2]
        if (Rz > self.allowed_tool_angles[0]) or (Rz <  self.allowed_tool_angles[1]):
            print("stopping due to reaching min angle:", Rz)
            self.DETECTED_SUCCESS = 1
            return 1
        return 0
    
    def on_enter_adaptive(self, hca_type = None, force_set_params = False, T_rel_1 = None, T_rel_2 = None):
        """Adaptive version of levering. Get levering params from rosparam server and just do the move. """
        assert hca_type in ['qundis', 'kalo']
        
        robot = self.robot
        # Do the move
        robot.ResetCurrentTarget()
        robot.GetState()
        x = robot.x
        #robot.CMoveFor([0,0,0.1],2)
        cur_T = x2t(x)
        
        if T_rel_1 is not None:
            dT_cart = T_rel_1
            dT_rot = T_rel_2
            
        else:
            name_dp = '/' + hca_type + '/' + 'd_position' # The tool move from initial position 
            name_rot = '/' + hca_type + '/' + 'd_angle' # How many degrees to rotate around tool y axis
            
            
            try:
                dp = rospy.get_param(name_dp)
                dr = rospy.get_param(name_rot)
            except KeyError:
                # If values are not set, set them now
                try:
                    if hca_type == 'kalo':
                        rospy.set_param(name_dp, [0, -0.015, 0.035])
                        rospy.set_param(name_rot, 70)
                    elif hca_type == 'qundis':
                        rospy.set_param(name_dp, [0, -0.00, 0.03])
                        rospy.set_param(name_rot, 50)
                except:
                    rospy.loginfo("Error setting levering parameters")
                # Now the params should be set so we can get them
                dp = rospy.get_param(name_dp)
                dr = rospy.get_param(name_rot)
                        
            dT_cart = copy.deepcopy(cur_T)
            
            dT_rot = copy.deepcopy(cur_T)
            
            dT_cart[0:3,-1] = dp
            dT_cart[0:3,0:3] = np.eye(3)
            print("rotating by deg:", dr)
            dT_rot[0:3,0:3] = rot_x(dr, unit='deg')
            dT_rot[0:3, -1] = 0
        
        dT_rot[1, -1] = -0.01 # For the robot to move a bit in Y tool direction while turning, to keep contact
        
        rospy.loginfo("Performing adaptive levering")
        
        if robot._control_strategy != 'CartesianImpedance':
            robot.Switch_controller(start_controller = 'CartesianImpedance')
            time.sleep(1)
            
        robot.ResetCurrentTarget()
        robot.error_recovery()
        robot.SetCartesianStiff_helper(m=1, n = 0.8)
        
        # Open the gripper
        robot.gripper.open(sleep=False)
        # First do the cartesian move
        target_T = cur_T@dT_cart
        robot.CMove(target_T, t = 2/self.sf)
        
        # Close the gripper so it doesnt jump out
        robot.gripper.close(0.8, sleep=False);time.sleep(0.2)
        
        # Then do the rotation
        robot.ResetCurrentTarget()
        #robot.GetState()
        robot.error_recovery()
        
        robot.SetCartImpContNullspace(robot.q,[0,0,0,0,0,0,0])
        robot.SetCartesianStiff_helper(m=1, n = 0.85)


        rospy.loginfo("Performing adaptive rotation")
        #robot.GetState()
        cur_T = x2t(robot.x)
        #time.sleep(1)
        target_T = cur_T@dT_rot
        rospy.loginfo("Adaptive levering : p2 active:{}".format(robot.ACTIVE))
        robot.CMove(target_T, t = 3/self.sf)

        #robot.gripper.close(1, sleep=True)
        
        return 0
        
    def return_adaptation_parameters(self, recorded_Ts = []):
        """ Take the three recorded Ts and convert them to a relative representation:
        - the relative tool move (position change only) from first to second position
        - the relative tool move (rotation only) from second to third position.
        Afterwards, the move is performed such as 
        - Move to init_T
        - move to init_T@T_rel_1
        - move to (init_T@T_rel_1)@T_rel_2"""
        
        if len(recorded_Ts) == 0 :
            recorded_Ts = self.recorded_Ts
        
        assert len(recorded_Ts) == 3
        
        # Get the relative tool move from T1 to T2
        T_rel_1 = np.linalg.solve(recorded_Ts[0], recorded_Ts[1])
        T_rel_1[0:3, 0:3] = np.eye(3) # We want to ignore the rotation here
        
        # Get the relative tool move from T2 to T3
        T_rel_2 = np.linalg.solve(recorded_Ts[1], recorded_Ts[2])
        T_rel_2[0:3, -1] = 0# Only take the rotational difference, EE position should be the same
        
        # Get tool space rotation from T2 to T3
        return T_rel_1, T_rel_2
    
    def on_exit(self):
        """Callback upon levering finish/success. 
        
        -Record the last position after levering success."""
        
        self.robot.GetState()
        self.recorded_Ts.append(self.robot.T)
        
        return self.recorded_Ts
        
        
    def on_enter(self, hca_type, adaptive = False):
        assert hca_type in ['qundis', 'kalo']
        
        #self.robot.SetJointStiffness([1500,1500,1500,600,300,300,300])
        #self.robot.SetJointDamping([30,30,30,30,12,12,12])
        self.robot.ResetCurrentTarget()
        self.robot.error_recovery()
        self.robot.GetState()
        
        if self.IF_go_to_init:
            # 1st step - Move above first position
            print("Getting into specified cart_position and angle")
            pos = self.robot.T
            pos[0:3, -1] = self.cart_pos
            # Try to get into vertical position if so specified. if not, it will stay in the current position
            if self.neutral_rot_vertical:
                pos[0:3, 0:3] = self.neutral_rot
            
            self.robot.CMove(x= pos, t=4/self.sf, max_vel = self.max_vel, max_acc = self.max_acc)
        
    
        self.robot.GetState()
        self.recorded_Ts.append(self.robot.T)
        
        self.robot.StartCapture() # Start capture when robot is at init pos
            
        if adaptive:
            self.on_enter_adaptive(hca_type=hca_type)
            return 0
        
        if self.IF_perform_Zminus_search:
            # 2nd step - move down until feeling contact
            print("starting downward move")
            fs = self.lv.move_until_contact(mv_direction = [0, 0, -self.default_dx], dt = self.dx_default_dt,
                            axes_to_monitor = [0,0,1,1,0,0],
                            max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)

            print("Got out of first move until contact")
            self.robot.error_recovery() # recover if 
            self.robot.GetState()
            pos = copy.deepcopy(self.robot.T) 
            pos[2,3] +=0.001; # raise z by 1 cm to get outta contact
            self.robot.CMove(x=pos,t=1/self.sf, task_space='World')
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
        
        if self.IF_perform_Xplus_search:
            # 3rd step - Try to get contact in x direction (where the levering load is located)
            print("Starting X+ move")
            fs = self.lv.move_until_contact(mv_direction = [self.default_dx*np.cos(self.Z_angle_rad), self.default_dx*np.sin(self.Z_angle_rad), 0], dt = self.dx_default_dt,
                            axes_to_monitor = [1,1,1,1,0,0],
                            max_allowed_values = self.x_FT_limits, max_allowed_t = self.max_allowed_t)
            
            self.robot.GetState()
            T = self.robot.T
            #T[0,3] -= 0.0005 # Decrease x to get out of contact
            #T[2,3] +=0.005 # Raise z by 0.5cm
            #self.robot.CMove(x = T, t = 1)
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
        
        self.robot.SetCartesianStiff_helper(m=1.2,n=0.85)
        
        # Now record this position
        self.robot.GetState()
        self.recorded_Ts.append(self.robot.T)
        
        if self.IF_perform_fulcrum_detect_rotation:
            # 4th step - Zero-in the torque sensors
            time.sleep(0.5)
            self.robot.GetState()
            zeroing_FT = np.array(self.robot.FT)
            print(zeroing_FT)
            
            # 5th step - Try to rotate back and get contact with the hinge/fulcrum
            print("FT sensor zeroed, rotating to detect fulcrum")
            self.robot.ResetCurrentTarget()
            
            # Hack for kalo. this works very well. important to keep contact while rotating.
            #if hca_type == 'kalo':
            #    self.robot.CMoveFor(rot_x(35,unit='deg'),t=2/self.sf, task_space='Tool')
            #    self.robot.CMoveFor([0,0.008,0],t=1/self.sf, task_space='World')
            #    self.robot.gripper.close(0.74, sleep=True)

            
            self.robot.CMoveFor([0.008*np.cos(self.Z_angle_rad),0.008*np.sin(self.Z_angle_rad),0],t=1/self.sf, task_space='World') # Move to get better contact
            self.robot.gripper.close(0.75)
            
            fs = self.lv.move_until_contact(mv_direction = None, rot_direction = [0,0,-self.default_rot_drpy * self.rot_sign],
                            dt = self.drpy_default_dt,
                            axes_to_monitor = [0,0,0,1,1,1],
                            max_allowed_values = self.fulcrum_FT_limits, use_abs = True, max_allowed_t=2*self.max_allowed_t,
                            zeroing_FT = zeroing_FT, allowed_tool_angles = self.allowed_tool_angles)
            
            #self.robot.gripper.close(0.74) # Close gripper so the hca doesnt jump out
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
            # Move in neg Z to assert contact
            #fs = self.lv.move_until_contact(mv_direction = [0, 0, -self.default_dx], dt = self.dx_default_dt,
            #                axes_to_monitor = [0,0,1,1,0,0],
            #                max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)
            
            
            
            # 6th step - rotate further and try to detect precise fulcrum location. afterwards change the TCP to be above the fulcrum.
            # Record EE position as EE_1
            self.robot.GetState()
            EE_1 = self.robot.T
            # Try to get second position and fulcrum location. 
            print("rotating further to detect precise fulcrum location")
            #self.robot.GetState()
            #self.robot.ResetCurrentTarget()
            #self.robot.error_recovery()
            
            # Record position
            EE_1 = self.robot.T
            # Increase max force slightly and rotate some more
            max_forces_rotation_2 = np.array(self.fulcrum_FT_limits)*1.2

            fs = self.lv.move_until_contact(mv_direction = None, rot_direction = [0,0, -self.default_rot_drpy * self.rot_sign],
                            dt = self.drpy_default_dt,
                            axes_to_monitor = [0,0,0,1,1,1],
                            max_allowed_values = max_forces_rotation_2, use_abs = True, max_allowed_t=2*self.max_allowed_t,
                                    zeroing_FT = zeroing_FT, allowed_tool_angles = self.allowed_tool_angles)
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
            # Move in neg Z to assert contact
            #fs = self.lv.move_until_contact(mv_direction = [0, 0, -self.default_dx], dt = self.dx_default_dt,
            #                axes_to_monitor = [0,0,1,1,0,0],
            #                max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)

            self.robot.GetState()
            EE_2 = self.robot.T # Record the second position

            # Calculate the point of the fulcrum
            EE_T_K = self.robot.state.EE_T_K
            xyz, z0_dist = EE_to_fulcrum_dist(EE_1, EE_2,EE_T_K)
            #print("HARDCODED z0_dist levering")
            #z0_dist = 0.02
            print("fulcrum xyz:", xyz,"- z0 dist:", z0_dist)
            #return xyz, z0_dist

            # NE dela neki dobr.
            #old_RBS_TCP = self.robot.TCPGripper
            #new_RBS_TCP = old_RBS_TCP.copy()
            #new_RBS_TCP[2, -1] -= z0_dist
            #self.robot.TCPGripper = new_RBS_TCP
            
            #self.robot.SetTCP()
            #self.robot.ResetCurrentTarget()
            
            #change_TCP_z(p, dz = -0.03)
            
            # Move until contact in Z direction again.
        if self.lv.DETECTED_SUCCESS==1:
            rospy.loginfo("Breaking out of levering because success was detected by move_until_contact")
            self.on_exit()
        
        if (self.IF_perform_PDMP and (self.lv.DETECTED_SUCCESS == 0)):
            
            self.robot.SetCartesianStiff_helper(m=1, n=0.95)
            #time.sleep(1.5)
            # While testing, set pose to current pose
            self.robot.GetState()
            self.robot.ResetCurrentTarget()
            pos = self.robot.T
            # Initial tool angle
            # If we know that we rotate around Z of RPY
            start_angle = (r2rpy(self.robot.R)*180/np.pi)[2]

            # running the DMP
            dec_traj, D_T = self.lv.init_dmp()

            n = 0
            prev_angle = self.rot_sign * -dec_traj[0]
            rospy.loginfo("PDMP sending commands with dt= {} s".format(D_T*self.pdmp_dt_factor))

            print("Starting P-DMP execution")
            self.plotter.execute(d_ang = 0, cond = 0)
            while (rospy.is_shutdown() == 0) and (n<dec_traj.shape[0]):
                
                # Move tool to the angle specified by Periodic DMP. Occasionally increase r
                angle = -self.rot_sign * -dec_traj[n]
                d_angle = angle - prev_angle
                #print("angle", angle)

                rot = rpy2r([0+self.hardcoded_tool_ang+ + self.Z_angle_deg, 0 , start_angle - angle], unit='deg', out = 'R')
                pos[0:3, 0:3] = rot       
                self.robot.CMove(x = pos, t=D_T*self.pdmp_dt_factor, task_space= 'World')
                #print("PDMP d_t is  ", D_T*self.pdmp_dt_factor)
                #time.sleep(D_T*self.pdmp_dt_factor)

                prev_angle = angle
                n +=1 *self.pdmp_n_factor       

                # If M_x dropped greatly WHILE we were doing the levering 
                cond = self.plotter.condition()
                self.plotter.execute(d_ang = d_angle, cond = cond)
                
                #if (d_angle > 0) and (cond > self.PDMP_M_THR_CONDITION):
                    #dx = [0, 0, -0.03]
                    #self.robot.CMoveFor(dx,t = 2, task_space = 'Tool')
                    #dx = [0, 0, 0.05]
                    #self.robot.CMoveFor(dx,t = 2, task_space = 'World')
                    
                    #print("Detected levering success due to condition")
                    #print("condition:{}, d_angle: {}".format(cond,d_angle))
                    #break
                
                if self.success_condition_angle() == 1:
                    print("Detected levering success due to angle")
                    break
                

                if n>=dec_traj.shape[0]-1:
                    ampl = self.lv.initial_angle_ampl +4
                    print("Increasing dmpl ampl to",ampl)
                    # We didnt manage to finish the levering procedure
                    n = 0
                    self.lv.initial_angle_ampl = ampl # + 2 degs
                    print("Prev goal:", self.lv.encoded_dmp.goal[0])
                    self.lv.encoded_dmp.goal[0] += 4 # deg
                    print("New goal:", self.lv.encoded_dmp.goal[0])
                
                    
                    dec_traj, D_T = self.lv.init_dmp()
                    #plt.plot(dec_traj)
                    
                # Printing
                tt = time.time()
                if tt - self.last_print_t > self.min_print_dt:
                    print("rot angle:", r2rpy(self.robot.T, out='deg')[2])
                    print("cond:", cond)
                    self.last_print_t = tt

                # Try to move slightly in the tool positive Z-direction
                # p.CMoveFor(dx = [0,0,0.001], t= 0.2, task_space = 'Tool', FT = [5,5,5, 2,2,2])

            self.plotter.on_exit()
            self.on_exit()
            
    def plot_forces(self):

        Fs = self.F_arr
        idxs = [Fs[i].shape[0] for i in range(0,len(Fs))]
        movement_end_idx = np.cumsum(idxs)

        combined_Fs = np.vstack(Fs)
        labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        plt.figure(figsize=(15,10))
        prv_axh = 0
        for i in range(0,combined_Fs.shape[1]):
            plt.plot(combined_Fs[:,i], label = labels[i])
            try:plt.axvline(movement_end_idx[i]-1)
            except:pass
        plt.legend(loc='upper left')
        plt.grid()
        plt.savefig('graphs/levering_Forces.jpg', dpi = 300)
        
        
if __name__ == '__main__':
    
    robot_name = 'panda_2'
    panda_1 = panda_ros(name = robot_name)

   
