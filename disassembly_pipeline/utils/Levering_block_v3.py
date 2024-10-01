#!/usr/bin/env python3

from ..dmp.dmp_periodic import PeriodicDMP
from .dmp_plotting import FT_plotter

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

from franka_msgs.msg import FrankaState as fs # import because break condition for move_until_contact checks if robot entered reflex (r.state.robot_mode != fs.ROBOT_MODE_MOVE).

last_estimate_if_contact_print_time = 0

class LeverBlock:
    def __init__(self,
                 robot = None,
                 end_effector = 'softclaw',
                 to_module = None,
                 direction = 0,
                 bbox = None,
                 initially_position_gripper_vertically = True,
                 F_contact_threshold = 10,
                 T_contact_threshold = 3,
                 max_allowed_t = 30,
                 allowed_tool_angles = (None,None),
                 speed_factor=1):
        
        assert end_effector in ['softclaw', 'screwdriver']
        r = robot 
        self.dx_default_dt = None
        if robot is not None:
            self.dx_default_dt = r.tsamp
            self.drpy_default_dt = r.tsamp
        self.end_effector = end_effector
        self.to_module = to_module
        self.max_allowed_t = max_allowed_t
        self.sf = speed_factor # Speed factor to speed up moves
        
        assert type(allowed_tool_angles) == tuple
        self.allowed_tool_angles = allowed_tool_angles # If this is in, we will perform levering AS LONG AS the current tool angle is WITHIN these allowed tool angles.
        # If this is 'out', we will perform levering as long as the current tool angle is OUT of these allowed angles.
        if self.allowed_tool_angles[0] is not None:
            assert self.allowed_tool_angles[0] < self.allowed_tool_angles[1]
        
        #####
        self.plotter = FT_plotter(r, dt = 0.05)
        self.F_contact_threshold = F_contact_threshold
        self.M_contact_threshold = T_contact_threshold
        
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
        self.initially_position_gripper_vertically = initially_position_gripper_vertically
        if self.initially_position_gripper_vertically:
            self.neutral_rot = rpy2r([0+self.hardcoded_tool_ang + self.Z_angle_deg, 0, 180], unit='deg', out='R')
        else:
            cur_rot = r2rpy(r.T[0:3, 0:3], out = 'deg')
            cur_rot[0] = self.hardcoded_tool_ang + self.Z_angle_deg
            self.neutral_rot = rpy2r(cur_rot, unit='deg', out = 'R')

        self.lv = PDMP_levering_controller(r, max_vel = self.max_vel, max_acc = self.max_acc)
        
        ## Create arrays        
        if self.rot_sign not in [-1, 1]: raise ValueError("Rotation sign must be +1 or -1.") # Must be -1 or +1
        
        self.F_arr = []
        self.recorded_Ts = [] # Record positions for later adaptation

    def on_enter(self,
                 robot = None,
                 hca_type = 'kalo',
                 starting_cartesian_pose = [-0.04780933, -0.60115153,  0.21001202,  0.04478305, -0.01314394, 0.99878274,  0.01596091],
                 max_allowed_lever_rotation_angle_deg_from_vertical = -60,
                 perform_adaptive_levering_using_known_positions = False,
                 step_1_move_above_init_pose = True,
                 step_2_perform_z_minus_search = True,
                 step_3_perform_x_plus_search = True,
                 step_4_perform_fulcrum_detect_rotation = True,
                 step_4_perform_PDMP = False,
                 linear_mv_velocity = 0.05,
                 rot_velocity = 7):
        """ 
        max_allowed_lever_rotation_angle_deg_from_vertial: If starting from vertical (or from wherever),
                                                           what is the maximum angle in degrees around gripper Y axis(VSGripper levers around Y), so that levering is considered successful.
                                                           E.g if you write max_allowed_lever_rotation_angle_deg_from_vertial=90, levering will stop when lever is horizontal.
        
        """
        if self.dx_default_dt == None:
            self.dx_default_dt = robot.tsamp
            self.drpy_default_dt = robot.tsamp

        assert hca_type in ['qundis', 'kalo']

        if robot is None:
            robot = r
        r = robot
        self.robot = r

        #r.SetJointStiffness([1500,1500,1500,600,300,300,300])
        #r.SetJointDamping([30,30,30,30,12,12,12])
        r.ResetCurrentTarget()
        r.error_recovery()
        r.GetState()
        
        if step_1_move_above_init_pose:
            # 1st step - Move above first position
            print("Getting into specified cart_position and angle")
            pos = starting_cartesian_pose
            # pos = r.T
            # pos[0:3, -1] = starting_cartesian_pose
            # Try to get into vertical position if so specified. if not, it will stay in the current position
            if self.initially_position_gripper_vertically:
                0
                #pos[0:3, 0:3] = self.neutral_rot
            r.CMove(x= pos, t=4/self.sf, max_vel = self.max_vel, max_acc = self.max_acc)

        r.GetState()
        self.recorded_Ts.append(r.T)

        self.initial_robot_control_strategy = copy.deepcopy(r._control_strategy) # Switch to initial controller in on_exit
        if r._control_strategy != 'CartesianImpedance':
            print("Switching robot to cartesian impedance.")
            r.Switch_controller(start_controller = 'CartesianImpedance')

        # Set stiffness
        #r.SetCartesianStiff_helper(m = 0.7, n = 0.7)
        r.SetCartesianStiff_helper(Kp = [2000, 2000, 100],Kr = [50,50,50],m=1, n=1, D=2, hold_pose = 'on')

        # Record initial pose so we can determine success criteria
        r.GetState()
        self.levering_start_R = copy.deepcopy(r.R)

        if perform_adaptive_levering_using_known_positions:
            self.on_enter_adaptive(hca_type=hca_type)
            return 0
        
        if step_2_perform_z_minus_search:
            # 2nd step - move down until feeling contact
            print("starting downward move")
            fs = move_until_contact(robot = r,
                                    mv_unit_direction = [0, 0, -1],
                                    mv_velocity = linear_mv_velocity,
                                    axes_to_monitor = [0,0,1,1,0,0],
                                    max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)

            print("Got out of first move until contact")
            r.error_recovery() # recover if 
            r.GetState()
            pos = copy.deepcopy(r.T) 
            pos[2,3] += 0.001; # raise z by 1 cm to get outta contact
            r.CMove(x=pos,t=1/self.sf, task_space='World')

            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)

        if step_3_perform_x_plus_search:
            # 3rd step - Try to get contact in x direction (where the levering load is located)
            print("Starting X+ move")
            fs = move_until_contact(robot = r, mv_unit_direction = [1*np.cos(self.Z_angle_rad), 1*np.sin(self.Z_angle_rad), 0], dt = self.dx_default_dt,
                            axes_to_monitor = [1,1,1,1,0,0],
                            mv_velocity = linear_mv_velocity,
                            max_allowed_values = self.x_FT_limits, max_allowed_t = self.max_allowed_t)
            
            r.GetState()
            T = r.T
            #T[0,3] -= 0.0005 # Decrease x to get out of contact
            #T[2,3] +=0.005 # Raise z by 0.5cm
            #r.CMove(x = T, t = 1)
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
        
        r.SetCartesianStiff_helper(m=1.2,n=0.85)
        
        # Now record this position
        r.GetState()
        self.recorded_Ts.append(r.T)
        
        if step_4_perform_fulcrum_detect_rotation:
            # 4th step - Zero-in the torque sensors
            time.sleep(0.5)
            r.GetState()
            zeroing_FT = np.array(r.FT)
            print(zeroing_FT)
            
            # 5th step - Try to rotate back and get contact with the hinge/fulcrum
            print("FT sensor zeroed, rotating to detect fulcrum")
            r.ResetCurrentTarget()
            
            # Hack for kalo. this works very well. important to keep contact while rotating.
            if hca_type == 'kalo':
               r.CMoveFor(rot_y(-35,unit='deg'),t=2/self.sf, task_space='Tool')
               r.CMoveFor(dx = [0,0, -0.01], t = 1, task_space = 'World')
            
            r.CMoveFor([0.008*np.cos(self.Z_angle_rad),0.008*np.sin(self.Z_angle_rad),0],t=1/self.sf, task_space='World') # Move to get better contact
            r.gripper.close(0.75)
            
            fs = move_until_contact(robot = r,
                            mv_unit_direction = [0,0, 1],
                            mv_velocity = 0.005,
                            rot_unit_direction = [0,-self.rot_sign,0],
                            rot_velocity = rot_velocity,
                            dt = r.tsamp,
                            axes_to_monitor = [0,0,0,1,1,1],
                            max_allowed_values = self.fulcrum_FT_limits, use_abs = True, max_allowed_t=2*self.max_allowed_t,
                            zeroing_FT = zeroing_FT,
                            max_allowed_lever_rotation_angle_deg_from_vertical = max_allowed_lever_rotation_angle_deg_from_vertical,
                            initial_vertical_levering_R = self.levering_start_R,
                            task_space = 'Tool')
            
            #r.gripper.close(0.74) # Close gripper so the hca doesnt jump out
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
            # Move in neg Z to assert contact
            #fs = self.lv.move_until_contact(mv_direction = [0, 0, -self.default_dx], dt = self.dx_default_dt,
            #                axes_to_monitor = [0,0,1,1,0,0],
            #                max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)
            
            
            
            # 6th step - rotate further and try to detect precise fulcrum location. afterwards change the TCP to be above the fulcrum.
            # Record EE position as EE_1
            r.GetState()
            EE_1 = r.T
            # Try to get second position and fulcrum location. 
            print("rotating further to detect precise fulcrum location")
            #r.GetState()
            #r.ResetCurrentTarget()
            #r.error_recovery()

            # Record position
            EE_1 = r.T
            # Increase max force slightly and rotate some more
            max_forces_rotation_2 = np.array(self.fulcrum_FT_limits)*1.2

            r.error_recovery()
            #r.ResetCurrentTarget()
            fs = move_until_contact(robot = r,
                            mv_unit_direction = [0,0, 1],
                            mv_velocity = 0,
                            rot_unit_direction = [0,-self.rot_sign,0],
                            rot_velocity = rot_velocity,
                            dt = r.tsamp,
                            axes_to_monitor = [0,0,0,1,1,1],
                            max_allowed_values = self.fulcrum_FT_limits*1.6, use_abs = True, max_allowed_t=2*self.max_allowed_t,
                            zeroing_FT = zeroing_FT, 
                            max_allowed_lever_rotation_angle_deg_from_vertical = max_allowed_lever_rotation_angle_deg_from_vertical,
                            initial_vertical_levering_R = self.levering_start_R,
                            task_space = 'Tool')
            #fs = move_until_contact(robot = r, mv_unit_direction = None, rot_unit_direction = [0,0, -self.default_rot_drpy * self.rot_sign],
            #                dt = self.drpy_default_dt,
            #                axes_to_monitor = [0,0,0,1,1,1],
            ##                max_allowed_values = max_forces_rotation_2, use_abs = True, max_allowed_t=2*self.max_allowed_t,
            #                zeroing_FT = zeroing_FT, allowed_tool_angles = self.allowed_tool_angles)

            r.error_recovery()
            
            try:self.F_arr.append(np.vstack(fs))
            except Exception as e:print(e)
            # Move in neg Z to assert contact
            #fs = self.lv.move_until_contact(mv_direction = [0, 0, -self.default_dx], dt = self.dx_default_dt,
            #                axes_to_monitor = [0,0,1,1,0,0],
            #                max_allowed_values = self.z_FT_limits,max_allowed_t= self.max_allowed_t)

            r.GetState()
            EE_2 = r.T # Record the second position

            # Calculate the point of the fulcrum
            EE_T_K = r.state.EE_T_K
            #xyz, z0_dist = EE_to_fulcrum_dist(EE_1, EE_2,EE_T_K)
            #print("HARDCODED z0_dist levering")
            #z0_dist = 0.02
            #print("fulcrum xyz:", xyz,"- z0 dist:", z0_dist)
            #return xyz, z0_dist

            # NE dela neki dobr.
            #old_RBS_TCP = r.TCPGripper
            #new_RBS_TCP = old_RBS_TCP.copy()
            #new_RBS_TCP[2, -1] -= z0_dist
            #r.TCPGripper = new_RBS_TCP
            
            #r.SetTCP()
            #r.ResetCurrentTarget()
            
            #change_TCP_z(p, dz = -0.03)
            
            # Move until contact in Z direction again.
        if self.lv.DETECTED_SUCCESS==1:
            rospy.loginfo("Breaking out of levering because success was detected by move_until_contact")
            self.on_exit()
        
        if (step_4_perform_PDMP and (self.lv.DETECTED_SUCCESS == 0)):
            
            r.SetCartesianStiff_helper(m=1, n=0.95)
            #time.sleep(1.5)
            # While testing, set pose to current pose
            r.GetState()
            r.ResetCurrentTarget()
            pos = r.T
            # Initial tool angle
            # If we know that we rotate around Z of RPY
            start_angle = (r2rpy(r.R)*180/np.pi)[2]

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
                r.CMove(x = pos, t=D_T*self.pdmp_dt_factor, task_space= 'World')
                #print("PDMP d_t is  ", D_T*self.pdmp_dt_factor)
                #time.sleep(D_T*self.pdmp_dt_factor)

                prev_angle = angle
                n +=1 *self.pdmp_n_factor       

                # If M_x dropped greatly WHILE we were doing the levering 
                cond = self.plotter.condition()
                self.plotter.execute(d_ang = d_angle, cond = cond)
                
                #if (d_angle > 0) and (cond > self.PDMP_M_THR_CONDITION):
                    #dx = [0, 0, -0.03]
                    #r.CMoveFor(dx,t = 2, task_space = 'Tool')
                    #dx = [0, 0, 0.05]
                    #r.CMoveFor(dx,t = 2, task_space = 'World')
                    
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
                    print("rot angle:", r2rpy(r.T, out='deg')[2])
                    print("cond:", cond)
                    self.last_print_t = tt

                # Try to move slightly in the tool positive Z-direction
                # p.CMoveFor(dx = [0,0,0.001], t= 0.2, task_space = 'Tool', FT = [5,5,5, 2,2,2])

            self.plotter.on_exit()
            self.on_exit()
            
        
    def success_condition_angle(self):
        """ Calc if tool angle is such that we estimate success """
        r.GetState()
        current_levering_R = copy.deepcopy(r.R)
        dR = np.transpose(self.levering_start_R)@current_levering_R
        angle_deg = np.rad2deg(r2rpy(dR))[1]
        print(angle_deg,self.max_allowed_lever_rotation_angle_deg_from_vertial )
        if angle_deg < self.max_allowed_lever_rotation_angle_deg_from_vertial:
            print("stopping due to reaching min angle:", angle_deg)
            self.DETECTED_SUCCESS = 1
            return 1
        return 0
    
    def on_enter_adaptive(self, hca_type = None, force_set_params = False, T_rel_1 = None, T_rel_2 = None):
        """Adaptive version of levering. Get levering params from rosparam server and just do the move. """
        assert hca_type in ['qundis', 'kalo']
        
        robot = r
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
        
        self.on_exit()
        
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
        
        r.GetState()
        self.recorded_Ts.append(r.T)

        #self.robot.Switch_controller(start_controller = self.initial_robot_control_strategy)
        
        return self.recorded_Ts
        
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
        

def move_until_contact(robot,
                       task_space = 'World',
                       mv_unit_direction = [1, 0, 0],
                       mv_velocity = 0.05,
                       rot_unit_direction= None,
                       rot_velocity = 10,
                       dt = None,
                       axes_to_monitor = [0,0,1,1,0,0],
                       max_allowed_values = [13,13,13,2,2,2],
                       use_abs = True,
                       max_allowed_t = 10,
                       max_allowed_lever_rotation_angle_deg_from_vertical = None,
                       initial_vertical_levering_R = None,
                       zeroing_FT = [0, 0,0,0,0,0],
                       min_F_measurement_dt = 0.01,
                       use_goto_T = False,
                       sendTf = None,
                       additional_break_fns = {},
                       **kwargs):

    """Function to move in world coords until detecting contact based on either Forces or Moments 
    Input args:
                max_allowed_t     max amount of time that movement will take, if no contact detected after we return 0
                
                additional_break_fns: any function in this list will be called each loop. If it returns 1, break out and stop."""

    if task_space not in ["World", "Tool"]: raise ValueError('task_space should be one of ["World", "Tool"]')

    r = robot

    prev_verbose = copy.deepcopy(r._verbose)
    r._verbose = 1 # So CMove_started msg is not shown

    sendTf_parent_frame = r.Base_link_name
    sendTf_child_frame_name = 'MOVE_UNTIL_CONTACT'
    
    if dt is None: dt = r.tsamp
    
    if mv_unit_direction is None:
        mv_unit_direction = [0,0,0]

    mv_direction = (np.array(mv_unit_direction) * mv_velocity) * dt

    if rot_unit_direction is None:
        rot_unit_direction = [0,0,0]

    rot_direction = (np.array(rot_unit_direction) * rot_velocity) * dt

    start_t = time.time()
    last_F_sample_t = start_t
    T = copy.deepcopy(r.T)
    contact = 0
    detected_success = 0
    F_arr = np.zeros((6,50000))
    F_n = 0
    forces, feeling_contact = estimate_if_contact(robot = r, axes_to_monitor= axes_to_monitor, use_abs = use_abs, max_allowed_values = max_allowed_values, zeroing_FT = zeroing_FT)

    # chkros() used to be in the while loop
    while (not feeling_contact):
        r.GetState()

        #T = copy.deepcopy(r.T)

        cur_t = time.time()
        #if self.estimate_if_contact(axes_to_monitor, use_abs = use_abs, max_allowed_values =max_allowed_values)
        #r.CMoveFor([0,0,-0.005], t=0.5, task_space = task_space)
        
        #if mv_direction is None:
        cur_rpy = r2rpy(T[0:3,0:3], out= 'deg')
        cur_rpy += rot_direction
        
        T[0:3, 0:3] = rpy2r(cur_rpy, unit='deg', out='R')
    
        if task_space == 'World':
            T[0:3, 3] += mv_direction
        elif task_space == 'Tool':
            mv_T = np.eye(4)
            mv_T[0:3, 3] = mv_direction
            T = T@mv_T
        
        if use_goto_T:
            r.GoTo_T(T, v = np.zeros(6), FT = np.zeros(6), task_space = 'World', last_move = True)
        else:
            r.CMove(x=T,t=dt, task_space = 'World')

            #sendTf.SendTransform2tf(p = T[0:3, -1], q = r2q(T[0:3, 0:3]), parent_frame = sendTf_parent_frame, child_frame = sendTf_child_frame_name)

        #r.GoTo_T(T=T, v =np.zeros((1,6)), FT = np.zeros((1,6)), wait=dt, varargs = None)

        # Break 1 - timeout
        if (cur_t-start_t) > max_allowed_t:
            print("move_until_contact breaking due to max_allowed_t")

            r._verbose = prev_verbose
            return F_arr, 0
        
        # Break 2 - robot reflex
        if r.state.robot_mode not in [fs.ROBOT_MODE_MOVE]:
                
            r._verbose = prev_verbose
            print("Move_until_contact aborting due to some robot reflex.")
            return F_arr, 0
        
        # Break 3 - all the additional_break_fns
        for key in additional_break_fns.keys():
            fn_output = key(**additional_break_fns[key])
            if fn_output == 1:
                print("Move_until_contact Breaking due to fn {0} with args:\n{1}".format(key, additional_break_fns[key]))
                r._verbose = prev_verbose
                return F_arr, 0

        # Sample Forces and append them to array.
        if cur_t - last_F_sample_t > min_F_measurement_dt:
            #o = chkros())
            if (max_allowed_lever_rotation_angle_deg_from_vertical is not None) and (initial_vertical_levering_R is not None):
                r.GetState()
                current_R = r.R
                dR = np.transpose(initial_vertical_levering_R)@current_R
                angle_deg = np.rad2deg(r2rpy(dR))[1]
                #print("Angle def diff", angle_deg, "max diff", max_allowed_lever_rotation_angle_deg_from_vertical)
                if angle_deg < max_allowed_lever_rotation_angle_deg_from_vertical:
                    print("move until contact breaking due to reaching min angle:", angle_deg)
                    detected_success = 1

                    r._verbose = prev_verbose
                    return F_arr, detected_success
            
            forces, feeling_contact = estimate_if_contact(robot = r, axes_to_monitor= axes_to_monitor, use_abs = use_abs, max_allowed_values = max_allowed_values, zeroing_FT = zeroing_FT)
            detected_success = feeling_contact
            F_arr[:,F_n] = forces
            F_n+=1
            #F_arr.append(forces)
            last_F_sample_t = cur_t
        
    print("move until contact breaking, success detected")
    r._verbose = prev_verbose
    return F_arr, detected_success

def estimate_if_contact(robot,
                        axes_to_monitor = [0,0,1, 1, 0, 0],
                        max_allowed_values = [10, 10, 10, 1.5, 1.5, 1.5],
                        use_abs = True,
                        zeroing_FT = [0,0,0,0,0,0],
                        min_print_dt = 0.1):
        """ Function to check if robot end effector is in contact with stuff"""
        global last_estimate_if_contact_print_time # Not the prettiest but then we don't have to keep this function within a class.
        
        forces_desc = ['F_x', 'F_y', 'F_z', 'M_x', 'M_y', 'M_z']
        forces_unit = ['N', 'N', 'N', 'Nm', 'Nm', 'Nm'] 
        
        r = robot
        
        r.GetState() # Refresh forces
        tmp_F_arr = np.array(getattr(r.state, 'K_F_ext_hat_K')) # can also use O_F_ext_hat_K

        
        tmp_F_arr = tmp_F_arr-zeroing_FT
            
        if use_abs: tmp_F_arr = np.abs(tmp_F_arr)
        
        #print(forces[3:])
        for i in range(0,6):
            force_val = tmp_F_arr[i]
            
            if (axes_to_monitor[i]) and (force_val> max_allowed_values[i]):
                print("Stopping cuz %s = %.3f %s"%(forces_desc[i], force_val, forces_unit[i]))        
                return tmp_F_arr, 1 
            
        tt = time.time()
        if tt-last_estimate_if_contact_print_time > min_print_dt:
            tmp_F_arr = np.array(tmp_F_arr, dtype = np.float16)
            #print("Fs:", tmp_F_arr)
            last_estimate_if_contact_print_time = tt
        ## If no conditions are hit, smooth sailing, return 0
        return tmp_F_arr, 0


class PDMP_levering_controller(object):
    def __init__(self, robotblockset_robot_object, initial_angle_ampl = 10, r = 1, max_vel = 0.5, max_acc = 0.5, max_allowed_t = 10):
        """ Args:
            robotblockset_robot_object - an object of class Robotblockset_python
            # so we can control the robot in this class
        """
        # We can call for example
        #r_object.CMoveFor for relative moves
        r = robotblockset_robot_object
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
        
        self.tmp_F_arr =np.zeros(6)
        
        self.DETECTED_SUCCESS = 0
   
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

        
if __name__ == '__main__':
    
    robot_name = 'panda_2'
    panda_1 = panda_ros(name = robot_name)
   
