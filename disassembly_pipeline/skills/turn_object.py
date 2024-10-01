import numpy as np
import json
import os
import time
import copy
from unified_planning.shortcuts import *

from disassembly_pipeline.utils.tf_utils import GenericTransformListener
from .base_skill import BaseSkill
from robotblockset_python.transformations import *


class FlipObject(BaseSkill):
    def __init__(self, robot = None, 
                 using_controller = 'JointPositionTrajectory',
                 move_above_z = 0.05):
        
        """Use robot to flip an object."""

        self.robot = robot
        #assert robot.gripper.Name in ['softhand', 'vsgripper']
        
        self.using_controller = using_controller
        self.move_above_z = move_above_z
    
        self.tflistener = GenericTransformListener()
        self.tf2x = self.tflistener.tf2x
        
    def on_enter(self, **kwargs):

        r = kwargs['robot']

        return 0

    def execute(self):
        0
    def on_exit(self):
        0


def robot_flip_object(self, robot, init_safe = True):
        """ Shitcode. Delete all and make new code later.
        Flipping a generic object is a useful skill..
        If we only detect hca_front, we might want to flip it using the softclaw """

        r = robot

        assert r.gripper.Name in ['softclaw', 'vsgripper']

        # Flip parameters
        # The parameters are a function of (gripper_type, object_type), where object type is kalo HCA, qundis HCA, smoke detector A, $
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
