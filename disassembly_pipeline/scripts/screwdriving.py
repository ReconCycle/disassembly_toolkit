# Autoreload modules during import so we don't have to restart the kernel after changing the code
import numpy as np
import sys
#import matplotlib.pyplot as plt
#%matplotlib inline

#%load_ext autoreload
#%autoreload 2

# Set print options for numpy arrays
#np.set_printoptions(precision=6,linewidth=110)
#%precision 6

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper

import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import rospy

import warnings
warnings.filterwarnings('ignore')

from disassembly_pipeline.yaml_parse_utils import parse_yaml_to_get_q
from disassembly_pipeline.pneumatics_utils import handle_pneumatics_jaws, move_slider, rotate_vice, move_cutter, activate_cutter_airblower, rotate_vice_eject_clamptray
from disassembly_pipeline.tf_utils import GenericTransformListener
from disassembly_pipeline.vision_utils import set_realsense_height,get_realsense_height
from disassembly_pipeline.multithreading import do_concurrently
from disassembly_pipeline.disassembly_blocks import MoveBlock

from disassembly_pipeline.saved_positions import p1_q1_init,p2_q1_init
from disassembly_pipeline.saved_positions import p1_q_vice_grip_hca, p1_q_above_vice_grip_hca,  p1_q1_above_sl
from disassembly_pipeline.saved_positions import p2_q_next_to_cutter, p2_q_in_cutter#, p2_q_pinpushinit, p2_q_pinpush_position

from disassembly_pipeline.demos_for_guests import demo_armwave, demo_kamera, demo_fastmove
from disassembly_pipeline.Levering_block_v3 import move_until_contact
 
import time
import copy

if __name__ == '__main__':
    q_init = (-0.214172, -0.447403, 0.120433, -2.095568, 0.109083, 1.680900, 0.688680)
    x_init = [ 0.424411, -0.053757,  0.439629,  0.026531, -0.999448,  0.009114, -0.017779]
    
    q1 = (0.008405, 0.712144, 0.624793, -1.251542, -0.420712, 1.882157, 1.409943)
    q2 = (0.000289, 0.931134, 0.626238, -1.280982, -0.541928, 2.018529, 1.427930)
    tocke_j = [q1, q2]
    
    
    # Panda 2 init
    #p2 = init_robot('panda_2', start_controller = 'joint_impedance_controller')
    p2 = panda_ros('panda_2', start_controller = 'position_joint_trajectory_controller', init_frankadesk_gripper_TCP = True)
    p2.SetCollisionBehavior(F=50, T= 20, tq = 30)
    p2.error_recovery()
    sc_grp = VariableStiffnessGripper()
    p2.gripper = sc_grp
    p2.gripper.open()

    p2.GetState()
    p2.ResetCurrentTarget()
    p2.JMove(q = p2._actual_int.q, t = 0.2, traj='poly')
        
    sleept = 0

    r = p2
    r.error_recovery()
    r.ResetCurrentTarget()
    if r._control_strategy != 'JointPositionTrajectory':
        r.Switch_controller(start_controller = 'JointPositionTrajectory')
        
        
    #old_tcp = copy.deepcopy()

    r.JMove(q_init, 2, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
    
    #ime.sleep(sleept)
    for tocka in tocke_j:
        r.JMove(tocka,2)
        #ime.sleep(sleept)

    r.ResetCurrentTarget()
    r.error_recovery()
    if r._control_strategy != 'CartesianImpedance':
        r.Switch_controller(start_controller = 'CartesianImpedance')
        
    r.CMoveFor([0.0025, 0.001,0], 1)
    move_until_contact(r,
                    task_space = 'World',
                    mv_unit_direction = [0, 0, -1],
                    mv_velocity = 0.01,
                    rot_unit_direction= None,
                    rot_velocity = 0.01,
                    dt = None,
                    axes_to_monitor = [0,0,1,1,0,0],
                    max_allowed_values = [5,5,7,2,2,2],
                    use_abs = True,
                    max_allowed_t = 10,
                    allowed_tool_angles = (None,None),
                    zeroing_FT = [0, 0,0,0,0,0],
                    min_F_measurement_dt = 0.01)
    if 1:
        time.sleep(0.5)
        # Remember the robot position
        r.GetState()
        r.ResetCurrentTarget()
    
        start_x_screwing = copy.deepcopy(r._command_int.x)
        r.GoTo_X(start_x_screwing, np.zeros(6), trq = [0,0,-10,0,0,0], wait=0)

        move_until_contact(r,
                        task_space = 'World',
                        mv_unit_direction = None,
                        mv_velocity = 0.01,
                        rot_unit_direction= [1,0,0],
                        rot_velocity = 3,
                        dt = None,
                        axes_to_monitor = [0,0,1,1,0,0],
                        max_allowed_values = [5,5,5,2,2,2],
                        use_abs = True,
                        max_allowed_t = 10,
                        allowed_tool_angles = (None,None),
                        zeroing_FT = [0, 0,0,0,0,0],
                        min_F_measurement_dt = 0.01)


        # Gremo v predzadnjo tocko 
        r.JMove(tocke_j[-2],2.5)


        #r.CMoveFor([0.05,0,0], 2)
        #r.CMoveFor([0,0.05,0], 2)
        #r.CMoveFor([-0.05,0,0], 2)
        #r.CMoveFor([0,-0.05,0], 2)
            
        #r.CMove(x1,2)
