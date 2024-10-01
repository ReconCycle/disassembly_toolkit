import rospy
import numpy as np

#from robotblockset_python.transformations import *

def time_to_move_to_x(robot, new_position, speed_factor=1):
    """pos_to_rot_factor: determines how much the move_time is influenced by position error and rotation error"""

    controller = robot._control_strategy

    pct_of_max_velocity = 0.1 # how fast relative to robot max velocities we move, if "speed==1"
    robot.GetState()

    assert 0.05<=speed_factor<=4
    assert controller in ['CartesianImpedance','JointImpedance']
    if controller=='CartesianImpedance':
        assert type(new_position) in [list, np.ndarray]
        new_position=np.array(new_position) # Make into an array
        if new_position.shape==(4,4):
            new_position = t2x(new_position)
        elif new_position.shape==(7,):
            0
        else:
            rospy.loginfo("time_to_move_to_x: Invalid new_position shape")
            return 1

        xdot = robot.v_max * pct_of_max_velocity
        x_err = xerr(robot.x,new_position)    # Difference x2-x1

        at_move = x_err/xdot
        t_move = np.amax(np.abs(at_move))/speed_factor + 0.1 # 0.1 for safety
        #len_p_err = np.linalg.norm(x_err[:3])
        #len_q_err = np.linalg.norm(x_err[3:])
    # Calculate move time
    #move_time = (pos_to_rot_factor[0]*len_p_err + pos_to_rot_factor[1]*len_q_err)/speed + 0.2 # 0.2 just for safety

    elif controller=='JointImpedance':
        # Convention to make a distinction between x (p + q ) and q as in joints
        assert type(new_position) == tuple
        qdot = robot.qdot_max * pct_of_max_velocity
        q_err= robot.JointDistance(new_position)
        at_move = q_err/qdot
        t_move = np.max(np.abs(at_move))/speed_factor+0.1
    else:
        rospy.loginfo("How the hell did you get in here")
        t_move=10

    return t_move
