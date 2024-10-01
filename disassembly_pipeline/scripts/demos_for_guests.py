import rospy
import time
import copy
import numpy as np

from robotblockset_python.transformations import x2t, t2x, rot_z

from time import time as ttt
from disassembly_pipeline.multithreading import do_concurrently

def predstavitev_p1(p1, standard_t = None):
    """ Demo ki je lavfal, ko so majstri snemali v labu. V grobem podobno kot cikel za HCA, ampak se gripperji ne odpirajo/zapirajo.""" 
    
    traj = 'poly'
    #traj = 'trap'
    
    # Init za do primeza
    q1 = (-0.259610, -0.557503, -0.394245, -2.556876, -0.216357, 2.055918, 0.673202)
    q2 = (-0.233435, 0.083605, -0.673825, -2.135081, 0.046796, 2.185985, 0.632073)
    q3 = (-0.229002, 0.459736, -0.702416, -1.678845, 0.017452, 1.776053, 0.454030)
    q4 = (-0.228406, 0.496641, -0.699558, -1.678133, 0.017832, 1.779094, 0.449679)
    # Potem za droppat roko
    #q14 = (0.171870, 0.049517, -0.803593, -2.085636, 0.138869, 2.044632, 0.011276)
    q14 = (-0.090545, -0.653724, -0.832242, -2.549161, -0.478019, 2.097942, 0.125971)
    
    p1_q_drop_1 = (0.492568, 0.463204, -0.480156, -2.345004, 0.390286, 2.760642, 0.964490)
    #p1_q_drop_2 = (0.489336, 0.338307, -0.575500, -2.538422, 0.224884, 2.767745, 0.100851)
    drop_qs = [p1_q_drop_1]#, p1_q_drop_2]
    to_pickup = np.random.randint(len(drop_qs))
    q_to_pickup = drop_qs[to_pickup]
    #p1_q_drop_3 = (0.491165, 0.404785, -0.446615, -2.380367, 0.395264, 2.788477, 0.969851)

    # Cikel za gospode

    #cy.robot_go_to_init_and_open_gripper(p1)
    #hca_type, has_pin = cy.p1_pickup_hca_and_put_into_vice(init_safe = False, double_tap_vise = False, return_to_via = True)
    r = p1
    r.error_recovery()
    if r._control_strategy != 'JointPositionTrajectory':
        r.ResetCurrentTarget()
        r.Switch_controller(start_controller =  'JointPositionTrajectory')
        
    starting_t = ttt()
    r.GetState();r.ResetCurrentTarget()
    r.JPath_new(path = np.stack([r._command_int.q, q1,q2]), t=4)
    #r.ResetCurrentTarget()
    #r.JMove(q1, 2, traj=traj)
    #r.JMove(q2, 1.5, traj=traj)
    #print(r._actual_int.q, r._command_int.q)
    r.gripper.close(0.5,sleep = False)
    #print("From q to q")
    
    r.JMove(q3, 2, traj=traj)
    r.JMove(q4, 1.5, traj=traj)
    r.JMove(q3, 1.5, traj=traj)
    
    r.GetState()
    path = np.stack([q3, q14, q_to_pickup])
    r.JPath_new(path = path, t = 3.5)
    
    #r.JMove(q14, 1.8, traj=traj)
    #r.JMove(q_to_pickup, 1.5, traj=traj)

    #cy.set_cycle_speed('ludicrous')
    #cy.p1_handle_hca_frame_to_bin(drop_back_on_table = True, return_to_init=False)
    #cy.set_cycle_speed('fast')

    #if r._control_strategy != 'JointPositionTrajectory':
    #    r.ResetCurrentTarget()
    #    r.Switch_controller(start_controller =  'JointPositionTrajectory')
    
    #r.gripper.open()
    end_t = ttt()
    diff = end_t - starting_t
    print("P1 took %.3f"%diff)
    
def predstavitev_p2(p2):
    """ Demo ki je lavfal, ko so majstri snemali v labu. V grobem podobno kot cikel za HCA, ampak se gripperji ne odpirajo/zapirajo.""" 
    
    traj = 'poly'
    #traj = 'trap'
    q_init = (-0.077788, -0.615181, 0.001935, -2.449091, -0.062205, 1.839013, 0.710934)
    q1 = (-1.326760, -0.528321, 0.072972, -2.435524, -0.039830, 1.919504, 0.774545)
    q2 = (-1.348096, 0.091679, 0.065733, -2.060958, -0.053441, 2.160539, 1.143584)
    
    #q_pickup_bat_1 = (-1.319089, 0.636093, 0.150723, -1.846739, -0.078336, 2.366209, 0.759513)
    #q_pickup_bat_2 = (-1.284040, 0.774444, 0.030317, -1.598763, 0.034807, 2.356234, 1.382217)
    q_pickup_bat_1 = (-1.261137, 0.666718, 0.021098, -1.691219, -0.006768, 2.371520, 1.332061)
    #q_pickup_bat_3 = (-1.319089, 0.636093, 0.150723, -1.846739, -0.078336, 2.366209, 0.759513)

    
    qs_pickup_bat = [q_pickup_bat_1]
    to_pickup = np.random.randint(len(qs_pickup_bat))
    print("P2 - Pickup index %d"%to_pickup)
    q_bat_pickup = qs_pickup_bat[to_pickup]
    
    q4 = (-1.268466, -0.166702, 0.283371, -2.549683, 0.032457, 2.375878, 0.948188)
    q5 = (-0.557155, -0.628876, 0.280503, -2.104714, 0.192002, 1.488340, 0.511694)
    q6 = (-0.558772, 0.024246, 0.542085, -1.537687, -0.027080, 1.549759, 0.727419)
    
    r = p2
    r.error_recovery()
    if r._control_strategy != 'JointPositionTrajectory':
        r.ResetCurrentTarget()
        r.Switch_controller(start_controller =  'JointPositionTrajectory')
        
    starting_t = ttt()
    r.JMove(q_init,2, traj = traj)
    r.JMove(q1,1.8, traj = traj)
    r.JMove(q2,1.5, traj = traj)
    r.JMove(q_bat_pickup,1.5, traj = traj)
    r.JMove(q4,1.5, traj = traj)
    r.JMove(q5,1.8, traj = traj)
    
    #r.JMove(q6,2, traj = traj)

    end_t = ttt()
    diff = end_t - starting_t
    print("P2 took %.3f"%diff)
    
    #try:cy.p2_pick_up_battery(get_into_camera_position = True, safe_to_camera = True, pick_up_battery = True)
    #except:pass

def predstavitev_za_gospode(p1 = None, p2 = None, n_retries = 1, standard_t = 4, robot_to_run = 'panda_1'):
    assert robot_to_run in ['panda_1', 'panda_2', 'both']
    
    cur_n = 0
    while cur_n < n_retries:
        cur_n += 1
        if robot_to_run == 'panda_1':
            predstavitev_p1(p1 = p1, standard_t = standard_t)
        elif robot_to_run == 'panda_2':
            predstavitev_p2(p2 = p2)
        
        elif robot_to_run == 'both':
            do_concurrently([[predstavitev_p1, {'p1':p1, 'standard_t' : standard_t}],
                            [predstavitev_p2, {'p2': p2}]], wait_until_task_completion = True)
            
#predstavitev_za_gospode(n_retries = 1, standard_t = 2, robot_to_run = 'panda_1')
#predstavitev_za_gospode(n_retries = 1, standard_t = 2, robot_to_run = 'panda_2')
#predstavitev_za_gospode(n_retries = 10, standard_t = 2, robot_to_run = 'both')

def demo_scanning(cy, robot):
    """
    
    Input args:
    cy - Disassembly cycle object
    robot - robot obj"""
    assert robot.Name == 'panda_2' # only panda 2 has the realsense
    
    x_1 = cy.tf2x(parent_frame = robot.Name + '/' + robot.Name + '_link0',
                        child_frame = 'cutter/realsense2')
    #
    x_2 = cy.tf2x(parent_frame = robot.Name + '/' +  'panda_EE',
                        child_frame = 'panda_2/realsense')
    
    T_end = x2t(x_1)@np.linalg.inv(x2t(x_2))
    x_above_cutter_plate = t2x(T_end)
    
        
    if robot._control_strategy != 'JointPositionTrajectory':
        robot.Switch_controller(start_controller =  'JointPositionTrajectory')
    robot.error_recovery()
    
    p2_q1_init = (-0.067808, -0.613722, -0.008163, -2.444180, -0.044385, 1.850346, 0.712032)
    
    # Go to init
    cy.prepare_vision(activate_realsense = True)
    rospy.set_param("/vision/realsense/publish_labeled_img", True)

    robot.JMove(p2_q1_init, t = 4, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
    
    robot.CMove(x_above_cutter_plate, 4)
    
    robot.CMoveFor([0.4, 0, 0], 4)
    robot.CMoveFor([0, 0.3, 0], 4)
    
    robot.CMoveFor(rot_z(-90, unit='deg'), t= 2, task_space = 'Tool')
    
    robot.gripper.close()
    
    robot.JMove(p2_q1_init, t = 4)
    #rospy.set_param("/vision/realsense/publish_labeled_img", False)

    return 0

def demo_kamera_cobra(cy, robot, direction = 'back', n_loop = 100):
    """
    The robot will raise up like a cobra, and use the eye-in-hand camera to scan the room.
    Input args:
    cy - Disassembly cycle object
    robot - robot obj"""
    
    assert direction in ['front','back']
    
    if robot._control_strategy != 'JointPositionTrajectory':
        robot.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        #robot.Switch_between_cart_imp_and_joint_imp()
    robot.error_recovery()
    
    p2_q1_init = (-0.067808, -0.613722, -0.008163, -2.444180, -0.044385, 1.850346, 0.712032)
    
    if direction == 'front':
        p2_q2 = (1.3296529318543902, -0.3565432202351549, -0.03437189025187276, -0.9784669078136231, 0.38664911872148516, 1.911226732298185, 0.5455267396743098)

        #p2_q2 = (0.541930, -0.267099, 0.010153, -1.581540, -0.097806, 1.958515, 0.711873)
        p2_q3 = np.array(copy.deepcopy(p2_q2))
        #p2_q3 = np.array((0.532293, 0, 0.113106, -0.808890, 0.287357, 1.948183, 0.622948))
        p2_q_lookaround_1 = copy.deepcopy(p2_q3)
        p2_q_lookaround_1[0] -=0.5
        
        
        p2_q_lookaround_2 = copy.deepcopy(p2_q3)
        p2_q_lookaround_2[0] +=0.5
    elif direction == 'back':
        p2_q2 = np.array((-1.912190, -0.826819, 0.306533, -1.450520, 0.146882, 2.046823, 1.083512))
    
        p2_q3 = np.array((-2.464032, -0.861952, 0.135725, -1.451482, 0.143052, 2.047680, 0.909697))
        
        p2_q_lookaround_1 = copy.deepcopy(p2_q3)
        p2_q_lookaround_1[0] -=0.3

        p2_q_lookaround_2 = copy.deepcopy(p2_q3)
        p2_q_lookaround_2[0] += 0.6
        
    # Go to init
    cy.prepare_vision(activate_realsense = True)
    rospy.set_param("/vision/realsense/publish_labeled_img", True)

    robot.JMove(p2_q1_init, t = 4, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
    
    robot.JMove(p2_q2, t = 3)
    
    robot.JMove(p2_q3, t = 2)
    
    for i in range(0,n_loop):
        try:
            robot.JMove(p2_q_lookaround_1, t = 3);time.sleep(1.5)
            robot.JMove(p2_q_lookaround_2, t = 3);time.sleep(1.5)
            robot.JMove(p2_q3, t = 2)
        except rospy.ROSInterruptException: 
            break
        except KeyboardInterrupt:
            break
    
    robot.gripper.close()
    
    robot.JMove(p2_q1_init, t = 2, qdot_max_factor = 0.2, qddot_max_factor = 0.2)
    #rospy.set_param("/vision/realsense/publish_labeled_img", False)

    return 0
    
def demo_fastmove(cy, robot_1, n_repetitions = 2, speed_factor = 1):
    
    assert speed_factor < 3
    
    dx = 0.25
    tx = 1.5/speed_factor
    
    dy = 0.2
    ty = 1.5/speed_factor
    
    #r1_init = p2_q1_init = [0.12272088397521683,-1.3181167800849902,-0.20597019964421703,-2.5176138428587596,
    # -0.20126330022266803,1.254942861398061,0.8085149702917125]

    #r2_init =  (-0.067808, -0.613722, -0.008163, -2.444180, -0.044385, 1.850346, 0.712032)
    #do_concurrently([[cy.robot_go_to_init_and_open_gripper, {'robot_obj':robot_1}],[cy.robot_go_to_init_and_open_gripper, {'robot_obj':robot_2}]], wait_until_task_completion=True)
    cy.robot_go_to_init_and_open_gripper(robot_1)
    
    if robot_1._control_strategy != 'JointPositionTrajectory':
        robot_1.Switch_controller(start_controller =  'JointPositionTrajectory')
    
    robot_1.error_recovery()
    #robot_1.SetCartesianStiff_helper(m=1.3,n=0.7)
    robot_1.ResetCurrentTarget()
    #robot_1.SetReconcycleCartesianCompliance(Kp=[2000,2000,2000],Kr=[60,60,60],D=1.2)
    time.sleep(1)
    
    print(1)
    robot_1.CMoveFor(dx=[-0.06, 0, 0],t=1.5, traj = 'trap')
    
    for i in range(0, n_repetitions):
        print(2)
        robot_1.CMoveFor(dx=[0, +dy, 0], t = ty, traj = 'trap')
        print(2)
        robot_1.CMoveFor(dx=[dx,0, 0], t = tx, traj = 'trap')
        print(3)
        robot_1.CMoveFor(dx=[0 , -dy, 0], t = ty, traj = 'trap')
        print(4)
        robot_1.CMoveFor(dx=[-dx , 0, 0], t = tx, traj = 'trap')

        robot_1.ResetCurrentTarget()

    return 0

def demo_juggle_p1(robot):
    """ An attempt to make a robot throw up a ball. Try replacing CMoveFor with CMove(apply_FT) to improve."""
    
    assert robot.Name =='panda_1'
    q_init = (-0.512903, -0.706052, -0.571840, -1.874793, -0.420342, 1.259678, 0.791259)
    q_juggle_init = (0.480050, -1.076803, -1.723027, -2.086875, 1.842278, 1.246483, 0.895657)
    traj = 'poly'
    tt=0.6
    
    #traj = 'trap'
    #tt = 0.8
    
    #traj = 'line'
    #tt=1
    
    r = robot
    r.error_recovery()
    r.gripper.close(0.4)
    #r.JMove(q_init,2)
    
    r.JMove(q_juggle_init,2)
    time.sleep(2)
    #r.CMoveFor([0,0,-0.1],2)
    r.gripper.open(1)

    r.CMoveFor(dx = [0,0,0.3], t= tt, traj=traj)
    time.sleep(3)
    r.CMoveFor(dx = [0,0,-0.3], t= tt, traj = traj)


#demo_juggle_p1(p1)

#demo_fastmove(cy, p1, p2, n_repetitions = 3)

#demo_kamera(cy = cy, robot= p2)
