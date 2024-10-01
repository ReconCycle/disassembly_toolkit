from disassembly_pipeline.utils.robot_quick_init import initialize_robot

if __name__ == '__main__':
    p1 = initialize_robot(robot_name = 'panda_1',
                     tool_name = 'ThreeJawChuck',
                     start_controller = 'position_joint_trajectory_controller',
                     collision_thresholds = {"F": 50, "T": 20, "tq": 30},
                     gripper_init_kwargs = {},
                     toolchanger = True,
                     toolchanger_init_kwargs = {})

    q_init = (0.14299200386122654, -0.7837136655774039, -0.36695189219952756, -2.0215913527597063, -0.2513166111906369, 1.2977900131543478, 0.5716469945249458)
    x1 = [ 0.42396338, -0.01733356,  0.48722725,  0.02186937,  0.7655018 ,0.64251233,  0.02658273]
    x2 = [ 0.23,  0.22,  0.43895892,  0.01578983,  0.61576657,0.78766315, -0.01299934]
    controller_to_use = 'CartesianImpedance'
    #controller_to_use = 'JointPositionTrajectory'

    p1.Switch_controller(start_controller = 'JointPositionTrajectory')
    p1.error_recovery()
    p1.GetState()
    p1.JMove(p1._actual_int.q, 0.1)

    p1.JMove(q_init, 1, qdot_max_factor = 0.3, qddot_max_factor = 0.3)
    p1.Switch_controller(start_controller = controller_to_use)
    p1.CMove(x1, 2)
    p1.CMove(x2, 2)