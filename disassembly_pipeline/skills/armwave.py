from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult


class Armwave(BaseSkill):
    def __init__(self):

        super().__init__()

    def on_enter(self, robot) -> SkillExecutionResult:
        demo_armwave(robot)

        result = SkillExecutionResult(success = True,
                                      textual_result = "Robot performed waving.",
                                      modified_variables_dict = {}
                                      )
        return result

    def execute(self):
        pass

    def on_exit(self):
        pass


def demo_armwave(robot, n_repetitions = 3):
    """ """
    ### PARAMS
    max_vel = 3
    max_acc = 3

    assert robot.gripper.Name == 'softhand'

    p1_q_init = (-0.043995, -0.430550, -0.409195, -1.916245, -0.226683, 1.558691, 0.353183)

    p1_q1 = (0.130702, -0.795076, -0.402820, -1.999278, -0.049360, 2.756914, 0.444195)
    p2_q2 = (0.474790, -0.761872, -0.905444, -2.005180, -0.047051, 2.928837, 0.941880)
    p2_q3 = (0.480880, -0.810580, -0.485103, -2.139690, -1.024069, 3.103375, 0.833179) 
    ### END PARAMS

    if robot._control_strategy == 'CartesianImpedance':
        robot.Switch_between_cart_imp_and_joint_imp()
    robot.error_recovery()

    robot.gripper.open()

    robot.JMove(p1_q_init, t = 4)
    robot.JMove(p1_q1, t = 2)

    for i in range(0,n_repetitions):    
        robot.JMove(p2_q2, t = 1, max_vel = max_vel, max_acc = max_acc)
        #robot.JMove(p1_q1, t = 2)
        robot.JMove(p2_q3, t = 1, max_vel = max_vel, max_acc = max_acc)

    robot.JMove(p1_q1, t = 2)

    robot.JMove(p1_q_init, t = 4)
