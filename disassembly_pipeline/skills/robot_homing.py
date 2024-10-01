import numpy as np
from typing import List, Union

from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.poses.saved_positions import p1_q1_init, p2_q1_init
from robotblockset_python.robots import robot as Robot


# TODO homed q's are imported and hardcoded per-robot, fix.
class RobotHoming(BaseSkill):
    def __init__(self,):
        """
        Example call:
        >>> homing_sk = RobotHoming()
        >>> homing_sk.on_enter(**{'robot': panda_2_robotblockset_object})"""

        super().__init__()

    def on_enter(self, robot: Robot, open_gripper = True, **kwargs) -> SkillExecutionResult:
        """ Function to call when first starting the skill."""

        # TODO improve, read from config.
        robot_name_to_init_q_dict = {'panda_1': p1_q1_init,
                                     'panda_2': p2_q1_init}

        robot_name = robot.Name

        if robot_name not in robot_name_to_init_q_dict.keys():
            raise Exception("Init position for robot {} not set!".format(robot_name))

        q_init = robot_name_to_init_q_dict[robot_name]
        success = robot_go_to_init_and_open_gripper(robot_obj = robot, init_q_positions = [q_init], open_gripper = open_gripper)
        adjective = "successful" if success else "unsuccessful"

        result = SkillExecutionResult(success = success,
                                    textual_result = f"Robot {robot_name} homing was {adjective}.",
                                    modified_variables_dict = {})

        return result

    def execute(self, **kwargs):
        """ Function that GETS CALLED periodically(possibly with high freq) during skill execution. """
        0

    def on_exit(self, **kwargs):
        """ Function that gets called after the skill finishes."""
        0


def is_max_joint_position_error_less_than_n_degrees(q_err: np.array, max_error_in_deg: Union[float, int] = 5):
    """Check that joint position error in any joint is less than max_error_in_deg degrees."""
    # TODO move to utils
    success = False

    q_err = np.abs(q_err)  # Convert to absolute since direction of error is irrelevant
    q_err_deg = q_err * 180 / np.pi  # Convert radian errors to degrees
    q_err_deg_max = np.max(q_err_deg)  # Find joint with maximal error
    if q_err_deg_max < max_error_in_deg:
        success = True

    return success


def robot_go_to_init_and_open_gripper(robot_obj: Robot, init_q_positions: List[Union[List, np.array]], open_gripper: bool = True):
        """Generic function for both robots. Go to q init and open the gripper
        Args:
        init_q_positions: List of either one or more joint positions to which to sequentially move

        """

        # TODO - Add path checking using moveit.

        if robot_obj._control_strategy != 'JointPositionTrajectory':
            robot_obj.ResetCurrentTarget()
            robot_obj.Switch_controller(start_controller =  'JointPositionTrajectory')

        robot_obj.error_recovery()
        #robot_obj.ResetCurrentTarget();time.sleep(0.1)
        init_q_pos = init_q_positions # self.robot_init_qs[robot_obj.Name]#

        if open_gripper: robot_obj.gripper.open(1, sleep=False)

        robot_obj.GetState()
        q_err = robot_obj.JointDistance(q = init_q_pos[0], state = 'Actual')
        q_configuration_reached = is_max_joint_position_error_less_than_n_degrees(q_err = q_err, max_error_in_deg = 5)
        if not q_configuration_reached:
            robot_obj.JMove(init_q_pos[0], t = 3, qdot_max_factor = 0.4, qddot_max_factor = 0.4) # The time will be overridden based on di$
        else:
            print("Robot homing not moving because current q is very close to desired_q")
        #do_concurrently([[robot_obj.JMove, {'q':init_q_pos[0], 't':4/self.sf, 'max_vel':0.8, 'max_acc':0.8}], [robot_obj.gripper.ope$        #robot_obj.WaitUntilStopped()
        # If we have more init Joint positions

        robot_obj.GetState()

        #robot_obj.ResetCurrentTarget()

        # Sometimes we want to move to a particular "dangerous" position so a sequence of known safe moves is performed
        # (instead of moving to final position immediately)
        if len(init_q_pos)>1:
            for i in range(1,len(init_q_pos)):
                robot_obj.JMove(init_q_pos[i], t = 1, qdot_max_factor = 0.5, qddot_max_factor = 0.5)

        # Check if position reached successfully.
        robot_obj.GetState()
        q_err = robot_obj.JointDistance(q = init_q_pos[-1], state = 'Actual')

        success = is_max_joint_position_error_less_than_n_degrees(q_err = q_err, max_error_in_deg = 5)
        return success
