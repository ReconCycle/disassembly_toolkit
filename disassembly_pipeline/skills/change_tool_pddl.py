from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill

from unified_planning.shortcuts import *


class ChangeRobotToolPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None,
                 robot_to_valid_gripper_dict: dict = None,
                 ):
        """
        PDDL wrapper for robot toolchanging.

        Args:
        robot_to_valid_gripper_dict: dict: dictionary mapping of robot name to list of valid tools, e.g. {'robot_name': ['valid tool 1', 'valid tool 2']}

        """
        name = "change_tool"
        description = "Change robot end-effector/tool."
        action_arg_names = {'robot':'robot', 'new_tool':'Tool to mount.'}

        if not isinstance(robot_to_valid_gripper_dict, dict): raise ValueError('Required input paramer robot_to_valid_gripper_dict is not set')
        self.robot_to_valid_gripper_dict = robot_to_valid_gripper_dict

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        robot_can_mount_tool = pddl_problem.fluent('robot_can_mount_tool')
        robot_has_tool = pddl_problem.fluent('robot_has_tool')

        if not pddl_problem.has_action(self.get_name()):
            # Other needed fluents

            robot_type = UserType('robot')
            robot_tool_type = UserType('robot_tool')

            change_tool = InstantaneousAction(self.get_name(), robot = robot_type, tool=robot_tool_type)

            robot_param = change_tool.parameter('robot')
            new_tool_param = change_tool.parameter('tool')

            # move.add_precondition(clear(obj)) # Hard to determine when object is "clear" to pick up
            change_tool.add_precondition(robot_can_mount_tool(robot_param, new_tool_param))

            # Previous tool on robot is unmounted
            t = Variable("t", robot_tool_type) # create a "loop" variable

            change_tool.add_effect(fluent = robot_has_tool(robot_param, t),
                                    forall = (t,),
                                    condition = Not(Equals(t, new_tool_param)),
                                    value = False)
            change_tool.add_effect(robot_has_tool(robot_param, new_tool_param), True)

            pddl_problem.add_action(change_tool)

        # Set initial conditions/fluent
        # Read config file to see which grippers are applicable for which robot

        # Add all grippers:
        robot_tool_type = pddl_problem.user_type('robot_tool')
        robot_type = pddl_problem.user_type('robot')
        for robot in self.robot_to_valid_gripper_dict.keys():
            # TODO cleanup: add robot if not existing yet (since robot wrapper modifies pddl later in environment.py)
            #             robot_obj = pddl_problem.object(robot)
            if not pddl_problem.has_object(robot): pddl_problem.add_object(robot, robot_type)
            robot_obj = pddl_problem.object(robot)
            grippers = self.robot_to_valid_gripper_dict[robot]
            for gripper in grippers:
                if not pddl_problem.has_object(gripper):
                    pddl_problem.add_object(gripper, robot_tool_type)

                # Add relations for which gripper is available to which robot
                gripper_obj = pddl_problem.object(gripper)
                pddl_problem.set_initial_value(robot_can_mount_tool(robot_obj, gripper_obj), True)

        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return pddl_problem, pddl_to_world_obj_links


if __name__ == '__main__':
    0
