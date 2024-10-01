from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill

from unified_planning.shortcuts import *


class VLMNoOpPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None,
                 robot_to_valid_gripper_dict: dict = None,
                 ):
        """
        PDDL wrapper for robot toolchanging.

        Args:
        robot_to_valid_gripper_dict: dict: dictionary mapping of robot name to list of valid tools, e.g. {'robot_name': ['valid tool 1', 'valid tool 2']}

        """
        name = "no_op"
        description = "Call this if:\n\t- none of the available actions is applicable,\n\t- the prompt is missing key information,\n\t- no objects to disassemble are detected.\n\t\tWrite your comment under action_input field."
        action_arg_names = {'reasoning' : 'The textual reason and explanation.'}


        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = None)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem, pddl_to_world_obj_links):
        return pddl_problem, pddl_to_world_obj_links


if __name__ == '__main__':
    0
