from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult

from unified_planning.shortcuts import *


class UncrewingPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None):
        """PDDL wrapper for Unscrewing skill: Generic function to pick up an object with whichever gripper.

    	Example call:
        >>> unscrewing_skill = UncrewingPDDLWrapper(skill_object = ActualUnscrewingSkill())
        """

        name = "unscrew"
        description = "Use robot to unscrew a screw."

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Check gripper type is screwdriver

        # Other needed fluents
        robot_type = UserType('robot')
        location_type = UserType('location')
        physical_object_type = UserType('physical_object')

        clamped = pddl_problem.fluent('clamped')

        unscrew = InstantaneousAction(self.get_name(), robot = robot_type, physical_object = physical_object_type)

        robot = unscrew.parameter('robot')
        obj = unscrew.parameter('physical_object')

        unscrew.add_precondition(clamped(obj))

        unscrew.add_effect(exists(obj), False)

        pddl_problem.add_action(unscrew)

        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return pddl_problem, pddl_to_world_obj_links


if __name__ == '__main__':
    0
