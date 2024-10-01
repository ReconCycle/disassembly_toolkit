from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill
from unified_planning.shortcuts import *
from unified_planning.model import Problem


class MoveObjectPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None
                 ):
        """Generic function to move an object to a new location.

	    Example call:
        >>>
        """
        name = "move"
        action_arg_names = {'robot':'robot', 'l_from':'location from which to pick', 'l_to':'location to place object', 'object': 'hca'}
        description = "Move an object to a particular location."

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem: Problem, pddl_to_world_obj_links: dict):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not pddl_problem.has_action(self.get_name()):
            # Other needed fluents
            at_location = pddl_problem.fluent('at_location')
            clamped = pddl_problem.fluent('clamped')
            gripper_empty = pddl_problem.fluent('gripper_empty')

            robot_type = UserType('robot')
            location_type = UserType('location')
            physical_object_type = UserType('physical_object')

            move = InstantaneousAction(self.get_name(), robot = robot_type, l_from = location_type, l_to=location_type, physical_object = physical_object_type)

            robot = move.parameter('robot')
            l_from = move.parameter('l_from')
            l_to = move.parameter('l_to')
            obj = move.parameter('physical_object')

            # move.add_precondition(clear(obj)) # Hard to determine when object is "clear" to pick up
            move.add_precondition(gripper_empty(robot))
            move.add_precondition(at_location(obj, l_from)) # Must be on table where we pick up from
            move.add_precondition(Not(at_location(obj, l_to))) # Can't place it on table where it already is
            move.add_precondition(Not(clamped(l_from, obj)))# Can't be clamped at any location

            move.add_effect(at_location(obj, l_from), False)
            move.add_effect(at_location(obj, l_to), True)

            pddl_problem.add_action(move)

        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem: Problem, pddl_to_world_obj_links: dict):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return pddl_problem, pddl_to_world_obj_links


if __name__ == '__main__':
    0
