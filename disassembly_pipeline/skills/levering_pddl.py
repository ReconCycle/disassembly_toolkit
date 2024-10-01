from disassembly_pipeline.skills.base_skill import BaseSkill
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from unified_planning.shortcuts import *


class LeveringPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                skill_object: BaseSkill = None):
        """

        """
        name = 'lever'
        action_arg_names = {'robot': 'robotblockset_python.robot.Robot', 'location': 'action_prediction_interface.pddl_modules.base_table.BaseTable','object': ['hca', 'pcb', 'context_action_framework.types.Detection']}
        description = "Lever out a PCB from an HCA object. Object must be clamped beforehand."
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not pddl_problem.has_action(self.get_name()):
            # Other needed fluents
            at_location = pddl_problem.fluent('at_location')
            clamped = pddl_problem.fluent('clamped')

            robot_type = UserType('robot')
            location_type = UserType('location')
            physical_object_type = UserType('physical_object')

            lever = InstantaneousAction(self.get_name(), robot = robot_type, location = location_type, object = physical_object_type)

            robot = lever.parameter('robot')
            location = lever.parameter('location')
            object = lever.parameter('object')

            lever.add_precondition(at_location(object, location))  # Must be on table where we lever up from
            lever.add_precondition(clamped(location, object))  # Must be clamped be clamped at any location

            # No effects as yet

            pddl_problem.add_action(lever)
        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links
