from unified_planning.shortcuts import *
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper


class ClampPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 pddl_constrain_based_on_predicates = False,
                 skill_object: BaseSkill = None,
                 **kwargs):

        name = "clamp"
        description = "Clamp an object to enable performing downstream operations."
        action_arg_names: str = {'location': 'location of clamping', 'object': 'object to clamp'}
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""
        # Add clamp action
        clamp_action_name = 'clamp'

        location_type = problem.user_type('location')
        physical_object_type = problem.user_type('physical_object')

        clamp_empty = problem.fluent('clamp_empty')
        clamped = problem.fluent('clamped')
        clamping_possible = problem.fluent('clamping_possible')
        at_location = problem.fluent('at_location')
        if not problem.has_action(clamp_action_name):
            clamp_operator = InstantaneousAction(clamp_action_name, location = location_type, obj = physical_object_type)
            clamp_location = clamp_operator.parameter('location')
            clamped_object = clamp_operator.parameter('obj')

            clamp_operator.add_precondition(clamp_empty(clamp_location))
            clamp_operator.add_precondition(at_location(clamped_object, clamp_location))
            clamp_operator.add_precondition(clamping_possible(clamp_location))

            clamp_operator.add_effect(clamped(clamp_location, clamped_object), True)
            clamp_operator.add_effect(clamp_empty(clamp_location), False)

            problem.add_action(clamp_operator)
        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links


class UnclampPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 pddl_constrain_based_on_predicates: bool = False,
                 skill_object: BaseSkill = None,
                 **kwargs):

        name = "unclamp"
        description = "Unclamp an object so it can be moved."
        action_arg_names: str = {'location': 'location of clamping', 'object': 'object to clamp'}

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Add unclamp action
        unclamp_action_name = 'unclamp'

        location_type = problem.user_type('location')
        object_type = problem.user_type('physical_object')

        clamp_empty = problem.fluent('clamp_empty')
        at_location = problem.fluent('at_location')
        clamped = problem.fluent('clamped')

        if not problem.has_action(unclamp_action_name):
            unclamp_operator = InstantaneousAction(unclamp_action_name, location = location_type, obj = object_type)
            clamp_location = unclamp_operator.parameter('location')
            clamped_object = unclamp_operator.parameter('obj')

            unclamp_operator.add_precondition(Not(clamp_empty(clamp_location)))
            unclamp_operator.add_precondition(clamped(clamp_location, clamped_object))
            unclamp_operator.add_precondition(at_location(clamped_object, clamp_location))

            unclamp_operator.add_effect(clamped(clamp_location, clamped_object), False)
            unclamp_operator.add_effect(clamp_empty(clamp_location), True)

            problem.add_action(unclamp_operator)
        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links