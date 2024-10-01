from unified_planning.shortcuts import *

from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill


class DropObjectPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None):

        """Generic function to drop an object with whichever gripper."""
        name = "drop"
        action_arg_names = {'robot': 'Robot which to use.', 'object': 'Object to be dropped.', 'location': 'Where to drop the object.'}
        description = "Use robot to drop an object at a particular location."
        super().__init__(name = name, description = description, action_arg_names = action_arg_names)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Other needed fluents
        holding = problem.fluent('holding')
        at_location = problem.fluent('at_location')

        robot_type = UserType('robot')
        location_type = UserType('location')
        physical_object_type = UserType('physical_object')

        if not problem.has_action(self.get_name()):

            drop = InstantaneousAction(self.get_name(), robot = robot_type, obj = physical_object_type, loc=location_type)
            robot = drop.parameter('robot')
            obj = drop.parameter('obj')
            loc = drop.parameter('loc')

            drop.add_precondition(holding(robot, obj))

            drop.add_effect(holding(robot, obj), False)
            drop.add_effect(at_location(obj, loc), True)

            
            problem.add_action(drop)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links
