from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill

from unified_planning.shortcuts import *


class WigglingBatteryRemovalPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None):

        """

        """
        name = "wiggling_battery_removal"
        action_arg_names = {'robot': 'Robot to perform operation.', 'location': 'location','object': 'the battery which to remove.'}
        description = "Attempt to remove a battery from a device by grasping it and wiggling/rocking it free using a robot."

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not pddl_problem.has_action(self.get_name()):
            # Other needed fluents
            at_location = pddl_problem.fluent('at_location')
            clamped = pddl_problem.fluent('clamped')

            holding = pddl_problem.fluent('holding')

            robot_type = UserType('robot')
            location_type = UserType('location')

            physical_object_type = UserType('physical_object')
            object_type = UserType('battery', father = physical_object_type)
            #object_type = UserType('physical_object')

            wiggling = InstantaneousAction(self.get_name(), robot = robot_type, location = location_type, object = object_type)

            robot = wiggling.parameter('robot')
            location = wiggling.parameter('location')
            object = wiggling.parameter('object')

            # Battery itself isn't clamped.
            # For these preconditions, consider adding "base object in which battery is" as a separate parameter
            #wiggling.add_precondition(at_location(object, location)) # Must be on table 
            #wiggling.add_precondition(clamped(location, object)) # Must be clamped be clamped at any location

            wiggling.add_effect(holding(robot, object), True)

            pddl_problem.add_action(wiggling)
        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links