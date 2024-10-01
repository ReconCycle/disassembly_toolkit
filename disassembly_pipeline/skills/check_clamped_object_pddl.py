from unified_planning.shortcuts import *
from unified_planning.exceptions import UPValueError
from disassembly_pipeline.skills.base_skill import BaseSkill
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper


class CheckClampedObjectPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                skill_object: BaseSkill = None):
        """Generic function to cut a smoke detector within the CNC machine."""
        name = "check_clamped_obj"
        action_arg_names = {'robot': 'robot with which to check', 'location': 'location where to check', 'object': 'clamped object to check'}
        description = "Look at clamped object to check and ensure secure clamping for downstream operations."

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        # Check clamped object at clamp location
        # The object must be clamped
        if not problem.has_action(self.get_name()):
            clamped = problem.fluent('clamped')
            robot_has_camera = problem.fluent('has_camera')

            robot_type = UserType('robot')
            location_type = UserType('location')
            physical_object_type = UserType('physical_object')

            check_clamped_obj = InstantaneousAction(self.get_name(), robot = robot_type, location = location_type, object = physical_object_type)

            robot = check_clamped_obj.parameter('robot')
            loc = check_clamped_obj.parameter('location')
            obj = check_clamped_obj.parameter('object')

            check_clamped_obj.add_precondition(clamped(loc, obj))# Can't be clamped at any location
            check_clamped_obj.add_precondition(robot_has_camera(robot))

            #check_clamped_obj.add_effect(securely_clamped(object)) # No effects for now

            problem.add_action(check_clamped_obj)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links
