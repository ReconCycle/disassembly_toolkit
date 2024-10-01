from unified_planning.shortcuts import *

from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill


class LookAtTablePDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None):
    
        """Generic function to look at a table with a robot with an eye-in-hand camera."""
        
        name = "look_at_table"
        action_arg_names = {'robot': 'Robot which to use.', 'location': 'Which table to look at.'}
        description = "Use robot with eye-in-hand camera to look at particular location."
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        #at_location = problem.fluent('at_location')

        robot_type = UserType('robot')
        location_type = UserType('location')
        has_camera = problem.fluent('has_camera')

        if not problem.has_action(self.get_name()):

            look_at = InstantaneousAction(self.get_name(), robot = robot_type, location=location_type)
            robot = look_at.parameter('robot')
            loc = look_at.parameter('location')

            look_at.add_precondition(has_camera(robot))
            #look_at.add_effect(looking_at(robot, loc), False)
            
            problem.add_action(look_at)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links
