from unified_planning.shortcuts import *

from disassembly_pipeline.skills.base_skill import BaseSkill
from action_prediction_interface.pddl_utils.pddl_utils import get_all_objects_of_class, generate_valid_args_regex
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper


class TurnObjectPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self, 
                 name = "turn",
                 action_arg_names = {'robot': 'robot to use', 'location': 'location where to turn it', 'object': 'object to turn around'},
                 description = "Use robot to turn/flip an object.",
                 skill_object: BaseSkill = None):
        
        """PDDL wrapper for TurnObject skill."""
            
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not problem.has_action(self.get_name()):
            # Other needed fluents
            #correct_side_up = problem.fluent('correct_side_up')
            clear = problem.fluent('clear')

            gripper_empty = problem.fluent('gripper_empty')

            robot_type = problem.user_type('robot')
            location_type = problem.user_type('location')
            physical_object_type = problem.user_type('physical_object')

            at_location = problem.fluent('at_location')
            turning_possible = problem.fluent('turning_possible')
            #table_vision = problem.object('table_vision')

            #physical_object_type = problem.user_type('physical_object')

            turn = InstantaneousAction(self.get_name(), robot = robot_type, loc = location_type, obj = physical_object_type)

            robot = turn.parameter('robot')
            loc = turn.parameter('loc')
            obj = turn.parameter('obj')

            turn.add_precondition(gripper_empty(robot))
            turn.add_precondition(turning_possible(loc))
            #turn.add_precondition(on_table(obj, table_vision)) # Only object on table_vision can be turned
            #turn.add_precondition(clear(obj))

            #turn.add_effect(correct_side_up(obj), True)

            problem.add_action(turn)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links