import numpy as np
import json
import os
import time
import copy
import rospy
from unified_planning.shortcuts import *
from unified_planning.exceptions import UPValueError
#from action_prediction_interface.pddl_utils.pddl_utils import get_all_objects_of_class, generate_valid_args_regex
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult


class LinearPneumaticCuttingPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self, skill_object: BaseSkill = None):

        name = "linear_cut"
        description = "Use pneumatic guillotine-style cutter to cut off the battery from a PCB, e.g. from a HCA. Only use when robot is holding a PCB!"
        action_arg_names = {'location': 'location/table where cutting is performed.', 'object': 'object to be cut apart.'}

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):

        location_type = problem.user_type('location')
        object_type = problem.user_type('physical_object')

        cutting_possible = problem.fluent('linear_cutting_possible')
        at_location = problem.fluent('at_location')
        clamped = problem.fluent('clamped')

        if not problem.has_action(self.get_name()):
            cutting_operator = InstantaneousAction(self.get_name(), location = location_type, obj = object_type)
            loc_param = cutting_operator.parameter('location')
            obj_param = cutting_operator.parameter('obj')

            cutting_operator.add_precondition(at_location(obj_param, loc_param))
            #cutting_operator.add_precondition(clamped(loc_param, obj_param))
            cutting_operator.add_precondition(cutting_possible(loc_param))

            #battery_visible_fluent = problem.fluent('battery_visible')
            #cutting_operator.add_effect(battery_visible_fluent(obj_param), True)

            problem.add_action(cutting_operator)
        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links