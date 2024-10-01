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


class CNCCutSmokeDetectorPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self, skill_object: BaseSkill = None):
        """Generic function to cut a smoke detector within the CNC machine."""

        name = "cnc_cut"
        description = "Use CNC milling machine to cut it open a smoke detector. Device should be dropped on table_cnc beforehand."
        action_arg_names = {'location': 'location/table where cutting is performed.', 'object': ['smoke_detector', 'hca']}

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        location_type = problem.user_type('location')
        object_type = problem.user_type('physical_object')

        cnc_cutting_possible = problem.fluent('cnc_cutting_possible')
        at_location = problem.fluent('at_location')
        clamped = problem.fluent('clamped')
        if not problem.has_action(self.get_name()):
            cnc_cutting_operator = InstantaneousAction(self.get_name(), location = location_type, obj = object_type)
            loc_param = cnc_cutting_operator.parameter('location')
            obj_param = cnc_cutting_operator.parameter('obj')

            cnc_cutting_operator.add_precondition(at_location(obj_param, loc_param))
            cnc_cutting_operator.add_precondition(clamped(loc_param, obj_param))
            cnc_cutting_operator.add_precondition(cnc_cutting_possible(loc_param))

            battery_visible_fluent = problem.fluent('battery_visible')
            cnc_cutting_operator.add_effect(battery_visible_fluent(obj_param), True)

            problem.add_action(cnc_cutting_operator)
        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links