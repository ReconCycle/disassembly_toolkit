from action_prediction_interface.pddl_modules.base_module import BaseModule

from unified_planning.shortcuts import *
from unified_planning.model.problem import Problem
from unified_planning.exceptions import UPProblemDefinitionError


class GenericModule(BaseModule):
    def __init__(self,
                 module_name: str = 'table_generic',
                 description: str = 'Random table. Change this description to reflect its meaning.'):

        super().__init__(module_name = module_name, description = description)

    def pddl_init(self, problem: Problem, pddl_to_world_obj_links: dict):

        # Add object(self) to pddl_to_world_obj_links
        pddl_to_world_obj_links[self.get_name()] = self

        # Get required env objects
        location = problem.user_type('location')

        # Add location/clamp_location object
        if not problem.has_object(self._name):
            problem.add_object(self._name, location)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem: Problem, pddl_to_world_obj_links: dict):
        0
        return problem, pddl_to_world_obj_links
        

class LinearPneumaticCutter(BaseModule):
    def __init__(self):

        module_name = 'table_cutter'
        description = "Contains a linear guillotine-style cutter which can cut apart PCBs to separate the battery."

        super().__init__(module_name = module_name, description = description)

    def pddl_init(self, problem: Problem, pddl_to_world_obj_links: dict):
        # Add object(self) to pddl_to_world_obj_links
        pddl_to_world_obj_links[self._name] = self

        # Get required env objects
        location = problem.user_type('location')

        clamp_empty = problem.fluent('clamp_empty')
        linear_cutting_possible = problem.fluent('linear_cutting_possible')
        clamping_possible = problem.fluent('clamping_possible')

        if not problem.has_object(self._name):
            problem.add_object(self._name, location)

        this_loc = problem.object(self._name)

        # Add initial condition that cutting is possible here
        problem.set_initial_value(linear_cutting_possible(this_loc), True)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem: Problem, pddl_to_world_obj_links: dict):
        0
        return problem, pddl_to_world_obj_links


class CNCMachine(BaseModule):
    def __init__(self):

        module_name = 'table_cnc'
        description = "Contains a CNC machine/mill. Smoke detectors dropped within this table can be cut open to reveal batteries and internal components."

        super().__init__(module_name = module_name, description = description)

    def pddl_init(self, problem: Problem, pddl_to_world_obj_links: dict):
        # Add object(self) to pddl_to_world_obj_links
        pddl_to_world_obj_links[self._name] = self

        # Get required env objects
        location = problem.user_type('location')

        clamp_empty = problem.fluent('clamp_empty')
        cnc_cutting_possible = problem.fluent('cnc_cutting_possible')
        clamping_possible = problem.fluent('clamping_possible')

        # Add location/clamp_location object
        if not problem.has_object(self._name):
            problem.add_object(self._name, location)

        # Add initial condition that clamp is empty
        clamp_loc = problem.object(self._name)
        problem.set_initial_value(clamp_empty(clamp_loc), True)
        # Add initial condition that cutting is possible here
        problem.set_initial_value(cnc_cutting_possible(clamp_loc), True)
        # Clamping is also possible
        problem.set_initial_value(clamping_possible(clamp_loc), True)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem: Problem, pddl_to_world_obj_links: dict):
        0
        return problem, pddl_to_world_obj_links


class ViseTable(BaseModule):
    def __init__(self, module_name = 'table_vise'):

        description = """Contains a vise/clamp for clamping Heat cost allocators (HCAs) so downstream operations (e.g. levering) can be performed."""

        super().__init__(module_name = module_name, description = description)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        # Add object(self) to pddl_to_world_obj_links
        pddl_to_world_obj_links[self._name] = self

        # Get required env objects
        location = problem.user_type('location')
        physical_object = problem.user_type('physical_object')
        clamped = problem.fluent('clamped')

        # Add location/clamp_location object
        location_type = problem.user_type('location')
        if not problem.has_object(self._name):
            problem.add_object(self._name, location_type)

        clamp_empty = problem.fluent('clamp_empty')

        # Add initial condition
        clamp_loc = problem.object(self._name)
        problem.set_initial_value(clamp_empty(clamp_loc), True)

        clamping_possible = problem.fluent('clamping_possible')
        problem.set_initial_value(clamping_possible(clamp_loc), True)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        0
        return problem, pddl_to_world_obj_links


class DropLocationTable(BaseModule):
    def __init__(self, pddl_location_type_name = 'location', module_name = 'table_battery_drop', description = 'Batteries should be moved here.'):
        """ PDDL interface to an empty table (such as that used by vision)"""
        self._pddl_location_type_name = pddl_location_type_name

        super().__init__(module_name = module_name, description = description)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ For vision table, we only add a Location object(self.pddl_location_type_name), called self.table_name"""
        problem, pddl_to_world_obj_links = super().pddl_init(problem, pddl_to_world_obj_links)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links: Problem):

        # Get visionFeeder results
        if self._vision_info_feeder is not None:
            0
        return problem, pddl_to_world_obj_links


class VisionTable(BaseModule):
    def __init__(self, vision_info_feeder = None, pddl_location_type_name = 'location', table_name = 'table_vision'):
        """ PDDL interface to an empty table (such as that used by vision)"""
        self._vision_info_feeder = vision_info_feeder
        self._pddl_location_type_name = pddl_location_type_name
        self._name = table_name

        self._description = f"""{self._name}: Contains waste electronics that should be disassembled."""

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ For vision table, we only add a Location object(self.pddl_location_type_name), called self.table_name"""

        problem, pddl_to_world_obj_links = super().pddl_init(problem, pddl_to_world_obj_links)
        pddl_to_world_obj_links[self._name] = self
        turning_possible = problem.fluent('turning_possible')

        vision_table = problem.object(self._name)

        problem.set_initial_value(turning_possible(vision_table), True)

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links: Problem):

        # Get visionFeeder results
        if self._vision_info_feeder is not None:
            0
        return problem, pddl_to_world_obj_links
