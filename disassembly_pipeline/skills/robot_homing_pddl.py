from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from disassembly_pipeline.skills.base_skill import BaseSkill

from unified_planning.shortcuts import *
from unified_planning.model import Problem


class RobotHomingPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                 skill_object: BaseSkill = None):
        """
        Args:
        is_robot_homed_fluent_name: str: what is the fluent/predicate, which determines whether robot is homed, called in given PDDL problem?
        pddl_robot_object_name: str: what is a robot object type called/defined as in given PDDL problem?

        Example call:
        >>> homing_sk = RobotHoming()
        >>> homing_sk.on_enter(**{'robot': panda_2_robotblockset_object})
        """

        name = 'robot_homing'
        description = 'Home a robot.'
        action_arg_names = {'robot': 'Robot to home.'}

        self._pddl_is_homed_fluent_name = 'is_robot_homed'

        self._pddl_robot_object_name = 'robot'

        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem: Problem, pddl_to_world_obj_links: dict):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not pddl_problem.has_action(self.get_name()):
            # Add robot_homing operator
            robot_type = UserType(self._pddl_robot_object_name)

            homing_operator = InstantaneousAction(self.get_name(), robot = robot_type)
            robot = homing_operator.parameter(self._pddl_robot_object_name)

            if not pddl_problem.has_fluent(self._pddl_is_homed_fluent_name):
                is_homed = unified_planning.model.Fluent(self._pddl_is_homed_fluent_name, BoolType(), robot =  robot_type)
            else:
                is_homed = pddl_problem.fluent(self._pddl_is_homed_fluent_name)
            homing_operator.add_effect(is_homed(robot), True)

            pddl_problem.add_action(homing_operator)

        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, pddl_problem: Problem, pddl_to_world_obj_links: dict):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return pddl_problem, pddl_to_world_obj_links
