from disassembly_pipeline.skills.base_skill import BaseSkill
from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper
from unified_planning.shortcuts import *


class LinearCutterPCBTouchOffAndCutPDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self,
                skill_object: BaseSkill = None):
        """

        """
        name = 'cut_pcb'
        action_arg_names = {'robot': 'robotblockset_python.robot.Robot', 'location': 'action_prediction_interface.pddl_modules.base_table.BaseTable','object': ['hca']}
        description = "Place a PCB within the cutter and cut off the battery so it can be later picked up."
        super().__init__(name = name, description = description, action_arg_names = action_arg_names, skill_object = skill_object)

    def pddl_init(self, pddl_problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""

        if not pddl_problem.has_action(self.get_name()):
            # Other needed fluents

            robot_type = UserType('robot')
            location_type = UserType('location')
            physical_object_type = UserType('physical_object')

            pcb_cut = InstantaneousAction(self.get_name(), robot = robot_type, location = location_type, object = physical_object_type)

            robot = pcb_cut.parameter('robot')
            location = pcb_cut.parameter('location')
            object = pcb_cut.parameter('object')

            #pcb_cut.add_precondition(holding(robot, object))
            #lever.add_precondition(at_location(object, location))  # Must be on table where we lever up from
            #lever.add_precondition(clamped(location, object))  # Must be clamped be clamped at any location

            # No effects as yet

            pddl_problem.add_action(pcb_cut)
        return pddl_problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links
