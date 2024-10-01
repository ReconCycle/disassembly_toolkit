from action_prediction_interface.pddl_modules.base_pddl_skill import BaseSkillPDDLWrapper


class ArmwavePDDLWrapper(BaseSkillPDDLWrapper):
    def __init__(self):

        name = "wave_robot_arm"
        description = "The robot will wave to greet guests/bystanders. Arguments: robot"

        super().__init__(name = name, description = description)

    def pddl_init(self, problem, pddl_to_world_obj_links):

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        return problem, pddl_to_world_obj_links