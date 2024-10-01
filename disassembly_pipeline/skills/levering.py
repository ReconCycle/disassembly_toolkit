from .base_skill import BaseSkill
from disassembly_pipeline.utils.Levering_block_v3 import LeverBlock


class Levering(BaseSkill):
    def __init__(self, 
                name = "lever",
                action_arg_names = ['object'],
                description = "Lever out a PCB from an HCA object. Object must be clamped beforehand."):

        """"""        
        self.levering_block = LeverBlock()
        super().__init__(name = name, description = description, action_arg_names = action_arg_names)
        
    def on_enter_pddl(self, **kwargs):
        return self.on_enter(**kwargs)

    def on_enter(self, **kwargs):
        #print(kwargs)
        robot = kwargs['robot']
        self.levering_block.on_enter(robot = robot)

    def execute(self):
        0
    def on_exit(self):
        0
    def pddl_init(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, static means it runs when the PDDL env is initialized (add objects, predicates/fluents, operators...)."""
        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Modify the PDDL problem, dynamic means it runs in a loop (add objects, predicates/fluents)."""
        return problem, pddl_to_world_obj_links