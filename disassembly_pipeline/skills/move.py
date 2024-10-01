import numpy as np
from robotblockset_python.transformations import *
from .base_skill import BaseSkill
from disassembly_pipeline.skills.drop_object import DropObject
from disassembly_pipeline.skills.pickup_object import PickupObject


class Move(BaseSkill):
    def __init__(self):
        0
        
    def on_enter_pddl(self, **kwargs):
        return self.on_enter(**kwargs)
        #return 0
        
    def on_enter(self, **kwargs):
        robot = kwargs['robot']
        location_from = kwargs['l_from']
        location_to = kwargs['l_to']
        object_to_move = kwargs['object']
        
        out = None
        # Pick up
        p = PickupObject()
        out = p.on_enter_pddl(**{'robot':robot, 'object': object_to_move, 'location_from': location_from})

        # Drop
        d = DropObject()
        out = d.on_enter(robot = robot, drop_location = location_to, drop_object = object_to_move)
        
        return out
        
    def execute(self):
        0

    def on_exit(self):
        0