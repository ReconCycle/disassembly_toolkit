from unified_planning.shortcuts import *
from unified_planning.exceptions import UPValueError
from disassembly_pipeline.skills.base_skill import BaseSkill


class Clamp(BaseSkill):
    def __init__(self):
        0
    def on_enter(self, location = None, **kwargs):
        """ 
        # from disassembly_pipeline.workcell_modules.vise_module import ViseModule
        location: BaseWorkcellModule with function clamp, unclamp
        
        location = ViseModule()
        
        """
        if not hasattr(location, 'clamp'):
            raise ValueError(f"Object {location} should have function 'clamp' defined.")
        location.clamp()
        
    def execute(self):
        0
    def on_exit(self):
        0


class Unclamp(BaseSkill):
    def __init__(self):
        0
    def on_enter(self, location, **kwargs):
        if not hasattr(location, 'unclamp'):
            raise ValueError(f"Object {location} should have function 'unclamp' defined.")
        location.unclamp()
    def execute(self):
        0
    def on_exit(self):
        0