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


class DrillBatteryContacts(BaseSkillPDDLWrapper):
    def __init__(self):
        pass
