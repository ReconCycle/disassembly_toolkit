from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class SkillExecutionResult:
    """ The expected output of a skill call."""
    success: bool = False
    textual_result: str = "Textual description of the result, e.g. 'the robot was homed'"
    # Dictionary to track changes in internal variables for each input argument.
    modified_variables_dict: dict = field(default_factory=dict)  # e.g {'robot': {'is_homed': True}}


class BaseSkill(ABC):
    def __init__(self,
                 **kwargs):
        """ Template object for a generic skill. Skills can be performed by agents (brain and robots) on
        environment objects and can change the environment state. """
        pass

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        """
        By default, pass all arguments directly to on_enter!
        Enable OPTIONALLY mapping from PDDL-argument-names to this skill's on_enter argument names.
        
        (e.g. pddl arg name might be robot, but the downstream function's on_enter arg might be called robot_to_home)
        1. e.g. pddl_kwargs = {'robot': robotblockset_python} 
        2. robot_homing_skill.on_enter(robot_to_home = None)
        3. robot_to_home = pddl_kwargs['robot']
        4. robot_homing_skill.on_enter(robot_to_home = robot_to_home)

        This function gets called by the PDDL wrapper."""
        result = self.on_enter(**kwargs)
        return result

    @abstractmethod
    def on_enter(self, **kwargs) -> SkillExecutionResult:
        """ Function to call when first starting the skill."""
        pass

    @abstractmethod
    def execute(self, **kwargs):
        """ Function that gets called periodically (possibly with high freq) during skill execution. """
        pass

    @abstractmethod
    def on_exit(self, **kwargs):
        """ Function that gets called after the skill finishes."""
        pass
