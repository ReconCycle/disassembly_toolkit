import copy
from typing import Union, List
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from robotblockset_python.robots import robot as Robot


class JMoveAboveTable(BaseSkill):
    def __init__(self,
                 pose_db = 'config/jmove_above_table.json'):
        """ 
        Look at a location within the workcell by moving a robot with an eye-in-hand camera there.

        Example call:
        >>> from disassembly_pipeline.skills.look_at_table import LookAtTable
        >>> look_at = LookAtTable()
        >>> look_at.on_enter(robot = panda_2, location = 'table_cnc')
        """
        self.pose_db = pose_db
        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        return self.on_enter(**kwargs)

    def on_enter(self,
                 robot: Robot,
                 location: Union[str, BaseWorkcellModule],
                 return_to_home = False) -> SkillExecutionResult:

        # Read config file
        pose_db = read_json_from_package('disassembly_pipeline', self.pose_db)
        location_name = location if isinstance(location, str) else location.get_name()        
        robot_name = robot.Name
        
        init_q = pose_db[location_name][robot_name]['initial_q'] 
        
        initial_robot_controller = copy.deepcopy(robot._control_strategy)
        if robot._control_strategy != 'JointPositionTrajectory':
            robot.Switch_controller(start_controller = 'JointPositionTrajectory')

        robot.error_recovery()
        robot.JMove(init_q, 2, qdot_max_factor = 0.55, qddot_max_factor = 0.55)

        if return_to_home:
            homing = RobotHoming()
            homing.on_enter(robot = robot)
            
    def execute(self):
        0
        
    def on_exit(self):
        0