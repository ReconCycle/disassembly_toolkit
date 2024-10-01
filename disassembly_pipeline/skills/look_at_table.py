import copy
from typing import Union, List
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from robotblockset_python.robots import robot as Robot
from disassembly_pipeline.utils.tf_utils import TFManager


class LookAtTable(BaseSkill):
    def __init__(self,
                 tf_manager: TFManager = None,
                 config_file = 'config/look_at_table.json'):
        """ 
        Look at a location within the workcell by moving a robot with an eye-in-hand camera there.
        Positions are defined in config file, with schematic "config[location_name][location_subframe][robot_name]"
        The fields in config should be "initial_q" and "final_x". If initial_q is defined, a joint move to this q configuration is first performed.

        Example call:
        >>> from disassembly_pipeline.skills.look_at_table import LookAtTable
        >>> look_at = LookAtTable()
        >>> look_at.on_enter(robot = panda_2, location = 'table_cnc')
        """

        if tf_manager is None:
            tf_manager = TFManager()
        
        self.tf_manager = tf_manager
        self.config_file = config_file
        super().__init__()

    def on_enter_pddl(self, **kwargs) -> SkillExecutionResult:
        return self.on_enter(**kwargs)

    def on_enter(self,
                 robot: Robot,
                 location: Union[str, BaseWorkcellModule],
                 location_subframe:str = 'default',
                 return_to_init_q = False,
                 return_to_home = False,
                 **kwargs) -> SkillExecutionResult:
        """ 
        location - name of table (table_cnc, ...)
        location_subframe - more specific pose within location, e.g. 'default', 'chuck_fumonic', 'chuck_hekatron' 
        """

        # Read config file
        config = read_json_from_package('disassembly_pipeline', self.config_file)
        location_name = location if isinstance(location, str) else location.get_name()        
        robot_name = robot.Name
        
        initial_q_and_final_x_poses = config[location_name][location_subframe][robot_name]
        init_q = initial_q_and_final_x_poses['initial_q'] if 'initial_q' in initial_q_and_final_x_poses.keys() else None
        final_x = initial_q_and_final_x_poses['final_x'] if 'final_x' in initial_q_and_final_x_poses.keys() else None
        
        initial_robot_controller = copy.deepcopy(robot._control_strategy)
        if robot._control_strategy != 'JointPositionTrajectory':
            robot.Switch_controller(start_controller = 'JointPositionTrajectory')

        robot.error_recovery()

        if init_q is not None:
            robot.JMove(init_q, 2, qdot_max_factor = 0.7, qddot_max_factor = 0.7)
        if final_x is not None:
            robot.CMove(final_x, 2, v_max_factor = 0.7, a_max_factor = 0.7)        

        if return_to_init_q:
            robot.JMove(init_q, 2, qdot_max_factor = 0.7, qddot_max_factor = 0.7)
        if return_to_home:
            homing = RobotHoming()
            homing.on_enter(robot = robot)
            
    def execute(self):
        0
        
    def on_exit(self):
        0