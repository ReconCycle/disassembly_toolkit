import rospy
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from disassembly_pipeline.utils.robot_quick_init import initialize_robot

from digital_interface_msgs.srv import PinStateWrite


class Panda1Module(BaseWorkcellModule):
    def __init__(self, name = 'panda_1_module'):

        self.r = initialize_robot(robot_name = 'panda_1',
                      tool_name = 'ThreeJawChuck',
                      start_controller = 'position_joint_trajectory_controller',
                      collision_thresholds = {"F": 70, "T": 20, "tq": 40},
                      gripper_init_kwargs = {},
                      toolchanger = True,
                      toolchanger_init_kwargs = {})

        # Airblock which provides pneumatics
        self.activate_panda1_airblock_svc = rospy.ServiceProxy('/airblock2_air_ON', PinStateWrite)
        
        super().__init__(name = name)

    def get_callable_submodules(self):
        return dict(robot = self.r,
                    airblock_service = self.activate_panda1_airblock_svc
                   )

    def enable_airblock(self):
        self.activate_panda1_airblock_svc.call(True)
        
    def disable_airblock(self):
        self.activate_panda1_airblock_svc.call(False)

        