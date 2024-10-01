from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from disassembly_pipeline.utils.robot_quick_init import initialize_robot
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package
from context_action_framework.vision_interface import VisionInterface


class Panda2Module(BaseWorkcellModule):
    def __init__(self,
                 name = 'panda_2_module',
                 activate_eye_in_hand_camera = True
                 ):

        self.r = initialize_robot(robot_name = 'panda_2',
                     tool_name = 'VariableStiffnessGripper',
                     start_controller = 'position_joint_trajectory_controller',
                     collision_thresholds = {"F": 50, "T": 20, "tq": 30},
                     gripper_init_kwargs = {**{'__ns':'/qbmove1'}},
                     toolchanger = True,
                     toolchanger_init_kwargs = {})

        if activate_eye_in_hand_camera:
            config_json = read_json_from_package(package_name = 'disassembly_pipeline',
                                                relative_json_path_within_package = 'config/camera_topic_params.json')
            camera_params = config_json['realsense']
            detections_topic = camera_params['detections_topic']
            activate_vision_topic = camera_params['activate_vision_topic']
            process_img_service_topic = camera_params['process_img_service_topic']
            vision_interface = VisionInterface(vision_topic = detections_topic,
                                            activate_vision_service_topic = activate_vision_topic,
                                            process_img_service_topic = process_img_service_topic,
                                            run_mode = 'topic',
                                            init_ros_node = True)
            
            self.r.camera_vision_interface = vision_interface
        
        super().__init__(name = name)

    def get_callable_submodules(self):
        return dict(robot = self.r)