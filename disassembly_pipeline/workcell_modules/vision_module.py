from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from disassembly_pipeline.utils.read_json_from_package import read_json_from_package
from context_action_framework.vision_interface import VisionInterface


class VisionModule(BaseWorkcellModule):
    def __init__(self,
                 name = 'vision_module',
                 config_file_location_within_disassembly_pipeline = '/config/camera_topic_params.json'):

        config_json = read_json_from_package(package_name = 'disassembly_pipeline',
                                             relative_json_path_within_package = 'config/camera_topic_params.json')

        self._cameras_dict = {}
        for key, value in config_json.items():
            camera_name = key
            
            location = value['location'] 
            image_topic = value['image_topic']
            detections_topic = value['detections_topic']
            activate_vision_topic = value['activate_vision_topic']
            process_img_service_topic = value['process_img_service_topic']

            vision_interface = VisionInterface(vision_topic = detections_topic,
                                               activate_vision_service_topic = activate_vision_topic,
                                               process_img_service_topic = process_img_service_topic,
                                               run_mode = 'topic',
                                               init_ros_node = True)
            
            self._cameras_dict[camera_name] = dict(location = location,
                                                   image_topic = image_topic,
                                                   detections_topic = detections_topic,
                                                   vision_interface = vision_interface)

        super().__init__(name = name)

    def get_callable_submodules(self):
        return self._cameras_dict