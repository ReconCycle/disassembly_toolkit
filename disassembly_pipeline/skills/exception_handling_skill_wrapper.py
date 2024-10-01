from disassembly_pipeline.skills.base_skill import BaseSkill, SkillExecutionResult
from disassembly_pipeline.skills.robot_homing import RobotHoming
from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from disassembly_pipeline.utils.vision_utils import check_if_detections_contain_label, get_most_centered_detection_of_particular_class_from_detections
from context_action_framework.types import Label
from robotblockset_python.robots import robot as Robot
from context_action_framework.vision_interface import VisionInterface
import time

class ExceptionHandlingSkillWrapper():
    def __init__(self,
                 vlm_client = None,
                 camera_raw_img_interface = None):
        """ 
        >>> from vlm_msgs.vlm_client import VLMClient
        >>> from disassembly_pipeline.utils.vision_utils import ROSImageListener
        
        >>> vlm_client = VLMClient()
        >>> camera_raw_img_interface = ROSImageListener(image_topic = '/realsense/color/image_raw')
        
        >>> exc_skill_wrp = ExceptionHandlingSkillWrapper(vlm_client = vlm_client, camera_raw_img_interface = )
        
        In case of skill failure, provide
        exception handling and re-execute skill.
        Example for battery cover removal:
        move_robot_above_cnc()
        success = False
        while success = False:
            look_at_location()

            
            if not battery_cover_is_detected():
                success = True
                break
            remove_battery_cover()
        """
        self.vlm_client = vlm_client
        self.camera_raw_img_interface = camera_raw_img_interface
        
    def on_enter(self,
                 robot: Robot,
                 vision_interface: VisionInterface, 
                 look_at_skill: BaseSkill,
                 look_at_skill_kwargs: dict,
                 base_skill: BaseSkill,
                 base_skill_kwargs: dict,
                 object_to_handle: Label,
                 do_initial_homing = True,
                 do_initial_look_at = False,
                 request_vlm_feedback = True,
                 disable_camera_afterwards = False,
                 max_n_retries = 2,
                 return_to_home = False):
        """ 
        Pseudocode:
        - home robot if specified
        - call look_at function and get vision detections
        - while success == False:
            - execute base skill
            - call look_at function and get detections again. 
            - if n_new_detections of Label==object_to_handle: success = True
        """
        if do_initial_homing:
            homing = RobotHoming()            
        
        initial_n_of_desired_class_objs = None
        MAX_N_RETRIES = max_n_retries
        current_n_retries = 0
        success = False

        vision_interface.enable_camera()

        success, all_detections, initial_n_of_desired_class_objs, cur_n_of_desired_class_objs = self.vision_loop(look_at_skill,
                                                                                look_at_skill_kwargs,
                                                                                vision_interface,
                                                                                object_to_handle,
                                                                                initial_n_of_desired_class_objs)

        while (success is False) and (current_n_retries < MAX_N_RETRIES):

            detection = get_most_centered_detection_of_particular_class_from_detections(all_detections, object_to_handle)
            base_skill_kwargs['detection'] = detection

            result = base_skill.on_enter(**base_skill_kwargs)

            success_vision_detections, all_detections, initial_n_of_desired_class_objs, cur_n_of_desired_class_objs = self.vision_loop(look_at_skill,
                                                                                look_at_skill_kwargs,
                                                                                vision_interface,
                                                                                object_to_handle,
                                                                                initial_n_of_desired_class_objs)
            success_vlm = True # True by default if vlm is not available
            if (request_vlm_feedback and 
                (self.vlm_client is not None) and 
                (self.camera_raw_img_interface is not None)):
                
                expected_action_result = ""
                camera_img = self.camera_raw_img_interface.get_pil_image()
                vlm_output = self.vlm_client.request_vlm_prediction(system_prompt = "You are an AI assistant determining if robotic skill execution was successful.",
                              images = [camera_img], 
                              image_descriptions=['Live camera image.'],
                              user_prompt = f'Expected result of robotic skill execution is: {expected_action_result}',
                              objective = "Determine if robotic skill execution was successful. Answer ONLY with yes or no!",
                              request_json_output = False)
                
                if 'no' in vlm_output.lower():
                    success_vlm = False
                
            success = success_vision_detections and success_vlm # Only consider it success if both vlm and vision report same result
                
            current_n_retries +=1
        
        if disable_camera_afterwards:
            vision_interface.enable_camera(False)

        return success

    def vision_loop(self,
                    look_at_skill,
                    look_at_skill_kwargs,
                    vision_interface,
                    object_to_handle,
                    initial_n_of_desired_class_objs):
        """ 
        Vision-based loop to determine if robotic action was successful.
        """
        success = False

        look_at_skill.on_enter(**look_at_skill_kwargs)
        time.sleep(0.5)
        all_detections = vision_interface.get_detections()

        # Break out if no relevant class objects detected
        if not check_if_detections_contain_label(all_detections, object_to_handle):
            success = True
            cur_n_of_desired_class_objs = 0
            print("Detection not found anymore")
        # Else remember the INITIAL number of relevant class objects
        elif initial_n_of_desired_class_objs is None:
            initial_n_of_desired_class_objs = len([det for det in all_detections if det.label == object_to_handle])
            cur_n_of_desired_class_objs = initial_n_of_desired_class_objs
        # Else determine if number of objects of relevant class HAS DECREASED (i.e. our action has succeeded so we can break out.)
        elif initial_n_of_desired_class_objs is not None:
            cur_n_of_desired_class_objs = len([det for det in all_detections if det.label == object_to_handle])
            if cur_n_of_desired_class_objs < initial_n_of_desired_class_objs:
                success = True
                print(f"Number of detected objects of class {object_to_handle} has decreased from \
                        {initial_number_of_detected_object_of_desired_class} to {current_n_retries}, so action is considered successful.")

        return success, all_detections, initial_n_of_desired_class_objs, cur_n_of_desired_class_objs