from robotblockset_python.transformations import x2t, t2x, q2r
from robotblockset_python.robots import robot as Robot
from disassembly_pipeline.utils.tf_utils import TFManager
from context_action_framework.vision_interface import VisionInterface
from context_action_framework.types import Label
import copy
import time


class VisualServoing:
    def __init__(self,
                 robot: Robot,
                 robot_camera_frame_tf_name: str,
                 robot_ee_frame_tf_name: str,
                 vision_system_interface: VisionInterface,
                 tf_manager: TFManager):

        """ 
        Example use:
        >>> vi = VisionInterface(vision_topic = '/vision/realsense/detections')
        >>> tf_manager = TFManager()
        >>> visual_servoing = VisualServoing(robot = p2, 
                                             robot_camera_frame_tf_name = 'panda_2/realsense',
                                             robot_ee_frame_tf_name = 'panda_2_EE',
                                             vision_system_interface = vi,
                                             tf_manager = tf_manager)
        >>> visual_servoing.step(detection_label_to_center_on = Label.battery)
        OR 
        >>> visual_servoing.on_enter(detection_label_to_center_on = Label.battery)

        """

        self.r = robot
        self.robot_camera_frame = robot_camera_frame_tf_name
        self.robot_ee_frame = robot_ee_frame_tf_name
        
        self.vision_interface = vision_system_interface
        self.tf_manager = tf_manager

        self.tracked_detection_name = None

    def on_enter(self, detection_label_to_center_on: Label = Label.battery):
        
        eps = 0.005 # m 
        max_n_steps = 5
        
        positioning_error = 100
        cur_n_steps = 0
        while positioning_error > eps:
            target_object_frame_name = self.step(detection_label_to_center_on = detection_label_to_center_on)
            time.sleep(0.5)
            positioning_error = self.get_positioning_error(object_tf_name = target_object_frame_name, camera_tf_name = self.robot_camera_frame)
            print(positioning_error)

            cur_n_steps += 1
            if cur_n_steps >= max_n_steps:
                print(f"Didn't achieve sufficiently low error within max retries {max_n_steps}")
                break

        return 0
        
    def step(self,
             detection_label_to_center_on: Label = Label.battery):

        robot_base_frame = self.r.Base_link_name
        
        if self.tracked_detection_name is None:
            0 #print(detections)
        else:
            0
        
        #object_center_in_camera_frame = 
        detections = self.vision_interface.get_detections(desired_class = detection_label_to_center_on)

        target_object = detections[0]
        target_object_frame_name = target_object.tf_name
        
        object_x_in_robot_base_frame = self.tf_manager.tf2x(parent_frame = robot_base_frame, child_frame = target_object_frame_name)
        object_T_in_robot_base_frame = x2t(object_x_in_robot_base_frame)

        self.r.GetState()
        cur_robot_x = self.r.x
        cur_robot_T = self.r.T

        camera_x = self.tf_manager.tf2x(parent_frame = robot_base_frame, child_frame = self.robot_camera_frame)
        camera_T = x2t(camera_x)

        # Calculate final camera pose so that it's above the object but keeping current rotation
        desired_final_camera_pose = copy.deepcopy(object_T_in_robot_base_frame)
        desired_final_camera_pose[0:3, 0:3] = camera_T[0:3, 0:3] # Keep robot camera orientation
        desired_final_camera_pose[2, -1] = camera_T[2, -1] # Keep camera height Z constant

        desired_final_camera_x = t2x(desired_final_camera_pose)
        #self.tf_manager.SendTransform2tf(p= desired_final_camera_x[0:3], q = desired_final_camera_x[3:], parent_frame = r.Base_link_name, child_frame = 'TEST')

        # Calculate EE pose so that camera is at desired_final_camera_pose
        robot_ee_to_camera = self.tf_manager.tf2x(parent_frame = self.robot_ee_frame, child_frame = self.robot_camera_frame)
        robot_ee_to_camera = x2t(robot_ee_to_camera)
        #desired_camera_pose = base_to_ee@ee_to_camera
        base_to_ee = x2t(desired_final_camera_x)@np.linalg.inv(robot_ee_to_camera)

        xx = t2x(base_to_ee)
        self.tf_manager.SendTransform2tf(p= xx[0:3], q = xx[3:], parent_frame = r.Base_link_name, child_frame = 'TEST')
        # Robot moves
        if 1:
            0
            distance_to_move = xx[0:3] - cur_robot_x[0:3]
            distance_to_move[2] = 0 # Keep robot camera height Z constant
            print("Distance: ", distance_to_move)
    
            #self.r.CMoveFor(distance_to_move, 1.5) # TODO parametric time
            self.r.CMove(xx, 1.5)
            
        return target_object_frame_name

    def get_positioning_error(self, object_tf_name, camera_tf_name):
        dx = self.tf_manager.tf2x(parent_frame = object_tf_name, child_frame = camera_tf_name)
        dx = dx[0:2] # only x,y. Ignore Z and rotation error

        error = np.linalg.norm(dx)
        return error