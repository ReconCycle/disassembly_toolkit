import rospy
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import numpy as np
import copy
import time
from abc import ABC, abstractmethod
from PIL import Image as PILImage
import cv2
from typing import Optional, List, Union
import tf2_ros as tf2
#from robotblockset_python.transformations import *
#from context_action_framework.msg import Detection, Detections
from context_action_framework.msg import Detection, Detections
from context_action_framework.types import Label, detections_to_py
from context_action_framework.vision_interface import VisionInterface

from .safety_utils import get_visiontable_pickup_safety, get_position_within_four_bboxes
from .geometry import is_on_right_side, is_inside_rectangle

from disassembly_pipeline.utils.disassembly_objects_classes import DisassemblyObject


def check_vision_is_working(vision_interfaces: Union[List[VisionInterface], VisionInterface], disable_vision_afterwards = False):
    vision_interfaces = [vision_interfaces] if not isinstance(vision_interfaces, list) else vision_interfaces
    for vision_interface in vision_interfaces:
        vision_interface.enable_camera(True)
        time.sleep(1)
        detections = vision_interface.get_detections()
        if detections is None:
            raise ValueError("No detection message received despite enabling vision pipeline camera.")

        if disable_vision_afterwards:
            vision_interface.enable_camera(False)
    print("Vision is working")

def get_most_centered_detection_of_particular_class_from_detections(detections: List[Detection],
                                                                    desired_detection_class: Label = Label.battery):
    """ 
    Parse list of detections:
    - get all detections of class/label desired_detection_class
    - if several detections of desired class are present, select the most-centered one in regard to camera frame (normed x-y values. z is ignored.)
    Camera frame defined as per REP 103 - z forward, x right, y down

    Example use:
    >>> from context_action_framework.vision_interface import VisionInterface
    >>> vision_interface = VisionInterface(vision_topic = '/vision/realsense/detections',
                                               activate_vision_service_topic = '/vision/realsense/enable',
                                               process_img_service_topic = '/vision/realsense/process_img',
                                               run_mode = 'topic',
                                               init_ros_node = True)
    >>> detections = vision_interface.get_detections()
    >>> most_centered_battery_covered_detection = get_most_centered_detection_of_particular_class_from_detections(detections = detections,
                                                                                              desired_detection_class = Label.battery_covered)
    """
    if detections is None: raise ValueError("No detections received, not even an empty list. Restart vision system!")

    detections_of_desired_class = [det for det in detections if det.label == desired_detection_class]

    if len(detections_of_desired_class) == 0:
        raise ValueError(f"Did not find any detections of desired class {desired_detection_class}")
    # If only one detection of desired_class, return it
    elif len(detections_of_desired_class) == 1:
        most_centered_detection = detections_of_desired_class[0]
    # If several detections, find most-centered one
    else:
        centers_of_detections_x_y_values = [det.center[0:2] for det in detections_of_desired_class] # x,y in camera frame
        normed_distances = [np.linalg.norm(xy_distance) for xy_distance in centers_of_detections_x_y_values]

        idx_of_most_centered_detection = np.argmin(normed_distances)
        most_centered_detection = detections_of_desired_class[idx_of_most_centered_detection]
    
    return most_centered_detection

def check_if_detections_contain_label(detections: List[Detection], desired_label: Label):
    """ Check if a particular label/class (e.g. battery) is detected within detections."""
    detections_contain_label = False
    for det in detections:
        if det.label == desired_label:
            detections_contain_label = True
            break
    return detections_contain_label

class ROSImageListener():
    def __init__(self, image_topic):

        self.image_topic = image_topic
        img_sub = rospy.Subscriber(self.image_topic, RosImage, self._img_cb)
        self.bridge = CvBridge()
        self.last_image = None
        
        # Init ROS node if not initalized yet
        if not rospy.get_node_uri():
            rospy.init_node('image_subscriber_test', anonymous = False)

    def _img_cb(self, data):
        self.last_image = data
    def get_pil_image(self):
        ros_image = self.last_image
        if ros_image is None:
            print("Image message not yet received.")
            return None
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        # Convert OpenCV image (numpy array) to PIL image
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        return pil_image


def get_hca_pickup_with_singular_orientation(self, robot, fn_tf2x, detections, max_wait_time_s = 5):
    """ Try to get a TF from robot to HCA, WHILE checking that the orientation is correct (by comparing the PCB and HCA_frame centers.
    Also check if there is a plastic clip inside the 

    Args:

    robot : robot object (so we can get the TF)
    fn_tf2x() : the tf2x function from disassembly cycle. (a transform listener + tf2x converter)
    max_wait_time_s : maximum time in seconds to wait for result
    """

    #pf = '/' + robot.Name + '/' + robot.Name + '_link0'
    pf = robot.Name + '/' + robot.Name + '_link0'

    fix_rotation = 0
    has_plastic_clip = 0

    got_result = 0
    start_time = time.time()
    x_hca_center = None # The case where no HCA will be detected

    while (time.time() - start_time < max_wait_time_s) and (got_result==0):
        # Loop over HCA backs
        for detection in detections:
            try:
                if (Label(detection.label).name == 'hca_back'):
                    hca_type = self.get_hca_type_from_detections([detection])[0]
                    
                    rospy.loginfo("HCA type: {}".format(hca_type))
                    if hca_type == 0:
                        # It is Kalo. We don't care for the pin
                        for det in detections:
                            
                            object_type = Label(det.label).name
                            
                            if object_type == 'battery':
                                if is_inside_rectangle(detection.obb_px, det.center_px):
                                    rospy.loginfo("Found a battery inside HCA rectangle, nice")
                                    
                                    child_frame = 'battery_%d_%s'%(det.id, self.detection_pf)
                                    p = detection.tf.translation
                                    r = detection.tf.rotation
                                    #x_hca_center = [p.x, p.y, p.z, r.w, r.x, r.y, r.z]
                                    x_hca_center = fn_tf2x(parent_frame=pf, child_frame='hca_back_%d_%s'%(detection.id, self.detection_pf))
                                    #x_hca_center = detection.tf.
                                    rospy.loginfo("x_hca_center: {}".format(x_hca_center))
                                    x_battery_center = fn_tf2x(parent_frame=pf, child_frame=child_frame)
                                    p_b = det.tf.translation
                                    r_b = det.tf.rotation
                                    #x_battery_center = [p_b.x, p_b.y, p_b.z, r_b.w, r_b.x, r_b.y, r_b.z]
                                    if x_hca_center[0] <= x_battery_center[0]:
                                        rospy.loginfo("Rotation is OK")
                                    else:
                                        rospy.loginfo("HCA rotation is not OK, fix rotation")
                                        fix_rotation = 1
                    elif hca_type == 1:
                        # It is Qundis
                        for det in self.detections:
                            object_type = Label(det.label).name
                            
                            if object_type == 'plastic_clip':
                                # Checking the "Plastic_clip" detection to determine if we should pinpush
                                if is_inside_rectangle(detection.obb_px, det.center_px):
                                    has_plastic_clip = 1
                            
                            # Checking the PCB for determining the orientation.
                            if object_type == 'pcb_covered':
                                if is_inside_rectangle(detection.obb_px, det.center_px):
                                    rospy.loginfo("Found a covered PCB inside HCA rectangle, nice")
                                    
                                    child_frame = 'pcb_covered_%d_%s'%(det.id, self.detection_pf)
                                    p_hca = detection.tf.translation
                                    r_hca = detection.tf.rotation
                                    #x_hca_center = [p_hca.x, p_hca.y, p_hca.z, r_hca.w, r_hca.x, r_hca.y, r_hca.z]
                                    x_hca_center = fn_tf2x(parent_frame=pf, child_frame='hca_back_%d_%s'%(detection.id, self.detection_pf))

                                    p_pcb = det.tf.translation
                                    r_pcb = det.tf.rotation
                                    
                                    #x_pcb_center = [p_pcb.x, p_pcb.y, p_pcb.z, r_pcb.w, r_pcb.x, r_pcb.y, r_pcb.z]
                                    x_pcb_center = fn_tf2x(parent_frame=pf, child_frame=child_frame)
                                    
                                    if x_pcb_center[0] <= x_hca_center[0]:
                                        rospy.loginfo("Rotation is OK")
                                        fix_rotation = 0
                                    else:
                                        rospy.loginfo("HCA rotation is not OK, fix rotation")
                                        fix_rotation = 1

                    # GO OUT OF LOOP, IT IS ENOUGH TO FIND ONE
                    got_result = 1
                    break
            except TypeError:
                    
                rospy.loginfo("VU Couldnt get transform")
                return 0

    if got_result:
        if x_hca_center == None:
            # For some reason we didn't find it. DEBUG.
            rospy.loginfo("HCA not found")
            return 1
                        
        T_hca = x2t(x_hca_center)
        # Now rotate the HCA frame so that the Z will be pointing downwards, like the Z of the robot EE
        R_hca = T_hca[0:3,0:3]@rot_x(180, unit= 'deg')
        
        if R_hca[0,0] < 0:
            R_hca = R_hca@rot_z(180, unit ='deg')
            
        # Now the X will be pointing away from the robot (that is, same of robot link0 X.
        #if fix_rotation:
        #    R_hca = T_hca[0:3,0:3]@rot_z(180, unit='deg')
            #T_hca[0:3, 0:3] = R_hca
        # Now rotate the HCA frame so that the Z will be pointing downwards, like the Z of the robot EE
        # Now check if HCA y is poining in the direction of the robot positive X (panda_1 !)
        
        T_hca[0:3,0:3] = R_hca
        # Fix rotation tells us whether it's rotated the wrong way, so later we have to position it in the vise differently
        return T_hca, fix_rotation, has_plastic_clip, Label(hca_type).name
    else: 
        return None, None, None, None

def get_battery(self, fn_tf2x, robot_frame = 'panda_2/panda_2_link0', timeout = 3):
    # Bounding box TFs
    origin_frame = "cutter"
    b1 = "cutter/plate_inedge1"
    b2 = "cutter/plate_inedge2"
    b3 = "cutter/plate_outedge1"
    b4 = "cutter/plate_outedge2"
    
    issafe = 0
    success = 0
    
    while (issafe ==0) or (success == 0):
            batteries = self.get_particular_class_from_detections(detections = [], desired_class = 'battery', check_size = True)
            
            # Keep track of all battery detection so we can choose the one in the middle
            all_detections = []
            
            #rospy.loginfo("Batteries: {}".format(batteries))
            for battery in batteries:
                bf = "battery_%d_realsense"%battery.id
                try:
                    rospy.loginfo("Bf: {}".format(bf))
                    
                    x_r2batt = fn_tf2x(parent_frame = robot_frame, child_frame=bf)
                    # Get the tf
                    #
                    # Check if its safe
                    issafe = get_position_within_four_bboxes(tf2x=fn_tf2x, origin_frame =origin_frame, b1=b1, b2 = b2,
                                                            b3 = b3, b4 = b4, target_frame = bf)
                    rospy.loginfo("Issafe: {}".format(issafe))
                    if issafe in [1,3,4,5,6,7]:
                        d = [battery, x_r2batt, issafe]
                        
                        all_detections.append(d)
                        
                        #success = 1
                        #break
                    
                except tf2.LookupException as e:
                    # This battery frame does not exist
                    rospy.loginfo("Lookup exc. - bf: {}".format(bf))
                    rospy.loginfo("{}".format(e))

                    pass
                except tf2.ExtrapolationException as e:
                    rospy.loginfo("Extr. exc. - bf: {}".format(bf))
                    rospy.loginfo("{}".format(e))
                    # This battery frame is old
                    pass
            
            # Now find best battery
            #for result in all_detections:
            
            # Temporary solution for now. Pick up a random battery. Guaranteed to not be on left edge
            if len(all_detections)>0:
                for i in range(0,len(all_detections)):
                    if issafe in [1,3,4,5,6,7]:
                        battery_detection = all_detections[i][0]
                        x_r2batt = all_detections[i][1]
                        issafe = all_detections[i][2]
                        bf = "battery_%d_realsense"%battery_detection.id
                        
                        success = 1
                        rospy.loginfo("KK - ISsafe {}".format(issafe))
                        return bf, x_r2batt, issafe
                        break
                        
            time.sleep(0.05)
            
    # If we get to here, we didnt get a good result
    return None, None, None

def get_smoke_detector_x(vision_utils, tf2x, robot_base_frame, pickup_z, pickup_quaternion = None):
    """ Args:
    
    example_pickup_pose_x: from vision we get only X and Y. From example_pickup_pose_x, we will use the Z value and the rotation quaternion.
                            """

    #vision_utils.update_detections(timeout = 1)
    #detected_smoke_detectors = vision_utils.get_particular_class_from_detections(desired_class = 'firealarm')
    detected_smoke_detectors = vision_utils.get_detections(desired_class = Label.smoke_detector, timeout = 2)
    if (detected_smoke_detectors is None) or (len(detected_smoke_detectors) == 0):
        return None

    print("Detected %d smoke detectors"%len(detected_smoke_detectors))

    random_idx = np.random.randint(0, len(detected_smoke_detectors))
    detection = detected_smoke_detectors[random_idx]
    if detection is not None:
        if 'front' in detection.tf_name:
            detector_type = 'fumonic'
        elif 'back' in detection.tf_name:
            detector_type = 'hekatron'

        #smokedet_tf_name = detection.tf_label.name + '_%d_%s'%(detection.id, vision_utils.parent_frame)
        smokedet_tf_name = detection.tf_name
        x_pickup = tf2x(child_frame = smokedet_tf_name, parent_frame = robot_base_frame)

    if x_pickup is None:return None
    smoke_detector_pickup_x = np.zeros(7)
    smoke_detector_pickup_x[0] = x_pickup[0]
    smoke_detector_pickup_x[1] = x_pickup[1]
    smoke_detector_pickup_x[2] = pickup_z
    # Allow forcing pickup rotation
    if pickup_quaternion is not None:
        smoke_detector_pickup_x[3:] = pickup_quaternion
    else:
        # Take rotation as detected by vision
        raise Exception("rotation as given by vision utils is incorrect and would result in gripper being upside down. Handle this case.")
        smoke_detector_pickup_x[3:] = x_pickup[3:]

    return {"smoke_detector_type": detector_type, "x": smoke_detector_pickup_x}

def set_realsense_height(realsense_height_param_name:str=None, Z:float=0.34):

    if realsense_height_param_name == None:
        realsense_height_param_name = '/vision/realsense_height'
    # Set param for realsense height ( distance between realsense tf and the PLANE in which you want to calculate real-world coordinates). 
    rospy.set_param(realsense_height_param_name, Z)

def get_realsense_height(topic:str='/vision/realsense_height') -> float:
    Z = rospy.get_param(topic)
    rospy.loginfo("Realsense: Z height set to {} m".format(Z))

    return Z