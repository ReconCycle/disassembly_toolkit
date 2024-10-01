# Send TF2 transformacijo - Izriše v rviz

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import Transform
import time
import numpy as np


def tf_obj2x(tfo:Transform) -> list:
    """
    Takes in ROS tf object and outputs it as a list of translation components and quaternion orientation

    Args:
    -----
        - tfo (Transform) : ROS transform message object

    Returns:
    --------
        - x (list) : Received transform message in list form [x, y, z, q_w, q_x, q_y, q_z]
    """
    x = [tfo.translation.x, tfo.translation.y, tfo.translation.z, tfo.rotation.w, tfo.rotation.x,tfo.rotation.y,
        tfo.rotation.z]
    return x

class TFManager:
    """
    Manages publishing and reading TF data to and from ROS
    """
    def __init__(self,
                 max_timeout = 1):
        """ Args:
        max_timeout [s]: maximum timeout in seconds """
        # check if ros node is not initialized
        if rospy.get_node_uri() is None:
            rospy.init_node('transform_manager', anonymous = True)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.br = tf2_ros.TransformBroadcaster()
        self.empty_tf_msg =  geometry_msgs.msg.TransformStamped()

        self.max_timeout = max_timeout # s

    def tf2x(self, parent_frame:str = 'world', child_frame:str ='world') -> list:
        """
        Manages reading transforms from the buffer

        Args:
        -----
            - parent_frame(str) : TF frame in respect to which, we want to acquire the coordinates of the child frame
            - child_frame(str) : TF frame, of which coordinates we want to obtain

        Returns:
        --------
            - x(list) : Received transform message in list form [x, y, z, q_w, q_x, q_y, q_z]

        Examples:
        ---------
            >>> pose=TFManager.tf2x(parent_frame='world', child_frame='hca_kalo')
        """
        success = 0
        start_time = time.time()
        
        while success == 0:
            try:
                tpose = self.buffer.lookup_transform(source_frame = child_frame,
                                                    target_frame = parent_frame, 
                                                    time = rospy.Time(0), 
                                                    timeout = rospy.Duration(1))
                x = tf_obj2x(tpose.transform)
                success = 1
                return x
            except tf2_ros.ExtrapolationException as e:
                rospy.loginfo("Extr. exc. - tf2x: {}".format(child_frame))
                rospy.loginfo("{}".format(e))
                # This battery frame is old
                time.sleep(0.1)
                pass
            except tf2_ros.LookupException as e:
                raise ValueError("tf frame lookup failure, {}".format(e))
            
            if (time.time() - start_time) > self.max_timeout:
                success = 1
                rospy.loginfo("Could not get transform from {} to {} within max_timeout time".format(parent_frame, child_frame))
            
        return None
    
    def SendTransform2tf(self, p:list=[0,0,0],q:list=[1,0,0,0], parent_frame:str="world",child_frame:str="TEST1"):
        """
        Publishes TF2 pose to ROS so we can see it in RViz

        Args:
        -----
            - p(list) : Cartesian position coordinates [x, y, z]
            - q(list) : Quaternion orientation [qw, qx, qy, qz]
            - parent_frame(str) : TF frame in respect to which we are inputing the coordinates
            - child_frame(str) : Name of the published TF frame

        Examples:
        ---------
        >>> TFManager.SendTransform2tf(p=[4,2,0],q=[1,3,1,2], parent_frame="world",child_frame="target_pose"):

        """
        self.empty_tf_msg.header.stamp = rospy.Time.now()
        self.empty_tf_msg.header.frame_id = parent_frame

        self.empty_tf_msg.child_frame_id = child_frame

        self.empty_tf_msg.transform.translation.x = p[0]
        self.empty_tf_msg.transform.translation.y = p[1]
        self.empty_tf_msg.transform.translation.z = p[2]    
        
        self.empty_tf_msg.transform.rotation.w = q[0]
        self.empty_tf_msg.transform.rotation.x = q[1]
        self.empty_tf_msg.transform.rotation.y = q[2]
        self.empty_tf_msg.transform.rotation.z = q[3]
        
        self.br.sendTransform(self.empty_tf_msg)
        
        return 0
