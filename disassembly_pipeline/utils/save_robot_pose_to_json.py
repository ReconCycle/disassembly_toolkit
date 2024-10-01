import rospy
import json
import numpy as np
import os
import sys
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
import shutil
from robotblockset_python.transformations import r2q


"""
You can either run this file as a standalone script and set
system args as 1) robot_name, 2) entry_name, 3) JSON file path
(you can leave out the JSON path to use the default one)

e.g. python3 save_robot_pose_to_json.py panda_1 test_entry ../poses/pose_database.json

or you can import the function below from some other Python script and use it
"""
def save_state_to_json(robot_name:str, entry_name:str, 
                       json_path:str='/devel_ws/src/disassembly_pipeline/disassembly_pipeline/poses/pose_database.json'):
    """
    This saves to JSON file

    State will be saved as:
        pose_db[robot_name][entry_name]["pose"]
        pose_db[robot_name][entry_name]["joints"]

    Returns (pose, joints)
    Pose is (position, xyzw_quaternion), joints are joints!
    """ 


    pose, joints = _get_current_state(robot_name)
    
    print("Creating backup of the pose database file...")
    shutil.copyfile(json_path, json_path + ".bak")
    print("Saved backup")
    
    with open(json_path) as fp:
        try:
            pose_db = json.load(fp)
        except Exception:
            raise ValueError("Could not open JSON file! Does it exist?")
       
        _update_pose_db(pose_db, robot_name, entry_name, pose, joints)

    with open(json_path, 'w') as fp:
        json.dump(pose_db, fp, indent=4)

    print("JSON file successfully updated with the new entry")

def save_state_to_rosparam(robot_name:str, entry_name:str, 
                       param_path:str='/pose_db'):
    """
    This updates entire pose db

    State will be saved as:
        pose_db[robot_name][entry_name]["pose"]
        pose_db[robot_name][entry_name]["joints"]

    Returns (pose, joints)
    Pose is (position, xyzw_quaternion), joints are joints!
    """ 


    pose, joints = _get_current_state(robot_name)
    
    try:
        pose_db = rospy.get_param(param_path)
    except Exception as e:
        raise print(e)
       
    _update_pose_db(pose_db, robot_name, entry_name, pose, joints)

    rospy.set_param(param_path, pose_db)

    print("Parameter server successfully updated with the new entry")
    

def _update_pose_db(pose_db:dict, robot_name:str, entry_name:str, pose, joints):
    if entry_name not in pose_db[robot_name]: 
        pose_db[robot_name][entry_name] = {'joints': joints, 'pose': pose}
    else:
        existing_entry = pose_db[robot_name][entry_name]
        if 'joints' in existing_entry or 'pose' in existing_entry:
            try:
                print("New joint entry:", joints)
                print("New pose entry:", pose)
                print("Existing entry:", existing_entry)
                input("Press Enter if you want to update an existing entry or CTRL+D to exit!")
                existing_entry.update({'joints': joints, 'pose': pose})
            except KeyboardInterrupt:
                sys.exit()
                    
def _get_current_state(robot_name):
    franka_state = rospy.wait_for_message('/' + robot_name + '/franka_state_controller/franka_states', FrankaState)
    joints = rospy.wait_for_message('/' + robot_name + '/franka_state_controller/joint_states', JointState)
    pose = np.zeros(7)
    pose[0:3] = franka_state.O_T_EE[12:15]
    rot_matrix = np.zeros((3, 3))
    rot_matrix[0, 0:3] = franka_state.O_T_EE[0:3]
    rot_matrix[1, 0:3] = franka_state.O_T_EE[4:7]
    rot_matrix[2, 0:3] = franka_state.O_T_EE[8:11]
    quat = r2q(rot_matrix)
    pose[3:] = quat
    pose = pose.tolist()

    joints = joints.position
    
    return pose, joints

if __name__ == "__main__":
    rospy.init_node('save_robot_pose_to_json')
    robot_name = sys.argv[1]
    entry_name = sys.argv[2]
    if len(sys.argv) == 4:
        json_path = sys.argv[3]
        save_state_to_json(robot_name, entry_name, json_path)
    else:
        save_state_to_json(robot_name, entry_name)

