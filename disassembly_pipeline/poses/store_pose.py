#!/usr/bin/env python
import rospy
import rospkg
r = rospkg.RosPack()
path = r.get_path('poses')
import numpy as np
import sys
import shutil
import json
print(f"Path: {sys.path}")
sys.path.append(path+"/depends/save_toolpost_to_json.py")
sys.path.append(path+"/depends/save_robot_pose_to_json.py")
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
from colorama import Fore, Back
from robotblockset_python.transformations import r2q




class PoseUpdater():
    def __init__(self) -> None:
        self.robot = None
        self.x = None
        self.q = None
        self.name = None
        self.type = None
        rospy.init_node("pose_updater")
        self._ROBOTS = {0: 'panda_1', 1: 'panda_2'}
        self._POSE_TYPES = {0:'endpoint', 1:'waypoint', 2:'toolpost'}
    
    def handle_inputs(self):
        print(f"{Fore.GREEN}Enter robot name from the listed choices:")
        for key, robot in self._ROBOTS.items():
            print(f"{Fore.LIGHTGREEN_EX}    -{Fore.LIGHTBLUE_EX}[{key}] {Fore.RED}-> {Fore.LIGHTGREEN_EX}{robot}{Fore.RESET}")
        key = int(input(f"{Fore.GREEN}Input robot name index:{Fore.RESET} "))
        if key not in self._ROBOTS.keys():
            raise KeyError(f"Invalid robot [{key}], should be one of: {self._ROBOTS}")
        else:
            self.robot = self._ROBOTS[key]
        print(f"{Fore.GREEN}Enter pose type from the listed choices:{Fore.RESET} ")
        for key, ptype in self._POSE_TYPES.items():
            print(f"{Fore.LIGHTGREEN_EX}    -{Fore.LIGHTBLUE_EX}[{key}] {Fore.RED}-> {Fore.LIGHTGREEN_EX}{ptype}{Fore.RESET}")
        self.type = int(input(f"{Fore.GREEN}Pose type:{Fore.RESET}"))
        if self.type not in self._POSE_TYPES.keys():
            raise KeyError(f"Invalid type [{self.type}], should be one of: {self._POSE_TYPES}")
        elif self.type==0:
            self.handle_endpoints()
        elif self.type==1:
            self.handle_waypoints()
        elif self.type==2:
            self.handle_toolposts()
        
    def handle_endpoints(self):
        self.name = input(f"{Fore.GREEN}Enter endpoint name:{Fore.RESET} ")
        self.save_state_to_json(robot_name=self.robot, entry_type='endpoints', entry_name=self.name)
        
    def handle_waypoints(self):
        self.name = input(f"{Fore.GREEN}Enter waypoint name:{Fore.RESET} ")
        self.save_state_to_json(robot_name=self.robot, entry_type='waypoints', entry_name=self.name)

    def handle_toolposts(self):
        self.name = input(f"{Fore.GREEN}Enter tool name:{Fore.RESET} ")
        self.name = "tp_" + self.name
        ALOWED_TYPES = {0: "mount", 1: "unmount"}
        print(f"{Fore.GREEN}Enter subpose type from the listed choices: ")
        for key, stype in ALOWED_TYPES.items():
            print(f"{Fore.LIGHTGREEN_EX}    -{Fore.LIGHTBLUE_EX}[{key}] {Fore.RED}-> {Fore.LIGHTGREEN_EX}{stype}{Fore.RESET}")
        key = int(input(f"{Fore.LIGHTGREEN_EX}Enter subtype: {Fore.RESET}"))
        if key not in ALOWED_TYPES.keys():
            raise KeyError(f"Invalid pose subtype [{key}], should be one of: {ALOWED_TYPES}")
        else:
            self.save_post_to_json(self.name, key)
    
    def save_post_to_json(self,entry_name:str, subtype:int,
                        json_path:str='/devel_ws/src/disassembly_pipeline/disassembly_pipeline/poses/test_db.json'):
        """
        This saves to JSON file

        State will be saved as:
            pose_db[robot_name][entry_name]["pose"]
            pose_db[robot_name][entry_name]["joints"]

        Returns (pose, joints)
        Pose is (position, xyzw_quaternion), joints are joints!
        """ 
        pose, joints = self._get_current_state('panda_1')
        
        shutil.copyfile(json_path, json_path + ".bak")
            
        with open(json_path) as fp:
            try:
                pose_db = json.load(fp)
            except Exception:
                raise ValueError("Could not open JSON file! Does it exist?")

        if subtype == 0:
            print(f"{Fore.MAGENTA}Make sure the toolchanger is empty and the EE config is correct!\n----------------------------------------------------------------\n")
            print(f"{Fore.MAGENTA}Place the robot's toolchanger into it's corresponding tool, then press ENTER to continue")
            input('Press enter to continue: ')
            mount_pose, __ = self._get_current_state('panda_1')
            
            print(f"{Fore.MAGENTA}Move robot up so the toolchanger pins are just barely above their holes, then press ENTER to continue")
            input('Press enter to continue: ')
            __, mount_joints = self._get_current_state('panda_1')
            self._update_mount(pose_db, entry_name, mount_pose, mount_joints)
        
        elif subtype==1:
            print(f"{Fore.MAGENTA}Place the robot's tool into it's corresponding toolpost, then press ENTER to continue")
            input('Press enter to continue: ')
            original_pose, original_q = self._get_current_state('panda_1')
            
            print(f"{Fore.MAGENTA}Raise the robot, so that the tool's mount clears the toolpost, then press ENTER to continue")
            input('Press enter to continue: ')
            raised_pose, raised_q = self._get_current_state('panda_1')
            dz = raised_pose[2]-original_pose[2]

            print(f"{Fore.MAGENTA}Move the robot out of the toolpost so that the tool's footprint clears the toolpost, then press ENTER to continue")
            input('Press enter to continue: ')
            out_pose, out_q = self._get_current_state('panda_1')
            dy = out_pose[1]-original_pose[1]
            
            print(f"{Fore.MAGENTA}Move the robot up so that it's able to continue working without hitting the toolposts, then press ENTER to continue")
            input('Press enter to continue: ')
            clear_pose, clear_q = self._get_current_state('panda_1')
            dz2 = clear_pose[2]-original_pose[2]
            
            relative_motions = [[0, 0, dz],
                                [0, dy, 0],
                                [0, 0, dz2]]
            
            ee_config = input(f"{Fore.LIGHTBLUE_EX}Enter the ee_config name: ")
            self._update_unmount(pose_db, entry_name, relative_motions, ee_config)

        with open(json_path, 'w') as fp:
            json.dump(pose_db, fp, indent=4)

        print("JSON file successfully updated with the new entry")
    
    def save_state_to_json(self, robot_name:str, entry_type:str, entry_name:str, 
                        json_path:str='/devel_ws/src/disassembly_pipeline/disassembly_pipeline/poses/test_db.json'):
        """
        This saves to JSON file

        State will be saved as:
            pose_db[robot_name][entry_name]["pose"]
            pose_db[robot_name][entry_name]["joints"]

        Returns (pose, joints)
        Pose is (position, xyzw_quaternion), joints are joints!
        """ 


        pose, joints = self._get_current_state(robot_name)
        
        print("Creating backup of the pose database file...")
        shutil.copyfile(json_path, json_path + ".bak")
        print("Saved backup")
        
        with open(json_path) as fp:
            try:
                pose_db = json.load(fp)
            except Exception:
                raise ValueError("Could not open JSON file! Does it exist?")
        
            self._update_pose_db(pose_db, robot_name, entry_type, entry_name, pose, joints)

        with open(json_path, 'w') as fp:
            json.dump(pose_db, fp, indent=4)

        print("JSON file successfully updated with the new entry")

    def _update_mount(self, pose_db:dict, entry_name:str, pose, joints):
        if entry_name not in pose_db['toolposts']: 
            pose_db['toolposts'][entry_name] = {
                "mount" : {},
                "unmount" : {}
            }
            pose_db['toolposts'][entry_name]["mount"] = {'joints': joints, 'pose': pose}
        else:
            existing_entry = pose_db['toolposts'][entry_name]['mount']
            if 'joints' in existing_entry or 'pose' in existing_entry:
                try:
                    print("New joint entry:", joints)
                    print("New pose entry:", pose)
                    print("Existing entry:", existing_entry)
                    input("Press Enter if you want to update an existing entry or CTRL+D to exit!")
                    existing_entry.update({'joints': joints, 'pose': pose})
                except KeyboardInterrupt:
                    sys.exit()
            else:
                existing_entry.update({'joints': joints, 'pose': pose})
                        
    def _update_unmount(self, pose_db:dict, entry_name:str, relative_motions:list, ee_config:str):
        if entry_name not in pose_db['toolposts']: 
            pose_db['toolposts'][entry_name]['unmount'] = {'relative_motions': relative_motions}
            pose_db['toolposts'][entry_name] = {'ee_config': ee_config}

        else:
            existing_entry = pose_db['toolposts'][entry_name]['unmount']
            if 'relative_motions' in existing_entry:
                try:
                    print("New relative motions:", relative_motions)
                    print("Existing entry:", existing_entry)
                    input("Press Enter if you want to update an existing entry or CTRL+D to exit!")
                    existing_entry.update({'relative_motions': relative_motions})
                except KeyboardInterrupt:
                    sys.exit()
            else: 
                existing_entry.update({'relative_motions': relative_motions})
    

            existing_entry = pose_db['toolposts'][entry_name]
            if 'ee_config' in existing_entry:
                try:
                    print("New ee config:", ee_config)
                    print("Existing ee config:", existing_entry['ee_config'])
                    input("Press Enter if you want to update an existing entry or CTRL+D to exit!")
                    existing_entry.update({'ee_config': ee_config})
                except KeyboardInterrupt:
                    sys.exit()
            else: 
                existing_entry.update({'ee_config': ee_config})

    def _update_pose_db(self, pose_db:dict, robot_name:str, entry_type:str, entry_name:str, pose, joints):
        available_types = ['endpoints', 'waypoints']
        assert entry_type in available_types, f"{Fore.RED}Invalid entry type: {entry_type}, must be one of: [{available_types}]!"
        
        if entry_name not in pose_db[entry_type][robot_name]: 
            pose_db[entry_type][robot_name][entry_name] = {'joints': joints, 'pose': pose}
        else:
            existing_entry = pose_db[entry_type][robot_name][entry_name]
            if 'joints' in existing_entry or 'pose' in existing_entry:
                try:
                    print("New joint entry:", joints)
                    print("New pose entry:", pose)
                    print("Existing entry:", existing_entry)
                    input("Press Enter if you want to update an existing entry or CTRL+D to exit!")
                    existing_entry.update({'joints': joints, 'pose': pose})
                except KeyboardInterrupt:
                    sys.exit()

    def _get_current_state(self, robot_name):
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
    pup = PoseUpdater()
    pup.handle_inputs()