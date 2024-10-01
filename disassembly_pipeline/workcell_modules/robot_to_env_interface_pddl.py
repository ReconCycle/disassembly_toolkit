import numpy as np
from abc import ABC, abstractmethod

import unified_planning
from unified_planning.shortcuts import *

class Robot_env_wrapper:
    def __init__(self):
        """ Abstract object for a generic robot in a RL setting. Make subobjects for different robots. """
        self._description = ''
        
    @abstractmethod
    def state(self):
        return 0

    @abstractmethod
    def action(self, dx, dt):
        return 0
    @abstractmethod
    def get_description(self):
        """ Return textual description (that can be passed to language model as state)"""
        return self._description
    @abstractmethod
    def pddl_init(self, problem):
        """ Add objects (robot in this case) to the PDDL problem, which is actually (problem + domain)
        Static - the function runs only when the entire environment/problem is initialized, NOT in a loop. 
        The pddl_loop function, however, runs in a loop as env. states are updated."""
        return 0

    @abstractmethod
    def pddl_loop(self, problem):
        """ Add objects to the PDDL problem, which is actually (problem + domain)
        dynamic - the function runs in a loop, as env. states are updated.
        """
        return 0

class Franka_robot_env_wrapper(Robot_env_wrapper):
    def __init__(self, robot_obj = None,
                  name = 'panda_1',
                  pddl_robot_type_name = 'robot',
                  robot_has_eye_in_hand_camera = False,
                  additional_description = ''):
        """ Wrapper for the Franka Emika Panda robot so that it works in a RL setting.
        Args:
        robot_obj : a robotblockset_python panda_ros instance
        pddl_robot_type_name : name of the robot object type in the PDDL environment. (usually 'robot')
        robot_has_eye_in_hand_camera: bool: Whether the robot has an eye-in-hand camera mounted.
        additional_description: str: Add this string to robot description. (You can textually specify that you prefer this robot be used for ...)
        """
        self._robot_obj = robot_obj
        # For testing (getting back description), robot_obj can be None
        if self._robot_obj is None:
            self._name = name
        else:
            self._name = self._robot_obj.Name

        self._pddl_robot_type_name = pddl_robot_type_name
        self._robot_has_eye_in_hand_camera = robot_has_eye_in_hand_camera

        self._dummy_state = np.zeros((5, 7))
           
        self._description = f"""{self._name}: An industrial robot arm. {additional_description}"""

    def state(self):
        self.robot_obj.GetState() # Update internal .state variable
        st = self.robot_obj.state
        q = st.q
        qdot = st.dq

        x, J = self.robot_obj.kinmodel(q)
        # FrankaState has O_T_EE but as T(4x4) matrix. Better not import Rot to quaternion tools and use this instead.

        xdot = np.dot(J, qdot)

        FT = st.K_F_ext_hat_K

        self.dummy_state[0:3, :] = q, qdot,x
        self.dummy_state[3:, 0:-1] = xdot, FT

        return self.dummy_state

    def action(self, dx, dt, task_space = 'World'):
        assert dx.shape[0] == 7
        assert task_space in ['World', 'Tool']

        return self.robot_obj.CMoveFor(dx, dt)

    def pddl_init(self, problem, pddl_to_world_obj_links):
        """
        Modify initially given PDDL problem by ADDING the robot object, if it's not present.
        This function is meant to run once, right after PDDL problem instantiation.
        If the environment/problem initial state does not contain the object robot_n (as defined in self._name), this object is added.
        Args:
        problem : unified_planning.model.problem.Problem instance
        """

        assert type(problem) == unified_planning.model.problem.Problem

        gripper_empty_name = 'gripper_empty'
        robot_homed_name = 'is_robot_homed'
        robot_has_camera_name = 'has_camera'

        if not problem.has_object(self._name):
            obj_type = UserType(self._pddl_robot_type_name)
            problem.add_object(self._name, obj_type)

        current_robot_object = problem.object(self._name)

        gripper_empty_fluent = problem.fluent(gripper_empty_name)
        problem.set_initial_value(gripper_empty_fluent(current_robot_object), True)

        robot_has_camera_fluent = problem.fluent(robot_has_camera_name)
        problem.set_initial_value(robot_has_camera_fluent(current_robot_object), self._robot_has_eye_in_hand_camera)

        robot_homed_fluent = problem.fluent(robot_homed_name)
        problem.set_initial_value(robot_homed_fluent(current_robot_object), False)

        # Grounding PDDL - add link from PDDL to real robot
        pddl_to_world_obj_links[self._name] = self._robot_obj

        return problem, pddl_to_world_obj_links

    def pddl_loop(self, problem, pddl_to_world_obj_links):
        """ Runs in a loop and modifies the PDDL problem.
        In the case of robot, we (perhaps) only need to modify gripper_empty parameter.
        (if the status changes UNEXPECTEDLY, such as if we drop the held item by mistake.
        """
        0
        return problem, pddl_to_world_obj_links
