#!/usr/bin/env python
# coding: utf-8

# # Run the robot

# ```bash
# source ~/panda_ws/devel/setup.bash
# roslaunch ijs_controllers cartesian_impedance.launch robot_ip:=192.16.0.1 __ns:=panda_1 arm_id:=panda_1 use_old:=true load_gripper:=false legacy_franka_ros:=false
# ```

# In[2]:


import numpy as np
import json
import os
import sys
import matplotlib.pyplot as plt



# Set print options for numpy arrays


from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
#from robotblockset_python.grippers import 

import time
import copy
import rostopic, rosgraph
from tf import TransformBroadcaster
from rospy import Time
import warnings
from colorama import Fore
import copy
from IPython.core.debugger import set_trace 


# In[3]:


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# In[7]:


r = panda_ros('panda_1', init_node= True, init_frankadesk_gripper_TCP = True, start_controller = 'cartesian_impedance_controller')
r._verbose = -1
r.SetNewEEConfig(tool_name='tc_and_3jaw_chuck.json')
r.Stop_controller()
r.SetJointImpedanceFranka(np.array([14000]*7), restart=False)
r.SetCollisionBehavior(F=70, T= 20, tq = 30,restart=False)
r.Start_controller()
r.error_recovery()
r.GetState()


# In[ ]:


#!/usr/bin/env python

import rospy
from robot_module_msgs.srv import SetString, SetStringResponse

def handle_panda_controls(req):
    response = SetStringResponse()  # Create a response object

    # Check the string in the request and perform actions accordingly
    if req.string == 'panda_soft':
        # Implement panda_soft action here
        r.SetJointImpedanceFranka(np.array([0]*7), restart=False)
        response.message = "Panda soft mode activated"
    elif req.string == 'panda_stiff':
        # Implement panda_stiff action here
        r.SetJointImpedanceFranka(np.array([14000]*7), restart=False)
        response.message = "Panda stiff mode activated"
    else:
        response.message = "Unknown command"

    return response

def panda_controls_server():
    #rospy.init_node('panda_controls_server')
    service = rospy.Service('/process_server/panda_controls', SetString, handle_panda_controls)
    rospy.loginfo("Panda controls server is ready.")
    rospy.spin()  # Keep the server running

if __name__ == "__main__":
    panda_controls_server()


# In[ ]:




