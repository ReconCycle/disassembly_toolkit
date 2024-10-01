## EE configs

Please note that all tool weights specified in the .json files **except for the ones named tc_and_adapter** pertain exclusively to the tool-side of the toolchanger and tool, they do not include the robot-side tool changer and flange. Therefore, the total tool weight consists of the sum of the mass specified in the ee_config and the mass of the flange and robot-side tool changer.

It's important to mention that Libfranka doesn't permit the modification of m_ee (end effector mass) while the robot is in operation. It only allows for changes to m_load (load mass). Consequently, m_ee solely represents the robot flange and the robot-side toolchanger.

Additionally, the selected TCP (Tool Center Point) configuration is added to the FrankDesk TCP. This implies that when you apply any of these JSON configurations in FrankaDesk to set the TCP, you must take into account the length of the tool-changer assembly. 

So, the values provided here are from the nominal end effector (NE_T_EE), referring to the robot-side toolchanger, to the end effector. Please be mindful that what you establish in FrankaDesk is denoted as F_T_NEE (Franka Flange to Nominal End Effector).


Tool descriptions:

- **tc_and_adapter_IN_FRANKADESK.json:** Panda-1 toolchanger and adapter this is SET IN FRANKADESK, since it is constant and always mounted to robot. Sets flange adapter and robot-side toolchanger values (mass, CoG, tf). ONLY AS REFERENCE, DO NOT apply this using robotblockset! (since then effectively there would be 2 robot-side toolchangers attached to robot flange)
tc_and_realsense_and_adapter_IN_FRANKADESK.json: panda-2 alu flange adapter with realsense mount, and robot-side toolchanger.

- **p2_tc_and_adapter.json:** Panda-2 toolchanger and adapter. This is SET IN FRANKADESK, since it is constant and always mounted to the robot. It sets the flange, adapter and robot-side toolchanger values (mass, CoG, tf). ONLY AS REFERENCE, DO NOT apply this using robotblockset! (since then effectively there would be 2 robot-side toolchangers attached to robot flange)

- **no_tool.json:** Used when the Robot-side Toolchanger is mounted, but there is NO tool actually attached to the robot
tc_realsense_and_vsg.json: On Panda-2. The Variable Stiffness Gripper mounted to an adapter and tool-side toolchanger.

- **tc_and_long_vacuum_gripper.json** The long 45deg vacuum gripper for pickup up batteries from within CNC
