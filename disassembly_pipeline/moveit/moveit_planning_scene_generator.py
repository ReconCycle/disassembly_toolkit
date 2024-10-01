from moveit_commander.planning_scene_interface import PlanningSceneInterface

import rospy
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np

from robotblockset_python.transformations import *

def create_moveit_planning_scene(robot_name = 'panda_1',
                                 ns="/panda_1",
                                 tf2x = None):
    """Create a moveit planning scene of the ReconCycle workcell, 
    with hardcoded positions of box objects.
    TODO improve to be less hardcoded. """

    DZ_TABLE_CENTER_TO_TABLE = 0
    
    base_link = 'table_rob1_center'

    # END Args
    
    # Define robot base links, as they must be excluded from collision checking.
    first_link = '%s_link0'%robot_name
    first_link_sc = '%s_link0_sc'%ns
    second_link = '%s_link1'%ns # Second 
    second_link_sc = '%s_link1_sc'%ns # Second 

    pl_scn_interface = PlanningSceneInterface(ns=ns)

    # Add ground BOX. adding ground plane would be perhaps preferable however some tools are mounted below the robot base frame.
    #add_ground_box(base_link = base_link, pl_scn_interface = pl_scn_interface, first_link = first_link, second_link = second_link)

    dz_base_to_table = 0.02 # height difference between robot base and top of table

    # Dynamically adding tables specified in tf_to_table_generation_fn_dict
    for key in TABLE_NAME_TO_TABLE_CENTER_TRANSFORM.keys(): 
        table_name = key
        generate_table_fn = TABLE_NAME_TO_TABLE_GENERATION_FUNCTION_NAME[key]
        table_center_tf = TABLE_NAME_TO_TABLE_CENTER_TRANSFORM[table_name]
        base_link_to_table_tf = tf2x(base_link, table_center_tf)
        rospy.loginfo("Base to {}: {}, {}, {}".format(table_name, base_link_to_table_tf[0], base_link_to_table_tf[1], base_link_to_table_tf[2]))
        generate_table_fn(base_link = base_link, table_name = table_name, pl_scn_interface = pl_scn_interface, base_to_table_tf = base_link_to_table_tf, \
                          dz_base_to_table = DZ_TABLE_CENTER_TO_TABLE, link = first_link, touch_links = [first_link, first_link_sc, second_link, second_link_sc])

    tool_name = ROBOT_NAME_TO_TOOL_NAME[robot_name]
    print("adding tool ", tool_name)

    add_tool_to_robot(ns, robot_name, tool_name, base_link, pl_scn_interface)
        
    # add_ground_plane(base_link, pl_scn_interface)
    
    #add_vise_model(base_link, pl_scn_interface)
    
    #add_camera_pillar_model(base_link = base_link, pl_scn_interface = pl_scn_interface)
    
    #add_cutter_model(base_link = base_link, pl_scn_interface = pl_scn_interface)
    ############################# ATTACHING OBJECTS TO IGNORE COLLISIONS !
    ## # Attach the box to link0 so as to ignore collisions
    #pl_scn_interface.attach_box(link = "", name = "worksurface_box", pose=None, size=None, touch_links = [base_link, first_link])
    #pl_scn_interface.attach_box(link = base_link, name = "worksurface_box", pose=None, size=None, touch_links = [])
    #pl_scn_interface.attach_box(link = first_link, name = "worksurface_box", pose=None, size=None, touch_links = [])
    
    return pl_scn_interface

def add_tool_to_robot(robot_ns, robot_name, tool_name, base_link, pl_scn_interface):
    EE_box = PoseStamped()
    EE_box.header.frame_id = base_link
    EE_box.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.1
    pose.position.y=0.1
    pose.position.z=0.1
    EE_box.pose = pose
    
    #robot_name = robot_ns[1:] # Ignore the initial slash / (ie /panda_1 -> panda_1)
    
    softhand_mesh_location = "/ros_ws/src/qb_hand_description/meshes/qbsofthand_fully_open_fixed_config.stl"
    vsg_mesh_location = "/ros_ws/src/qb_vsa_gripper/qb_move_description/meshes/qbmove_frame_low_definition.stl"
    
    first_link = '%s_link0'%robot_name
    second_link = "%s_link1"%robot_name
    EE_link = '%s_link7'%robot_name
    
    touch_links = [EE_link, '%s_link7'%robot_name, '%s_link7_sc'%robot_name, '%s_link8'%robot_name, '%s_tc_robot'%robot_name, '%s_flange'%robot_name]

    for i in range(0,6):
        touch_links.append("%s_link%d_sc"%(robot_name, i))
    
    pose= Pose()
    
    if tool_name == 'softhand_box':
            softhand_shape = (0.33,0.2,0.1)
            x_dist_offset = 0.12
            rad_offset = -45 * np.pi / 180
            softhand_offset = (-x_dist_offset*np.sin(rad_offset),-x_dist_offset*np.cos(rad_offset),0.1)
            
            EE_box.header.frame_id = EE_link
            EE_box.pose.orientation.w, EE_box.pose.orientation.x, EE_box.pose.orientation.y, EE_box.pose.orientation.z = 0.92388 , -0.      , -0.      ,  -0.382683
            
            EE_box.pose.position.x, EE_box.pose.position.y, EE_box.pose.position.z = softhand_offset
            
            pl_scn_interface.attach_box(link = EE_link, name = tool_name, pose=EE_box, size=softhand_shape, touch_links = touch_links)

    elif tool_name == 'softhand_mesh':
        EE_box = PoseStamped()
        EE_box.header.frame_id = 'panda_1_flange'
        EE_box.header.seq = 1
        
        q_0 = [1,0,0,1]
        rot = q2r(q_0)
        rot = rot@rot_z(-60+15, unit='deg')@rot_x(90, unit='deg')
        q_0 = r2q(rot)
        pose.orientation.w=q_0[0]
        pose.orientation.x=q_0[1]
        pose.orientation.y=q_0[2]
        pose.orientation.z=q_0[3]
        pose.position.x=0.05
        pose.position.y=-0.05
        pose.position.z=0.1
        EE_box.pose = pose
        
        softhand_shape = (0.001,0.001,0.001)
        #pl_scn_interface.attach_mesh(link = EE_link, name = "gripper_softhand", pose=EE_box, filename= "/ros_ws/src/qb_hand_description/qbsofthand_fully_open_fixed_config.stl", size=softhand_shape, touch_links = [EE_link, 'panda_1_link7', #'panda_1_link8', 'panda_1_tc_robot','panda_1_flange'])
        pl_scn_interface.attach_mesh(link = EE_link, name = tool_name, pose=EE_box, filename=softhand_mesh_location, size=softhand_shape, touch_links = touch_links)
    
    elif tool_name == 'vsg_box':
        vsg_shape = (0.15,0.15,0.17)
        x_dist_offset = 0.02
        rad_offset = -45 * np.pi / 180
        softclaw_offset = (-x_dist_offset*np.sin(rad_offset),-x_dist_offset*np.cos(rad_offset),0.23)
        EE_box.header.frame_id = EE_link
        EE_box.pose.orientation.w, EE_box.pose.orientation.x, EE_box.pose.orientation.y, EE_box.pose.orientation.z = 0.92388 , -0.      , -0.      ,  -0.382683
        EE_box.pose.position.x, EE_box.pose.position.y, EE_box.pose.position.z = softclaw_offset
        try:
            pl_scn_interface.attach_box(link = EE_link, name = tool_name, pose=EE_box, size=vsg_shape, touch_links = touch_links)
            print(EE_link, tool_name, EE_box, vsg_shape, touch_links)
        except Exception as e:
            print("exception when adding tool: ", e)
        
    elif tool_name == 'vsg_mesh':
        EE_box.header.frame_id = 'panda_2_link8'
        vsg_shape = (0.001, 0.001, 0.001)
        q_0 = [1,0,0,1]
        rot = q2r(q_0)
        rot = rot@rot_z(-15-30, unit='deg')@rot_x(0, unit='deg')
        q_0 = r2q(rot)
        pose.orientation.w=q_0[0]
        pose.orientation.x=q_0[1]
        pose.orientation.y=q_0[2]
        pose.orientation.z=q_0[3]
        pose.position.x=0
        pose.position.y=0
        pose.position.z=0.05
        EE_box.pose = pose
        pl_scn_interface.attach_mesh(link = EE_link, name = tool_name, pose=EE_box, filename= vsg_mesh_location, size=vsg_shape, touch_links = touch_links)

    else:
        raise Exception("Unhandled tool (name) in moveit_planning_scene_generator")
    
    return 0

def add_table(base_link, table_name, pl_scn_interface, base_to_table_tf, dz_base_to_table, link, touch_links):
    
    table_lwh = [0.6, 0.6, 0.9]
    table_collision_size = [0.595, 0.595, 0.9] # Make model a bit smaller so tables dont overlap

    # Add table itself
    table_pose = PoseStamped()
    table_pose.header.frame_id = base_link
    table_pose.header.seq = 1
    pose= Pose()
    pose.orientation.w=base_to_table_tf[3]
    pose.orientation.x=base_to_table_tf[4]
    pose.orientation.y=base_to_table_tf[5]
    pose.orientation.z=base_to_table_tf[6]
    pose.position.x=base_to_table_tf[0]
    pose.position.y=base_to_table_tf[1]

    z_position = base_to_table_tf[2] - table_lwh[2]/2 + dz_base_to_table

    # If i dont do this, the table is half a meter too low. Idk why, since all table tfs are at the same height. Fix if you know how.
    #if ('table_rob_2' in table_name) and ('panda_1_link0' in touch_links[0]): z_position = 0 
    #elif ('table_rob_1' in table_name) and ('panda_2_link0' in touch_links[0]): z_position = -table_lwh[2]/2 + dz_base_to_table
    #elif ('table_rob_2' in table_name) and ('panda_2_link0' in touch_links[0]): z_position = 0

    pose.position.z= z_position
    table_pose.pose = pose

    print("Moveit scn: Creating table {} at {}, {}, {}".format(table_name, pose.position.x, pose.position.y, pose.position.z))
    print("Moveit scn: table_size {}, {}, {}".format(table_lwh[0],table_lwh[1],table_lwh[2]))
    print("Moveit scn: base_frame {}".format(link))

    #pl_scn_interface.add_box(name=table_name, pose = table_pose, size = (table_lwh[0],table_lwh[1],table_lwh[2]))
    pl_scn_interface.attach_box(name=table_name, pose = table_pose, size = (table_collision_size[0],table_collision_size[1],table_collision_size[2]), link=link, touch_links = touch_links)

def x_to_pose_object(x):
    assert len(x) == 7
    pose= Pose()
    pose.position.x= x[0]
    pose.position.y= x[1]
    pose.position.z= x[2]
    pose.orientation.w=x[3]
    pose.orientation.x=x[4]
    pose.orientation.y=x[5]
    pose.orientation.z=x[6]
    return pose

def add_vision_table(base_link, table_name, pl_scn_interface, base_to_table_tf, dz_base_to_table, link, touch_links):
    """ Calls the add_table function to add the table. This fn then adds the vision table camera pillars."""

    add_table(base_link, table_name, pl_scn_interface, base_to_table_tf, dz_base_to_table, link = link, touch_links = touch_links)

    PILLAR_ROTATION = 90
    
    table_lwh = [0.6, 0.6, 0.9]
    DX = 0
    
    # Add Camera pillar - upper part
    camera_pillar_lwh = (0.05, 0.05, 1.34)
    camera_pillar_x = [base_to_table_tf[0] - table_lwh[0]/2 + camera_pillar_lwh[0] /2,
                       base_to_table_tf[1],
                       base_to_table_tf[2] + dz_base_to_table + camera_pillar_lwh[2]/2,
                       base_to_table_tf[3],
                       base_to_table_tf[4],
                       base_to_table_tf[5],
                       base_to_table_tf[6]]
    camera_pillar_T = x2t(camera_pillar_x)
    camera_pillar_T[0:3, 0:3] = camera_pillar_T[0:3, 0:3]@rot_z(PILLAR_ROTATION, unit='deg')
    camera_pillar_x = t2x(camera_pillar_T)

    camera_pillar_pose = PoseStamped()
    camera_pillar_pose.header.frame_id = base_link
    camera_pillar_pose.header.seq = 1    
    camera_pillar_pose.pose = x_to_pose_object(camera_pillar_x)
    pl_scn_interface.add_box(name=table_name + 'camera_pillar', pose = camera_pillar_pose, size = camera_pillar_lwh)

    # Add Camera pillar - lower part
    camera_pillar_bottom_lwh = (0.05,0.5,0.045)
    camera_pillar_bottom_x = [base_to_table_tf[0] - table_lwh[0]/2 + camera_pillar_bottom_lwh[0]/2,
                              base_to_table_tf[1],
                              base_to_table_tf[2] + dz_base_to_table + camera_pillar_bottom_lwh[2]/2 ,
                              base_to_table_tf[3],
                              base_to_table_tf[4],
                              base_to_table_tf[5],
                              base_to_table_tf[6]]
    camera_pillar_bottom_T = x2t(camera_pillar_bottom_x)
    camera_pillar_bottom_T[0:3, 0:3] = camera_pillar_bottom_T[0:3, 0:3]@rot_z(PILLAR_ROTATION, unit='deg')
    camera_pillar_bottom_x = t2x(camera_pillar_bottom_T)

    bottom_camera_pillar_pose = PoseStamped()
    bottom_camera_pillar_pose.header.frame_id = base_link
    bottom_camera_pillar_pose.header.seq = 1
    bottom_camera_pillar_pose.pose = x_to_pose_object(camera_pillar_bottom_x)
    pl_scn_interface.add_box(name=table_name + 'cam_pillar_bottom', pose = bottom_camera_pillar_pose, size = camera_pillar_bottom_lwh)

    return 0
    
def add_ground_box(base_link, pl_scn_interface, first_link, second_link):
    # Add box to represent worksurface

    # How much higher is the robot base frame than the table surface/top part
    # This makes sure the ground box is level with all the tables in the worksurface
    robot_base_to_table_top_distance = 0.02 
    table_width = 0.6

    l=2*table_width # Box length
    w=2*table_width # Box width
    h=0.9 # Box height
    
    # Add worksurface box
    worksurface_box = PoseStamped()
    worksurface_box.header.frame_id = base_link
    worksurface_box.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1

    pose.position.x=l/2 - table_width/2 - table_width
    pose.position.y=table_width/2
    pose.position.z=-h/2 + robot_base_to_table_top_distance
    worksurface_box.pose = pose
    pl_scn_interface.attach_box(link = first_link, name = "worksurface_box", pose=worksurface_box, size=(l,w,h), touch_links = [first_link, second_link])
    #pl_scn_interface.add_box(name = "worksurface_box", pose=worksurface_box, size=(l,w,h)) # This allows for transparency but touch_links can't be set...
    
    return 0

def add_ground_plane(base_link, pl_scn_interface):
    raise Exception("Big fail")
    """
    This does add a plane, but how to ignore collisions between link0 of robot and this plane ? Probably in URDF. The function add_plane does not allow for specifying 
    # of touch links.
    
    # Add worksurface plane
    ground_plane_pose = PoseStamped()
    ground_plane_pose.header.frame_id = base_link
    ground_plane_pose.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0
    pose.position.y=0
    pose.position.z=0

    ground_plane_pose.pose = pose
    pl_scn_interface.add_plane(name='table_plane', pose =ground_plane_pose, normal = (0,0,1), offset=dz)
    """
    return 0

def add_vise_model(base_link, pl_scn_interface):
    DX = -0.24 # Change for using table_rob_1_pnp_connector_2 instead of table_rob_1
    
    dz = 0.065 # Change for using table_rob_1_pnp_connector_2 instead of table_rob_1
    
    # Add vise
    vise_pose = PoseStamped()
    vise_pose.header.frame_id = base_link
    vise_pose.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.53 + DX
    pose.position.y=-0.54
    pose.position.z=0.07 + dz
    vise_pose.pose = pose
    pl_scn_interface.add_box(name='vise', pose =vise_pose, size = (0.22,0.22,0.14))
    
    return 0


def add_cutter_model(base_link, pl_scn_interface):
    
    DX = -0.24 # Change for using table_rob_1_pnp_connector_2 instead of table_rob_1
    
    dz = 0.065 # Change for using table_rob_1_pnp_connector_2 instead of table_rob_1
    
    # Add cutter - left part BOXY
    cutter_upper_box = PoseStamped()
    cutter_upper_box.header.frame_id = base_link
    cutter_upper_box.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.6+0.6+0.12 + DX
    pose.position.y=0
    pose.position.z=0.57 + dz
    cutter_upper_box.pose = pose
    pl_scn_interface.add_box(name='cutter_upper_box', pose =cutter_upper_box, size = (0.4,0.21,0.3))

    # Add cutter - Lower part BOXY
    cutter_lower_box = PoseStamped()
    cutter_lower_box.header.frame_id = base_link
    cutter_lower_box.header.seq = 1
    h = 0.23
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.6+0.6+0.12 + DX
    pose.position.y=0
    pose.position.z=h/2 + 0.01 + dz
    cutter_lower_box.pose = pose
    pl_scn_interface.add_box(name='cutter_lower_box', pose =cutter_lower_box, size = (0.4,0.14,h))
    
    # Add cutter left and right boxy parts (vodila)
    l=0.08
    w=0.2
    h=0.42

    dy = 0.06

    # Add cutter - right part BOXY
    cutter_rightside_box = PoseStamped()
    cutter_rightside_box.header.frame_id = base_link
    cutter_rightside_box.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.6+0.6+0.12 + DX
    pose.position.y=+w/2 +dy
    pose.position.z=h/2+0.01 + dz
    cutter_rightside_box.pose = pose
    pl_scn_interface.add_box(name='cutter_rightside_box', pose =cutter_rightside_box, size = (l,w,h))

    # Add cutter - left part BOXY
    cutter_leftside_box = PoseStamped()
    cutter_leftside_box.header.frame_id = base_link
    cutter_leftside_box.header.seq = 1
    pose= Pose()
    pose.orientation.w=1
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=1
    pose.position.x=0.6+0.6+0.12 + DX
    pose.position.y=-w/2 -dy
    pose.position.z=h/2 + 0.01 + dz
    cutter_leftside_box.pose = pose
    pl_scn_interface.add_box(name='cutter_leftside_box', pose =cutter_leftside_box, size = (l,w,h))
    
    return 0


def add_cnc_table_model(base_link, pl_scn_interface, table_tf_name):
    
    raise Exception("Unfinished")

TABLE_NAME_TO_TABLE_CENTER_TRANSFORM = {'table_vision': 'table_vision_center',
                                        'table_cnc': 'table_cnc_center',
                                        'table_cutter': 'table_cutter_center',
                                        'table_vise': 'table_vise_center',
                                        'table_rob_1': 'table_rob1_center',
                                        'table_rob_2': 'table_rob2_center'}

TABLE_NAME_TO_TABLE_GENERATION_FUNCTION_NAME = {'table_vision':add_vision_table,
                                      'table_cnc': add_table, 
                                      'table_cutter': add_table,
                                      'table_vise': add_table,
                                      'table_rob_1': add_table, 
                                      'table_rob_2': add_table}
    
ROBOT_NAME_TO_TOOL_NAME = {'panda_1': 'softhand_box',
                           'panda_2': 'vsg_box'}

TOOL_NAME_TO_TOOL_MESH = {'vsgripper': '/ros_ws/src/qb_move_description/meshes/qbmove_frame_low_definition.stl'}