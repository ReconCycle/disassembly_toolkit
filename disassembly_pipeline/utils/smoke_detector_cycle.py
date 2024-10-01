

def find_battery_covered_rotation():
    0

def working_first_half_of_cycle():
    """ The tested first half of cycle, with placing smoke detector into CNC; looking at it, generating G-code, and performing initial cut."""
    vi_basler.enable_camera()
    home_all_robots()
    cnc_cut_skill.cnc_cl.move_chuck('open')
    smoke_detector = panda_1_pick_up_smoke_detector_from_table_vision_and_place_in_cnc()
    vi_basler.enable_camera(False)
    ### Look at smoke detector with second robot, determine gcode
    cnc_cut_skill.clamp_skill.on_enter()
    result = check.on_enter(robot = p2, location = None, object_to_check = smoke_detector)
    ### Cut open with CNC 
    cnc_cut_skill.on_enter(object = smoke_detector, enable_spindle = True)
    return smoke_detector

def in_progress_second_half_of_cycle():
    """ The in-progress second half of cycle.
    STARTS AFTER INITIAL CNC CUTTING, after the function 'working_first_half_of_cycle()'
    """
    0

# "improved" cycle
PREVIOUS_ACTION = 'None'

vi_realsense.enable_camera()
### Look at smoke detector: A) Fumonic: Cut battery contacts if fumonic. B) Hekatron: Remove cover

def step_1():
    vi_realsense.enable_camera(True)
    result = check.on_enter(robot = p2, location = None, object_to_check = None, return_to_home = False, only_move_to_position_and_ignore_detecting_objects = True)
    time.sleep(1)
    all_detections = vi_realsense.get_detections() # Detect either battery_covered or 
    # Pick up battery_covered from hekatron
    if check_if_detections_contain_label(all_detections, Label.battery_covered):
        battery_covered = get_most_centered_detection_of_particular_class_from_detections(detections = all_detections,
                                                                                          desired_detection_class = Label.battery_covered)
        print("DEBUG: Removing battery cover")
        #cnc_cut_skill.clamp_skill.on_enter()
        cnc_cut_skill.unclamp_skill.on_enter()
        #pickup_top.on_enter(robot = p2,
        #            object_class = battery_covered.label.name,
        #            object_tf_name= battery_covered.tf_name,
        #            gripper_close_command = 1.5,
        #            move_time_to_pickup = 2)
        skill_exception_handling_wrapper.on_enter(robot = p2,
                                          look_at_skill = skill_look_at_table,
                                          look_at_skill_kwargs = {'robot':p2, 'location': 'table_cnc', 'return_to_home':False},
                                          vision_interface = p2.camera_vision_interface,
                                          base_skill = pickup_top,
                                          base_skill_kwargs = {'robot': p2, 'gripper_close_command': 1.2},
                                          object_to_handle = Label.battery_covered,
                                          return_to_home = False)
        
        panda_2_drop_battery_cover()
        PREVIOUS_ACTION = 'remove_battery_cover'
    # CNC cut battery contacts from Fumonic
    elif check_if_detections_contain_label(all_detections, Label.battery):
        print("DEBUG: Cutting off battery solder contacts")
        print("DEBUG: Generating drill battery contacts G-code")
        img_raw = realsense_img_listener_raw.get_pil_image()
        out_dict = estimate_battery_pose_skill.on_enter(detections = all_detections,
                                                   image = img_raw,
                                                   machine_base_frame = 'cnc_machine_home',
                                                   safe_height_above_drill_point_in_meters = 0.01,
                                                   drill_depth_in_meters = 0.0001,
                                                   hardcoded_drill_point_raise_z_in_meters = 0.03)
        drilling_gcode = out_dict['drilling_gcode']
        
        # Move robot back
        skill_homing.on_enter(p2)
        #cnc_cut_skill.cnc_cl.call_server(gcode = drill_gcode)
        PREVIOUS_ACTION = "cut_off_soldered_battery_contacts"
    # Neither battery nor battery_covered is detected. This means these components are removed. Place smoke detector in trash bin
    elif check_if_detections_contain_label(all_detections, Label.pcb):
        print("DEBUG: only PCB seen, placing smoke detector in trash")
        panda_1_pick_up_smoke_detector_from_cnc_and_place_on_table()
        PREVIOUS_ACTION = "drop_smoke_detector_in_trash"
    else:
        raise ValueError("Did not find either battery, battery_covered or PCB. Handle this case.")
    return PREVIOUS_ACTION

def step_2(previous_action = 'None'):
    print("DEBUG: previous_action: ", PREVIOUS_ACTION)
    if PREVIOUS_ACTION == "drop_smoke_detector_in_trash":
        return 0 # Lol
    
    result = check.on_enter(robot = p2, location = None, object_to_check = None, return_to_home = False, only_move_to_position_and_ignore_detecting_objects = True)
    time.sleep(1)
    all_detections = vi_realsense.get_detections() # Detect either battery_covered or 

    if check_if_detections_contain_label(all_detections, Label.battery) and (PREVIOUS_ACTION == 'remove_battery_cover'):
        print("DEBUG: Removing battery by wiggling")
        # Hekatron remove battery after removing battery cover
        # Remove battery by wiggling
        panda_2_wiggling_battery_removal()
        # Drop battery
        # Hardcoded JMove
        panda_2_drop_battery()
    elif check_if_detections_contain_label(all_detections, Label.battery) and (PREVIOUS_ACTION == 'cut_off_soldered_battery_contacts'):
        print("DEBUG: Picking up battery with vacuum gripper")
        # Fumonic - pick up battery with vacuum gripper
        0
        #panda_1_drop_battery()
    else:
        print("DEBUG: Unhandled case. Don't see a battery. Dropping smoke detector into trash bin.")
    # Home robot, place smoke det into trash
    skill_homing.on_enter(p2)
    panda_1_pick_up_smoke_detector_from_cnc_and_place_on_table()
    skill_homing.on_enter(p1)


if __name__ == '__main__':
    # Move robot to initial position (be careful)

    skill_homing.on_enter(robot=p1)
    skill_homing.on_enter(robot=p2)
    vi_realsense.enable_camera(True)
    vi_basler.enable_camera()
    smoke_detector = working_first_half_of_cycle()
    PREVIOUS_ACTION = step_1()
    step_2(previous_action = PREVIOUS_ACTION)