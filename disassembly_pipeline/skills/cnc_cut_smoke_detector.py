import numpy as np
import json
import os
import time
import copy
import rospy
from unified_planning.shortcuts import *
from unified_planning.exceptions import UPValueError
from disassembly_pipeline.utils.tf_utils import TFManager
from robotblockset_python.transformations import x2t, r2q, rot_z

from disassembly_pipeline.utils.cnc_utils import circular_cut_gcode_parametric, cut_smokedet_hekatron_tab, cut_hca
# from disassembly_pipeline.utils.pddl_utils import get_all_objects_of_class
from disassembly_pipeline.cnc_manager.src.action_client_cnc import CNCActionClient
from .base_skill import BaseSkill

class CNCCutSmokeDetector(BaseSkill):
    def __init__(self,
                cnc_action_client: CNCActionClient = None,
                wait_for_server = False,
                init_ros_node = False,
                simulate = False,
                description = "Use CNC milling machine to cut it open a smoke detector. Device should be dropped on table_cnc beforehand."):

        """Generic function to cut a smoke detector within the CNC machine."""

        if simulate == False:
            if cnc_action_client is None:
                cnc_action_client =  CNCActionClient(wait_for_server=wait_for_server, init_ros_node=init_ros_node)
            self.cnc_cl = cnc_action_client

            self.tf_manager = TFManager()

    def on_enter_pddl(self, **kwargs):
        out = self.on_enter()
        return out

    def on_enter(self, enable_spindle = False, **kwargs):
        """ Take as input a kwargs dict containing the Detection or disassembly_pipeline.utils.disassembly_object_classes.DisassemblyObject
        Input args example
        {'object': DisassemblyObject}

        Example call:

        """
        result = 'Finished cutting.'
        object_to_be_cut = kwargs['object']
        object_name = object_to_be_cut.label_precise_name

        self.cnc_cl.move_chuck('close')
        if 'fumonic' in object_name:
            self.on_enter_fumonic(enable_spindle = enable_spindle)
        elif 'hekatron' in object_name:
            battery_rotation = object_to_be_cut.battery_rotation if (hasattr(object_to_be_cut, 'battery_rotation') and object_to_be_cut.battery_rotation is not None) else 0
            self.on_enter_hekatron(battery_rectangle_rotation_deg = battery_rotation, enable_spindle = enable_spindle)
        else:
            result = f"Unknown object {object_name}, don't know how to cut"
        self.cnc_cl.move_chuck('open')
        return result

    def on_enter_hekatron(self,
                          device_center_in_cnc_base_frame_mm = [-313, -159, -21],
                          rectangle_dimension_mm = [70, 45, 0],
                          rectangle_center_to_device_center_offset_mm = [11, 0, 0],
                          battery_rectangle_rotation_deg = 0,
                          machine_base_frame = 'cnc_machine_home',
                          perform_actual_cnc_call = True,
                          perform_dry_run_above = False,
                          enable_spindle = False):
        """
        Based on a known rotation of hekatron battery, generate G-code using parametric function.
        Args:
        z_rot_deg: rotation around Z axis, as determined by
        perform actual_cnc_call: If True, call the CNC to execute G-code. If False, just return gcode
        """

        ADDITIONAL_ROT = battery_rectangle_rotation_deg
        RECTANGLE_DIMENSION =np.array(rectangle_dimension_mm) / 1000 # Hekatron rectangle dim
        CENTER_TO_SQUARE_OFFSET = np.array(rectangle_center_to_device_center_offset_mm) / 1000 # Hekatron rectangle translation in regards to center
        #CENTER_TO_SQUARE_OFFSET = [0.05,0,0] # Hekatron rectangle translation in regards to center

        SMOKE_DETECTOR_CENTER_BASE_FRAME = machine_base_frame  # You can specify in relation to another frame (e.g. table_cnc), but then calibration (table_cnc -> cnc_machine_home) has to be perfect
        SMOKE_DETECTOR_CENTER = np.array(device_center_in_cnc_base_frame_mm)/1000 # Center of chuck in cnc_machine_home frame
        cnc_machine_home_FRAME =  machine_base_frame

        RECTANGLE_DIMENSION = RECTANGLE_DIMENSION[0:2]


        gcode_dry_run = []
        if perform_dry_run_above:
            tmp_device_center = copy.deepcopy(SMOKE_DETECTOR_CENTER)
            tmp_device_center[2] = min(0.03, tmp_device_center[2]+0.025) # Ensure limit switch is not hit
            gcode_dry_run = cut_smokedet_hekatron_tab(device_center_position = tmp_device_center,
                                battery_cover_rotation_deg = ADDITIONAL_ROT,
                                rectangle_dimension = RECTANGLE_DIMENSION,
                                center_to_rectangle_offset = CENTER_TO_SQUARE_OFFSET,
                                cnc_table_base_frame = SMOKE_DETECTOR_CENTER_BASE_FRAME,
                                cnc_machine_home_frame = cnc_machine_home_FRAME,
                                cnc_table_base_to_cnc_machine_home = None,
                                tf_manager = self.tf_manager,
                                feed_rate = 700,
                                initial_raise_above_mm = 2,
                                initial_move_feed = 1000,
                                enable_spindle = False)

        gcode_actual_run = cut_smokedet_hekatron_tab(device_center_position = SMOKE_DETECTOR_CENTER,
                                battery_cover_rotation_deg = ADDITIONAL_ROT,
                                rectangle_dimension = RECTANGLE_DIMENSION,
                                center_to_rectangle_offset = CENTER_TO_SQUARE_OFFSET,
                                cnc_table_base_frame = SMOKE_DETECTOR_CENTER_BASE_FRAME,
                                cnc_machine_home_frame = cnc_machine_home_FRAME,
                                cnc_table_base_to_cnc_machine_home = None,
                                tf_manager = self.tf_manager,
                                feed_rate = 700,
                                initial_raise_above_mm = 20,
                                initial_move_feed = 1000,
                                enable_spindle = enable_spindle)

        gcode = gcode_dry_run + gcode_actual_run

        if perform_actual_cnc_call:
            self.cnc_cl.call_server(gcode=gcode)
        return gcode

    def on_enter_fumonic(self, enable_spindle = False):
        """ Cuts out a circular shape. """
        gcode = circular_cut_gcode_parametric(enable_spindle = enable_spindle)

        self.cnc_cl.call_server(gcode=gcode)

        return gcode

    def execute(self):
        0

    def on_exit(self):
        0
    def clamp(self):
        self.cnc_cl.move_chuck('close')
    def unclamp(self):
        self.cnc_cl.move_chuck('open')
