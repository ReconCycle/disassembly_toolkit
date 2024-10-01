# Debug PCB touch off and cutting
from disassembly_pipeline.skills.base_skill import BaseSkill
from disassembly_pipeline.workcell_modules.linear_pneumatic_cutter_module import LinearPneumaticCutterModule
from robotblockset_python.robots import robot as Robot
from disassembly_pipeline.utils.Levering_block_v3 import move_until_contact
import copy
import numpy as np
import time


class LinearCutterPCBTouchoffAndCut(BaseSkill):
    def __init__(self):
        0
        #super().__init__(name = name, description = description, action_arg_names = action_arg_names)

    def on_enter_pddl(self, **kwargs):
        print("Hit a")
        robot = kwargs['robot']
        location = kwargs['location']
        self.on_enter(robot = robot, cutter_module = location, perform_cutting = True)
        
    def on_enter(self,
                 robot: Robot,
                 cutter_module: LinearPneumaticCutterModule,
                 cutter_touch_off_height_in_robot_base_frame = 0.21224096,
                 gripper_close_command = 3,
                 perform_cutting = False,
                 **kwargs
                 ):
        """ 
        - Move the PCB to the cutter, 
        - perform a touch off to determine how much it's sticking out,
        - place into cutter,
        - cut,
        - move robot back out

        Expected initial state: Robot holding a PCB within (VariableStiffness) gripper.

        Args:
        
        cutter_touch_off_height_in_robot_base_frame: float: Move robot so gripper touches the cutter at the point where PCB is expected to touch it.
                Record this height in robot base frame (robot.Base_link_name)
        """
        q_above_cutter_touch_off_point = [-1.144043275051187,0.20613962880654238,0.0960601738210308,-1.383473400149429,0.014085290800465158,1.607780242284139,1.3099728782143856]
        q_above_cutter_cutting_point = [-1.0682315560492304, 0.6906324768181071, 0.27944753845419446, -0.7546409624597507, -0.17465348089476435, 1.468029906290833, 1.5014671851619377]
        fixed_z_offset_to_move_into_cutter = 0.145 
        #x_above_cutter_touch_off_point = []
        
        r = robot
        r.error_recovery()
        #if r._control_strategy != 'JointPositionTrajectory':
        r.Switch_controller(start_controller = 'JointPositionTrajectory')

        r.JMove(q = q_above_cutter_touch_off_point, t = 2, qdot_max_factor = 0.5, qddot_max_factor = 0.5)
        r.CMoveFor(dx = [0, 0, -0.03], t =  1.5)

        r.Switch_controller(start_controller = 'CartesianImpedance')
        r.gripper.close(command = gripper_close_command, sleep = False)
        # Move down until PCB touches the cutter
        fs = move_until_contact(robot = r,
            mv_unit_direction = [0, 0, -1],
            mv_velocity = 0.05,
            axes_to_monitor = [0,0,1,0,0,0],
            max_allowed_values = [0,0,10, 0, 0 ,0],max_allowed_t= 10)

        # PCB is not touching cutter. Read off the height of the EE
        r.ResetCurrentTarget()
        r.GetState()
        current_height = copy.deepcopy(r.x[2])

        distance_from_robot_ee_to_pcb = current_height - cutter_touch_off_height_in_robot_base_frame
        print(f"The PCB is sticking out from the robot gripper's z-axis by a value of {distance_from_robot_ee_to_pcb} m.")

        r.Switch_controller(start_controller = 'JointPositionTrajectory')
        r.error_recovery()
        # Move back up 
        r.JMove(q = q_above_cutter_touch_off_point, t = 1.2, qdot_max_factor = 0.5, qddot_max_factor = 0.5)
        r.error_recovery()
        # Move above cutter cutting point
        r.JMove(q = q_above_cutter_cutting_point, t = 1.2, qdot_max_factor = 0.5, qddot_max_factor = 0.5)
        r.error_recovery()

        if perform_cutting:
            #r.Switch_controller(start_controller = 'JointPositionTrajectory')

            cutter_module.close_tray()
            cutter_module.open_cutter()
            time.sleep(2)

        # Move down by detected distance plus some fixed offset
        r.CMoveFor(dx = [0,0, -fixed_z_offset_to_move_into_cutter + distance_from_robot_ee_to_pcb], t = 1.2)

        # Move a bit to the side so that we are next to the cutter wall (less shock)
        r.CMoveFor(dx=[-0.007,0,0], t= 1.2)

        time.sleep(2)

        # Check robot is not in error state and has reached commanded position!
        x_error = r.x_err
        normed_error = np.linalg.norm(x_error)
        if perform_cutting and (normed_error < 1e-3):
            # Perform cutting
            user_feedback = input('press y if you want to cut')
            if user_feedback == 'y':
                cutter_module.close_cutter()
                time.sleep(3)
                cutter_module.open_tray()
        # Move back up
        #r.CMoveFor(dx = [0,0, +fixed_z_offset_to_move_into_cutter - distance_from_robot_ee_to_pcb], t = 1.2)
        r.error_recovery()
        r.JMoveFor([0,-0.1,0,0,0,0,0], t=1.2)

        # Drop PCB into pcb_dropoff_location

        return distance_from_robot_ee_to_pcb
        
    def execute(self):
        0
    def on_exit(self):
        0