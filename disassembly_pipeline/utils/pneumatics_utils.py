import numpy as np
from robotblockset_python.transformations import *
from .multithreading import do_concurrently
import time
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from reconcycle_flexbe_states.activate_raspi_output import ActivateRaspiDigitalOutput

class PneumaticsManager:
    def __init__(self,
                 userdata,
                 cnc_airblock_state,
                 cnc_chuck_state,
                 panda_1_airblock,
                 activate_block2_st,
                 mainjaws_st,
                 sidejaws_st,
                 slider_st,
                 clamptray_st,
                 rotate_holder_st,
                 cutter_st, 
                 cutter_airblower_st):
        
        # Generic FlexBe userdata object
        self.userdata = userdata 

        # CNC states
        self.cnc_airblock_state = cnc_airblock_state
        self.cnc_chuck_state = cnc_chuck_state

        # Activating distribution airblock to other airblocks
        self.panda1_airblock_state = panda_1_airblock
        self.activate_block2_st = activate_block2_st

        # Vise stuff
        self.mainjaws_st = mainjaws_st
        self.sidejaws_st = sidejaws_st
        self.slider_st = slider_st
        self.clamptray_st = clamptray_st
        self.rotate_holder_st = rotate_holder_st

        # Cutter/guilloting stuff
        self.cutter_st = cutter_st
        self.cutter_airblower_st = cutter_airblower_st

        self.cnc_chuck_state = ActivateRaspiDigitalOutput(service_name = '/cnc_chuck_open')

    def handle_pneumatics_jaws(self, command:str = 'close', sidejaws:bool= False):
        """
        Function to open and close the main and side vise jaws

        Args:
        -----
            - command(str) : Either 'open' or 'close', tells the vise what to do
            - sidejaws(bool) : Tells vise whether to open sidejaws too, or just the main jaws

        Examples:
        ---------
            Open main jaws only: 
            >>> handle_pneumatics_jaws(command='open', sidejaws=False)

            Close both jaws:
            >>> handle_pneumatics_jaws(command='close', sidejaws=True)
        """
        userdata = self.userdata
        mainjaws_st = self.mainjaws_st
        sidejaws_st = self.sidejaws_st

        assert command in ['close', 'open'], "Invalid command, should be either 'close' or 'open'"
        
        if command == 'close':
            # No multithreading
            # Operating the sidejaws - close
            userdata.value = True
            #userdata.value = False
            sidejaws_st.on_enter(userdata)
            time.sleep(1.7)
            # Operating the mainjaws - close
            userdata.value = False
            mainjaws_st.on_enter(userdata)
            if sidejaws:
                time.sleep(1)
                userdata.value = False
                sidejaws_st.on_enter(userdata)
        else:
            # Operating the sidejaws - open
            userdata.value = True
            userdata.value = False
            sidejaws_st.on_enter(userdata)
            # Operating the mainjaws - open
            userdata.value = True
            #userdata.value = False
            mainjaws_st.on_enter(userdata)
        print('Done')
        return 0

    def move_slider(self, position:str = 'front'):
        """
        Move the slider behind the vise
        
        Args:
        -----
            - position(str) : Determines whether we want the slide to move to the front or back position
        
        Examples:
        ---------
            Move slider to back position:
            >>> move_slider(position='back')

            Move slider to front position:
            >>> move_slider(position='front')
        """
        userdata = self.userdata
        slider_st = self.slider_st

        assert position in ['front', 'back'], "Invalid position argument, should be either 'front' or 'back'"
        if position == 'front':
            userdata.value = True
        else:
            userdata.value = False
        slider_st.on_enter(userdata)

    def rotate_vise(self, position:str='up'):
        """ 
        Tilting the vise
        
        Args:
        -----
            - position(str) : Determines the orientation of the vise

        Examples:
        ---------
            Tilting the vise down:
            >>> rotate_vise(position='down')

            Returning vise to normal, upright position:
            >>> rotate_vise(position='up')
        
        """

        userdata = self.userdata
        rotate_vice_st = self.rotate_holder_st

        assert position in ['up', 'down'], "Invalid position argument, should be either 'up' or 'down'"
        if position == 'down':
            userdata.value = True
        else:
            userdata.value = False
        rotate_vice_st.on_enter(userdata)
        
    def rotate_vise_eject_clamptray(self):
        """ 
        Rotate vise and eject the tray below it
        """

        userdata = self.userdata
        slider_st = self.slider_st
        rotate_holder_st = self.rotate_holder_st
        clamptray_st = self.clamptray_st

        min_t = 4
        # Move the slider back
        # Operating the slider
        userdata.value = False
        slider_st.on_enter(userdata)
        time.sleep(min_t)
        # NOT DONE CURRENTLY Open the mainjaws and sidejaws
        # Rotate holder down
        # Rotate clamptray
        userdata.value = True
        rotate_holder_st.on_enter(userdata)
        time.sleep(min_t)
        # Move clamptray out
        # Open sliding tray below the clamp
        userdata.value = True
        clamptray_st.on_enter(userdata)
        time.sleep(min_t)
        # Rotate holder back up
        userdata.value = False
        rotate_holder_st.on_enter(userdata)
        time.sleep(min_t)
        # close the clamptray
        # Open sliding tray below the clamp
        userdata.value = False
        clamptray_st.on_enter(userdata)
        time.sleep(min_t)

        return 0
    
    def move_cutter(self, position:str = 'up'):
        """ 
        DEPRECATED: CUTTER IS NO LONGER IN USE
        """

        userdata = self.userdata
        cutter_st = self.cutter_st

        assert position in ['up', 'down'], "Invalid position argument, sholude be either 'up' or 'down'"
        if position == 'up':
            userdata.value = False
        else:
            userdata.value = True
        cutter_st.on_enter(userdata)

    def activate_cutter_airblower(self, position:str = 'on'):
        """ 
        DEPRECATED: CUTTER IS NO LONGER IN USE
        """
        userdata = self.userdata
        cutter_airblower_st = self.cutter_airblower_st
        if position == 'on':
            userdata.value = True
        else:
            userdata.value = False
        cutter_airblower_st.on_enter(userdata)

        return 0
    
    def helper_cut(self, only_blow_air = True):
        """ 
        DEPRECATED: CUTTER IS NO LONGER IN USE
        Cutter - Helper function to activate cutter to move down, then move up, then activate airblower to blow stuff out.
        Move this to pneumatics utils later. A helper function for activating the cutter sequence"""

        if not only_blow_air:
            self.move_cutter(position = 'down'); time.sleep(4)
            self.move_cutter(position = 'up');   time.sleep(1.5)
        self.activate_cutter_airblower(position = 'on'); time.sleep(1.5)
        self.activate_cutter_airblower(position = 'off')
        return 0
    
    def prepare_pneumatics(self, activate_vise:bool = False, activate_cnc:bool = False):
        """ 
        Helper function to activate the pneumatics block which provide air to downstream users.
        
        Args:
        ----
            - activate_vise(bool) : Determines whether the function enables the vise pneumatics distro block
            - activate_cnc(bool) : Determines whether the function enables the CNC pneumatics distro block

        Examples:
        ---------
            Only activate the CNC block:
            >>> prepare_pneumatics(activate_vise=False, activate_cnc=True)

            Activate both blocks:
            >>> prepare_pneumatics(activate_vise=True)
        """
        
        if activate_vise:
            #Turn on air to block 2 (Vise)
            self.userdata.value = True; self.activate_block2_st.on_enter(self.userdata)
            # Open vice
            self.handle_pneumatics_jaws(command = 'open');time.sleep(0.1)
        
        if activate_cnc:
            self.userdata.value=True
            self.cnc_airblock_state.on_enter(self.userdata)
        
        # Turn on air to panda_1 airblock (so toolchanger and pneumatic tools can be operated)
        self.userdata.value = True; self.panda1_airblock_state.on_enter(self.userdata)
        
        return 0

    def handle_cnc_chuck(self, position:str='open'):
        """
        CNC chuck handler function

        Args:
        ----
            - position(str) : Determines the desired state of the CNC chuck
        
        Examples:
        --------
            Open the CNC chuck:
            >>> handle_cnc_chuck(position='open')

            Close the CNC chuck:
            >>> handle_cnc_chuck(position='closed')
        """
        userdata = self.userdata
        if position == 'open':
            userdata.value = True
            self.cnc_chuck_state.on_enter(userdata)
        elif position == 'close':
            userdata.value = False
            self.cnc_chuck_state.on_enter(userdata)