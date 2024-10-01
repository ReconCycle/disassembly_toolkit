import time
import rospy
from std_srvs.srv import SetBool, Trigger
from linear_rail_service.srv import SetPosition

from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule
from disassembly_pipeline.cnc_manager.src.action_client_cnc import CNCActionClient


class CNCModule(BaseWorkcellModule):
    def __init__(self, name = 'table_cnc'):
        """ 
        
        """
        
        self._cnc_client = CNCActionClient(wait_for_server=True, init_ros_node=True)

        super().__init__(name = name)

    def get_callable_submodules(self):
        return dict(cnc_client = self._cnc_client)

    def get_name(self):
        return self._name

    def clamp(self, **kwargs):
        self._cnc_client.move_chuck('close')

    def unclamp(self, **kwargs):
        self._cnc_client.move_chuck('open')