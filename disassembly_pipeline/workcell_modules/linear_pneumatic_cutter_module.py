import time
import rospy
from std_srvs.srv import SetBool, Trigger
from linear_rail_service.srv import SetPosition

from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule


class LinearPneumaticCutterModule(BaseWorkcellModule):
    def __init__(self, 
                 name = 'linear_pneumatic_cutter',
                 cutter_srv_topic = "/cutter_open",
                 linear_rail_init_service_topic = "/linear_rail_service/init",
                 linear_rail_reset_service_topic = "/linear_rail_service/reset",
                 linear_rail_service_topic = "/linear_rail_service/drive",
                 initialize_linear_rail = True,
                 init_ros_node = True):
        """ 
        Enables controlling the Linear Cutter with integrated linear tray.

        Args: 
            initialize_linear_rail: Bool: If True, call the linear_rail/reset service, and then the /linear_rail/init service.
        
        Example call:
        >>> from disassembly_pipeline.workcell_modules.linear_pneumatic_cutter_module import LinearPneumaticCutterModule
        >>> c = LinearPneumaticCutterModule()
        # Basic functionality primitives
        >>> c.open_cutter()
        >>> c.close_cutter()

        >>> c.open_tray()
        >>> c.close_tray()

        # High-level operations
        >>> c.prepare_for_cutting()
        >>> c.cut_and_open_tray(sleep_duration_seconds = 4)
        """

        if init_ros_node:
            # Check if node is already initialized
            if not rospy.get_node_uri():
                rospy.init_node("linear_pneumatic_cutter", anonymous = True)

        self._cutter_srv_topic = cutter_srv_topic
        self._cutter_srv_proxy = rospy.ServiceProxy(name=self._cutter_srv_topic , service_class=SetBool)

        self._linear_rail_srv_topic = linear_rail_service_topic
        self._linear_rail_init_srv_topic = linear_rail_init_service_topic
        self._linear_rail_reset_srv_topic = linear_rail_reset_service_topic
        self._linear_rail_init_srv_proxy = rospy.ServiceProxy(name=self._linear_rail_init_srv_topic, service_class=Trigger)
        self._linear_rail_reset_srv_proxy = rospy.ServiceProxy(name=self._linear_rail_reset_srv_topic, service_class=Trigger)
        self._linear_rail_srv_proxy = rospy.ServiceProxy(name=self._linear_rail_srv_topic, service_class=SetPosition)
        
        # Init linear rail
        if initialize_linear_rail:
            self._init_linear_rail()

        super().__init__(name = name)

    def _init_linear_rail(self):
        """
        Init linear rail
        """
        result_reset = self._linear_rail_reset_srv_proxy.call()
        time.sleep(2)
        result_init = self._linear_rail_init_srv_proxy.call()
        return result_reset, result_init

    def get_callable_submodules(self):
        return dict()

    def open_cutter(self):
        result = self._cutter_srv_proxy.call(True)
        return result
        
    def close_cutter(self):
        result = self._cutter_srv_proxy.call(False)
        return result
        
    def open_tray(self):
        result = self._linear_rail_srv_proxy.call(pos=5)
        return result
        
    def close_tray(self):
        result = self._linear_rail_srv_proxy.call(pos=0)
        return result
        
    def prepare_for_cutting(self):
        """ 
        Prepare for cutting by opening the cutter and closing the tray (so stuff can fall into it after cutting)
        """
        result_tray = self.close_tray()
        result_cutter = self.open_cutter()
        return result_cutter, result_tray
    
    def cut_and_open_tray(self, sleep_duration_seconds = 4):
        result_cutter = self.close_cutter()
        time.sleep(sleep_duration_seconds)
        result_tray = self.open_tray()
        return result_cutter, result_tray