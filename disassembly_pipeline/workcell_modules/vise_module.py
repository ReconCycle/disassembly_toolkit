import time
import rospy
from std_srvs.srv import SetBool

from disassembly_pipeline.workcell_modules.base_workcell_module import BaseWorkcellModule


class ViseModule(BaseWorkcellModule):
    def __init__(self, 
                 name = 'vise_module',
                 main_jaws_service_topic = "/mainjaws_open",
                 side_jaws_service_topic = "/sidejaws_open",
                 init_ros_node = True):
        """ 
        Enables controlling the Vise.
        
        Example call:
        >>> from disassembly_pipeline.workcell_modules.vise_module import ViseModule
        >>> c = ViseModule()
        # Basic functionality primitives
        >>> c.open_main_jaws()
        >>> c.close_main_jaws()

        >>> c.open_side_jaws()
        >>> c.close_side_jaws()

        # High-level operations
        >>> c.open()
        >>> c.close(sleep_duration = 2) # Close side jaws, wait a bit, close main jaws
        """
 
        if init_ros_node:
            # Check if node is already initialized
            if not rospy.get_node_uri():
                rospy.init_node("vise_module", anonymous = True)

        self._main_jaws_srv_topic = main_jaws_service_topic
        self._main_jaws_srv_proxy = rospy.ServiceProxy(name=self._main_jaws_srv_topic , service_class=SetBool)

        self._side_jaws_srv_topic = side_jaws_service_topic
        self._side_jaws_srv_proxy = rospy.ServiceProxy(name=self._side_jaws_srv_topic , service_class=SetBool)

        self._init_module()

        super().__init__(name = name)

    def _init_module(self):
        """
        Init
        """
        return 0

    def get_callable_submodules(self):
        return dict()

    def open_main_jaws(self):
        result = self._main_jaws_srv_proxy.call(True)
        return result
        
    def close_main_jaws(self):
        result = self._main_jaws_srv_proxy.call(False)
        return result
        
    def open_side_jaws(self):
        result = self._side_jaws_srv_proxy.call(True)
        return result
        
    def close_side_jaws(self):
        result = self._side_jaws_srv_proxy.call(False)
        return result

    def open(self):
        result_side_jaws = self.open_side_jaws()
        result_main_jaws = self.open_main_jaws()
        return result_side_jaws, result_main_jaws

    def close(self, sleep_duration = 2):
        result_side_jaws = self.close_side_jaws()
        time.sleep(sleep_duration)
        result_main_jaws = self.close_main_jaws()
        return result_side_jaws, result_main_jaws

    def clamp(self, **kwargs):
        self.close()

    def unclamp(self, **kwargs):
        self.open()
