import yaml 
import rospy
from std_srvs.srv import SetBool, Trigger, Empty
from colorama import Fore
from pprint import pprint

class ServiceClientManager():
    """
    A service client wrapper that streamlines the creation of service client proxies
    by initialising them based on a human-readable YAML document

    Args:
    -----
        - config_path(str) : Path to the YAML configuration file

        - Other args will be output in the terminal
    """
    def __init__(self, config_path:str) -> None:
        self.configPath = config_path
        with open(self.configPath, 'r') as configFile:
            config = yaml.load(configFile)
            for configline in config['pin_configs']:
                if '/' in configline['service_name']:
                    splitname = configline['service_name'].split('/')
                    name = '_'.join(splitname)
                else:
                    name = configline['service_name']
                if configline['actual_config'] == 'DigitalInput':
                    setattr(self, name, rospy.ServiceProxy(configline['service_name'], Trigger))
                elif configline['actual_config'] == 'DigitalOutput':
                    setattr(self, name, rospy.ServiceProxy(configline['service_name'], SetBool))
                else:
                    pass

if __name__ == '__main__':
    manager = ServiceClientManager('/ros_ws/src/disassembly_pipeline/disassembly_pipeline/utils/testconf.yaml')
    manager.realsense_alphapose_enable(False)
    atrs = vars(manager)
    pprint(atrs)