import abc


class BaseWorkcellModule(abc.ABC):
    def __init__(self, name = 'base_module'):
        self._name = name

    @abc.abstractmethod
    def get_callable_submodules(self):
        """ Return dict of {name:object} with any callable submodules on the particular module, e.g. robotblockset_python.robot.Robot panda_2 if
        module is panda_2_module, or cnc_client if module is cnc_module"""
        return 

    def get_name(self):
        return self._name