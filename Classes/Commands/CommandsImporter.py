import abc

class CommandsImporter():
    def __init__():
        pass


class Command():
    """
    Declare an interface for executing an operation.
    """
    #__metaclass__ = abs.ABCMeta

    def __init__(self, receiver):
        self._receiver = receiver

    @abc.abstractmethod
    def execute(self):
        pass

