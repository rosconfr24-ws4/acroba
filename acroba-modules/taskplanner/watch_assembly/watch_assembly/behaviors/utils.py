
import py_trees 
import typing 


class Remap(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, remap_to: typing.Dict[str, str]):
        """
        remap a variable 
        Args:
            name: behaviour name
            remap_to: remappings (from variable name to variable name)
        """
        super().__init__(name=name)
        self._remap_to = remap_to
        pass 

    def setup(self, timeout): 
        self._bb = py_trees.blackboard.Blackboard()


    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        for key, new_key in self._remap_to.items(): 
            value = self._bb.get(key)
            self._bb.set(new_key, value)
        return py_trees.common.Status.SUCCESS
    
    