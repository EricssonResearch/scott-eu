"""
Scene
@author Ricardo Souza
@date 30-06-2016
Description: Defines abstract class Scene
"""

from abc import *

class Scene(ABC):
    
    @abstractmethod
    def start(self):
        """Starts simulation"""
        pass

    @abstractmethod
    def stop(self):
        """Stops simulation"""
        pass

    @abstractmethod
    def get_poi_list(self):
        """Gets scene/sim points of interest list/dictionary"""
        pass

    @abstractmethod
    def get_poi_matrix(self):
        """Get matrix cost between all points of interest"""
        pass

        
