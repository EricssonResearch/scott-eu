"""
ShelfBase
@author Ricardo Souza
@date 21-06-2016
Description: Defines abstract class ShelfBase
"""

from abc import *

class StdShelfBase(ABC):

    @abstractmethod
    def fill(self, shelf_ID, stock):
        """Fill shelf with products"""
        pass

    @abstractmethod
    def add_product(self, shelf_ID, type_):
        """Add product of type_ to shelf"""
        pass

    @abstractmethod
    def get_pickable_list(self, shelf_ID):
        """Returns vector containing prickable products"""
        pass
