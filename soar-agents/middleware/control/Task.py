"""
Task Class

@author Ricardo Souza
@date 15-08-2016
"""

import simplejson as json

class Task:

    """
    Object that represents a task
    """

    def __init__(self, json_):
        """
        Constructor
        """
        self.operation = json_["operation"]
        self.id_ = json_["id"]
        self.parameters= json_["par"]
        self.op_id = json_["op_id"]
        self.target = json_["target"]

    def get_target(self):
        """
        Return task's target
        """
        return self.target

    def get_operation(self):
        """
        Return task's operation
        """
        return self.operation

    def get_ID(self):
        """
        Return task ID
        """
        return self.id_

    def get_parameters(self):
        """
        Return operation parameters structure
        """
        return self.parameters

    def get_operation_id(self):
        return self.op_id
