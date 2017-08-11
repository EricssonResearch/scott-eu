"""
Agent

i.e. youBot

@author Ricardo Souza
"""

from queue import *

class Agent:
    
    """
    Object that represent an agent with the following attributes
    ID, Task Queue, Current Task, Plan
    """
    
    def __init__(self, id_, current_task=None, plan=None):
        """
        Constructor
        """
        self.id_ = id_
        self.current_task = current_task
        if plan == None:
            self.plan = {}
            self.plan["plan"] = []
        else:
            self.plan = plan
        self.queue = Queue()


    def get_id(self):
        return self.id_

    def get_queue(self):
        return self.queue

    def get_current_task(self):
        return self.current_task

    def get_plan(self):
        return self.plan

    def set_current_task(self, ct):
        self.current_task = ct

    def append_task_to_plan(self, tsk):
        self.plan["plan"].append(tsk)

    def set_plan(self, pl):
        self.plan = pl
    
    #debug
    def print_(self):
        print(self.id_)
        print(self.current_task)
        print(self.plan)
        print(self.queue)
