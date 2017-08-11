from subprocess import Popen
from py4j.java_gateway import JavaGateway
import os
import subprocess
import re
import time

class SoarAgent(object):
    """A class for creating SOAR agent objects. This SoarAgent connects to a java SoarAgent which in turn is responsible for interacting with JSOAR. 
    Soar Agents are created with the following properties:

    Attributes:
        name: A string representing the agent's name.
        port: An integer specifying which port the python side of the agent will use to talk to JSOAR
    @author Klaus Raizer (eklarai)
    @date 2017-07-24
    """

    def __init__(self, name, port=25333):
        """Returns a Soar Agent named *name* and which uses port *port* for communicating with JSOAR."""
        self.name = name
        self.port = port
        
        self.free_port(port)

        self.start_agent_gateway()

       # self.initialize_agent(self.name,"simple-example.soar")        

    # Local methods

    def close_agent(self):
        """ Closes the agent and terminates its process."""
        print("Closing agent...")
        print("Terminating gateway...")
        self.p.terminate()
        print("...done.")

    def free_port(self, port):
        """Makes sure the passed port is free."""
        print("Making sure port "+str(port)+" is free...")
        command="fuser -k "+str(port)+"/tcp" #TODO: For osx users you can use sudo lsof -t -i tcp:8000 | xargs kill -9
        print(command)
        pf=Popen([command, 'ls'], shell=True)
        time.sleep(1)
        pf.terminate()

    def start_agent_gateway(self):
        """Starts a new agent gateway for communication with JSOAR"""
        print("Starting agent gateway in subprocess...")
        command="java -cp \".:*:jsoar/*:\" P4jSoarAgent"
        print(command)
        p=Popen([command, 'ls'], shell=True)
        self.p=p
        time.sleep(1)

        print("Getting gateway entry point...")
        gateway = JavaGateway()                   # connect to the JVM at (127.0.0.1:25333)
        time.sleep(1)
        gateway = gateway.entry_point        # get the agent instance
        self.gateway=gateway
    
    def initialize_agent(self, rules):
        """Initializes agent with given name and rules"""
    #TODO: rule loading should have its own method
        result=self.gateway.newSoarAgent(self.name)    
        print(result)
        result=self.gateway.loadRules(rules)    
        print(result)
        print("Initializing...")
        self.gateway.initialize()     

    # Methods inside java gateway
    def UpdateWme(self, wmeKey, value): #TODO values could be of other types, such as floats
        return self.gateway.UpdateWme(wmeKey,value)

    def InputWme(self,identifier,symbol, value):#TODO values could be of other types, such as floats
        return self.gateway.InputWme(identifier,symbol, value)

    def getWMEsUnder(self, name_letter, name_number):
        return self.gateway.getWMEsUnder(name_letter, name_number)
    def getOutputWMEs(self):
        return self.getWMEsUnder('I',3)
    def getInputLink(self):
        return self.gateway.getInputLink()

    def getAllWmesInRete(self):
        """" Returns a string with all elements in working memory."""
        return self.gateway.getAllWmesInRete()

    def addSensorReadingsToInput(self, sensorReadings):
        """Updates working memory inputs with given sensor readings."""
        #TODO: move code from java gateway to here?
        self.gateway.addSensorReadingsToInput(sensorReadings)
        time.sleep(1)

    def printAllWME(self):
        """ Prints all elements in working memory in a readable fashion."""
        self.gateway.printAllWME()

    def runFor(self, n):
        """ Runs agents for "n" decision cycles"""
    #TODO: implement other types of run?
        print("Running for ",n," decison(s)...")
        self.gateway.runFor(n)

    def runForever(self):
        """ Runs agent forever """
        print("Running forever...")
        self.gateway.runForever()



