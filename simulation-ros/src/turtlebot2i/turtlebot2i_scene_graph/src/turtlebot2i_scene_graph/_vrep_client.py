"""
Make sure to have the remote API server running in V-REP.

By default, V-REP launches it on port 19997, so you do not need to do anything if you connect to this port.
See https://forum.coppeliarobotics.com/viewtopic.php?t=7514.

Alternatively, you can open a server on other ports adding the following in a child script of a V-REP scene:
simRemoteApi.start(19999)

http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
"""

import vrep
import socket


class VrepClient(object):
    def __init__(self):
        self.clientID = -1

    def start_connection(self, host, port):
        """Opens remote API connection with V-REP. If there are other open connections, they are closed.

        :param host: hostname or IP address of V-REP remote API server
        :param port: port of V-REP remote API server
        """
        # close old connection
        if self.is_connected():
            vrep.simxFinish(self.clientID)

        # start new connection
        ip = socket.gethostbyname(host)
        self.clientID = vrep.simxStart(ip, port, True, True, 5000, 5)
        if not self.is_connected():
            raise Exception('Connection to V-REP remote API server failed')

    def close_connection(self):
        """Closes remote API connection with V-REP."""
        vrep.simxFinish(self.clientID)

    def is_connected(self):
        return self.clientID != -1
