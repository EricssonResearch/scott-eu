"""
Make sure to have the remote API server running in V-REP.

By default, V-REP launches it on port 19997, so you do not need to do anything if you connect to this port.
See https://forum.coppeliarobotics.com/viewtopic.php?t=7514.

Alternatively, you can open a server on other ports in two ways: temporary or continuous.

See https://www.coppeliarobotics.com/helpFiles/en/remoteApiServerSide.htm
"""

import vrep
import socket


class VrepClient(object):
    def __init__(self):
        self.clientID = -1
        self._host = None
        self._port = None

    def open_connection(self, host, port):
        """Opens remote API connection with V-REP. If there are other open connections, they are closed.

        :param host: hostname or IP address of V-REP remote API server
        :param port: port of V-REP remote API server
        """
        self._host = socket.gethostbyname(host)
        self._port = port
        self.close_connection()
        self.clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)
        if not self._is_connected():
            raise Exception('Connection to V-REP remote API server failed')

    def close_connection(self):
        """Closes remote API connection with V-REP."""
        if self._is_connected():
            vrep.simxFinish(self.clientID)

    def reset_connection(self):
        """Closes and re-opens connection to the same remote API server"""
        self.close_connection()
        self.open_connection(self._host, self._port)

    def _is_connected(self):
        return self.clientID != -1
