import rospy
import subprocess
import re
import numpy as np
from std_msgs.msg import Header
from turtlebot2i_edge.srv import Stamp, MeasureSnr, Ping, PingResponse


class NetworkMonitor:
    def __init__(self, server_host=None, ns3_simulation=False):
        if not ns3_simulation and server_host is None:
            raise ValueError('Server host is necessary for pinging if it not a simulation')

        self.server_host = server_host
        self.ns3_simulation = ns3_simulation

        rospy.loginfo('Waiting for ROS service to stamp...')
        self._stamp = rospy.ServiceProxy('edge/stamp', Stamp)
        self._stamp.wait_for_service()

        if ns3_simulation:
            rospy.loginfo('Waiting for ROS service to ping...')
            self._ns3_ping = rospy.ServiceProxy('edge/ping', Ping)
            self._ns3_ping.wait_for_service()
        else:
            self._ns3_ping = None

    def measure_rtt(self, max_rtt=0.1):
        if self.ns3_simulation:
            max_rtt = rospy.Time.from_sec(max_rtt)
            try:
                self._ns3_ping.wait_for_service(timeout=0.1)
                response = self._ns3_ping(max_rtt=max_rtt)
                rtt = response.rtt.to_sec() * 1e3
            except (rospy.ROSException, rospy.ServiceException):
                rtt = (max_rtt.to_sec() + 1) * 1e3
        else:
            rtt = self._ping(max_rtt=max_rtt)
        return rtt

    def measure_throughput(self, n_bytes=2**20, max_duration=0.1):
        max_duration = rospy.Time.from_sec(max_duration)
        try:
            self._stamp.wait_for_service(timeout=0.1)
            time_start = rospy.Time.now()
            response = self._stamp(to_stamp=n_bytes, max_duration=max_duration)
            time_end = response.header.stamp
            stamped = response.stamped
            time_elapsed = time_end - time_start
            time_elapsed = time_elapsed.to_sec()
            throughput = stamped * 8 * 1e-6 / time_elapsed
        except (rospy.ROSException, rospy.ServiceException):
            throughput = 0
        return throughput

    def _ping(self, max_rtt):
        # TODO: use scapy or other libraries, Linux ping does not support timeout < 1 second
        ping = subprocess.Popen(
            args=['ping', self.server_host, '-c', '1', '-w', str(max_rtt)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        output, _ = ping.communicate()

        packet_loss = float(re.findall(r'(\S+)% packet loss', output)[0]) / 100
        if packet_loss != 1:
            summary = re.findall(r'rtt min/avg/max/mdev = (\S+)', output)[0]
            rtt_min, rtt_avg, rtt_max, rtt_mdev = (float(x) * 1e-3 for x in summary.split('/'))
        else:
            rtt_min = rtt_avg = rtt_max = np.inf
            rtt_mdev = 0

        return PingResponse(
            header=Header(stamp=rospy.Time.now()),
            rtt_min=rtt_min,
            rtt_avg=rtt_avg,
            rtt_max=rtt_max,
            rtt_mdev=rtt_mdev,
            packet_loss=packet_loss
        )
