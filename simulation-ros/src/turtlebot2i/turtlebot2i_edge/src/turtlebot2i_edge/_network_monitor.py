import rospy
import subprocess
import re
import numpy as np
from std_msgs.msg import Header
from turtlebot2i_edge.srv import Stamp, Ping, PingResponse


class NetworkMonitor:
    def __init__(self, server_host, ns3_simulation=False):
        self.server_host = server_host
        self.ns3_simulation = ns3_simulation

        rospy.loginfo('Waiting for ROS service to stamp...')
        self._stamp = rospy.ServiceProxy('edge/stamp', Stamp)
        self._stamp.wait_for_service()

        if ns3_simulation:
            rospy.loginfo('Waiting for ROS service to ping...')
            self._ping = rospy.ServiceProxy('edge/ping', Ping)
            self._ping.wait_for_service()
        else:
            self._ping = self._linux_ping

    def measure_latency(self, duration=1):
        """Measure RTT and packet loss of the network using a ping test.

        :param duration: int, duration of the measurement in seconds
        :return: rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss
        """
        duration = rospy.Time.from_sec(duration)
        ping_result = self._ping(duration)
        return (    # return tuple, not ROS message, to be consistent with throughput
            ping_result.rtt_min,
            ping_result.rtt_avg,
            ping_result.rtt_max,
            ping_result.rtt_mdev,
            ping_result.packet_loss
        )

    def measure_throughput(self, n_bytes=2**20, max_duration=1):
        """Measures the throughput of the network using a ROS service.

        :param n_bytes: int, bytes to send for the measurement. Since there is a communication overhead given by the
            TCP 3-way handshake and the ROS service, the higher the size, the better the approximation of the
            throughput. Do not use a too small value.
        :param max_duration: double, max duration of the measurement in seconds
        :return: throughput (Mbps)
        """
        max_duration = rospy.Time.from_sec(max_duration)
        try:
            self._stamp.wait_for_service(timeout=0.1)
            bytes_ = b'\x00' * n_bytes
            time_start = rospy.Time.now()
            response = self._stamp(bytes=bytes_, max_duration=max_duration)
            time_end = response.header.stamp
            stamped = response.stamped

            time_elapsed = time_end - time_start
            time_elapsed = time_elapsed.to_sec()
            throughput = stamped * 8 * 1e-6 / time_elapsed
        except rospy.ROSException, rospy.ServiceException:
            throughput = 0

        return throughput

    def _linux_ping(self, duration):
        duration = duration.secs
        ping = subprocess.Popen(
            args=['ping', self.server_host, '-A', '-w', str(duration)],
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
