import rospy
import subprocess
import re
from turtlebot2i_edge.srv import Stamp


class NetworkMonitor:
    def __init__(self, server_host, throughput=False):
        self.server_host = server_host
        self.throughput = throughput

        if throughput:
            rospy.loginfo('Waiting for ROS service to measure throughput...')
            self._stamp = rospy.ServiceProxy('edge/stamp', Stamp)
            self._stamp.wait_for_service()

    def measure_latency(self, time=1):
        """Adapted from https://github.com/larics/python-network-tools/blob/master/network-test.py"""
        ping = subprocess.Popen(
            args=['ping', self.server_host, '-A', '-w', str(time)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        output, _ = ping.communicate()

        summary = re.findall(r'rtt min/avg/max/mdev = (\S+)', output)[0]
        rtt_min, rtt_avg, rtt_max, rtt_mdev = (float(x) * 1e-3 for x in summary.split('/'))
        packet_loss = float(re.findall(r'(\S+)% packet loss', output)[0]) / 100

        return rtt_min, rtt_avg, rtt_max, rtt_mdev, packet_loss

    def measure_throughput(self, size=2**10):
        if not self.throughput:
            raise Exception('NetworkMonitor initialized with throughput=False')

        try:
            self._stamp.wait_for_service(timeout=0.1)
            bytes_ = b'\x00' * size
            time_start = rospy.Time.now()
            response = self._stamp(bytes=bytes_)
            time_end = response.header.stamp

            time_elapsed = time_end - time_start
            time_elapsed = time_elapsed.to_sec()
            throughput = size * 8 / time_elapsed
        except rospy.ROSException, rospy.ServiceException:
            throughput = 0

        return throughput
