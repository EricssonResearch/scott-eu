import rospy
import subprocess
import re
import numpy as np
from std_msgs.msg import Header
from turtlebot2i_edge.srv import Stamp, MeasureSnr, Ping, PingResponse


class NetworkMonitor:
    def __init__(self, server_host=None, bandwidth=None, shannon=False, ns3_simulation=False):
        if not ns3_simulation and server_host is None:
            raise ValueError('Server host is necessary for pinging if it not a simulation')
        if bandwidth is None and shannon:
            raise ValueError('Bandwidth is necessary for throughput estimation with Shannon-Hartley theorem')
        if shannon:     # TODO: investigate, SNR is always the same independently of the congestion
            raise NotImplementedError('Throughput estimation with Shannon-Hartley does not work properly yet')

        self.server_host = server_host
        self.bandwidth = bandwidth
        self.shannon = shannon
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

        if shannon:
            rospy.loginfo('Waiting for ROS service to measure SNR...')
            self._ns3_measure_snr = rospy.ServiceProxy('edge/measure_snr', MeasureSnr)
            self._ns3_measure_snr.wait_for_service()
        else:
            self._ns3_measure_snr = None

    def measure_rtt(self, max_rtt=0.1):
        if self.ns3_simulation:
            max_rtt = rospy.Time.from_sec(max_rtt)
            rtt = None
            while rtt is None:      # while loop because the ns-3 can crash, even if rarely
                try:
                    response = self._ns3_ping(max_rtt=max_rtt)
                    rtt = response.rtt.to_sec() * 1e3
                except (rospy.ROSException, rospy.ServiceException):
                    pass
        else:
            rtt = self._ping(max_rtt=max_rtt)
        return rtt

    def measure_throughput(self, n_bytes=2**20, max_duration=0.1):
        max_duration = rospy.Time.from_sec(max_duration)

        if self.shannon:
            if self.ns3_simulation:
                snr = None
                while snr is None:
                    try:
                        response = self._ns3_measure_snr(max_duration=max_duration)
                        snr = response.snr
                    except (rospy.ROSException, rospy.ServiceException):
                        pass
            else:
                snr = self._measure_snr(max_duration=max_duration)
            throughput = self.bandwidth * np.log2(1 + snr)

        else:
            throughput = None
            while throughput is None:   # while loop because the ns-3 can crash, even if rarely
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
                except (rospy.ROSException, rospy.ServiceException):
                    if not self.ns3_simulation:
                        throughput = 0
                        break

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

    def _measure_snr(self, max_duration):
        # TODO: implement with real network
        raise NotImplementedError
