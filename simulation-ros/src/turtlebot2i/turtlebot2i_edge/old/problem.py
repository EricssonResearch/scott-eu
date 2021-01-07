import numpy as np


class Network:
    """
    Hybrid network with 5G cell + fiber link.

    The 5G cell is used to connect mobile devices (MDs) to the base station (BS). Since 5G uses OFDM (Orthogonal
    Frequency Division Multiplexing), the cell can be represented with K channels with a certain bandwidth. So, maximum
    K mobile devices can use the cell simultaneously.

    The fiber link is used to connect the MEC server to the BS. We represent it just with the uplink data rate, since
    we assume that the responses from the MEC server to the mobile devices are very small.

    MD1  MD2  MD3  MD4 ... MDN
    |    |    |    |        |
    =========================
      5G      |
             BS -------------- MEC server
                  fiber link
    """

    def __init__(self, n_channels, channel_bandwidth, fiber_bandwidth, background_noise, path_loss):
        """
        Constructs a network.

        :param n_channels: number of orthogonal frequency channels
        :param channel_bandwidth: channel bandwidth (Hz)
        :param fiber_bandwidth: uplink data rate of optical fiber network (bit/s)
        :param background_noise: background noise power
        :param path_loss: path loss factor
        """
        self.n_channels = n_channels
        self.channel_bandwidth = channel_bandwidth
        self.fiber_bandwidth = fiber_bandwidth
        self.background_noise = background_noise
        self.path_loss = path_loss

    def transmit(self, mobile_devices):
        """
        Transmit tasks from mobile devices to MEC server.

        :param mobile_devices: 1D numpy array, mobile_devices that want to transmit their task
        :return: 1D numpy array, latencies
        """
        gains = [md.bs_distance ** (-self.path_loss) for md in mobile_devices]
        tot_power = sum([g * md.robot_transmit_power for g, md in zip(gains, mobile_devices)])
        latencies = np.zeros(len(mobile_devices))

        for i, md in enumerate(mobile_devices):
            # Shannon-Hartley theorem
            signal_power = md.robot_transmit_power * gains[i]
            interference_power = tot_power - signal_power + self.background_noise
            uplink_data_rate = self.channel_bandwidth * np.log2(1 + signal_power / interference_power)

            # latency from MD to MEC server
            if uplink_data_rate == 0:   # underflow
                uplink_data_rate += np.finfo(float).eps
            latency = md.task.input_size / uplink_data_rate + md.task.input_size / self.fiber_bandwidth
            latencies[i] = latency

        return latencies


class MobileDevice:
    """Mobile device."""

    def __init__(self, computing_ability, cpu_energy_consumption, transmit_power, bs_distance, task):
        """
        Constructs a mobile device.

        :param computing_ability: computing ability (cycle/s)
        :param cpu_energy_consumption: energy consumption per CPU cycle (J/cycle)
        :param transmit_power: transmit power (W)
        :param bs_distance: distance from BS in the cell (m)
        :param task: task
        """
        self.computing_ability = computing_ability
        self.cpu_energy_consumption = cpu_energy_consumption
        self.transmit_power = transmit_power
        self.bs_distance = bs_distance
        self.task = task

    def compute_task(self):
        """
        Simulates local computation of the task.

        :return: tuple (latency, energy)
        """
        latency = self.task.cpu_cycles / self.computing_ability
        energy = self.task.cpu_cycles * self.cpu_energy_consumption
        return latency, energy


class MECServer:
    """MEC server."""

    def __init__(self, computing_ability):
        """
        Constructs a MEC server.

        :param computing_ability: computing ability (cycle/s).
        """
        self.computing_ability = computing_ability

    def compute_task(self, task):
        """
        Simulates computation of a task.

        :param task: task to compute
        :return: latency
        """
        latency = task.cpu_cycles / self.computing_ability
        return latency


class Task:
    """Task."""

    def __init__(self, input_size, cpu_cycles, max_latency):
        """
        Constructs a task.

        :param input_size: input data size (bytes)
        :param cpu_cycles: required CPU cycles (cycles)
        :param max_latency: maximum permissible latency (seconds)
        """
        self.input_size = input_size
        self.cpu_cycles = cpu_cycles
        self.max_latency = max_latency


class Problem:
    def __init__(self, n_mobile_devices, seed=None):
        if seed is not None:
            np.random.seed(seed)

        self.network = Network(
            n_channels=50,
            channel_bandwidth=40e6,                 # 40 MHz
            fiber_bandwidth=1e9,                    # 1 Gbps
            background_noise=1e-13,                 # -100 dBm
            path_loss=4
        )

        self.mec_server = MECServer(
            computing_ability=10e9                  # 10 GHz
        )

        computing_abilities = np.random.choice([0.5e9, 0.8e9, 1e9], size=n_mobile_devices)  # 0.5-0.8-1 GHz
        input_sizes = np.random.uniform(300e3, 800e3, size=n_mobile_devices)                # 300-800 KB
        cpu_cycles = np.random.uniform(100e6, 1e9, size=n_mobile_devices)                   # 100-1000 Megacycles
        max_latencies = np.random.uniform(0.1, 2, size=n_mobile_devices)                    # 0.1-2 s
        bs_distances = np.random.uniform(0, 50, size=n_mobile_devices)                      # 0-50 m

        self.mobile_devices = np.array([
            MobileDevice(
                computing_ability=computing_abilities[i],
                cpu_energy_consumption=500e-12,     # 500 mJ/Gigacycle (equal for all)
                transmit_power=100e-3,              # 100 mW (equal for all)
                bs_distance=bs_distances[i],
                task=Task(input_sizes[i], cpu_cycles[i], max_latencies[i])
            )
            for i in range(n_mobile_devices)
        ])
