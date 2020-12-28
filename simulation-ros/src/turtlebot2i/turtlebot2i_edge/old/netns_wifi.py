#!/usr/bin/env python3

"""
Utility for setting up network namespaces for the following network:

              WiFi 10.0.0.0/24
      *       *       ...     *       *
      |       |               |       |    10.0.2.0/29
   robot1  robot2          robotN     AP --------------- MEC server
                                  .254  .1

TODO: sistema
Each robot and the MEC server are run within a different network namespaces. The network can be simulated with
turtlebot2i_edge/ns3_wifi. The ROS master and V-REP are run in the global namespace.

Each network namespace can communicate:
1. With other network namespaces over the simulated network. This requires a veth pair and a tap, i.e. 3 IP addresses.
2. With the global network namespace directly. This requires a veth pair, i.e. 2 IP addresses.

For the WiFi network, given that we use 10.0.0.0/24, that each network namespace requires 3 IP addresses and that the AP
requires another IP address, we have a constraint on the maximum number of robots: floor((256-3)/3) = 84.

For the point-to-point network, given that each network namespace requires 3 IP addresses and that the AP requires
another IP address, we need /29, so we use 10.0.2.0/29.

For the direct connections to the global namespace, given that each network namespace requires 2 IP addresses, we need
a /30 for each network namespace. For simplicity we use 10.0.1.0/24, so we have a constraint on the maximum number of
robots: floor(256 / 4) - 1 = 63.

For satisfying both constraints, we allow 63 robots.

Adapted from: https://github.com/nps-ros2/ns3_gazebo/blob/master/scripts/nns_setup.py
Reference: https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks
"""

import os
import subprocess
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pathlib import Path


class NetDevice:
    def __init__(self, name, ip=None, prefix_length=None):
        self.name = name
        if ip is not None and prefix_length is not None:
            self.ip_device = ip + '/' + str(prefix_length)      # with prefix length
            self.ip_route = ip                                  # without prefix length


def run_cmd(cmd, verbose=False):
    if verbose:
        print('Command: %s' % cmd)
    subprocess.run(cmd.split(), stdout=subprocess.PIPE).stdout.decode('utf-8')


def get_global_netns():
    return 'global'


def get_robot_netns(i):
    return 'robot%d' % i


def get_server_netns():
    return 'server'


def get_wifi_net_devices(i):
    veth_pair = (
        NetDevice('w%dveth' % i),                                   # for global netns
        NetDevice('w%dvethb' % i, '10.0.0.%d' % (i * 2 + 1), 24)    # for netns
    )
    tap = NetDevice('w%dtap' % i)
    bridge = NetDevice('w%dbridge' % i)
    return veth_pair, tap, bridge


def get_p2p_net_devices():
    # TODO: sistema
    veth_pair = (
        NetDevice('p2pveth', '10.0.2.2', 29),
        NetDevice('p2pvethb', '10.0.2.3', 29)
    )
    tap = NetDevice('p2ptap', '10.0.2.4', 29)
    bridge = NetDevice('p2pbridge')         # no IP address
    return veth_pair, tap, bridge


def get_direct_net_devices(i):
    veth_pair = (
        NetDevice('d%dveth' % i, '10.0.1.%d' % (i * 4 + 1), 30),
        NetDevice('d%dvethb' % i, '10.0.1.%d' % (i * 4 + 2), 30)
    )
    return veth_pair


def setup_netns(netns):
    run_cmd('ip netns add %s' % netns)
    run_cmd('ip netns exec %s ip link set dev lo up' % netns)


def setup_indirect(netns, veth_pair, tap, bridge):
    veth_global, veth_netns = veth_pair

    # create net devices
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_netns.name))
    run_cmd('ip tuntap add %s mode tap' % tap.name)
    run_cmd('ip link add name %s type bridge' % bridge.name)

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_netns.name, netns))

    # connect veth and tap by means of a bridge
    run_cmd('ip link set %s master %s' % (veth_global.name, bridge.name))
    run_cmd('ip link set %s master %s' % (tap.name, bridge.name))

    # assign IP addresses
    run_cmd('ip netns exec %s ip address add %s dev %s' % (netns, veth_netns.ip_device, veth_netns.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip netns exec %s ip link set %s up' % (netns, veth_netns.name))
    run_cmd('ip link set %s promisc on up' % tap.name)
    run_cmd('ip link set %s up' % bridge.name)


def setup_direct(netns, veth_pair):
    veth_global, veth_netns = veth_pair

    # create net devices
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_netns.name))

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_netns.name, netns))

    # assign IP addresses
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip netns exec %s ip address add %s dev %s' % (netns, veth_netns.ip_device, veth_netns.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip netns exec %s ip link set %s up' % (netns, veth_netns.name))


def setup_network(n_robots):
    for i in range(n_robots):
        netns = get_robot_netns(i)
        setup_netns(netns)
        setup_indirect(netns, *get_wifi_net_devices(i))
        # setup_direct(netns, get_direct_net_devices(i))

    # netns = get_server_netns()
    # setup_netns(netns)
    # setup_indirect(netns, *get_p2p_net_devices())
    # setup_direct(netns, get_direct_net_devices(n_robots))


def setup_firewall_net_device(net_device, trust):
    action = 'I' if trust else 'D'
    run_cmd('iptables -%s INPUT -i %s -j ACCEPT' % (action, net_device.name))
    run_cmd('iptables -%s FORWARD -i %s -j ACCEPT' % (action, net_device.name))


def setup_firewall(n_robots):
    # bridge does not look at iptables
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('0')

    # accept all packets from network namespaces
    for i in range(n_robots):
        setup_firewall_net_device(get_wifi_net_devices(i)[0][0], trust=True)
        setup_firewall_net_device(get_direct_net_devices(i)[0], trust=True)
    setup_firewall_net_device(get_p2p_net_devices()[0][0], trust=True)
    setup_firewall_net_device(get_direct_net_devices(n_robots)[0], trust=True)


def setup_dns(n_robots):
    global_netns = get_global_netns()
    server_netns = get_server_netns()
    network_namespaces = [get_robot_netns(i) for i in range(n_robots)] + [server_netns]

    # global network namespace
    with open(Path('/') / 'etc' / 'hosts', mode='a') as f:
        # needed for ROS (e.g. ROS_HOSTNAME=global, global must be resolvable)
        f.write('127.0.0.1\t%s\n' % global_netns)

        # other network namespaces via direct connection
        for j, netns in enumerate(network_namespaces):
            veth_netns = get_direct_net_devices(j)[1]
            f.write('%s\t%s\n' % (veth_netns.ip_route, netns))

    # other network namespaces
    for i, netns in enumerate(network_namespaces):
        path = Path('/') / 'etc' / 'netns' / netns
        path.mkdir(exist_ok=True)

        with open(path / 'hosts', mode='w') as f:
            # needed for ROS (e.g. ROS_HOSTNAME=robot0, robot0 must be resolvable)
            f.write('127.0.0.1\t%s\n' % netns)

            # global network namespace via direct connection
            veth_global = get_direct_net_devices(i)[0]
            f.write('%s\t%s\n' % (veth_global.ip_route, global_netns))

            # other network namespaces via network
            for j, netns_ in enumerate(network_namespaces):
                if netns == netns_:
                    continue
                veth_netns = get_wifi_net_devices(j)[0][1] if netns_ != server_netns else get_p2p_net_devices()[0][1]
                f.write('%s\t%s\n' % (veth_netns.ip_route, netns_))

    # restart to have effect
    run_cmd('systemctl restart networking')


def setup(n_robots):
    if os.geteuid() != 0:
        raise Exception('Run as root for setting up')
    print('Setting up network namespaces for %d robots and 1 MEC server...' % n_robots)
    setup_network(n_robots)
    setup_firewall(n_robots)
    setup_dns(n_robots)


def clean_netns(netns):
    run_cmd('ip netns del %s' % netns)


def clean_indirect(veth_pair, tap, bridge):
    veth_global, _ = veth_pair
    run_cmd('ip link set dev %s down' % veth_global.name)
    run_cmd('ip link set dev %s down' % tap.name)
    run_cmd('ip link set dev %s down' % bridge.name)
    run_cmd('ip link delete dev %s' % veth_global.name)
    run_cmd('ip link delete dev %s' % tap.name)
    run_cmd('ip link delete dev %s' % bridge.name)


def clean_direct(veth_pair):
    veth_global, _ = veth_pair
    run_cmd('ip link set dev %s down' % veth_global.name)
    run_cmd('ip link delete dev %s' % veth_global.name)


def clean_network(n_robots):
    for i in range(n_robots):
        netns = get_robot_netns(i)
        clean_indirect(*get_wifi_net_devices(i))
        clean_direct(get_direct_net_devices(i))
        clean_netns(netns)

    netns = get_server_netns()
    clean_indirect(*get_p2p_net_devices())
    clean_direct(get_direct_net_devices(n_robots))
    clean_netns(netns)


def clean_firewall(n_robots):
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('1')

    for i in range(n_robots):
        setup_firewall_net_device(get_wifi_net_devices(i)[0][0], trust=False)
        setup_firewall_net_device(get_direct_net_devices(i)[0], trust=False)
    setup_firewall_net_device(get_p2p_net_devices()[0][0], trust=False)
    setup_firewall_net_device(get_direct_net_devices(n_robots)[0], trust=False)


def clean_dns(n_robots):
    global_netns = get_global_netns()
    network_namespaces = {get_robot_netns(i) for i in range(n_robots)} | {get_server_netns(), global_netns}

    # global network namespace
    with open(Path('/') / 'etc' / 'hosts') as f:
        lines = f.readlines()
        lines = [line for line in lines if line == '\n' or line[0] == '#' or line.split()[1] not in network_namespaces]
    with open(Path('/') / 'etc' / 'hosts', mode='w') as f:
        f.writelines(lines)

    # other network namespaces
    network_namespaces.remove(global_netns)
    for netns in network_namespaces:
        path = Path('/') / 'etc' / 'netns' / netns / 'hosts'
        path.unlink()
        path = path.parent
        path.rmdir()

    # restart to have effect
    run_cmd('systemctl restart networking')


def clean(n_robots):
    if os.geteuid() != 0:
        raise Exception('Run as root for cleaning up')
    print('Cleaning up network namespaces for %d robots and 1 MEC server...' % n_robots)
    clean_firewall(n_robots)
    clean_dns(n_robots)
    clean_network(n_robots)


def spawn_netns(netns):
    subprocess.run('gnome-terminal -- bash -c "sudo ./activate_netns.sh %s"' % netns, shell=True)


def spawn(n_robots):
    if os.geteuid() == 0:
        raise Exception('Do not run as root for spawning')
    print('Spawning shells for %d robots and 1 MEC server...' % n_robots)
    spawn_netns(get_global_netns())
    spawn_netns(get_server_netns())
    for i in range(n_robots):
        spawn_netns(get_robot_netns(i))


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', type=str, help='Command to execute', choices=['setup', 'clean', 'spawn'])
    parser.add_argument('-r', '--robots', type=int, default=1, help='Number of robots')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    if args.robots < 0 or args.robots >= 63:
        raise ValueError('Invalid number of network namespaces')

    if args.command == 'setup':
        setup(args.robots)
    elif args.command == 'clean':
        clean(args.robots)
    elif args.command == 'spawn':
        spawn(args.robots)
    else:   # should never happen, ArgumentParser already checks for valid commands
        raise ValueError('Invalid command: %s' % args.command)

    print('Done')


if __name__ == '__main__':
    main()
