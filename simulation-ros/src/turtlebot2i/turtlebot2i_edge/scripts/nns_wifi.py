#!/usr/bin/env python3

"""
Utility for setting up network namespaces.

The two network namespaces communicate with each other over the WiFi network simulated by ns-3, while each network
namespace can communicate with the global network namespace (which runs V-REP or Gazebo for example) directly.

For the WiFi network, we need a veth pair and a tap for each network namespace, i.e. 3 IP addresses. For simplicity,
we use 10.0.0.0/24 for the WiFi network. So, recalling that there is also an access point which takes 1 IP address (the
last one, .254), the WiFi network gives a constraint on the maximum number of network namespaces: floor((256-3)/3) = 84.

For the direct connection to the global namespace, we need a veth pair, i.e. 2 IP addresses. For simplicity, we use
10.0.1.0/24 and each direct connection takes 2+2 = 4 IP addresses. So, the direct connections give a constraint on the
maximum number of network namespaces: floor(256 / 4) = 64.

For satisfying both constraints, we allow 64 network namespaces.

The names of the created network devices are structured in the following way: <goal><namespace><type> (e.g. w0bridge)
Where:
- <goal> is one of 'w' (WiFi) and 'd' (direct)
- <namespace> is the index of the network namespace
- <type> is 'veth', 'tap', 'bridge' etc.

Adapted from: https://github.com/nps-ros2/ns3_gazebo/blob/master/scripts/nns_setup.py
Reference: https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks
"""

import os
import re
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


def get_network_namespace(i):
    return 'nns%d' % i


def get_wifi_net_devices(i):
    veth_pair = (
        NetDevice('w%dveth' % i, '10.0.0.%d' % (i * 3 + 1), 24),
        NetDevice('w%dvethb' % i, '10.0.0.%d' % (i * 3 + 2), 24)
    )
    tap = NetDevice('w%dtap' % i, '10.0.0.%d' % (i * 3 + 3), 24)
    bridge = NetDevice('w%dbridge' % i)     # no IP address
    return veth_pair, tap, bridge


def get_direct_net_devices(i):
    veth_pair = (
        NetDevice('d%dveth' % i, '10.0.1.%d' % (i * 4 + 1), 30),
        NetDevice('d%dvethb' % i, '10.0.1.%d' % (i * 4 + 2), 30)
    )
    return veth_pair


def setup_nns(i):
    nns = get_network_namespace(i)
    run_cmd('ip netns add %s' % nns)
    run_cmd('ip netns exec %s ip link set dev lo up' % nns)


def setup_wifi(i):
    nns = get_network_namespace(i)

    # create net devices
    (veth_global, veth_nns), tap, bridge = get_wifi_net_devices(i)
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_nns.name))
    run_cmd('ip tuntap add %s mode tap' % tap.name)
    run_cmd('ip link add name %s type bridge' % bridge.name)

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_nns.name, nns))

    # connect veth and tap by means of a bridge
    run_cmd('ip link set %s master %s' % (veth_global.name, bridge.name))
    run_cmd('ip link set %s master %s' % (tap.name, bridge.name))

    # assign IP addresses
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip netns exec %s ip address add %s dev %s' % (nns, veth_nns.ip_device, veth_nns.name))
    run_cmd('ip addr flush dev %s' % tap.name)
    run_cmd('ip address add %s dev %s' % (tap.ip_device, tap.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip netns exec %s ip link set %s up' % (nns, veth_nns.name))
    run_cmd('ip link set %s up' % tap.name)
    run_cmd('ip link set %s up' % bridge.name)


def setup_direct(i):
    nns = get_network_namespace(i)

    # create net devices
    veth_global, veth_nns = get_direct_net_devices(i)
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_nns.name))

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_nns.name, nns))

    # assign IP addresses
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip netns exec %s ip address add %s dev %s' % (nns, veth_nns.ip_device, veth_nns.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip netns exec %s ip link set %s up' % (nns, veth_nns.name))


def setup_firewall(n):
    # bridge does not look at iptables
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('0')

    # accept all packets from network namespaces
    for i in range(n):
        veth_global = get_wifi_net_devices(i)[0][0]
        run_cmd('iptables -I INPUT -i %s -j ACCEPT' % veth_global.name)
        run_cmd('iptables -I FORWARD -i %s -j ACCEPT' % veth_global.name)

        veth_global = get_direct_net_devices(i)[0]
        run_cmd('iptables -I INPUT -i %s -j ACCEPT' % veth_global.name)
        run_cmd('iptables -I FORWARD -i %s -j ACCEPT' % veth_global.name)


def setup_dns(n):
    # global network namespace
    with open(Path('/') / 'etc' / 'hosts', mode='a') as f:
        # needed for ROS (e.g. ROS_HOSTNAME=global_nns, global_nns must be resolvable)
        f.write('\n127.0.0.1\tglobal_nns\n')

        # other network namespaces via direct connection
        for j in range(n):
            nns = get_network_namespace(j)
            veth_nns = get_direct_net_devices(j)[1]
            f.write('%s\t%s\n' % (veth_nns.ip_route, nns))

    # other network namespaces
    for i in range(n):
        nns = get_network_namespace(i)
        path = Path('/') / 'etc' / 'netns' / nns
        path.mkdir(exist_ok=True)

        with open(path / 'hosts', mode='w') as f:
            # needed for ROS (e.g. ROS_HOSTNAME=nns0, nns0 must be resolvable)
            f.write('127.0.0.1\t%s\n' % nns)

            # global network namespace via direct connection
            veth_global = get_direct_net_devices(i)[0]
            f.write('%s\tglobal_nns\n' % veth_global.ip_route)

            # other network namespaces via WiFi
            for j in range(n):
                nns = get_network_namespace(j)
                veth_nns = get_wifi_net_devices(j)[0][1]
                f.write('%s\t%s\n' % (veth_nns.ip_route, nns))

    # restart to have effect
    run_cmd('systemctl restart networking')


def clean_nns(i):
    nns = get_network_namespace(i)
    run_cmd('ip netns del %s' % nns)


def clean_wifi(i):
    (veth_global, _), tap, bridge = get_wifi_net_devices(i)
    run_cmd('ip link set dev %s down' % veth_global.name)
    run_cmd('ip link set dev %s down' % tap.name)
    run_cmd('ip link set dev %s down' % bridge.name)
    run_cmd('ip link delete dev %s' % veth_global.name)
    run_cmd('ip link delete dev %s' % tap.name)
    run_cmd('ip link delete dev %s' % bridge.name)


def clean_direct(i):
    veth_global, _ = get_direct_net_devices(i)
    run_cmd('ip link set dev %s down' % veth_global.name)
    run_cmd('ip link delete dev %s' % veth_global.name)


def clean_firewall(n):
    # bridge does not look at iptables
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('1')

    # accept all packets from network namespaces
    for i in range(n):
        veth_global = get_wifi_net_devices(i)[0][0]
        run_cmd('iptables -D INPUT -i %s -j ACCEPT' % veth_global.name)
        run_cmd('iptables -D FORWARD -i %s -j ACCEPT' % veth_global.name)

        veth_global = get_direct_net_devices(i)[0]
        run_cmd('iptables -D INPUT -i %s -j ACCEPT' % veth_global.name)
        run_cmd('iptables -D FORWARD -i %s -j ACCEPT' % veth_global.name)


def clean_dns(n):
    # global network namespace
    regex = re.compile('^(global_nns)|(nns[0-9]+)$')
    with open(Path('/') / 'etc' / 'hosts') as f:
        lines = f.readlines()
        lines = [line for line in lines if line == '\n' or line[0] == '#' or not regex.fullmatch(line.split()[1])]
    with open(Path('/') / 'etc' / 'hosts', mode='w') as f:
        f.writelines(lines)

    # other network namespaces
    for i in range(n):
        nns = get_network_namespace(i)
        path = Path('/') / 'etc' / 'netns' / nns / 'hosts'
        path.unlink()
        path = path.parent
        path.rmdir()

    # restart to have effect
    run_cmd('systemctl restart networking')


def spawn_nns(nns):
    subprocess.run('gnome-terminal -- bash -c "sudo ./activate_nns.sh %s"' % nns, shell=True)


def setup(n):
    if os.geteuid() != 0:
        raise Exception('Run as root for setting up')
    print('Setting up %d network namespaces...' % n)
    for i in range(n):
        setup_nns(i)
        setup_wifi(i)
        setup_direct(i)
    setup_firewall(n)
    setup_dns(n)


def clean(n):
    if os.geteuid() != 0:
        raise Exception('Run as root for cleaning up')
    print('Cleaning up %d network namespaces...' % n)
    clean_firewall(n)
    clean_dns(n)
    for i in range(n):
        clean_wifi(i)
        clean_direct(i)
        clean_nns(i)


def spawn(n):
    if os.geteuid() == 0:
        raise Exception('Do not run as root for spawning')
    print('Spawning shells for %d network namespaces...' % n)
    spawn_nns('global_nns')
    for i in range(n):
        spawn_nns('nns%d' % i)


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', type=str, help='Command to execute', choices=['setup', 'clean', 'spawn'])
    parser.add_argument('-c', '--count', type=int, default=2, help='Number of network namespaces')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    if args.count < 0 or args.count >= 64:
        raise ValueError('Invalid number of network namespaces')

    if args.command == 'setup':
        setup(args.count)
    elif args.command == 'clean':
        clean(args.count)
    elif args.command == 'spawn':
        spawn(args.count)
    else:
        # should never happen, ArgumentParser already checks for valid commands
        raise ValueError('Invalid command: %s' % args.command)

    print('Done')


if __name__ == '__main__':
    main()
