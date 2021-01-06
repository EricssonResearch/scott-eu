#!/usr/bin/env python3

"""
Utility for setting up network namespaces for the following network:

              WiFi 10.0.0.0/24
      *       *       ...     *         *
      |       |               |         |
   robot1  robot2          robotN   MEC server

TODO
"""

import os
import subprocess
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pathlib import Path


# TODO: fai classe per Veth
# TODO: fai file utils con NetDevice etc. + file wifi con setup_wifi + file 5g con setup_5g, etc...

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


def get_netns(i, n_netns):
    return 'robot_%d' % i if i != n_netns-1 else 'mec_server'


def get_wifi_net_devices(i):
    veth_pair = (
        NetDevice('w%dveth' % i),                                   # for global netns
        NetDevice('w%deth' % i, '10.0.0.%d' % (i + 1), 24)          # for netns
    )
    tap = NetDevice('w%dtap' % i)
    bridge = NetDevice('w%dbridge' % i)
    return veth_pair, tap, bridge


def get_direct_net_devices(i):
    veth_pair = (
        NetDevice('d%dveth' % i, '10.0.1.%d' % (i * 4 + 1), 30),
        NetDevice('d%deth' % i, '10.0.1.%d' % (i * 4 + 2), 30)
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

    # assign IP address
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

    # assign IP address
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip netns exec %s ip address add %s dev %s' % (netns, veth_netns.ip_device, veth_netns.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip netns exec %s ip link set %s up' % (netns, veth_netns.name))


def setup_network(n_netns):
    for i in range(n_netns):
        netns = get_netns(i, n_netns)
        setup_netns(netns)
        setup_indirect(netns, *get_wifi_net_devices(i))
        setup_direct(netns, get_direct_net_devices(i))


def setup_firewall_net_device(net_device, trust):
    action = 'I' if trust else 'D'
    run_cmd('iptables -%s INPUT -i %s -j ACCEPT' % (action, net_device.name))
    run_cmd('iptables -%s FORWARD -i %s -j ACCEPT' % (action, net_device.name))


def setup_firewall(n_netns):
    # bridge does not look at iptables
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('0')

    # accept all packets from network namespaces
    for i in range(n_netns):
        setup_firewall_net_device(get_wifi_net_devices(i)[0][0], trust=True)
        setup_firewall_net_device(get_direct_net_devices(i)[0], trust=True)


def setup_dns(n_netns):
    global_netns = get_global_netns()
    network_namespaces = [get_netns(i, n_netns) for i in range(n_netns)]

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
                veth_netns = get_wifi_net_devices(j)[0][1]
                f.write('%s\t%s\n' % (veth_netns.ip_route, netns_))

    # restart to have effect
    run_cmd('systemctl restart networking')


def setup(n_netns):
    if os.geteuid() != 0:
        raise Exception('Run as root for setting up')
    print('Setting up %d network namespaces...' % n_netns)
    setup_network(n_netns)
    setup_firewall(n_netns)
    setup_dns(n_netns)


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


def clean_network(n_netns):
    for i in range(n_netns):
        netns = get_netns(i, n_netns)
        clean_indirect(*get_wifi_net_devices(i))
        clean_direct(get_direct_net_devices(i))
        clean_netns(netns)


def clean_firewall(n_netns):
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('1')

    for i in range(n_netns):
        setup_firewall_net_device(get_wifi_net_devices(i)[0][0], trust=False)
        setup_firewall_net_device(get_direct_net_devices(i)[0], trust=False)


def clean_dns(n_netns):
    global_netns = get_global_netns()
    network_namespaces = {get_netns(i, n_netns) for i in range(n_netns)} | {global_netns}

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


def clean(n_netns):
    if os.geteuid() != 0:
        raise Exception('Run as root for cleaning up')
    print('Cleaning up %d network namespaces...' % n_netns)
    clean_firewall(n_netns)
    clean_dns(n_netns)
    clean_network(n_netns)


def spawn_netns(netns):
    subprocess.run('gnome-terminal -- bash -c "sudo ./activate_netns.sh %s"' % netns, shell=True)


def spawn(n_netns):
    if os.geteuid() == 0:
        raise Exception('Do not run as root for spawning')
    print('Spawning shells for %d network namespaces...' % n_netns)
    spawn_netns(get_global_netns())
    for i in range(n_netns):
        spawn_netns(get_netns(i, n_netns))


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', type=str, help='Command to execute', choices=['setup', 'clean', 'spawn'])
    parser.add_argument('-r', '--robots', type=int, default=1, help='Number of robots')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    if args.robots < 0 or args.robots > 255:
        raise ValueError('Invalid number of network namespaces')
    n_netns = args.robots + 1

    if args.command == 'setup':
        setup(n_netns)
    elif args.command == 'clean':
        clean(n_netns)
    elif args.command == 'spawn':
        spawn(n_netns)
    else:   # should never happen, ArgumentParser already checks for valid commands
        raise ValueError('Invalid command: %s' % args.command)

    print('Done')


if __name__ == '__main__':
    main()
