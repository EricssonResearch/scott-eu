#!/usr/bin/env python3

"""
Utility for setting up network namespaces for the following networks:

- WiFi:

                    WiFi
        *       *       ...     *       *
        |       |               |       |
     robot1  robot2          robotN  MEC server



- 5G:

                    5G NR
          *       *       ...     *       *
          |       |               |       |      fiber
       robot1  robot2          robotN    gNB ------------ MEC server


TODO: explain
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


class VethPair:
    def __init__(self, external, internal):
        self.external = external    # in netns
        self.internal = internal    # in global netns


class IndirectNetDevices:
    def __init__(self, veth_pair, bridge, tap):
        self.veth_pair = veth_pair
        self.bridge = bridge
        self.tap = tap


def run_cmd(cmd, verbose=False):
    if verbose:
        print('Command: %s' % cmd)
    subprocess.run(cmd.split(), stdout=subprocess.PIPE).stdout.decode('utf-8')


def get_global_netns():
    return 'global'


def get_netns(i, n_netns):
    return 'robot_%d' % i if i != n_netns-1 else 'mec_server'


def get_default_gw():
    return '10.0.0.1'


def setup_netns(netns):
    run_cmd('ip netns add %s' % netns)
    run_cmd('ip netns exec %s ip link set dev lo up' % netns)


def setup_indirect(netns, indirect_net_devices):
    veth_pair = indirect_net_devices.veth_pair
    veth_ext = veth_pair.external
    veth_int = veth_pair.internal
    bridge = indirect_net_devices.bridge
    tap = indirect_net_devices.tap
    default_gw = get_default_gw()

    # create net devices
    run_cmd('ip link add %s type veth peer name %s' % (veth_int.name, veth_ext.name))
    run_cmd('ip tuntap add %s mode tap' % tap.name)
    run_cmd('ip link add name %s type bridge' % bridge.name)

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_ext.name, netns))

    # connect veth and tap by means of a bridge
    run_cmd('ip link set %s master %s' % (veth_int.name, bridge.name))
    run_cmd('ip link set %s master %s' % (tap.name, bridge.name))

    # assign IP address
    run_cmd('ip netns exec %s ip address add %s dev %s' % (netns, veth_ext.ip_device, veth_ext.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_int.name)
    run_cmd('ip netns exec %s ip link set %s up' % (netns, veth_ext.name))
    run_cmd('ip link set %s promisc on up' % tap.name)
    run_cmd('ip link set %s up' % bridge.name)

    # set default gateway
    run_cmd("ip netns exec %s ip route add default via %s" % (netns, default_gw))


def setup_direct(netns, veth_pair):
    veth_ext = veth_pair.external
    veth_int = veth_pair.internal

    # create net devices
    run_cmd('ip link add %s type veth peer name %s' % (veth_int.name, veth_ext.name))

    # move veth end to network namespace
    run_cmd('ip link set %s netns %s' % (veth_ext.name, netns))

    # assign IP address
    run_cmd('ip address add %s dev %s' % (veth_int.ip_device, veth_int.name))
    run_cmd('ip netns exec %s ip address add %s dev %s' % (netns, veth_ext.ip_device, veth_ext.name))

    # bring net devices up
    run_cmd('ip link set %s up' % veth_int.name)
    run_cmd('ip netns exec %s ip link set %s up' % (netns, veth_ext.name))


def setup_network(direct_net_devices, indirect_net_devices):
    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)
    for i in range(n_netns):
        netns = get_netns(i, n_netns)
        setup_netns(netns)
        setup_direct(netns, direct_net_devices[i])
        setup_indirect(netns, indirect_net_devices[i])


def setup_firewall_net_device(net_device, trust):
    action = 'I' if trust else 'D'
    run_cmd('iptables -%s INPUT -i %s -j ACCEPT' % (action, net_device.name))
    run_cmd('iptables -%s FORWARD -i %s -j ACCEPT' % (action, net_device.name))


def setup_firewall(direct_net_devices, indirect_net_devices):
    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)

    # bridge does not look at iptables
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('0')

    # accept all packets from network namespaces
    for i in range(n_netns):
        setup_firewall_net_device(indirect_net_devices[i].veth_pair.internal, trust=True)
        setup_firewall_net_device(direct_net_devices[i].internal, trust=True)


def setup_dns(direct_net_devices, indirect_net_devices):
    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)
    global_netns = get_global_netns()
    network_namespaces = [get_netns(i, n_netns) for i in range(n_netns)]

    # global network namespace
    with open(Path('/') / 'etc' / 'hosts', mode='a') as f:
        # needed for ROS (e.g. ROS_HOSTNAME=global, global must be resolvable)
        f.write('127.0.0.1\t%s\n' % global_netns)

        # other network namespaces via direct connection
        for j, netns in enumerate(network_namespaces):
            veth_ext = direct_net_devices[j].external
            f.write('%s\t%s\n' % (veth_ext.ip_route, netns))

    # other network namespaces
    for i, netns in enumerate(network_namespaces):
        path = Path('/') / 'etc' / 'netns' / netns
        path.mkdir(exist_ok=True)

        with open(path / 'hosts', mode='w') as f:
            # needed for ROS (e.g. ROS_HOSTNAME=robot0, robot0 must be resolvable)
            f.write('127.0.0.1\t%s\n' % netns)

            # global network namespace via direct connection
            veth_int = direct_net_devices[i].internal
            f.write('%s\t%s\n' % (veth_int.ip_route, global_netns))

            # other network namespaces via network
            for j, netns_ in enumerate(network_namespaces):
                if netns == netns_:
                    continue
                veth_ext = indirect_net_devices[j].veth_pair.external
                f.write('%s\t%s\n' % (veth_ext.ip_route, netns_))

    # restart to have effect
    run_cmd('systemctl restart networking')


def setup(direct_net_devices, indirect_net_devices):
    if os.geteuid() != 0:
        raise Exception('Run as root for setting up')

    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)

    print('Setting up %d network namespaces...' % n_netns)
    setup_network(direct_net_devices, indirect_net_devices)
    setup_firewall(direct_net_devices, indirect_net_devices)
    setup_dns(direct_net_devices, indirect_net_devices)


def clean_netns(netns):
    run_cmd('ip netns del %s' % netns)


def clean_indirect(indirect_net_devices):
    veth_int = indirect_net_devices.veth_pair.internal
    bridge = indirect_net_devices.bridge
    tap = indirect_net_devices.tap

    run_cmd('ip link set dev %s down' % veth_int.name)
    run_cmd('ip link set dev %s down' % tap.name)
    run_cmd('ip link set dev %s down' % bridge.name)
    run_cmd('ip link delete dev %s' % veth_int.name)
    run_cmd('ip link delete dev %s' % tap.name)
    run_cmd('ip link delete dev %s' % bridge.name)


def clean_direct(veth_pair):
    veth_int = veth_pair.internal
    run_cmd('ip link set dev %s down' % veth_int.name)
    run_cmd('ip link delete dev %s' % veth_int.name)


def clean_network(direct_net_devices, indirect_net_devices):
    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)
    for i in range(n_netns):
        netns = get_netns(i, n_netns)
        clean_direct(direct_net_devices[i])
        clean_indirect(indirect_net_devices[i])
        clean_netns(netns)


def clean_firewall(direct_net_devices, indirect_net_devices):
    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)

    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('1')

    for i in range(n_netns):
        setup_firewall_net_device(direct_net_devices[i].internal, trust=False)
        setup_firewall_net_device(indirect_net_devices[i].veth_pair.internal, trust=False)


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


def clean(direct_net_devices, indirect_net_devices):
    if os.geteuid() != 0:
        raise Exception('Run as root for cleaning up')

    assert len(direct_net_devices) == len(indirect_net_devices)
    n_netns = len(direct_net_devices)

    print('Cleaning up %d network namespaces...' % n_netns)
    clean_firewall(direct_net_devices, indirect_net_devices)
    clean_dns(n_netns)
    clean_network(direct_net_devices, indirect_net_devices)


def spawn_netns(netns):
    subprocess.run('gnome-terminal -- bash -c "sudo ./activate_netns.sh %s"' % netns, shell=True)


def spawn(n_netns):
    if os.geteuid() == 0:
        raise Exception('Do not run as root for spawning')
    print('Spawning shells for %d network namespaces...' % n_netns)
    spawn_netns(get_global_netns())
    for i in range(n_netns):
        spawn_netns(get_netns(i, n_netns))


def get_5g_net_devices(i, n_netns):
    veth_pair = VethPair(
        external=NetDevice('nr%deth' % i, '10.0.0.%d' % (n_netns + 1 + (i+1)), 24),
        internal=NetDevice('nr%dveth' % i)
    )
    bridge = NetDevice('nr%dbridge' % i)
    tap = NetDevice('nr%dtap' % i)
    net_devices = IndirectNetDevices(veth_pair=veth_pair, bridge=bridge, tap=tap)

    # TODO: MEC server
    return net_devices


def get_wifi_net_devices(i):
    veth_pair = VethPair(
        external=NetDevice('w%deth' % i, '10.0.0.%d' % (i + 1), 24),
        internal=NetDevice('w%dveth' % i)
    )
    bridge = NetDevice('w%dbridge' % i)
    tap = NetDevice('w%dtap' % i)
    net_devices = IndirectNetDevices(veth_pair=veth_pair, bridge=bridge, tap=tap)
    return net_devices


def get_direct_net_devices(i):
    veth_pair = VethPair(
        external=NetDevice('d%deth' % i, '10.1.0.%d' % (i * 4 + 1), 30),
        internal=NetDevice('d%dveth' % i, '10.1.0.%d' % (i * 4 + 2), 30)
    )
    return veth_pair


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('network', type=str, help='Network to setup for', choices=['5g', 'wifi'])
    parser.add_argument('command', type=str, help='Command to execute', choices=['setup', 'clean', 'spawn'])
    parser.add_argument('-r', '--robots', type=int, default=1, help='Number of robots')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    if args.robots < 0 or args.robots > 20:
        raise ValueError('Invalid number of network namespaces')
    n_netns = args.robots + 1

    if args.network == '5g':
        indirect_net_devices = [get_5g_net_devices(i, n_netns) for i in range(n_netns)]
    elif args.network == 'wifi':
        indirect_net_devices = [get_wifi_net_devices(i) for i in range(n_netns)]
    else:
        raise ValueError
    direct_net_devices = [get_direct_net_devices(i) for i in range(n_netns)]

    if args.command == 'setup':
        setup(direct_net_devices, indirect_net_devices)
    elif args.command == 'clean':
        clean(direct_net_devices, indirect_net_devices)
    elif args.command == 'spawn':
        spawn(n_netns)
    else:
        raise ValueError

    print('Done')


if __name__ == '__main__':
    main()
