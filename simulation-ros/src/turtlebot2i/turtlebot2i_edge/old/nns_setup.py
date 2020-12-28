"""
Sets up Linux network namespaces for communication over the WiFi network simulated by ns-3.

For the WiFi network, we need a veth pair and a tap for each network namespace, i.e. 3 IP addresses. For simplicity,
we use 10.0.0.0/24 for the WiFi network. So, the WiFi network gives a constraint on the maximum number of network
namespaces: floor((256-2)/3) = 84.

For the direct connection to the global namespace, we need a veth pair, i.e. 2 IP addresses. For simplicity, we use
10.0.1.0/24 and each direct connection takes 2+2 = 4 IP addresses. So, the direct connections give a constraint on the
maximum number of network namespaces: floor(256 / 4) = 64.

For satisfying both constraints, we allow 64 network namespaces.

Adapted from: https://github.com/nps-ros2/ns3_gazebo/blob/master/scripts/nns_setup.py
Reference: https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks
"""

from __future__ import print_function
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import subprocess
import os


class NetworkDevice:
    def __init__(self, name, ip=None, prefix_length=None):
        self.name = name
        if ip is not None and prefix_length is not None:
            self.ip_device = ip + '/' + str(prefix_length)      # with prefix length
            self.ip_route = ip                                  # without prefix length


def get_network_namespace(i):
    return 'nns%d' % i


def get_wifi_net_devices(i):
    if i < 0 or i >= 64:
        raise ValueError('Bad network namespace index')

    veth_pair = (
        NetworkDevice('wifi_veth%d' % i, '10.0.0.%d' % (i * 3 + 1), 24),
        NetworkDevice('wifi_vethb%d' % i, '10.0.0.%d' % (i * 3 + 2), 24)
    )
    bridge = NetworkDevice('wifi_bridge%d' % i)     # no IP address
    tap = NetworkDevice('wifi_tap%d' % i, '10.0.0.%d' % (i * 3 + 3), 24)

    return veth_pair, bridge, tap


def get_direct_net_devices(i):
    veth_pair = (
        NetworkDevice('direct_veth%d' % i, '10.0.1.%d' % (i * 4 + 1), 30),
        NetworkDevice('direct_vethb%d' % i, '10.0.1.%d' % (i * 4 + 2), 30)
    )
    return veth_pair


def run_cmd(cmd):
    print('Command: %s' % cmd)
    with open(os.devnull, 'w') as dev_null:
        ret = subprocess.call(cmd.split(), stdout=dev_null, stderr=subprocess.STDOUT)
    if ret != 0:
        print('Error. Return value: %d' % ret)


def setup_nns(i):
    nns = get_network_namespace(i)
    run_cmd('ip netns add %s' % nns)
    run_cmd('ip netns exec %s ip link set dev lo up' % nns)     # helps Ctrl-C activate on first rather than second


def setup_wifi(i):
    nns = get_network_namespace(i)
    veth_pair, bridge, tap = get_wifi_net_devices(i)
    veth_global, veth_nns = veth_pair

    # setup veth pair
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_nns.name))
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip link set %s netns %s' % (veth_nns.name, nns))
    run_cmd('ip netns exec %s ip addr add %s dev %s' % (nns, veth_nns.ip_device, veth_nns.name))
    run_cmd('ip netns exec %s ip link set %s up' % (nns, veth_nns.name))

    # setup tap device (link to ns-3)
    run_cmd('ip tuntap add %s mode tap' % tap.name)
    run_cmd('ip addr flush dev %s' % tap.name)
    run_cmd('ip address add %s dev %s' % (tap.ip_device, tap.name))
    run_cmd('ip link set %s up' % tap.name)

    # setup bridge
    run_cmd('ip link add name %s type bridge' % bridge.name)
    run_cmd('ip link set %s up' % bridge.name)
    run_cmd('ip link set %s master %s' % (veth_global.name, bridge.name))
    run_cmd('ip link set %s master %s' % (tap.name, bridge.name))

    # set bridge to work at level 2, so not to use iptables (no TCP packets dropped)
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('0')


def setup_direct(i):
    nns = get_network_namespace(i)
    veth_global, veth_nns = get_direct_net_devices(i)

    # setup veth pair
    run_cmd('ip link add %s type veth peer name %s' % (veth_global.name, veth_nns.name))
    run_cmd('ip address add %s dev %s' % (veth_global.ip_device, veth_global.name))
    run_cmd('ip link set %s up' % veth_global.name)
    run_cmd('ip link set %s netns %s' % (veth_nns.name, nns))
    run_cmd('ip netns exec %s ip addr add %s dev %s' % (nns, veth_nns.ip_device, veth_nns.name))
    run_cmd('ip netns exec %s ip link set %s up' % (nns, veth_nns.name))

    # set default route to global namespace
    run_cmd("ip netns exec %s ip route add default via %s" % (nns, veth_global.ip_route))

    # add net device in global namespace to trusted zone (no TCP packets dropped)
    run_cmd('firewall-cmd --zone=trusted --permanent --add-interface=%s' % veth_global.name)


def clean_wifi(i):
    veth_pair, bridge, tap = get_wifi_net_devices(i)
    veth_global, _ = veth_pair
    run_cmd('ip link set %s down' % veth_global.name)
    run_cmd('ip link set %s down' % tap.name)
    run_cmd('ip link set %s down' % bridge.name)
    run_cmd('ip link delete %s' % veth_global.name)
    run_cmd('ip link delete %s' % tap.name)
    run_cmd('ip link delete %s' % bridge.name)
    with open('/proc/sys/net/bridge/bridge-nf-call-iptables', mode='w') as f:
        f.write('1')


def clean_direct(i):
    veth_global, _ = get_direct_net_devices(i)
    run_cmd('ip link set %s down' % veth_global.name)
    run_cmd('ip link delete %s' % veth_global.name)
    run_cmd('firewall-cmd --zone=trusted --permanent --remove-interface=%s' % veth_global.name)


def clean_nns(i):
    nns = get_network_namespace(i)
    run_cmd('ip netns del %s' % nns)


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', type=str, help='Command to execute', choices=['setup', 'clean'])
    parser.add_argument('-c', '--count', type=int, default=2, help='Number of network namespaces')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    print('Providing %s for %d network namespaces...' % (args.command, args.count))

    if args.command == 'setup':
        for i in range(args.count):
            setup_nns(i)
            setup_wifi(i)
            setup_direct(i)
    elif args.command == 'clean':
        for i in range(args.count):
            clean_wifi(i)
            clean_direct(i)
            clean_nns(i)
    else:   # should never happen, ArgumentParser already checks for valid commands
        raise ValueError('Invalid command: %s' % args.command)

    print('Done')


if __name__ == '__main__':
    main()
