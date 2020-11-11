#!/usr/bin/env python3

"""
Sets up Linux network namespaces for communication over the network simulated by ns-3.

Adapted from: https://github.com/nps-ros2/ns3_gazebo/blob/master/scripts/nns_setup.py
Reference: https://www.nsnam.org/wiki/HOWTO_Use_Linux_Containers_to_set_up_virtual_networks
"""

from __future__ import print_function
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import subprocess
import sys


def ip_from_index(i):
    """
    For the connection over WiFi, we need a veth pair and a tap (for linking to ns-3). That means:
    - Required IP addresses: 5 (3 for interfaces + 2 reserved for network and broadcast)
    - Prefix length: /29 (8 IP addresses, enough)

    For simplicity, we use only 10.0.0.0/24 for connections over WiFi, that means we can have max 32 network namespaces.

    For the direct connection to the special namespace, we need a veth pair for each namespace, including the special
    one. That means:
    - Required IP addresses: 68 (66 for interfaces + 2 reserved for network and broadcast)
    - Prefix length: /25 (128 IP addresses, enough)
    
    For simplicity, we use only 10.0.1.0/24 for direct connections.
    
    :param i: integer, index of network namespace
    :return: tuple (wifi_veth, wifi_vethb, wifi_tap, direct_veth, direct_vethb)
    """
    if i < 0 or i >= 32:
        raise ValueError('Bad network namespace index')

    wifi_veth = '10.0.0.%d/29' % (i * 8 + 1)
    wifi_vethb = '10.0.0.%d/29' % (i * 8 + 2)
    wifi_tap = '10.0.0.%d/29' % (i * 8 + 3)

    direct_veth = '10.0.1.%d/25' % (i * 2 + 1)
    direct_vethb = '10.0.1.%d/25' % (i * 2 + 2)

    return wifi_veth, wifi_vethb, wifi_tap, direct_veth, direct_vethb


def run_cmd(cmd):
    print('Command: %s' % cmd)
    ret = subprocess.call(cmd.split())
    if ret != 0:
        print('Error. Return value: %d' % ret)


def setup_nns(i):
    run_cmd('ip netns add nns%d' % i)
    run_cmd('ip netns exec nns%d ip link set dev lo up' % i)    # helps Ctrl-C activate on first rather than second


def setup_wifi(i):
    wifi_veth, wifi_vethb, wifi_tap, _, _ = ip_from_index(i)

    # setup veth pair
    run_cmd('ip link add wifi_veth%d type veth peer name wifi_vethb%d' % (i, i))
    run_cmd('ip address add %s dev wifi_vethb%d' % (wifi_vethb, i))
    run_cmd('ip link set wifi_vethb%d up' % i)
    run_cmd('ip link set wifi_veth%d netns nns%d' % (i, i))
    run_cmd('ip netns exec nns%d ip addr add %s dev wifi_veth%d' % (i, wifi_veth, i))
    run_cmd('ip netns exec nns%d ip link set wifi_veth%d up' % (i, i))

    # setup tap device (link to ns-3)
    run_cmd('ip tuntap add wifi_tap%d mode tap' % i)
    run_cmd('ip addr flush dev wifi_tap%d' % i)  # clear IP
    run_cmd('ip address add %s dev wifi_tap%d' % (wifi_tap, i))
    run_cmd('ip link set wifi_tap%d up' % i)

    # setup bridge
    run_cmd('ip link add name wifi_br%d type bridge' % i)
    run_cmd('ip link set wifi_br%d up' % i)
    run_cmd('ip link set wifi_vethb%d master wifi_br%d' % (i, i))
    run_cmd('ip link set wifi_tap%d master wifi_br%d' % (i, i))


def setup_direct(i):
    _, _, _, direct_veth, direct_vethb = ip_from_index(i)

    # setup veth pair
    run_cmd('ip link add direct_veth%d type veth peer name direct_vethb%d' % (i, i))
    run_cmd('ip address add %s dev direct_vethb%d' % (direct_vethb, i))
    run_cmd('ip link set direct_vethb%d up' % i)
    run_cmd('ip link set direct_veth%d netns nns%d' % (i, i))
    run_cmd('ip netns exec nns%d ip addr add %s dev direct_veth%d' % (i, direct_veth, i))
    run_cmd('ip netns exec nns%d ip link set direct_veth%d up' % (i, i))


def clean_wifi(i):
    run_cmd('ip link set wifi_vethb%d down' % i)
    run_cmd('ip link set wifi_tap%d down' % i)
    run_cmd('ip link set wifi_br%d down' % i)
    run_cmd('ip link delete wifi_vethb%d' % i)
    run_cmd('ip link delete wifi_tap%d' % i)
    run_cmd('ip link delete wifi_br%d type bridge' % i)


def clean_direct(i):
    run_cmd('ip link set direct_vethb%d down' % i)
    run_cmd('ip link delete direct_vethb%d' % i)


def clean_nns(i):
    run_cmd('ip netns del nns%d' % i)


def get_cmd_args():
    parser = ArgumentParser(description='Setup network namespaces for communication over WiFi simulated by ns-3.',
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('command', type=str, help='The command to execute.', choices=['setup', 'cleanup'])
    parser.add_argument('-c', '--count', type=int, default=2,
                        help='The number of network namespaces communicating over WiFi.')
    parser.add_argument('-d', '--include_direct', action='store_true',
                        help='Include direct connections to the global namespace.')
    return parser.parse_args()


def main():
    args = get_cmd_args()
    print('Providing %s for %d network namespaces...' % (args.command, args.count))

    if args.command == 'setup':
        for i in range(args.count):
            setup_nns(i)
            # setup_wifi(i)
            if args.include_direct:
                setup_direct(i)
    elif args.command == 'cleanup':
        for i in range(args.count):
            clean_wifi(i)
            if args.include_direct:
                clean_direct(i)
            clean_nns(i)
    else:   # should never happen, ArgumentParser already checks for valid commands
        raise ValueError('Invalid command: %s' % args.command)

    print('Done')


if __name__ == '__main__':
    main()
