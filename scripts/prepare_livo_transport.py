#!/usr/bin/env python3
"""Generate a run-local Fast DDS profile for this WSL mirrored interface."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--initial-peer-range', type=int, default=4,
                        help='Expected participant discovery range; 4 preserves the small replay default')
    args = parser.parse_args()
    if not 4 <= args.initial_peer_range <= 100:
        parser.error('--initial-peer-range must be between 4 and 100')
    route = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'get', '1.1.1.1'], text=True))[0]
    address, interface = str(ipaddress.IPv4Address(route['prefsrc'])), route['dev']
    if ipaddress.ip_address(address).is_loopback:
        raise ValueError('a verified active mirrored interface is required')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = '''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
 <profiles>
  <transport_descriptors><transport_descriptor>
   <transport_id>impact_replay_udp</transport_id><type>UDPv4</type>
   <maxInitialPeersRange>INITIAL_PEER_RANGE</maxInitialPeersRange>
   <interfaceWhiteList><address>LOCAL_ADDRESS</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="impact_replay_participant" is_default_profile="true"><rtps>
   <useBuiltinTransports>false</useBuiltinTransports>
   <userTransports><transport_id>impact_replay_udp</transport_id></userTransports>
   <builtin>
    <discovery_config><discoveryProtocol>SIMPLE</discoveryProtocol></discovery_config>
    <metatrafficUnicastLocatorList><locator><udpv4><address>LOCAL_ADDRESS</address></udpv4></locator></metatrafficUnicastLocatorList>
    <initialPeersList><locator><udpv4><address>LOCAL_ADDRESS</address></udpv4></locator></initialPeersList>
   </builtin>
  </rtps></participant>
 </profiles>
</dds>
'''.replace('LOCAL_ADDRESS', address).replace('INITIAL_PEER_RANGE', str(args.initial_peer_range))
    (args.output_dir / 'fastdds-replay.xml').write_text(profile)
    result = {'status': 'READY', 'interface': interface, 'ipv4': address,
              'ROS_DOMAIN_ID': os.environ['ROS_DOMAIN_ID'], 'ROS_LOCALHOST_ONLY': '0',
              'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
              'maxInitialPeersRange': args.initial_peer_range,
              'scope': 'Run-local transport; system/WSL/firewall configuration is unchanged'}
    (args.output_dir / 'transport-environment.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
