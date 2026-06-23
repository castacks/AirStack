#!/usr/bin/env python3
"""
Receives and decodes OptiTrack NatNet UDP packets from a Motive PC over ethernet.

MoCap setup (AirLab, as of 2026-06-09):
    Motive PC IP:  192.168.0.77
    Data port:     1511
    Command port:  1510
    Stream type:   multicast (multicast addr: 239.255.42.99)
    Laptop iface:  en9 (AX88179B USB ethernet adapter, static IP: 192.168.0.100)

Requirements:
    pip install scapy

Usage:
    sudo ~/miniconda3/bin/python3 natnet_receive.py               # receive + decode frames
    sudo ~/miniconda3/bin/python3 natnet_receive.py --discover    # show all NatNet UDP traffic (sanity check)
    sudo ~/miniconda3/bin/python3 natnet_receive.py --debug       # also print raw hex bytes (for diagnosing packet format)
    sudo ~/miniconda3/bin/python3 natnet_receive.py --iface en9 --port 1511
    sudo ~/miniconda3/bin/python3 natnet_receive.py --relay localhost:1511  # relay packets to another host/port

To test relay locally (two terminals):
    Terminal 1: nc -u -l 1512
    Terminal 2: sudo ~/miniconda3/bin/python3 natnet_receive.py --relay localhost:1512

To relay to another machine on the same network:
    On their machine: nc -u -l 1512
    Find their IP:    ifconfig | grep "inet "
    On your machine:  sudo ~/miniconda3/bin/python3 natnet_receive.py --relay THEIR_IP:1512

Goal pipeline:
    MoCap cameras â Motive PC â UDP over ethernet â laptop (natnet_receive.py) â [BLOCKED] â Osmo cloud â NatNet ROS2 node

Known blockers for MoCap â Osmo cloud (as of 2026-06-09):
    BLOCKER 1 â Osmo CLI UDP port-forward is broken.
        Bug location: /usr/local/osmo/osmo (compiled binary, not editable)
        Error: TypeError('Passing coroutines is forbidden, use tasks explicitly.')
              in port_forward.py line 303 run_udp()
        TCP port-forward works fine. Report this bug to whoever maintains the Osmo CLI in Airlab.

    BLOCKER 2 â NatNet ROS2 package not installed in the cloud container image.
        The package natnet_ros2 is missing from:
            airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev
        Needs to be added to the Docker image build and pushed as a new version.
        (airstack setup --natnet needs to be run during image build)

    Once both are fixed, run these in separate terminals:

        Terminal 1 â UDP tunnel to cloud:
            osmo workflow port-forward <wf-id> workspace \
                --port 1511 \
                --udp \
                --connect-timeout 86400

        Terminal 2 â relay MoCap data through tunnel:
            sudo ~/miniconda3/bin/python3 ~/Downloads/natnet_receive.py \
                --relay localhost:1511
"""

import argparse
import socket
import struct
from datetime import datetime

try:
    from scapy.all import sniff, UDP, IP
except ImportError:
    print("error: scapy not installed. Run: pip install scapy")
    raise SystemExit(1)

MSG_NAMES = {
    0: "NAT_CONNECT",
    1: "NAT_SERVERINFO",
    2: "NAT_REQUEST",
    3: "NAT_RESPONSE",
    4: "NAT_REQUEST_MODELDEF",
    5: "NAT_MODELDEF",
    6: "NAT_REQUEST_FRAMEOFDATA",
    7: "NAT_FRAMEOFDATA",
    8: "NAT_MESSAGESTRING",
    9: "NAT_DISCONNECT",
    10: "NAT_KEEPALIVE",
    14: "NAT_DISCOVERY",
}

def decode_packet(data):
    if len(data) < 4:
        return "short packet", {}
        
    message_id, payload_size = struct.unpack_from("<HH", data, 0)
    name = MSG_NAMES.get(message_id, f"unknown({message_id})")
    details = {"message": name, "payload_bytes": payload_size}
    
    if message_id == 7:
        offset = 4
        
        # 1. Frame Number (4 bytes)
        if len(data) >= offset + 4:
            details["frame"] = struct.unpack_from("<i", data, offset)[0]
            offset += 4
            
        # 2. Number of Named Marker Sets (4 bytes)
        if len(data) >= offset + 4:
            n_marker_sets = struct.unpack_from("<i", data, offset)[0]
            details["marker_sets"] = n_marker_sets
            offset += 4
            
            # Loop and advance past Named Marker Sets if any exist
            for _ in range(n_marker_sets):
                # Marker set name is a null-terminated string
                end_idx = data.find(b'\x00', offset)
                if end_idx == -1:
                    break
                offset = end_idx + 1 # Skip past the string name
                
                # Each set has a 4-byte marker count
                n_markers = struct.unpack_from("<i", data, offset)[0]
                offset += 4
                # Each marker is 3 floats (x, y, z) = 12 bytes
                offset += n_markers * 12

        # 3. NatNet v3.x/4.x Unlabeled/Other Markers Section (4 bytes)
        # This is the hidden block throwing off your byte alignments!
        if len(data) >= offset + 4:
            n_unlabeled_markers = struct.unpack_from("<i", data, offset)[0]
            details["unlabeled_markers"] = n_unlabeled_markers
            offset += 4
            # If there are unlabeled markers, they are 12 bytes each
            # (But wait! In some configurations this might just be a flat count 
            #  followed by coordinates. Let's just monitor the count for now)

        # 4. Rigid Bodies Count (4 bytes)
        if len(data) >= offset + 4:
            details["rigid_bodies"] = struct.unpack_from("<i", data, offset)[0]
            offset += 4

    return name, details

def discover_handler(pkt):
    if not (UDP in pkt and IP in pkt):
        return
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}]  {pkt[IP].src}:{pkt[UDP].sport} -> {pkt[IP].dst}:{pkt[UDP].dport}  {len(bytes(pkt[UDP].payload))}B", flush=True)


def make_handler(port, motive_ip, debug=False, relay=None):
    relay_sock = None
    if relay:
        relay_host, relay_port = relay.rsplit(":", 1)
        relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        relay_addr = (relay_host, int(relay_port))
        print(f"Relaying packets to {relay_host}:{relay_port}", flush=True)

    def handle(pkt):
        if not (UDP in pkt and IP in pkt):
            return
        if pkt[UDP].dport != port:
            return
        if motive_ip and pkt[IP].src != motive_ip:
            return
        data = bytes(pkt[UDP].payload)
        if relay_sock:
            relay_sock.sendto(data, relay_addr)
        src = f"{pkt[IP].src}:{pkt[UDP].sport}"
        name, details = decode_packet(data)
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        parts = [f"[{ts}]", f"from {src}", f"{len(data)}B", name]
        if "frame" in details:
            parts.append(f"frame={details['frame']}")
        if "marker_sets" in details:
            parts.append(f"marker_sets={details['marker_sets']}")
        if "unlabeled_markers" in details:
            parts.append(f"unlabeled={details['unlabeled_markers']}")
        if "rigid_bodies" in details:
            parts.append(f"rigid_bodies={details['rigid_bodies']}")

        print("  ".join(parts), flush=True)

        if debug:
            print(f"  hex: {data[:32].hex(' ')}", flush=True)
    return handle


def main():
    parser = argparse.ArgumentParser(description="NatNet receiver via libpcap")
    parser.add_argument("--iface", default="en9", help="Network interface (default: en9)")
    parser.add_argument("--port", type=int, default=1511, help="NatNet data port (default: 1511)")
    parser.add_argument("--motive-ip", default="", help="Filter to this source IP only (default: any)")
    parser.add_argument("--discover", action="store_true", help="Show all NatNet UDP traffic (sanity check)")
    parser.add_argument("--debug", action="store_true", help="Print raw hex of first 32 bytes of each packet")
    parser.add_argument("--relay", default="", help="Relay raw packets to HOST:PORT via UDP (e.g. localhost:1511)")
    args = parser.parse_args()

    print(f"Interface: {args.iface}")
    print("Press Ctrl-C to stop.\n")

    if args.discover:
        bpf = "udp and (port 1511 or port 1510 or port 1001 or host 239.255.42.99)"
        print(f"Discover mode â showing all NatNet UDP traffic")
        print(f"Filter: {bpf}\n")
        sniff(iface=args.iface, filter=bpf, prn=discover_handler, store=0)
    else:
        bpf = f"udp dst port {args.port}"
        print(f"Receive mode â decoding NatNet frames on port {args.port}")
        print(f"Motive IP filter: {args.motive_ip or 'any'}\n")
        sniff(iface=args.iface, filter=bpf, prn=make_handler(args.port, args.motive_ip, args.debug, args.relay or None), store=0)


if __name__ == "__main__":
    main()
