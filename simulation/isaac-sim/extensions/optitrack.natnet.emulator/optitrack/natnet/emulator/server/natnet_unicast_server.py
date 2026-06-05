from . import natnet_data_types as DataTypes
from . import natnet_server_types as ServerTypes
from enum import Enum
import socket
import threading
import queue
import time
import signal
import ctypes
from .natnet_server import TransmissionType, Client, NatNetServer


class NatNetUnicastServer(NatNetServer):
    def __init__(self, 
            local_interface="172.31.0.200",
            transmission_type: TransmissionType = TransmissionType.UNICAST, 
            multicast_address=None,
            command_port=1510,
            data_port=1511
        ):

        if not transmission_type == TransmissionType.UNICAST:
            raise ValueError("Transmission type 'MULTICAST' is not supported in NatNetUnicastServer. Please use NatNetMulticastServer instead.")

        super().__init__(local_interface, transmission_type, multicast_address, command_port, data_port)

    def _data_update_loop(self):
        # Loop to update mocap data and send packets at regular intervals
        while not self.shutdown_event.is_set():
            time.sleep(1 / self.publish_rate)

            with self.clients_lock:
                clients = list(self.connected_clients)
            if not clients:
                continue

            data_messages = self._get_latest_mocap_packet()
            if not data_messages:
                continue

            for client in clients:
                try:
                    self._send_data_packet(client, data_messages)
                except ValueError as e:
                    print(str(e))
                    continue

    def _command_listener_loop(self): # Stub: Different betweeen multicast and unicast server implementations, as they will need to handle client connections differently (e.g. unicast will need to manage a list of connected clients and send packets directly to their IPs, while multicast will just send to the multicast group address)
        # Listens on UDP command socket for incoming command requests from clients.
        # Handles incoming client handshakes and teardown.

        print(f"[Command Listener] Command listener thread started. Listening for incoming client command requests on UDP address:port {self.local_interface}:{self.command_port}...")

        while not self.shutdown_event.is_set():
            try:
                data, addr = self.command_socket.recvfrom(1024) # Buffer size of 1024 bytes should be sufficient for command requests
                if not data:
                    continue
                print(f"[Command Listener] Received command request from client {addr}.")
                self._handle_command_request(data, addr)
            except Exception as e:
                if self.shutdown_event.is_set():
                    break
                print(f"[Command Listener] Error receiving command request: {e}")
                time.sleep(0.1) # Sleep briefly to avoid tight loop on errors
    
    def _handle_command_request(self, request_data: bytes, client_address: tuple): # Stub: Different betweeen multicast and unicast server implementations, as they will need to handle client connections differently (e.g. unicast will need to manage a list of connected clients and send packets directly to their IPs, while multicast will just send to the multicast group address)
        """
        Processes standard binary headers and registers unicast endpoints.
        """
        header_size = ctypes.sizeof(ServerTypes.sPacketHeader)
        if len(request_data) < header_size:
            return

        # Parse the header via ctypes
        header = ServerTypes.sPacketHeader.from_buffer_copy(request_data[:header_size])

        # Handle Connection Handshake
        if header.iMessage == int(ServerTypes.MessageId.NAT_CONNECT):
            client_requested_version = self.natnet_version  # Fallback to server's version. Version handshaking not supported in this extension.

            client_ip, client_port = client_address

            # Create and store a new client object
            new_client = Client(client_ip, client_port, version=client_requested_version)
            try:
                with self.clients_lock:
                    self.connected_clients.discard(new_client) # Remove any existing client with the same IP and port
                    self.connected_clients.add(new_client) # Add the new client to the connected clients list
                    print(f"[Command Handler] Added client {new_client.ip}:{new_client.port} to connected clients list.")
            except Exception as e:
                print(f"[Command Handler] Error adding client {new_client.ip}:{new_client.port} to connected clients list: {e}")
                return

            try:
                self._send_packet_to_client(
                    new_client,
                    ServerTypes.MessageId.NAT_SERVERINFO,
                    self.server_description.pack(),
                )
            except ValueError as e:
                raise ValueError(
                    f"[Command Handler] Error sending server description to client {client_address}: {e}"
                ) from e
            print(
                f"[Command Handler] Sent server description to client address "
                f"through its port {client_address}."
            )
            return

        # Non-handshake commands require a prior NAT_CONNECT from this endpoint.
        client_ip, client_port = client_address
        client = self._find_client(client_ip, client_port)
        if client is None:
            print(
                f"[Command Handler] Ignoring message {header.iMessage} from "
                f"unregistered client {client_address}."
            )
            return

        print(
            f"[Command Handler] Unhandled message id {header.iMessage} from "
            f"registered client {client_address}."
        )

    def _find_client(self, ip: str, port: int) -> Client | None:
        target = Client(ip, port)
        with self.clients_lock:
            for client in self.connected_clients:
                if client == target:
                    return client
        return None
