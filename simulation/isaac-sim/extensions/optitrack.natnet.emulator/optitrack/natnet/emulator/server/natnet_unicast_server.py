import ctypes
import time

from . import natnet_server_types as ServerTypes
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
        # Loop to update mocap data and send packets at regular intervals. 
        # When auto_stream is False the frames are pumped externally (Isaac physics step),
        # so this thread only idles — but stays alive for clean shutdown.
        while not self.shutdown_event.is_set():
            time.sleep(1 / self.publish_rate)
            if not self.auto_stream:
                continue
            self.flush_mocap_data()

    def flush_mocap_data(self):
        """Send the latest (or last) mocap frame to every connected client, once."""
        with self.clients_lock:
            clients = list(self.connected_clients)
        if not clients:
            return

        data_messages = self._get_latest_mocap_packet()

        if data_messages is None: # If the server stops producing frames, use the last known frame.
            data_messages = self._get_last_known_mocap_frame()
        if data_messages is None:
            return

        for client in clients:
            try:
                self._send_data_packet(client, data_messages)
            except ValueError as e:
                print(str(e))
                continue

    def _command_listener_loop(self):
        # Listens on UDP command socket for incoming command requests from clients.
        # Handles incoming client handshakes and teardown.

        print(f"[Command Listener] Command listener thread started. Listening for incoming client command requests on UDP address:port {self.local_interface}:{self.command_port}...")

        while not self.shutdown_event.is_set():
            try:
                data, addr = self.command_socket.recvfrom(1024) # Buffer size of 1024 bytes should be sufficient for command requests
                if not data:
                    continue
                self._handle_command_request(data, addr)
            except Exception as e:
                if self.shutdown_event.is_set():
                    break
                print(f"[Command Listener] Error receiving command request: {e}")
                time.sleep(0.1) # Sleep briefly to avoid tight loop on errors
    
    def _handle_command_request(self, request_data: bytes, client_address: tuple):
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
                    self._build_connect_response_payload(),
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

        if header.iMessage == int(ServerTypes.MessageId.NAT_REQUEST_MODELDEF):
            try:
                self._send_packet_to_client(
                    client,
                    ServerTypes.MessageId.NAT_MODELDEF,
                    self._get_model_def_payload(),
                )
            except ValueError as e:
                print(
                    f"[Command Handler] Error sending MODELDEF to client "
                    f"{client_address}: {e}"
                )
            return

        if header.iMessage == int(ServerTypes.MessageId.NAT_KEEPALIVE):
            # Receiving a keepalive refreshes the client's liveness; nothing to send back.
            return

        if header.iMessage == int(ServerTypes.MessageId.NAT_ECHOREQUEST):
            echo_payload = request_data[header_size : header_size + header.nDataBytes]
            # libNatNet expects clientRequestTimestamp + hostReceivedTimestamp (8 + 8 bytes).
            host_ts = int(time.time() * 1_000_000_000).to_bytes(8, "little", signed=False)
            response_payload = echo_payload[:8].ljust(8, b"\x00") + host_ts
            try:
                self._send_packet_to_client(
                    client,
                    ServerTypes.MessageId.NAT_ECHORESPONSE,
                    response_payload,
                )
            except ValueError as e:
                print(
                    f"[Command Handler] Error sending ECHORESPONSE to client "
                    f"{client_address}: {e}"
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
