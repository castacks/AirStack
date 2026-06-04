from . import natnet_data_types as DataMessages
from . import natnet_server_types as ServerMessages
from enum import Enum
import socket
import threading
import queue
import signal
import ctypes
import typing


class TransmissionType(str, Enum):
    UNICAST = "unicast"
    MULTICAST = "multicast"

class Client:
    def __init__(self, ip: str, port: int, version: typing.Tuple[int, int, int, int] = (4, 4, 0, 0)):
        self.ip = ip
        self.port = port
        self.version = version
        self.subscribed_assets = set()
        self.socket_lock = threading.Lock()

    def __hash__(self):
        # We uniquely identify a client session by their IP and their unique command port.
        # Why? A single machine could technically run multiple separate NatNet clients
        # simultaneously, and they would share an IP but have unique command ports.
        return hash((self.ip, self.port))

    def __eq__(self, other):
        return (isinstance(other, Client) and 
                self.ip == other.ip and 
                self.port == other.port)
    

class NatNetServer:
    def __init__(self, 
            local_interface : str = "172.31.0.200",
            transmission_type: TransmissionType = TransmissionType.MULTICAST, 
            multicast_address : str = "239.255.42.99",
            command_port: int = 1510,
            data_port : int = 1511,
            motive_app_version : typing.Tuple[int, int, int, int]=(3, 1, 0, 0),
            natnet_version : typing.Tuple[int, int, int, int]=(4, 4, 0, 0),
            high_res_clock_freq : int = 1_000_000_000, 
            publish_rate : int = 100 # Hz (default 100Hz)
        ):

        self.local_interface = local_interface
        self.transmission_type = transmission_type
        self.multicast_address = multicast_address
        self.command_port = command_port
        self.data_port = data_port
        self.motive_app_version = motive_app_version
        self.natnet_version = natnet_version
        self.high_res_clock_freq = high_res_clock_freq
        self.publish_rate = publish_rate
        
        self._validate_init_params()

        self.server_description = self._build_server_description()
        # Initialize synchronously safe data structures for server state and mocap data

        # Thread-safe queue for Mocp frames
        self.mocap_data_queue = queue.Queue(maxsize=100)
        
        # Thread list and shutdown event
        self.threads = []
        self.shutdown_event = threading.Event()

        # Connected clients for unicast mode
        self.connected_clients : typing.Set[Client] = set()
        self.clients_lock : threading.Lock = threading.Lock()

        # Sockets
        self.command_socket : socket.socket | None = None
        self.data_socket : socket.socket | None = None

        self.running = False

        # Start up process with threads
        # # One thread to listen and manage command requests (TCP/UDP)
        # # One thread to send data packets (UDP)
        # # One thread to manage server state and data updates (mocap system or other source)

        # Overall design:
        # - On initialization, validate parameters and set up server description
        # - Start threads for command listening and data sending
        # - Perhaps separate into Unicast and Multicast server classes that inherit from a common base, or handle both in one class with conditional logic based on transmission type
        # - Multicast server will need to join multicast group and manage socket options accordingly, while unicast server will send directly to client IPs
        # - General helper functions will break packets down into appropriate sizes, serialize data messages, and manage client connections for unicast case
        # - General main loop will manage server lifecycle and clean shutdown, while threads will handle their respective tasks for command listening and data sending
        # - General main data management function will take in new mocap data (e.g. from mocap system or other source), update the latest data state, and trigger packet sending to clients at regular intervals (e.g. 100Hz)
        # - Unicast:
        # - On command request, respond with server description and data packets sent directly to requesting client's IP
        # - Have a synchronously safe list of connected clients to manage multiple unicast recipients
        # - General main data structure or queue to hold the latest mocap data that will be sent in packets to clients, with thread-safe access for updates and retrievals
        # - Have a function to continously send packets at regular intervals (e.g. 100Hz) with the latest mocap data to unicast clients.
        # - - Use the general helper function to take in new mocap data (e.g. from mocap system or other source). The server will break the message down into packets 
        # - Multicast:
        # - On command request, respond with server description but data packets will be sent to the multicast group address rather than directly to client IPs
        # - General main data structure or queue to hold the latest mocap data that will be sent in packets to clients, with thread-safe access for updates and retrievals
        # - Have a function to continously send packets at regular intervals (e.g. 100Hz) with the latest mocap data to the multicast group address.
        # - - Use the general helper function to take in new mocap data (e.g. from mocap system or other source). The server will break the message down into packets and send to the multicast group address.

    def _signal_handler(self, signum, frame):
        print(f"\n[NatNetServer] Received interrupt signal {signum}. Initiating shutdown...")
        self.shutdown()

    def enqueue_mocap_data(self, new_data: DataMessages.sFrameOfMocapData):
        # Thread-safe method to push new physics frames (called by Isaac-Sim extension)
        if self.mocap_data_queue.full():
            try:
                # Drop oldest frame if falling behind
                self.mocap_data_queue.get_nowait()
            except queue.Empty:
                pass
        self.mocap_data_queue.put(new_data)

    def start(self):
        # Bind sockets and launch worker threads automatically on init
        
        # Register signal handlers for graceful shutdown (Catches Ctrl+C and kill)
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)
        except ValueError:
            pass # Safe fallback if not called from the main thread
        
        # 1. Setup Command Socket (Receives connection/discovery requests)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.command_socket.bind(('', self.command_port))

        # 2. Setup Data Socket (Sends outward Mocap frames)
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Note: Data socket doesn't need to bind to the multicast explicitly if it's only sending.
        # It just routes via the local interface.
        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.local_interface))

        # 3. Launch Threads
        cmd_thread = threading.Thread(target=self._command_listener_loop, daemon=True)
        data_thread = threading.Thread(target=self._data_update_loop, daemon=True)

        self.threads.extend([cmd_thread, data_thread])
        
        for t in self.threads:
            t.start()
        
        self.running = True

    def shutdown(self):
        # Cleanly shutdown threads and close sockets
        self.running = False
        self.shutdown_event.set()
        
        if self.command_socket:
            self.command_socket.close()
            
        if self.data_socket:
            self.data_socket.close()

        for t in self.threads:
            if t.is_alive():
                t.join(timeout=1.0)

    def _validate_init_params(self):

        # Validate the local_interface is a valid IP address
        if not self.local_interface or not isinstance(self.local_interface, str) or self.local_interface.count('.') != 3:
            raise ValueError(f"Invalid local interface IP address: {self.local_interface}")

        # Validate between transmission types and address requirements
        if self.transmission_type not in TransmissionType:
            raise ValueError(f"Invalid transmission type: {self.transmission_type}. Must be 'unicast' or 'multicast'.")

        if self.transmission_type == TransmissionType.MULTICAST and not self.multicast_address:
            raise ValueError("Multicast address must be provided for multicast transmission type.")
        
        if self.transmission_type == TransmissionType.UNICAST and self.multicast_address:
            raise ValueError("Multicast address should not be provided for unicast transmission type.")

        if not (0 < self.command_port < 65536):
            raise ValueError(f"Invalid command port: {self.command_port}. Must be between 1 and 65535.")

        if not (0 < self.data_port < 65536):
            raise ValueError(f"Invalid data port: {self.data_port}. Must be between 1 and 65535.")
        
        if self.command_port == self.data_port:
            raise ValueError("Command port and data port must be different.")

        if self.motive_app_version and (not isinstance(self.motive_app_version, tuple) or len(self.motive_app_version) != 4):
            raise ValueError(f"Invalid Motive app version: {self.motive_app_version}. Must be a tuple of 4 integers (major, minor, build, revision).")

        if self.natnet_version and (not isinstance(self.natnet_version, tuple) or len(self.natnet_version) != 4):
            raise ValueError(f"Invalid NatNet version: {self.natnet_version}. Must be a tuple of 4 integers (major, minor, build, revision).")
        
        if self.motive_app_version and not self.motive_app_version[0] == 3:
            raise ValueError(f"Unsupported Motive app version: {self.motive_app_version}. Minimum supported version is 3.0.0.0. Recommended to use 3.1.0.0")

        if not self.natnet_version[0] == 4:
            raise ValueError(f"Unsupported NatNet version: {self.natnet_version}. Minimum supported version is 4.0.0.0. Recommended to use 4.4.0.0")

        if self.high_res_clock_freq <= 0:
            raise ValueError(f"Invalid high resolution clock frequency: {self.high_res_clock_freq}. Must be a positive integer representing the frequency in Hz.")

    def _get_latest_mocap_packet(self) -> DataMessages.sFrameOfMocapData | None:
        # Thread-safe method to retrieve the latest mocap data to be sent
        try:
            return self.mocap_data_queue.get_nowait()
        except queue.Empty:
            return None

    @staticmethod
    def _pad_fixed_string(value: bytes) -> bytes:
        """Null-pad a byte string to MAX_NAMELENGTH for fixed-size NatNet name fields."""
        truncated = value[: ServerMessages.MAX_NAMELENGTH - 1]
        return truncated + b"\x00" * (ServerMessages.MAX_NAMELENGTH - len(truncated))

    @staticmethod
    def _assign_version_bytes(field: ctypes.Array, version: typing.Tuple[int, int, int, int]) -> None:
        for index, component in enumerate(version):
            field[index] = component

    @staticmethod
    def _assign_ipv4_bytes(field: ctypes.Array, address: str | bytes) -> None:
        octets = socket.inet_aton(address) if isinstance(address, str) else address
        for index, octet in enumerate(octets):
            field[index] = octet

    def _build_server_description(self) -> ServerMessages.sServerDescription:
        # Helper to build the server description struct with current server info (e.g. on startup or in response to command request)
        description = ServerMessages.sServerDescription()
        description.HostPresent = True
        description.szHostComputerName = self._pad_fixed_string(
            socket.gethostname().encode("utf-8")
        )
        self._assign_ipv4_bytes(description.HostComputerAddress, self.local_interface)
        description.szHostApp = self._pad_fixed_string(b"Motive")
        self._assign_version_bytes(description.HostAppVersion, self.motive_app_version)
        self._assign_version_bytes(description.NatNetVersion, self.natnet_version)
        description.HighResClockFrequency = self.high_res_clock_freq
        description.bConnectionInfoValid = True
        description.ConnectionDataPort = self.data_port
        description.ConnectionMulticast = self.transmission_type == TransmissionType.MULTICAST

        if self.transmission_type == TransmissionType.MULTICAST:
            self._assign_ipv4_bytes(description.ConnectionMulticastAddress, self.multicast_address)
        else:
            self._assign_ipv4_bytes(description.ConnectionMulticastAddress, b"\x00\x00\x00\x00")

        return description

    def _send_packet_to_client(
        self,
        client: Client,
        message_id: ServerMessages.MessageId | int,
        payload: bytes,
    ) -> None:
        """Send a NatNet packet to a unicast client on the command socket (libNatNet 4.4)."""
        if not self.command_socket:
            raise ValueError("[NatNetServer] Command socket not initialized. Cannot send packet.")

        header = ServerMessages.sPacketHeader(
            iMessage=int(message_id),
            nDataBytes=len(payload),
        )
        packet = header.pack() + payload
        try:
            with client.socket_lock:
                self.command_socket.sendto(packet, (client.ip, client.port))
        except OSError as e:
            raise ValueError(
                f"[NatNetServer] Error sending message {int(message_id)} to "
                f"client {client.ip}:{client.port}: {e}"
            ) from e

    def _data_update_loop(self): # Stub: Different betweeen multicast and unicast server implementations, as they will need to handle client connections differently (e.g. unicast will need to manage a list of connected clients and send packets directly to their IPs, while multicast will just send to the multicast group address)
        # Loop to update mocap data and send packets at regular intervals (e.g. 100Hz)
        pass

    def _send_data_packet(self, client: Client, data_message: DataMessages.sFrameOfMocapData):
        # Serialize frame payload and send via command socket (unicast libNatNet 4.4).
        try:
            packet_bytes = data_message.pack()
        except Exception as e:
            raise ValueError(f"[NatNetServer] Error serializing data message: {e}") from e

        self._send_packet_to_client(
            client,
            ServerMessages.MessageId.NAT_FRAMEOFDATA,
            packet_bytes,
        )

    def _command_listener_loop(self): # Stub: Different betweeen multicast and unicast server implementations, as they will need to handle client connections differently (e.g. unicast will need to manage a list of connected clients and send packets directly to their IPs, while multicast will just send to the multicast group address)
        # Loop to listen for and handle incoming command requests (e.g. from client apps)
        pass

    def _handle_command_request(self, request_data: bytes): # Stub: Different betweeen multicast and unicast server implementations, as they will need to handle client connections differently (e.g. unicast will need to manage a list of connected clients and send packets directly to their IPs, while multicast will just send to the multicast group address)
        # Parse incoming command request, perform requested action, and send response if needed
        pass

