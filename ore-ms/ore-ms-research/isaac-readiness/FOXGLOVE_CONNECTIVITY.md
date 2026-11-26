# Foxglove connectivity

The GCS runs the ROS 2 C++ Foxglove bridge in domain 0 on TCP 8765. Docker publishes pod TCP 8766 and the OSMO helper forwards local TCP 8766 to that pod port. Isaac, robot and GCS share the Compose bridge network. The robot's DDS Router selectively forwards domain 1 streams into domain 0. Local WebSocket probes negotiated `foxglove.sdk.v1` with HTTP 101 and received live data.

The tested local bridge accepted the connection without credentials. External access depends on the OSMO tunnel. Direct public exposure and authentication enforcement were not separately tested. The user-provided screenshot showed a rendered right-camera frame and a map axis through the tunnel, and the user reported a 1011 keepalive timeout. The helper installed custom panels into Foxglove Desktop. The web client showed Robot Tasks as an unknown panel. Fresh OSMO startup and a new desktop login were not verified.
