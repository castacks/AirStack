# Motive / NatNet Emulator

This directory is the future home of **integration tests** that drive a real
NatNet wire-protocol mock server against `natnet_ros2_node`.

## Why here, not in the package test/ dir?

Unit tests for pure logic live in
`tests/robot/perception/natnet_ros2/test_natnet_logic.cpp` and run via `colcon
test` with no network or SDK required (uses `FakeNatNetClient`).

The emulator tests here will require an actual UDP server that speaks the NatNet
protocol, so they belong in the `sensors` mark of the system test suite alongside
other topic-streaming tests.

## Planned implementation

The mock server should:

1. Open a UDP socket on the NatNet command port (default 1510).
2. Respond to `NAT_CONNECT` (message type 0) with a `NAT_SERVERINFO` (type 1)
   packet containing a canned `sServerDescription`.
3. Respond to `NAT_REQUEST_MODELDEF` (type 4) with a `NAT_MODELDEF` (type 5)
   packet describing one or more rigid bodies.
4. Stream `NAT_FRAMEOFDATA` (type 7) packets to the client's data port at a
   configurable rate with synthetic pose data.

### Reference

The NatNet wire format is documented in the NatNet SDK developer notes and the
`PacketClient` example shipped with the SDK (available inside the robot Docker
container after `airstack setup --natnet`).

## Relationship to `FakeNatNetClient`

```
                ┌──────────────────────────────────────┐
                │         Test boundary                 │
 colcon gtest   │  FakeNatNetClient (in-process)        │  ← unit tests (no network)
                │  test_natnet_logic.cpp                │
                └──────────────────────────────────────┘

                ┌──────────────────────────────────────┐
                │         Network boundary              │
 pytest sensors │  MotiveEmulator (UDP server, Python)  │  ← integration tests
                │  NatNetClientAdapter → NatNetClient   │
                │  natnet_ros2_node (full ROS node)     │
                └──────────────────────────────────────┘
```

The `FakeNatNetClient` seam (already implemented) lets unit tests verify all
connection-outcome logic paths.  The emulator here will verify the full
end-to-end path including the NatNet SDK's own parser.

## When to add this

Implement the emulator when:
- The OptiTrack emulator service is placed under `simulation/optitrack-emulator/`
  or `tests/sim/motive_emulator/`
- The `sensors` test mark is extended to include `natnet_ros2` topic checks
- CI has access to the robot container with the NatNet SDK installed
