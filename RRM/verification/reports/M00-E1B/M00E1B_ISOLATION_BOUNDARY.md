# M00-E1B isolation boundary and exposure evidence

**UNVERIFIED ISOLATION. Target designation UNVERIFIED. PREPARED — NOT AUTHORIZED FOR EXECUTION.**

Solid arrows below describe observed socket relationships or configured/running software paths as labeled; dotted arrows describe potential/unverified actuation/external paths, not observed physical connections. Separate Linux network namespaces still communicate over the shared bridge. Domain boundaries are bridged by configuration; unique domain alone does not establish containment.

```mermaid
flowchart LR
  subgraph Workspace[Current candidate workspace — not assigned Tier B target]
    subgraph Sim[Privileged simulator container]
      PX4[PX4 SITL executing backing file]
      PEG[Pegasus / Isaac]
      PX4 -->|observed loopback TCP4561 + FD ownership| PEG
    end
    subgraph Robot[Privileged robot container — domain1]
      MAV[MAVROS UDP14541 binding]
      GUARD[Interface / guard + control APIs]
      AUTO[Observed planners/controllers/task/setpoint processes]
      GUARD -->|source-configured arming/mode/setpoint APIs| MAV
      AUTO -.->|possible commands; no mission queried| GUARD
    end
    GCS[GCS domain0 action relay and bridge processes]
    ROUTER[Running DDS routers — configs1↔0 and1↔99]
    MAV -->|startup-file peer172.31.0.200:14581; UDP bindings observed| PX4
    Robot -->|configured robot-command service bridge| ROUTER
    ROUTER -->|configured domain0| GCS
    ROUTER -.->|configured gossip domain99; peers unverified| Gossip[Other DDS peers]
    RW[Shared writable checkout / ROS workspace / cache]
    Sim -->|observed RW mounts| RW
    Robot -->|observed RW mounts| RW
  end
  Host[Host device namespace — ttyS0..31 + broad devices]
  Sim -.->|privilege + device-node exposure| Host
  Robot -.->|privilege + device-node exposure| Host
  Host -.->|possible serial/USB/CAN/radio path; not proven connected| Physical[Physical FC / actuators — UNVERIFIED]
  External[Host ingress / external routes]
  External -.->|published ports; no active probe| GCS
  External -.->|published WebRTC ports| Sim
  MAV -.->|uninspected routes/peers; no deny attestation| Physical
```

EX01–EX12 exposure register ties each source/destination/interface to direct evidence and configured/observed/possible status, consequence and mitigation. Device metadata shows ttyS nodes without opening them. Selected process FD snapshots did not show serial/USB handles, and no ttyACM/ttyUSB names were present, but these negative snapshots do not exclude future access, other processes/namespaces, network FCs, hotplug or physical devices behind existing nodes. The broad privileged device namespace remains a Class D blocker.

Socket tables prove local bindings and selected loopback connections; process FD inode matching attributes some sockets to current PX4/Pegasus/MAVROS processes. They do not identify MAVLink payload commands or authenticate a physical peer. Startup-file fcu_url is a configured destination, not proof every packet follows only that route. No packet capture or active scan occurred. DDS/router service-name configuration and process presence identify potential command forwarding without claiming active mission/arming/messages.

Closure requires an exact PD-designated independent rig; owner-provided device/capability/host isolation and effective route/firewall/peer/process allowlist, no unintended autolaunch/mission stack, private mounts/resources, named stop authority and independent safety review. Class D safety restrictions take precedence over every proposed Class C test. Existing framework stop/recovery documents and low/no observed traffic do not prove these controls. No isolation configuration, firewall, mount, process or service was changed in E1B.
