# Ground Controller PC Onboarding

Set up an Ubuntu computer to replace this machine as the AirStack ground
controller for BV (`drone_3`) and the drone-soccer policy. This is host and
network setup only. Use [SKILL.md](SKILL.md) for lab bringup and safety checks,
and [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for flight/policy commands.

## Topology

All three computers join the `SVGTeam` LAN:

| System | Address/identity | Ground-PC connection |
|---|---|---|
| Motive Windows PC | `192.168.50.5` | NatNet multicast `239.255.42.99`, UDP `1510/1511` |
| BV VOXL/PX4 | `192.168.50.12`, `drone_3` | SSH `voxl-bv`; Micro XRCE-DDS to ground UDP `8892`, ROS domain `1` |
| Ground controller | AirStation: `192.168.50.6` | AirStack, DDS agent, NatNet client, QGroundControl |

AirStation currently uses the reserved address `192.168.50.6`; BV intentionally
targets it instead of XiaoXin at `192.168.50.139`. For any later replacement
ground computer, determine its reserved address after joining the lab LAN and
use that same address as both `DDS_AGENT_IP` and NatNet `clientIP`.

## 1. Host prerequisites and repositories

Install Docker, give the user Docker access, and install QGroundControl. This
computer uses a QGC AppImage under `~/Applications` and the repository-local
`./airstack.sh`; it has no global `airstack` command or `~/.airstack.conf`.

Clone the repositories as siblings—the relative Compose mount requires this
layout:

```text
<workspace>/
├── AirStack/
└── drone_soccer/
```

Current matching branches are:

```bash
git clone https://github.com/castacks/AirStack.git
git clone git@github.com:neelay-j/drone_soccer.git
git -C AirStack switch yutong/drone_soccer
git -C drone_soccer switch hw_test
```

The AirStack container mounts `../drone_soccer` read-only at
`/root/drone_soccer` and adds it to `PYTHONPATH`. Do not install it editable.
Complete the policy dependency/build step in
[QUICK_REFERENCE.md §4](QUICK_REFERENCE.md).

## 2. Lab Wi-Fi and firewall

Create and connect the NetworkManager profile:

```bash
nmcli device wifi connect SVGTeam password passme24
ip -4 -brief address
ip route
```

Record the `192.168.50.x` address on the Wi-Fi interface. Ensure the host
firewall permits inbound UDP `8892`, `1510`, and `1511`; also permit the UDP
port configured for QGC telemetry (commonly `14550`). Host networking is
required for the `robot-desktop` container so multicast and DDS reach it.

On the Motive PC, enable Data Streaming, select its `192.168.50.x` lab
interface, and use the NatNet multicast address/ports in the topology table.
On BV, configure the PX4 MicroDDS client to target the **new ground-PC IP**,
domain `1`, port `8892`, and namespace `drone_3`; follow
**Configure VOXL/PX4** in [SKILL.md](SKILL.md#configure-voxlpx4).

## 3. VOXL SSH aliases

Generate separate keys on the new PC or securely obtain the approved lab
keys. Never commit private keys. Have an existing administrator install each
new public key in the corresponding VOXL `authorized_keys`, then add:

```sshconfig
Host voxl-bs
  HostName 192.168.50.73
  User root
  IdentityFile ~/.ssh/id_voxl_m0054
  IdentitiesOnly yes

Host voxl-bv
  HostName 192.168.50.12
  User root
  IdentityFile ~/.ssh/id_voxl_bv
  IdentitiesOnly yes
```

Do not create a `voxl-by` alias or key placeholder during onboarding. BY's
historical `192.168.50.6` configuration is not flight-ready and can conflict
with a ground controller's DHCP address. Restore BY access only after its
network identity, DDS settings, mocap mapping, and estimator have been
reconfigured and revalidated.

Protect the files and validate alias expansion without needing the lab:

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/config ~/.ssh/id_voxl_*
chmod 644 ~/.ssh/id_voxl_*.pub
ssh -G voxl-bv | grep -E '^(hostname|user|identityfile) '
```

The VOXL hostname may be `m0054`; use the alias/IP and configured drone name
as the operational identity.

## 4. Static checks before going to the lab

These checks work away from `SVGTeam`:

```bash
docker info
id | grep docker
test -d ../drone_soccer/drone_soccer
grep -E '^(COMPOSE_PROFILES|AUTOLAUNCH|NUM_ROBOTS)=' .env
```

Expected AirStack values are desktop profile, `AUTOLAUNCH="false"`, and one
robot. When convenient, start the container with `./airstack.sh up
robot-desktop` and verify that `/root/drone_soccer` exists and is read-only.

Lab IP, Motive, VOXL, DDS, QGC, mocap-frame, and estimator tests cannot pass
off-site. Run **Start the Lab Stack**, **Core Checks Before Any Arm**, and the
full **Flight Readiness Gate** in [SKILL.md](SKILL.md) after arriving.
