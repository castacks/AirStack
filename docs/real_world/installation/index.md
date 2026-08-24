# Installation on ORIN AGX/NX

We have tested installation and running the robot container on Jetson ORIN AGX/NX with Ubuntu 22.04 (L4T / JetPack).

## Setup

Ensure you have Docker installed (`airstack install` can install it for you).

### Clone

```bash
git clone --recursive -j8 git@github.com:castacks/AirStack.git
cd AirStack
```

## Configure

Run `./airstack.sh setup` and follow the prompts to do an initial configuration of the repo (this also adds the `airstack` command to your PATH).

Pull the correct image:

```bash
docker compose --profile l4t pull robot-l4t
```

## Run

```bash
airstack --profile l4t up
```

The autonomy stack launches inside a tmux session in the `robot-l4t` container. Verify it with `airstack status` and follow the output with `airstack logs robot-l4t` (or `airstack connect robot-l4t` to attach to the tmux session).
