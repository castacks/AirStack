# Installation on ORIN AGX/NX

We have tested installation and running robot container on Jetson ORIN AGX/NX and Ubuntu 22.04.

## Setup
Ensure you have docker installed.
### Clone

```
git clone --recursive -j8 git@github.com:castacks/AirStack.git
```
Checkout to the correct branch:
```
git checkout jkeller/jetson_36.4
```
## Configure

Run `./configure.sh` and follow the instructions in the prompts to do an initial configuration of the repo.

Pull the correct image:
```
docker compose pull robot_l4t
```

## Run
```
docker compose up robot_l4t
```
You should be able to see the rviz GUI being launched.