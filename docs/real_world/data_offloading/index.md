# Data Offloading

We have a tool to automatically offload and sync your robot data to the AirLab internal storage server.

## Setup Storage Tools Server Locally

### Clone and install

``` bash
git clone https://github.com/castacks/storage_tools_server
cd storage_tools_server
python -m venv venv
. venv/bin/activate
pip install -r requirements.txt
```

### Configure

Edit the `config/config.yaml` file to match your configuration.

### REQUIRED UPDATES

- `upload_dir` is the location for uploads.  This must be readable and writeable by the user running the Server.
- `volume_root` sets the prefix for all entries in the `volume_map`.  This must be readable and writeable by the user running the Server.
- `volume_map` is a mapping from project name to `volume_root/{path}`.  All projects must have a mapping.

### Set Environment and Run

- `CONFIG` is the full path to the `config.yaml` in use.  By default, the app will use `$PWD/config/config.yaml`
- `PORT` is the same port as define in the optional setup. The default is 8091.

``` bash
export CONFIG=$PWD/config/config.yaml
export PORT=8091

gunicorn -k gevent -w 1 -b "0.0.0.0:${PORT}" --timeout 120 "server.app:app"
```

Open a web browser to http://localhost:8091 (or the PORT you set). The default user is `admin` and the default password is `NodeNodeDevices`.

### Create an API Key for your robot

- Log into the Server
- Go to Configure -> Keys
- Enter a name for the device key in the "Add a new key name" field.
- Click "Generate Key"

## Set up Storage Tools Device on your Robot

### Install Requirements

- [Docker Compose](https://docs.docker.com/compose/install/standalone/)

### Clone Device Repo

```bash
cd /opt 
git clone https://github.com/castacks/storage_tools_device
cd stroage_tools_device
```

### Update the config.yaml

Update `config/config.yaml` to match your environment.  Things you should update:

- `API_KEY_TOKEN`.  The api key that your admin gave you, or the key that you set up in the [Server Setup](#create-an-api-key-for-your-robot)
- `watch`.  The list of directories that have your robot's files.

Update the `env.sh` to match your system.

- `CONFIG_FILE`.  If you have multiple config files, make sure `CONFIG_FILE` points to the one you want to use.
- `DATA_DIR`. This is the top level data directory that all of the `watch` dirs share.  For example, if you `watch` directories are `/mnt/data/processor_1` and `/mnt/data/processor_2`, set the `DATA_DIR` to `/mnt/data`.  

### Build and Run

``` bash
cd /opt/storage_tools_device
. env.sh
docker compose up --build
```
