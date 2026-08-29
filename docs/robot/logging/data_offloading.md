# Data Offloading

The supported workflow for offloading ROS bags and logs from robots is the
storage-tools pair — [storage_tools_server](https://github.com/castacks/storage_tools_server)
on the receiving machine and storage_tools_device on the robot. Setup and usage
are documented in the [Real World Data Offloading guide](../../real_world/data_offloading/index.md).

## Quick Manual Copy

For a one-off manual transfer, plain rsync works. Bags land in `robot/bags/`
on the host (mounted at `/bags` in the robot container); on Jetson (`l4t`
profile) they land in `${BAG_STORAGE_PATH}` (default
`/media/airlab/Storage/airstack_collection`):

```bash
rsync -avz --progress /media/airlab/Storage/airstack_collection/ user@groundstation:/data/robot_1/bags/
```

## See Also

- [Real World Data Offloading](../../real_world/data_offloading/index.md) - The supported storage-tools workflow
- [Logging Overview](index.md) - Where bags land and how recording is configured
- [ROS Bags](rosbags.md) - Recording data
