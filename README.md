# imav2025_swarm

## Running the mission

It is recommended to use tmux.

Running MicroXRCEAgnt:
```
MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600
```

Running the system:
```
ros2 launch imav2025_swarm swarm_multi.launch.py swarm_count:=<int> ns_prefix:=<px4|uav>
```

Starting the mission:
```
ros2 service call /p<px4|uav>_1/swarm_start std_srvs/srv/Trigger "{}"
```

### PX4 parameters
 - `UXRCE_DDS_CGF`: serial port set up (for IMAV it is `TELEM/SERIAL 4`)
 - `SER_TEL4_BAUD`: baud rate for telem/serial4 port (for imav it is `921600 8N`)
 - `UXRCE_DDS_NS_IDX`: sets up the namespace
 - `MAV_SYS_ID`: should be `UXRCE_DDS_NS_IDX` + 1 

## Setting up the development environment

Building the image locally:

```
docker build -t highflyers/imav-2025:latest . --build-arg USER_UID=$(id -u)
```

Running the container:

Run `scripts/set_GPU_param.sh` to detect the GPU support and set the `DOCKER_GPU_PARAM` value. Defualts to `device=/dev/dri` (no GPU support).

It is recommended to develop inside a devcontainer.

For running the container manually, use:

```
docker run --name imav --network host --ipc host --privileged -it highflyers/imav-2025:latest /bin/bash
```
