# imav2025_swarm

## Setting up the development environment

Building the image:

```
docker build -t highflyers/imav-2025:latest . --build-arg USER_UID=$(id -u)
```

Running the container:

Run `scripts/set_GPU_param.sh` to detect the GPU support and set the `DOCKER_GPU_PARAM` value. Defualts to `device=/dev/dri` (no GPU support).

It is recommended to develop inside a devcontainer.

For running the container manully, use:

```
docker run --name imav --network host --ipc host --privileged -it highflyers/imav-2025:latest /bin/bash
```
