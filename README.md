# RoBorregos RoboCup @HOME 2024

## Docker environment setup

### Machine prerequisites

- Docker Engine (Ubuntu) / Docker Desktop
- NVIDIA Driver

### Installation

Clone the repo:

```bash
git clone https://github.com/RoBorregos/robocup-home2.git
cd robocup-home2
```

Build Docker image:
```bash
docker build -t ros-humble-home .
```

Run a Docker container:
```bash
xhost + # Allow visual tools display (RViz, RQT, ...)
docker run -it --name main_home --net=host --gpus all --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 -v $(pwd):/workspace ros-humble-home:latest bash
```

If getting a similar error as the following:

```
docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
ERRO[0000] error waiting for container: context canceled
```

Check the [Installing the toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) page of NVIDIA Container Toolkit, from installation to configuration. Then, repeat the docker run command.

To open multiple terminals with the container already running, type:
```bash
xhost + && docker exec -it --user $(id -u):$(id -g) main_home bash
```

The docker container won't be removed from the machine when turned off, unless specified in the run command. After the initial setup, start the container using:

```bash
docker start main_home
```

For efficiency when working with multiple terminals, add an alias command inside the `~/.bashrc` file:
```bash
nano ~/.bashrc
```
At the bottom of the file add the line:
```bash
# Alias should be different than the container name, could be personalized at will
alias home_main='xhost + && docker exec -it --user $(id -u):$(id -g) main_home bash'
```

Exit and save the file (`Ctrl X + Y + Enter`), and after opening a new terminal, the command `home_main` will open the container bash.
