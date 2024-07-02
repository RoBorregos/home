# RoBorregos RoboCup @HOME 2024

Current development of [RoBorregos](www.roborregos.com), for the RoboCup @HOME competition, in the Open League Platform (OPL). The goal is to built a service robot capable of accomplishing tasks in domestic applications. The software areas being developed are: Human-Robot Interaction and Cooperation, Object Manipulation, Computer Vision, and Navigation and Mapping.

This repository features the `main_engine` package, in charge of managing Behavior Integration with all the areas.
For more information on the project, check our [documentation](https://docs.rbrgs.com/home/).

## Setup

Each development area has its own repository, added as a submodule here. To only clone this repo run:

```bash
git clone https://github.com/Roborregos/home
```

And to update the submodules content execute:

```bash
git submodule update --init --recursive
```

All the development environments use Docker, follow the instructions below or the [README](docker/README.md) inside the `docker` folder for insights.

## Software Architecture

![home-2](https://github.com/RoBorregos/home/assets/25570636/ea6f9551-27c7-4b4e-8fcb-8733a6eb7284)

## Docker Development
The project uses Docker for easier development within ROS and CUDA/Jetson compatibility. Both this main engine repository and each area's contain a `docker` folder with dockerfiles and a Makefile for easier image and container creation and modification. 
### Requirements

- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
- [Post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/)
If using GPU:
- NVIDIA Driver 
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html))
### Container Creation
To build an image, run:

```bash
# For CPU
make main.build
# For GPU
make main.build.cuda
# For Jetson L4T: 35.4.1
make main.build.jetson
```
To create a container, run the following commands. The `ws` folder is mounted by default, and additional folders can be added with the `volumes` argument, with both absolute and relative paths allowed:

```bash
# For CPU
make main.create volumes="another_folder1/,~/another_folder2"
# For GPU
make main.create.cuda volumes="another_folder1/,~/another_folder2"
# For Jetson L4T: 35.4.1
make main.create.jetson volumes="another_folder1/,~/another_folder2"
```

To enter the container, run:

```bash
make main.up
make main.shell
```

You can stop and remove the container with:

```bash
make main.down
make main.remove
```

Additional commands can be added within the Makefile and the scripts inside the `docker/scripts` folder can help for easier integration and sharing. These include a build script to run the dockerfile and create a new image and a run script to create containers from it. Any additional dependency or system/environment configuration should be added to these scripts.

## Task manager

The `frida_task_manager` is the central package in charge of processing the command information and directing the tasks with its respective area. For code readability, each area development its inside a Python module `area_tasks.py`, and they are called by `task_manager_server.py`, the script were the ROS node is created.

At the top of this mentioned file, there are four constants to enable or disable the tasks, you can change any of these to `False` is for the current test there is no need of an area:
```python
NAV_ENABLED = True 
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True 
VISION_ENABLED = False
```

For executing the `task_manager`, follow the above steps of [Docker setup](#Docker%20Development). Inside the `bash` terminal, setup the ROS network and execute the node:
```bash
export ROS_MASTER_URI=http://192.168.31.23:11311 # IP of the ROS Master (if master is in another machine)
export ROS_IP=192.168.31.97 # IP of own machine
rosrun frida_task_manager task_manager_server.py
```

## Team Members

| Name                    | Github                                                       | Role      |
| ----------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------ |
| Adán Flores | [@afr2903](https://github.com/afr2903) | Integration, HRI & Manipulation |
| Emiliano Flores | [@EmilianoHFlores](https://github.com/EmilianoHFlores) | Integration, Manipulation & Computer Vision |
| Iván Romero | [@IvanRomero03](https://github.com/IvanRomero03) | Integration, HRI & Computer Vision |
| Alejandra Coeto | [@Ale-Coeto](https://github.com/Ale-Coeto) | Computer Vision & Manipulation |
| Oscar Arreola | [@Oscar-gg](https://github.com/Oscar-gg) | HRI & Web |
| Alexis Chapa | [@Chapa-1810](https://github.com/Chapa-1810) | Manipulation & Navigation |
| Marina Villanueva | [@mariinaVillanueva](https://github.com/mariinaVillanueva) | HRI |
| David Vázquez | [@Deivideich](https://github.com/Deivideich) | Electronics, Navigation & Manipulation | 
| Diego Hernández | [@Diego-HC](https://github.com/Diego-HC) | Navigation |
| Franciso Salas | [@Francisco-SP3](http://github.com/Francisco-SP3) | HRI, Navigation |
| Leonardo Sánchez | [@LeoLFSH](https://github.com/LeoLFSH) | Mechanics |
| Alex Guerrero | [@alex-guerreroc](https://github.com/alex-guerreroc) | Mechanics |
