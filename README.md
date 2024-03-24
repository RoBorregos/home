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

All the development environments use Docker, follow the instructions below or the README inside the `docker` folder for insights.

## Software Architecture

![home-2](https://github.com/RoBorregos/home/assets/25570636/ea6f9551-27c7-4b4e-8fcb-8733a6eb7284)

## Docker Development

The project uses Docker for easier development within ROS and CUDA/Jetson compatibility. Both this main engine repository and each area's contain a `docker` folder with dockerfiles and a Makefile for easier image and container creation. To build an image, run:

```bash
# For CPU
make main.build
# For GPU
make main.build.cuda
# For Jetson L4T: 35.4.1
make main.build.jetson
```
To create a container, run:

```bash
# For CPU
make main.create
# For GPU
make main.create.cuda
# For Jetson L4T: 35.4.1
make main.create.jetson
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

Additional commands can be added with the Makefile and the scripts inside the `docker/scripts` folder can help for easier integration and sharing.

## Team Members

| Name                    | Github                                                       | Role      |
| ----------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------ |
| Adán Flores | [@afr2903](https://github.com/afr2903) | Integration, HRI & Manipulation |
| Emiliano Flores | [@EmilianoHFlores](https://github.com/EmilianoHFlores) | Integration, Manipulation & Computer Vision |
| Iván Romero | [@IvanRomero03](https://github.com/IvanRomero03) | Integration, HRI & Computer Vision |
| Alejandra Coeto | [@Ale-Coeto](https://github.com/Ale-Coeto) | Computer Vision & Manipulation |
| Oscar Arreola | [@Oscar-gg](https://github.com/Oscar-gg) | HRI & Web |
| Alexis Chapa | [@Chapa-1810](https://github.com/Chapa-1810) | Manipulation & Navigation |
| David Vázquez | [@Deivideich](https://github.com/Deivideich) | Electronics, Navigation & Manipulation | 
| Diego Hernández | [@Diego-HC](https://github.com/Diego-HC) | Navigation & Integration |
| Franciso Salas | [@Francisco-SP3](http://github.com/Francisco-SP3) | HRI |
| Leonardo Sánchez | [@LeoLFSH](https://github.com/LeoLFSH) | Mechanics |
| Alex Guerrero | [@]() | Mechanics |
