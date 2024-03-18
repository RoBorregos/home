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

All the development environments use Docker, follow the instructions inside the `docker` folders.

## Software Architecture



## Team Members

| Name                    | Github                                                       | Role      |
| ----------------------- | ------------------------------------------------------------------- | ------------------------------------------------------------ | --------- |
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