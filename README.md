# MuJoCo-franka-panda
**MuJoCo Simulation for Franka Emika Panda**  

<img src="https://github.com/ARC-KIST/MuJoCo-franka-panda/assets/113012648/e9dcf0f8-5e37-40b0-8b10-f4aa8f386b1a" width="80%"/>  

</br></br>

## Installation
__Essential 1.__ Clone the repository
```shell
$ git clone https://github.com/ARC-KIST/MuJoCo-franka-panda.git
```

__Essential 2.__ Install Dependencies
> RBDL, GLFW etc. should be installed!
- [RBDL](https://github.com/rbdl/rbdl.git): C++ library that contains some essential and efficient rigid body dynamics algorithms.
- [GLFW](https://www.glfw.org/): GLFW (Graphics Library Framework) is a lightweight utility library for use with OpenGL.

__Essential 3.__ Build the project
> Make the **build** directory
```shell
$ mkdir build
```
```shell
$ cd build
```
```
$ cmake ..
```
```
$ make
```

__Essential 4.__ Check if it was installed correctly
> Make sure you are in the build directory!
```shell
$ ./run
```

</br></br>
### Written by:
- 240520: [Sol Choi](https://github.com/S-CHOI-S)
