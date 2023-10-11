# ALPINE: A cLimbing robot for oPerations In mouNtain environmEnts




Michele Focchi, Andrea Del Prete, Luigi Palopoli, Daniele Fontanelli, Marco Frego, Angelika Peer

Corresponding author's email: Michele Focchi

This repository is a reduced version of [Locosim](https://github.com/mfocchi/locosim) ([preprint](https://arxiv.org/abs/2305.02107)) and it is intended for reproducing simulations and experiments
presented in the manuscript : TBD



# Installing the code

To install the code follow these detailed installation [instructions](https://github.com/mfocchi/climbing_robots2/tree/master/Install.md).



# **Running the Code**  

### IDE Pycharm

We recommend to use an IDE to run and edit the Python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/Python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the home directory. 

We ask you to download this specific version (2021.1.1) that we am sure it works: newer versions seem to be failing to load environment variables. 

To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 

### Terminal

To run from a terminal we  use the interactive option that allows  when you close the program have access to variables:

```
$ Python3 -i $LOCOSIM_DIR/robot_control/base_controllers/climbingrobot_controller2.py
```

to exit from Python3 console type CTRL+Z



### Simulation settings

The file **climbingrobot_controller2.py** has certain option flags that are summarized in the following table:

| Flag                | Value                                                        |
| ------------------- | ------------------------------------------------------------ |
| robotName           | 'climbingrobot2' (without landing mechanism), 'climbingrobot2landing' (with landing mechanism) |
| MPC_control         | MPC control Enabled/Disabled                                 |
| OBSTACLE_AVOIDANCE  | Add Elllipsoidal obstacle on the wall                        |
| type_of_disturbance | Type of disturbance enabled at the lift-off ('none', 'impulse', 'const') |
| PROPELLERS          | Propellers Enabled/Disabled                                  |
| MULTIPLE_JUMPS      | Perform multiple jumps on an ellipsoid path around p0        |

Note: You need the matlab runtime Environment installed to be able to run the simulations that make use of C++ Matlab generated code. We provide a docker image with Ubuntu 20 that already contains Matlab R2023 and Matlab Runtime Environment  and all the required code dependencies already installed (you will need only to clone the code and compile it), by following this  [wiki](https://github.com/mfocchi/lab-docker). Docker has been tested to work with windows machines, Linux and old MACs (not ARM processors).  To be able to run Matlab you just need to 1) log into the docker as root (dock-root alias), 2) copy your license file into the folder /usr/local/MATLAB/R2023a/licenses 3) do a docker commit for future uses.



### Matlab

The repo provides a simulation with the reduced order model and both an offline jump optimization (optimal control) and online (mpc). In both of them it is possible to generated C++ code uncommenting the appropriate lines. For the MPC it is also possible to emulate an MPC loop.



# Tips and Tricks 

1) Some machines, do not have support for GPU. This means that if you run Gazebo Graphical User Interface (GUI) it can become very **slow**. A way to mitigate this is to avoid to start the  Gazebo GUI and only start the gzserver process that will compute the dynamics, you will keep the visualization in Rviz. This is referred to planners that employ BaseController or BaseControllerFixed classes. In the Python code where you start the simulator you need to pass this additional argument as follows:

```
additional_args = 'gui:=false'
p.startSimulator(..., additional_args =additional_args)
```

2) Another annoying point is the default timeout to kill Gazebo that is by default very long. You can change it (e.g. to 0.1s) by setting the  _TIMEOUT_SIGINT = 0.1 and _TIMEOUT_SIGTERM = 0.1:

```
sudo gedit /opt/ros/ROS_VERSION/lib/PYTHON_PREFIX/dist-packages/roslaunch/nodeprocess.py
```

 this will cause ROS to send a `kill` signal much sooner.

3) if you get this annoying warning: 

```
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame...
```

a dirty hack to fix it is to clone this repository in your workspace:

```
git clone --branch throttle-tf-repeated-data-error git@github.com:BadgerTechnologies/geometry2.git
```

## Code usage	

Describe how to use the code
