# Panda Dagger #

This repository implements a simple algorithm for imitation learning: [DAGGER](https://www.cs.cmu.edu/%7Esross1/publications/Ross-AIStats11-NoRegret.pdf).
In this example the agent is the Panda Emika robot and learn how to move in a goal position. The goal is to do a simple grasping task of a ring. It uses a monocular camera RGB to detect the ring position.

## Getting started ##

This repositories need [this](https://github.com/falfab/franka_ros) fork of _franka_ros_ repository, clone it and follow the standard install instruction. It is a modified version of the original repository with a robot model which works offline and a different kinematic solver (trac_ik_kinematics).

It has been used the robot simulator V-REP (version 3.5.0 proedu), download it [here](http://www.coppeliarobotics.com/downloads.html).

Optional but suggested step:

- Create a python virtual environment:

```bash
sudo apt-get install virtualenv
virtualenv venv
source venv/bin/activate
```

Nedded steps:

- Install python requirements:

```bash
pip install -r requirements.txt
```

- If you have Nvidia GPU with cuda you should install also:

```bash
pip install tensorflow-gpu
```

- Install ROS C++ dependencies:

```bash
rosdep init
catkin_make
```

Now you're ready to start!

### Preliminary steps necessary for the tasks below ###

- Launch the controllers (this step is required for all node):

```bash
roslaunch panda_dagger panda_controllers.launch
```

- Open VREP (it is supposed you have an alias to vrep.sh)

```bash
vrep ./vrep/scene/panda_dagger.ttt
```

## 1. Generate dataset ##

- Run dataset generator:

```bash
rosrun panda_dagger dataset_generator
```

This node will iterate and will generate a dataset of optimal movement to reach the goal. You can stop the execution when you want, every single iteration is saved as single dataset in the dataset directory

## 2. Dataset merge ##

- Run dataset merge:

```bash
rosrun panda_dagger dataset_merge
```

This node will merge the datasets in the dataset folder into a single dataset called merged.pkl

## 3. Dagger ##

- Run dagger itself:

```bash
rosrun panda_dagger dagger
```

This node will execute the dagger algoritm. To customize execution look at the config file in config folder.
When dagger execution is done it will save the trained net into the models folder.

## 4. Use trained model ##

- Execute trained model:

```bash
rosrum panda_dagger trained_controller
```
