# Biotac SP ROS

This repository is for hosting the Biotac SP sensors software released under the lgpl3.0 license.

# Requirements

## Documentation
* [Biotac SP user manual](https://github.com/pedrombmachado/biotac_sp/blob/master/doc/BioTac_SP_Product_Manual.pdf)

## Hardware
* 3x Biotac SP sensors
* Biotac Board
* micro-USB power cable for connecting the Biotac board
* Cheetah SPI host
* USB-B cable.
* Host pc

## Software
### ROS
* Ubuntu Linux 18.04 LTS
* ROS Melodic [installed](http://wiki.ros.org/melodic/Installation/Ubuntu) and [configured](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
### ROS 2
* Ubuntu 24.04 LTS
* ROS 2 Jazzy [installed](https://docs.ros.org/en/jazzy/Installation.html) and [configured](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

# Installation procedure:
## Step 1: Connect the equipment 
Figure 1 shows the configuration setup

![](https://github.com/pedrombmachado/biotac_sp/blob/master/doc/Biotac.png)

Figure 1: Biotac setup
  
## Step 2: Update the OS and install base packets

```
$ sudo apt update & sudo apt upgrade -y & sudo apt dist-upgrade -y & sudo apt autoremove -y & sudo apt autoclean -y
$ sudo apt install build-essential git terminator
```

## Step 3: clone the repository
### ROS
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNCR-NTU/biotac_sp_ros.git
```
### ROS 2
$ cd ~/ros2-ws/src
$ git clone https://github.com/CNCR-NTU/biotac_sp_ros.git
## Step 4: install the drivers
```
$ cd biotac_sp_ros
$ ./installCheetahDriver.sh
```

## Step 5: compile and install

### ROS only
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.sh
```
### ROS2 only
```
$ cd ~/ros2-ws
$ colcon build --packages-select biotac_sp_ros
$ source devel/setup.sh
```

## Step 6: run

### ROS only
`$ roslaunch biotac_sp_ros biotac.launch`


### ROS2 only
`$ ros2 launch biotac_sp_ros2 biotac_launch.py`


# Understanding the data
4x data frames are colected per second. Each frame is composed follows the recommended Dafault sampling sequence.

Figure 2 shows the Default Sampling	Sequence

![](https://github.com/CNCR-NTU/biotac_sp/blob/master/doc/data_sampling.png)

Figure 2: Default sampling sequence

Table 1: Bandwidth and Sampling rate for Default Sampling Sequence at 4.4kHz

![](https://github.com/CNCR-NTU/biotac_sp/blob/master/doc/data_sampling_bandwidth.png)

Figure 3 shows the Biotac SP electrodes distribution

![](https://github.com/CNCR-NTU/biotac_sp/blob/master/doc/Electrodes_distribution.png)

Figure 3 Biotac SP electrodes distribution

The output is a vector of 163 columns where:

[time, E1_s01, PAC_s01, E2_s01, PAC_s01,...,E24_s01, PAC_s01, PDC_s01, PAC_s01, PAC_s01, TAC_s01, PAC_s01, TDC_s01, PAC_s01,
E1_s02, PAC_s02, E2_s02, PAC_s02,...,E24_s02, PAC_s02, PDC_s02, PAC_s02, PAC_s02, TAC_s02, PAC_s02, TDC_s02, PAC_s02,
E1_s03, PAC_s03, E2_s03, PAC_s03,...,E24_s03, PAC_s03, PDC_s03, PAC_s03, PAC_s03, TAC_s03, PAC_s03, TDC_s03, PAC_s03]

where:

s01 - sensor 1

s02 - sensor 2

s03 - sensor 3

With exception of the time, each of the values is the average of the individual values of the 4x frames collected during 1 sec.

# Howto cite?
Please cite as: 
Machado, Pedro, & McGinnity, T.M. (2019, November 5). BioTac SP ROS stack (Version 1.0.0). Zenodo. http://doi.org/10.5281/zenodo.3529376

DOI: ![](https://github.com/CNCR-NTU/biotac_sp_ros/blob/master/doc/zenodo.3529376.svg)

# Contacts
Computational Neurosciences and Cognitive Robotics Group at the Nottingham Trent University.

Pedro Machado <pedro.baptistamachado@ntu.ac.uk>

