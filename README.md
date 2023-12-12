# HDVS_Franka
A novel Hybrid Decoupled Visual Servoing (HDVS) method implemented in C++ and ROS for Franka Panda robots.

![FrankaSimDm](https://github.com/aaflakiyan/HDVS_Franka/assets/48828461/502be807-bfee-41e5-877b-ade7fb297783)

## Installaion 

```
git clone https://github.com/frankaemika/franka_ros.git
git clone https://github.com/erdalpekel/panda_simulation.git
git clone https://github.com/neka-nat/gazebo_domain_randomization.git
```

Unzip the visp_ros.zip file inside this repository to your src folder in your ros workspace. 
The zip file contains visp_ros and vision_visp ros nodes based on the VISP library [GitHub Pages](https://github.com/lagadic).


## Usage 
To run the simulation without domain randomization: 

```
roslaunch hdvs_launch simulation-wo-DM.launch
```

To run the simulation with domain randomization: 

```
roslaunch hdvs_launch simulation.launch
```

Inside the src folder of hdvs_code, you'll find codes dedicated to various types of visual servoing. Notably, the Pure_vs code is designed for universal use across different robot types. This code calculates the generated velocity from each visual servoing method and publishes the resulting data on the assigned ROS topic. 

