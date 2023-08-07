# Introduction
This library has been my learning experience for a period of time, and I have made countless mistakes and still haven't learned well in many aspects. The basic learning process is described here.  
## ROS learning  
In my experience, ROS does not require deliberate learning. What we need to do is learn how to use ROS. Therefore, as long as we start to understand the usage of ROS and understand the mechanism of ROS messages, we can handle the vast majority of tasks. The rest only require us to learn while using ROS. 

Here are some links related to ROS learning：  
1.[wiki](http://wiki.ros.org/ROS/Tutorials)  
2.[古月居](https://www.bilibili.com/video/BV1zt411G7Vn/?spm_id_from=333.337.search-card.all.click&vd_source=d6ea4dbc61d9452fed12a5669810253d)  
## Opencv and PCl  
These two libraries do not need to be deliberately studied, just learned during use. We use the version that comes with the system for PCL. We can download opencv-4.5-* in [website](https://opencv.org/releases/) . First, extract it and enter the folder:
```bash
cd opencv
madir build
cd build && cmake ..
make -j8
sudo make install
```

## How to use the current repository  
This library is actually pieced together by many corresponding libraries, using many libraries, resulting in a relatively awkward situation. And the implementation effect is not particularly good, in fact, it may be an essential starting point design issue, and the first step is to build the corresponding simulation environment.  
### GAZEBO Environment  
```bash
git clone -b v1.12.3 https://github.com/PX4/PX4-Autopilot.git --recursive     
```
add the bashrc file  
```bash
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
source ~/.bashrc
```
We use iris_depth_camera model and the world which is chose by you. you can see the model in " ~/PX4-Autopilot/Tools/sitl_gazebo/models " and world in " ~/PX4-Autopilot/Tools/sitl_gazebo/worlds ". The initial file connection to Mavros is in " ~/PX4-Autopilot/launch " which is called " mavros_posix_sitl.launch ". As show in the picture, you can change the position, orientation and model,world. [Mavros](http://wiki.ros.org/mavros) is a secondary wrapper used for controlling drones and onboard computers. There are somethings about it.  

![image](https://github.com/xxje-library/DEF/blob/main/picture/mavros.png)  
### Eigen3  
```bash
sudo apt-get install libeigen3-dev
```
### [OpenVINO](https://software.intel.com/content/www/us/en/develop/tools/openvino-toolkit/download.html)  
you will get a l_openvino_toolkit_p_<version>.tgz. It should be 2021 version. Unzip it and Enter folder.
```bash
sudo ./install_GUI.sh

cd opt/intel/openvino_2021/install_dependencie
sudo -E ./install_openvino_dependencies.sh

sudo gedit ~/.bashrc
source /opt/intel/openvino_2021/bin/setupvars.sh
source ~/.bashrc
```
you will see " [setupvars.sh] OpenVINO environment initialized ". The purpose of this module is neural network recognition, and we are using the Openvino version of [Nanodet](https://github.com/RangiLyu/nanodet).  

### [GLPK](http://ftp.gnu.org/gnu/glpk/)  
The version is 5.0. Enter the folder.
```bash
cd glpk
configure
make -j8
sudo make install
```

### Test
All the necessary files have been installed above, and below, all you need to do is use the library.
#### Step1  
```bash
roscore
```
#### Step2  
```bash
roslaunch px4 mavros_posix_sitl.launch
```
#### Step3 
```bash
cd key_control
python3 communicate.py
python3 key.py
```
you should press "t" "b" to arm UAV and press"i" "s" to fly and hover.
#### Step4
```bash
cd DEF
catkin_make
source devel/setup.bash
roslaunch plan_manage rsastar_replan.launch
```
Press "G" to give the goal.







