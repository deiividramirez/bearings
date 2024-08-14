# Implementation of an Image Based Visual Servo (IBVS) for a Quadrotor UAV with follower-leader formation control

This repository was made to implement a modification to the algorithm described in the paper "Image-based estimation, planning, and control for high-speed flying through multiple openings" by Guo, Dejun and Leang for the leaders, and only-bearing based control proposed on "Translational and scaling formation maneuver control via bearing-bases approach" by Shiyu Zhao, Daniel Zelazo.



## Let's begin 🚀

The next instructions will allow you to get a copy of the project running on your local machine for development and testing purposes. There are many prerequisites that you need to install before running the code. The code was developed and tested on Ubuntu 20.04 LTS, ROS Noetic, and Gazebo 11.11.0. The code was developed using Python 3.8 and C++ 17.

### Prerequisites 📋

* Python 3.8+
* OpenCV 4+
* ROS Noetic
* Gazebo 11.11.0

### Instalation 🔧

First, we are going to install ROS Noetic, if you already have it installed, you can skip this step.

#### Installation in one line by ROS
```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

and then, you can check if the installation was successful by trying to install the following packages:

```bash
sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox python3-wstool python3-catkin-tools
```

#### Python setup

Now, it is important to have Python PIP installed, if you don't have it, you can install it by running the following command:

```bash
sudo curl https://bootstrap.pypa.io/get-pip.py | sudo python3
sudo curl https://bootstrap.pypa.io/get-pip.py | python3
```

and then, add the path to the python's executables to the PATH variable, by adding the following lines to the ~/.bashrc file:

* Bash
  ```bash
  echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.bashrc
  source ~/.bashrc
  ```

* Zsh
  ```bash
  echo 'export PATH=$PATH:"$HOME/.local/bin"' >> ~/.zshrc
  source ~/.zshrc
  ```

#### Python dependencies

Now, we are going to install the Python dependencies, by running the following command:

```bash
sudo python3 -m pip install -U rosdep catkin_pkg future
python3 -m pip install -U rosdep catkin_pkg future empy defusedxml numpy matplotlib imageio opencv-contrib-python
```

#### ROS dependencies

For ROS dependencies, we are going to create a new catkin workspace with all the dependencies, by running the following commands:

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
catkin_init_workspace
```

Then, we are going to clone the corresponding repositories MavROS and Rotors Simulators, by running the following commands:

```bash
cd ~/catkin_ws/src
wstool init
wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
git clone https://github.com/ethz-asl/rotors_simulator
git clone https://github.com/ethz-asl/mav_comm
wstool merge rotors_hil.rosinstall
wstool update
```

Now, the "Bearing" package is going to be cloned, by running the following commands:

```bash
cd ~/catkin_ws/src
git clone https://github.com/deiividramirez/bearings
git clone https://github.com/deiividramirez/placing_iris.git
```

The "placing_iris" package is for placing the iris drone in a specific position in the Gazebo world (in a global reference frame), and the "bearings" package is for the whole control description.

Finally, we are going to build the workspace, by running the following commands:

```bash
sudo rosdep init
rosdep update
cd ~/catkin_ws/
# The next command is useful if you have problems with the dependencies by installing them
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
catkin build
```

Finally, you can add the scripts to terminal environment by running the following commands:

* Bash
  ```bash
  echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
  source ~/.bashrc
  ```
* Zsh
  ```bash
  echo 'source ~/catkin_ws/devel/setup.zsh' >> ~/.zshrc
  source ~/.zshrc
  ```

## Execution ⚙️

You have to run the following commands in different terminals:

### Gazebo launch file
  
```bash
roslaunch bearings bebop2_4.launch
```

Each drone has a different namespace with "iris_n" where n is the number of the drone, for example, the first drone has the namespace "iris_1", the second drone has the namespace "iris_2", and so on.

You can move any drone along Gazebo's world by using the following command:

```bash
rosrun placing_iris placing_iris bebop2_n pos_x pos_y pos_z pos_yaw
```
  
where n is the number of the drone, pos_x, pos_y, pos_z, and pos_yaw are the position and orientation of the drone in the Gazebo's world.

### Bearing controller

Before you launch the IBVS controller, you have to make sure the target images are correctly. You can modify the "general.yaml" file inside "config" folder, and then, change then change the variable as 

```bash
SAVE_DESIRED_IMAGES: 1
```

Then, make sure the drones are placed in the correct position with whole visibility of the ArUco marker and then, launch the control as usual:

```bash
roslaunch bearings control.launch
```

Then, you can change the variable "SAVE_DESIRED_IMAGES" to 0, and then, you can run the control for the IBVS controller.

### Disclaimer ⚠️

Each drone has its own namespace and its own params file in "config" folder, so, you can modify the parameters for each drone; including the ArUco marker ID, the camera topic, and the camera calibration matrix, gains, front camera or bottom camera, and so on. Remember, changing the camera topic and the camera calibration matrix could bring some problems, so, be careful.

## Additional information 📖

* You can find the individual controllers simulated in Python in the "Python" folder. The "Python" folder contains the IBVS controller with GUO version and classic version. Also, it contains the IBVS controller bearing-only controller. 

* There are some scripts in the "src/data" folder, which are useful for plotting the data, making a gif, and making a mp4 video. Also, there is a script for detecting the ArUco marker in the images. These scripts can be used by temporal alias in the terminal, by executing

  ```bash
  alias clean="python ~/catkin_ws/src/bearings/src/data/aux_clean.py"
  alias plot="python ~/catkin_ws/src/bearings/src/data/aux_plot.py"
  alias gif="python ~/catkin_ws/src/bearings/src/data/aux_gif.py"
  alias mp4="python ~/catkin_ws/src/bearings/src/data/aux_makeMP4.py"
  alias aruco="python ~/catkin_ws/src/bearings/src/data/aux_aruco_detector.py"
  alias control="roslaunch bearings control.launch"
  alias bebop4="roslaunch bearings bebop2_4.launch"
  alias bebop5="roslaunch bearings bebop2_5.launch"
  alias iris4="roslaunch bearings iris_wall_4.launch"
  alias iris5="roslaunch bearings iris_wall_5.launch"
  ```

* After every controller simulation, the are some data saved in "src/data" folder, including error's data, integral part control data, adaptative gains data, velocity data and, position data. This data can be easily plotted with the `plot` alias.

  It can be used for an specific drone, by running `plot n`, where n is the number of the drone.

* If you want to make a gif, you can use the `gif` alias, by running `gif n`, where n is the number of the drone. The gif will be saved in the "src/data" folder as "out_sim_n.gif".

* In "general.yaml" exists an option called "SAVE_IMAGES" which is useful for saving the images of the drone's camera. If you want to save the images, you have to change the variable to 1, and then, you can change it to 0, and then, you can run the control for the IBVS controller.

  Also, if you saved the previous images, and want to run again the control, it is recommended to clean the previous images, by running `clean`, which will delete all the images in the "src/data/img" folder.

* If you want to make a mp4 video, you can use the `mp4` alias, by running `mp4 n`, where n is the number of the drone. The mp4 video will be saved in the "src/data" folder as "out_sim_n.mp4".

* If you want to make sure the ArUco marker is detected in the desired images, you can use the `aruco` alias, by running `aruco n`, where n is the number of the drone. The ArUco marker will be detected in the images, and plot the detected marker in a matplotlib window.


## Autores ✒️

* **David Ramírez** - *Implementación* - [deiividramirez](https://github.com/deiividramirez)


<!-- ## License 📄 -->

<!-- Este proyecto está bajo la Licencia (Tu Licencia) - mira el archivo [LICENSE.md](LICENSE.md) para detalles -->


---
⌨️ with ❤️ by [deiividramirez](https://github.com/deiividramirez) 😊
