# Robocup 2021 World

1. **Create a docker folder**
    ```
    $ mkdir ~/docker_robocup
    ```

2. **Download official docker Simulator**
    Clone the [repo](https://github.com/RoboCupAtHome/Simulation-OPL-TiaGO) and build the image.
    ```
    $ cd ~/docker_robocup
    $ git clone --recursive https://github.com/RoboCupAtHome/Simulation-OPL-TiaGO.git
    $ cd Simulation-OPL-TiaGO
    $ ./build.sh
    ```
3. **Download our docker bridges**
    Clone the [repo](https://github.com/gentlebots/tmc_wrs_docker) and pull the image.
    ```
    $ cd ~/docker_robocup
    $ git clone --recursive https://github.com/gentlebots/tmc_wrs_docker.git -b rc_melodic_pks_and_bridges
    $ cd tmc_wrs_docker
    $ ./pull-images.sh
    ```
4. **Create deps ws**
    ```
    $ mkdir -p ~/robocup_melodic_ws/src
    $ cd ~/robocup_melodic_ws/src
    $ wget https://raw.githubusercontent.com/gentlebots/gentlebots/main/melodic_deps.repos
    $ vcs import < melodic_deps.repos
    ```
5. **Create a ROS2 ws and clone our repos**
    ```
    $ mkdir -p ~/robocup_ws/src
    $ cd ~/robocup_ws/src
    $ wget https://raw.githubusercontent.com/gentlebots/gentlebots/main/gentlebots.repos
    $ vcs import < gentlebots.repos
    $ cd ..
    $ colcon build --symlink-install
    
6. **Launch official docker Simulator**
    ```
    $ cd ~/docker_robocup/Simulation-OPL-TiaGO
    $ docker-compose -f docker-compose.tiago.yml up
    ```
    **Navigate to simulator [webvnc](http://localhost:3000) and press Play Button**
    
7. **Launch moveit and ros1_bridges docker**
    ```
    $ cd ~/docker_robocup/tmc_wrs_docker
    $ docker-compose -f docker-compose.nvidia.yml up
    ```
8. **Compile melodic deps. If is not your first time, skip this step.**
     ```
    $ docker ps
    ```
    Locate and copy the CONTAINER_ID of **ghcr.io/gentlebots/tmc_wrs_docker_rc_melodic_and_bridges:nvidia**
    
    ```
    $ docker exec -it CONTAINER_ID /bin/bash
    $ cd home/developer/robocup_melodic_ws/
    $ source /opt/ros/melodic/setup.bash 
    $ catkin_make
    ```
    Restart tmc_wrs_docker
    
9. **Launch the test**
    ```
    $ cd  ~/robocup_ws/
    $ source install/setup.bash
    $ ros2 launch clean_up clean_up_launch.py
    ```



# DEPRECATED

## Description:

This repository contains the necessary packages and resources to run the simulated tmc_wrs_gazebo world with TIAGo robot and its ROS2 bridges, allowing run your ROS2 Eloquent/Foxy software from your host.

## LXD Container

* Firt of all, you need to have installed LXD. There are multiples tutorials to install it, e.g. [this](https://www.linode.com/docs/guides/beginners-guide-to-lxd/), [this](https://www.digitalocean.com/community/tutorials/how-to-set-up-and-use-lxd-on-ubuntu-18-04) or [this - Spanish](https://www.adictosaltrabajo.com/2018/07/11/amaras-lxd-por-encima-de-todas-las-cosas/).

* Now, you have to download [the container](https://urjc-my.sharepoint.com/:u:/g/personal/jonatan_gines_urjc_es/EQ9010b24zBMhWB-UwSCJlcBIgf-SF-fBSjt8fElaWX01A?e=oR8d2E)[Updated 09-04-2021]. This is a *tar.gz* file.

* Import the image from the downloaded file:

  ```
  lxc image import 027a8a0d3a2f3a23bd753edc8645dc4ddb77d9477a889cdf2d30cb2ab48e7ccd.tar.gz --alias rc2021world
  ```
* Launch the lxd container from the imported image:

  ```
  lxc launch rc2021world robocup2021world
  ```

Now, you should have the container running. Check it typing ``lxc list``

### Change IP address

We need to change the container IP address because it should take part of the same subnet of the host.

* Create a new bridge interface:

  ```
  lxc network create lxdbr1
  ```

* Change the subnet of this new bridge:

  ```
  lxc network edit lxdbr1
  ```

  Locate the next line: ``ipv4.address: 10.72.119.1/24`` and change it by ``ipv4.address: 192.168.1.240/24``

  > NOTE: It is assumed that the host subnet is of type 192.168.1.X... If it is not of this way, put an IP that take part of your subnet. For example, if your PC is 192.168.0.33, you must put 192.168.0.240/24

* Now, we are going to attach our container to this new bridged interface and change the IP of the container. Don't worry about the interfaces id:

  ```
  lxc stop robocup2021world
  lxc network attach lxdbr1 robocup2021world eth0 eth0
  lxc config device set robocup2021world eth0 ipv4.address 192.168.1.241
  ```

* Also, is important to run the next command. Otherwise, ROS1 bridges will not can be launched:

  ```
  lxc config set robocup2021world security.nesting=true
  ```

### Creating a GUI Profile
To allow the access from your container to you NVIDIA card, you must to create a new lxd profile and assign it to your robocup container.
If it is not a NVIDIA card, you can use the profile given in [this link](https://blog.simos.info/how-to-easily-run-graphics-accelerated-gui-apps-in-lxd-containers-on-your-ubuntu-desktop/).

Copy this lines into a configuration txt file, for example ``lxd_gui_profile.txt``
```
config:
  environment.DISPLAY: :0
  environment.PULSE_SERVER: unix:/home/ubuntu/pulse-native
  nvidia.driver.capabilities: all
  nvidia.runtime: "true"
  user.user-data: |
    #cloud-config
    runcmd:
      - 'sed -i "s/; enable-shm = yes/enable-shm = no/g" /etc/pulse/client.conf'
    packages:
      - x11-apps
      - mesa-utils
      - pulseaudio
description: GUI LXD profile
devices:
  PASocket1:
    bind: container
    connect: unix:/run/user/1000/pulse/native
    listen: unix:/home/ubuntu/pulse-native
    security.gid: "1000"
    security.uid: "1000"
    uid: "1000"
    gid: "1000"
    mode: "0777"
    type: proxy
  X0:
    bind: container
    connect: unix:@/tmp/.X11-unix/X1
    listen: unix:@/tmp/.X11-unix/X0
    security.gid: "1000"
    security.uid: "1000"
    type: proxy
  mygpu:
    type: gpu
name: x11
used_by: []
```
  > If this gives you an error such as ``Error: Can't open display: :0`` change the line
  > ``connect: unix:@/tmp/.X11-unix/X1`` to ``connect: unix:@/tmp/.X11-unix/X0``


Then, create the profile in lxd:
```
lxc profile create gui
cat lxd_gui_profile.txt | lxc profile edit gui
```
Finally, add this profile to the robocup2021world instance and start it or restart it if it wasn't stopped - ``lxc list`` can be used to see its state.
```
lxc profile add robocup2021world gui
lxc restart robocup2021world
```
* At this moment, we can start our container:

  ```
  lxc start robocup2021world
  ```

* And enter to it:

  ```
  lxc exec robocup2021world -- su --login ubuntu
  ```

  > NOTE: I recommend to create an alias for this command


## Robocup referee set up

When you login in the container at the first time, you have to configure the score system.

### Set up the score widget

```
./catkin_ws/src/referee/tmc_gazebo_task_evaluators/scripts/setup_score_widget
```

## Run the Simulation


### Gazebo
From the *robocup2021world* container, run the next command:

```
roslaunch tiago_sim_robocup2021 tiago_sim_robocup2021.launch
```

It should run the GUI gazebo interface with the world.

### ros1_bridges

Open a new shell and login to *robocup2021world* container. Run the next command:

```
source .bashrc_bridges
ros2 launch ros1_bridge dedicated_bridges_launch.py
```

That's all! Your simulated TIAGo is ready to be controlled from ROS2.
