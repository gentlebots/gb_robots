# Robocup 2021 World

## Description:

This repository contains the necessary packages and resources to run the simulated tmc_wrs_gazebo world with TIAGo robot and its ROS2 bridges, allowing run your ROS2 Eloquent/Foxy software from your host.

## LXD Container

* Firt of all, you need to have installed LXD. There are multiples tutorials to install it, e.g. [this](https://www.linode.com/docs/guides/beginners-guide-to-lxd/), [this](https://www.digitalocean.com/community/tutorials/how-to-set-up-and-use-lxd-on-ubuntu-18-04) or [this - Spanish](https://www.adictosaltrabajo.com/2018/07/11/amaras-lxd-por-encima-de-todas-las-cosas/).

* Now, you have to download [the container](https://urjc-my.sharepoint.com/:u:/g/personal/jonatan_gines_urjc_es/ETgFyp-JSA1AhyEoEUfsKH8BDbBwIhZNBVzLJYs3PKSWWg?e=BqaSsv). This is a *tar.gz* file.

* Import the image from the downloaded file:

  ```
  lxc image import e4aec40b07128a30dd8d55ed6e3bdff85f3dc88ae39e32be45d836923f472eb9.tar.gz --alias rc2021world
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
```
lxc profile create gui
lxc profile edit gui
```
Copy and paste the below profile and save the changes.

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
    gid: "1000"
    listen: unix:/home/ubuntu/pulse-native
    mode: "0777"
    security.gid: "1000"
    security.uid: "1000"
    type: proxy
    uid: "1000"
  X0:
    bind: container
    connect: unix:@/tmp/.X11-unix/X1
    listen: unix:@/tmp/.X11-unix/X0
    security.gid: "1000"
    security.uid: "1000"
    type: proxy
  mygpu:
    type: gpu
name: gui

```

Attach the profile to the container:

```
lxc profile add robocup2021world gui
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
