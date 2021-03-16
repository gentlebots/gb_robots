# Robocup 2021 World

## Description:

This repository contains the necessary packages and resources to run the simulated tmc_wrs_gazebo world with TIAGo robot and its ROS2 bridges, allowing run your ROS2 Eloquent/Foxy software from your host.

## LXD Container

* Firt of all, you need to have installed LXD. There are multiples tutorials to install it, e.g. [this](https://www.linode.com/docs/guides/beginners-guide-to-lxd/), [this](https://www.digitalocean.com/community/tutorials/how-to-set-up-and-use-lxd-on-ubuntu-18-04) or [this - Spanish](https://www.adictosaltrabajo.com/2018/07/11/amaras-lxd-por-encima-de-todas-las-cosas/).

* Now, you have to download [the container](https://urjc-my.sharepoint.com/:u:/g/personal/jonatan_gines_urjc_es/ER01wTemVpJBmqno6zkP1AQBure4hsUgyQBL6w7I-ZbUIQ?e=fHE51X). This is a *tar.gz* file.

* Import the image from the downloaded file:

  ```
  lxc image import 42b0ec65a03cfa6fcb5acfe94bb8ddaa6d8cfc91da47fa4487dad31a1f8c7bd3.tar.gz --alias rc2021world
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
[Follow this guide to create the profile](https://blog.simos.info/how-to-easily-run-graphics-accelerated-gui-apps-in-lxd-containers-on-your-ubuntu-desktop/)=

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
