# Leader Switcher

![alt tag](https://raw.githubusercontent.com/Redshift92/leader_switcher/master/screens/simul_1.gif)

ROS node implementing MHA* based leader switching formation planning algorithm presented in:

  - Siddharth Swaminathan, Mike Phillips and Maxim Likhachev, "Planning for multi-agent teams with leader switching," Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2015.
    [Full PDF](https://www.cs.cmu.edu/~maxim/files/planforleader_icra15.pdf)

### Usage

    roslaunch leader_switcher leader_switcher.launch
    
Try different simulations varying algorithm parameters and maps.

###### parameters:
    
  - *ltc*: leader transfer cost
  - *wh*, *wa*: as described on paper
  - *wf*: keep formation weight
  - *cache*: *s* to save next simulation to cache, *y* to check if already in cache before running algorithm.

###### maps:

  - intuitive maps described through *.png* images placed in *maps* folder.

###### examples:

    roslaunch leader_switcher leader_switcher.launch wh:=20 map:=2
    roslaunch leader_switcher leader_switcher.launch wh:=50 wa:=2 map:=4 wf:=40
    roslaunch leader_switcher leader_switcher.launch wh:=20 map:=15 ltc:=500

### Dependencies

  - [plan2rviz](https://github.com/Redshift92/plan2rviz): formation planning algorithms visualizer

###### This project is licensed under the terms of the MIT license.
