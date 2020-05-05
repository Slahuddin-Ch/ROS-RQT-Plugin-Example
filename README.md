# RQT-Plugin-Example

## Discription
RQT Plugin and a ROS Node communication

The RQT Plugin wait for ROS Node to become available and display its status if its available or not in its UI.

The ROS Node publish Fibonacci Series, one number at a time. The RQT Plugin calculates ratio of n'th to (n-1)th number and adds the ratio to a list.

When the RQT Plugin thinks that the number has become too large (your choice on how to define too large), It informs the ROS Node to restart from 1. 

The list of ratios in plugin updates in circular buffer manner from this point. The UI indicates this.



# FILES DISCRIPTION
-----------------------------------------------------------------------------------
1) The program consists of two parts qtros_plugin and a ros_node node.
2) qtros plugin is connected with a QT ui
3) both parts are independent and talk with each other using publisher/subsriber model on service chatter and chatter1   


1) qtros plugin
----------------
.
├── include
│   └── qtros
│       ├── main_window.hpp
│       └── qnode.hpp
│
├── resources
│   ├── images
│   │   └── icon.png
│   └── images.qrc
├── src
│   ├── main.cpp
│   ├── main_window.cpp
│   └── qnode.cpp
├── ui
│   └── main_window.ui
├── mainpage.dox
├── package.xml
├── CMakeLists.txt


2) A minimal ros node
---------------------
.
├── CMakeLists.txt
├── package.xml
└── src
    └── ros_node.cpp



# FEATURES
-----------------------------------------------------------------------------------
1) MAX_NUMBER can be changed durring runtime via UI.
2) all the logs for plugin are displayed in UI
3) ROS NODE (which publishes fibonacci series) status is displayed on UI
4) clear_output buttton on UI will clear the list output in display box
4) clear-logs button on UI will clear the log history list 

# HOW TO RUN PROJECT
-------------------
1) create a catkin workspace
2) add both packages in source folder (src)
3) build and make the workplace
4) execute roscore
5) execute qtros (rosrun qtros qtros)
6) execute ros_node (rosrun ros_node ros_node)


