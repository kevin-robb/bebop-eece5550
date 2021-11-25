# bebop-eece5550

This repository itself will be treated as a ROS package. It will contain one or more nodes that are intended to be run on the turtlebot itself, and it will contain one or more nodes that will be run on the remote PC. The same package will be cloned to both machines for convenience. We will probably create two separate launch files, one that will be run on the robot and one that will be run on the remote PC.

To connect to the robot:
 - Make sure your machine and the robot are connected to the same network. If this is done, skip to the final bullet point.
 - Connect a display to the robot's HDMI port, connect a keyboard to a USB port, and power it on by flipping the switch on the OpenCR board. After logging in (username `ubuntu`, password `turtlebot`), the IP address should be printed to the screen with a bunch of other information. You can also check with the command `ifconfig`. 
 - If no IP prints, it is not connected to a network. You must edit the netplan file with the command `sudo nano ~/../../etc/netplan/50-cloud-init.yaml`. At the bottom of the file, the final two lines follow the format `WIFI_SSID:` and `password: WIFI_PASSWORD`. Replace the SSID and password with those for the network you plan to use. Save and exit the editor, then restart the robot with the command `sudo shutdown now`; after the green light on the raspberry pi stays off, flip the power switch off and back on to restart the pi. It should connect to the new network when it turns on, as long as the network is available.
 - The command `sudo netplan apply` is a good way to diagnose errors in the netplan file.
 - Once the robot and your computer are connected to the same network, you can connect to it with ssh, and the display/keyboard are no longer necessary for the robot. On linux, you can connect with the command `ssh ubuntu@IP_ADDRESS`, using the IP of the robot. On windows, you can connect by installing the application PuTTY. 



## Question 1: Steps to generate a PGM and YAML file of the mapped region
### Starting Recording 
* Connect turtlebot3 and Host Computer to same WiFi network
* SSH into the Raspi: `ssh ubuntu@192.168.___.___` 
* New MASTER terminal: `roscore`
* New MASTER terminal: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
* Initiate sensors in HOST terminal: `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
* New MASTER terminal: `cd /<Bag File Destination>`
  * Start Bag file recording: `rosbag record -a`
###Stopping recording of map:
  * Stop Bag file recorder: `Ctrl+C`
    * Bag file is created in the working directory of the recorder terminal
    * Name of bag file: _`YYYY-MM-DD-Time.bag`_ 
  * Finish Trajectory and write as a pbstream file
    * `rosservice call /finish_trajectory 0`
    * `rosservice call /write_state "{filename: '<Destination path>/<Filename>.pbstream', include_unfinished_submaps: "false"}" `
  * Stop cartographer node on MASTER
  * Using asset_writer service to obtain PGM and YAML files
    * Navigate to file directory for pbstream and bag files 
    * `roslaunch cartographer_ros assets_writer_ros_map.launch bag_filenames:=<Bag File Path>.bag pose_graph_filename:=<pbstream File poath>.pbstream`

### Viewing recorded data
  * Check pbstream file data:
    * `roslaunch cartographer_ros visualize_pbstream.launch pbstream_filename:=<pbstream File Path>/<Filename>.pbstream`
  * Play Rosbag file: `rosbag play <FilePath>.bag`
  * Visualize created map in Rviz
    * `rosrun rviz rviz`
    * `rosrun map_server map_server <YAML File Path>/<filename>.bag_map.yaml`
    
  The image below is a mapping of NEU's Colab 130P in the Snell Library
![Generated Image](./Lab3Q1.bag_map.png)



##Potential Issues
Installation of Cartographer done using documentation
Anything works
In Abhinav's computer, python2 and python3 both were installed
But in case of final installation sometimes by default python2 is used.
In such a case
* `sudo apt install python-is-python3`
* `alias python=python3`

Saving to a pbstream file had some issues. The exact command I used:

*`rosservice call /write_state "{filename: '${HOME}/Desktop/NEU/EECE_5550/ROS_ASSIGNMENTS/LAB3_Setup/revo_lds.pbstream', include_unfinished_submaps: "false"}"`

