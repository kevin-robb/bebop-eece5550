# bebop-eece5550

This repository itself will be treated as a ROS package. It will contain one or more nodes that are intended to be run on the turtlebot itself, and it will contain one or more nodes that will be run on the remote PC. The same package will be cloned to both machines for convenience. We will probably create two separate launch files, one that will be run on the robot and one that will be run on the remote PC.

To connect to the robot:
 - Make sure your machine and the robot are connected to the same network. If this is done, skip to the final bullet point.
 - Connect a display to the robot's HDMI port, connect a keyboard to a USB port, and power it on by flipping the switch on the OpenCR board. After logging in (username `ubuntu`, password `turtlebot`), the IP address should be printed to the screen with a bunch of other information. You can also check with the command `ifconfig`. 
 - If no IP prints, it is not connected to a network. You must edit the netplan file with the command `sudo nano ~/../../etc/netplan/50-cloud-init.yaml`. At the bottom of the file, the final two lines follow the format `WIFI_SSID:` and `password: WIFI_PASSWORD`. Replace the SSID and password with those for the network you plan to use. Save and exit the editor, then restart the robot with the command `sudo shutdown now`; after the green light on the raspberry pi stays off, flip the power switch off and back on to restart the pi. It should connect to the new network when it turns on, as long as the network is available.
 - The command `sudo netplan apply` is a good way to diagnose errors in the netplan file.
 - Once the robot and your computer are connected to the same network, you can connect to it with ssh, and the display/keyboard are no longer necessary for the robot. On linux, you can connect with the command `ssh ubuntu@IP_ADDRESS`, using the IP of the robot. On windows, you can connect by installing the application PuTTY. 
