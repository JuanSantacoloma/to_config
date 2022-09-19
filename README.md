# Step by step
1. Connect the 2 batteries needed for the robot to function. Ask for help if you never used these LiPo batteries.
2. Connect the power supply of the computer (intel NUC) to the wall and turn it on.
3. Turn on the robot using the on/off button.
4. Make sure the keyboard and the screen are connected to the computer (intel NUC).
5. After the ubuntu loading procces, log in. Ask Juan Camilo for the creedentials.
6. In the terminal use the command:

    `startx`

This command will run i3, this is a windows manager for ubuntu. The computer in the robot runs ubuntu server.
7. For i3 you need to know 2 basic commands
* This open a new terminal 

    `cntrl + enter`

* This opens a menu in the top, this let's you search the programs installed in the computer.

    `cntrl + d`
    
8. Now, finally you can run the commands:
* The first command activate the CAN BUS communication.

    `sudo ip link set can0 type can bitrate 100000`

* The next command runs the ROS package for CAN communication, this creates 2 important topics: \send_messages and \recive_messages

   `rosrun socketcan_bridge socketcan_bridge_node`

* This full command runs the Champ package with the configuration for the quadruped robot TO, with Rviz on, hardware_connected is set to true wich means that you have motors and connection allowed and finally the parameter publish_joint_control, this is set to False because we want to move the joints in manual mode first. 

    `roslaunch to_config bringup.launch rviz:=true hardware_connected:=true publish_joint_control:=false`

* joint_gui.launch allows to have a graphic interface with sliders for every single motor/joint in the robot.

    `roslaunch champ_bringup joints_gui.launch`

* Finally the code where we send commands.

    `rosrun to_config interface_node.py`
