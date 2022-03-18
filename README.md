# DevliveryHelper
By Marco Morales

[DeliveryHelper Portfolio Post](https://mmorales45.github.io/2022/02/11/DeliveryHelper/)

## Starting

1. Install MediaPipe
    - This will be important for detecting hand/poses.
    ```
    pip install mediapipe
    ```
2. Create the workspace and move into the directory.
    ```
    mkdir -p ~/DeliveryHelper/src
    cd ~/DeliveryHelper/src
    ```
3. Clone the DeliveryHelper repository. 
    ```
    git clone git@github.com:mmorales45/deliveryhelper.git
    ```
4.  Catkin make in the base of the workspace
    ```
    cd ../
    catkin_make
    ``` 

For installing the repositories for the Ridgeback and Sawyer, please click on the following links to get detailed instructions to setup those workspaces.

[Ridgeback](https://github.com/jimas95/nu_ridgeback/blob/master/nuridgeback_robot/launch/accessories.launch)

[Sawback](https://github.com/jimas95/sawback)

## Startup

To start the Ridgeback and arm follow the instructions below. Make sure to have the emergency E-STOP available at all times and keep at least one eye on the robot at all times.

1. Make sure the power cable is unplugged from any power source.

2. Press the back right button on the Ridgeback. This button is located by the desktop computer and is the right one between the two. 

3. On the left panel, assuming you're facing the desktop, turn on the power inverter. 

4. After pressing the inverter, wait for about 5 seconds before pressing the power button on the desktop to start up the Saywer arm. It may take some time to fully boot up.

5. Once some time has passed, the Ridgeback lights should be flashing different colors. Make sure the E-STOP is not active if the light are not flashing different colors. If the E-STOP is active, the lights on the ridgeback would flash only RED.

6. Press the left button, again between the pair closest to the desktop, on the Ridgeback. The lights should not flash anymore and instead the lights on the back are a constant red and the front are a constant white.

7. Use the controller to move the Ridgeback! Make sure to press the left bumper to move the Ridgeback. It is the dead mans switch. 

## Instructions to run files
For the ridgeback
1. `Connect to the HOSTNAME network on the Ridgeback`
    - This will allow the computer to connect to the Ridgeback.
2. `con_sawback`
    - This commands sets the computer's ROS_MASTER_URI to that of the Ridgeback.
    - bash function to complete the three commands at once. The ROS_IP is set the HOSTNAME to the laptop, yours will be different than the one below.
        - `export ROS_MASTER_URI=http://192.168.131.40:11311`
        - `export ROS_IP=10.42.0.171`
        - `unset ROS_HOSTNAME`
3. `sudo ip route add 192.168.131.0/24 via 10.42.0.1`	
    - Assign the computer's address to that of the Ridgeback.
4. `traceroute 192.168.131.40`	
    - Checks to see if the computer has established a connection to both the Ridgeback and Sawyer arm.
5. `ssh administrator@192.168.131.1`	
    - Allows remote access to the Ridgeback.
6. `systemctl stop ridgeback.service`	
    - Stops the base.launch and laser_slam.launch files.
7. `consawyer`	
    - This commands sets the computer's ROS_MASTER_URI to that of the Sawyer.
    - bash function to complete the following three commands at once. There will be no difference between laptops/users and is present on the Ridgeback.
        - `export ROS_MASTER_URI='http://192.168.131.40:11311'`
        - `export ROS_IP='192.168.131.1'`
        - `unset ROS_HOSTNAME`
8. `delrefs`
    - bash functions to kill saywer's ref state publishers and TFs
        - `rosnode kill /robot_ref_publisher`
        - `rosnode kill /ref_base_to_world`
        - `rosnode kill /base_to_world`
9. `source ~test_dir/Deliveryhelper/devel/setup.bash`	
    - Source the package directory. 
10. `rosrun intera_interface enable_robot.py -e`	
    - Enable the robot arm to be used.
    - Will need to run again if the E-STOP is pushed.
11. `roslaunch deliveryhelper launch_all.launch`
    - Launches all the necessary launch files and nodes such as the helper, or navigation node, and the manipulation node.

On the laptop, opens RVIZ
1. `con_saywer`
2. `cd ridge_ws`
3. `roslaunch nuridgeback_robot visualization.launch viz_lab:=true`
    - Launch the RVIZ configuration to see the arm and mobile base.

To see the camera view of what the robot is seeing on your laptop.
1. `source DeliveryHelper/devel/setup.bash`
2. `rosrun deliveryhelper perception.py`


To send a package or node to the Ridgeback, use below as an example.

`
rsync -av /home/mmoral45/DeliveryHelper/src/deliveryhelper/nodes/ administrator@192.168.131.1:/home/administrator/test_dir/DeliveryHelper/src/deliveryhelper/nodes/
`

To save a map 

`rosrun map_server map_saver -f marco_map  map:=/ridgeback/rtabmap/grid_map`

To open a map 

`rosrun map_server map_server marco_lab.yaml /map:=/ridgeback/rtabmap/grid_map1`

- Make sure to change the grid map on RVIZ to the new topic/grid_map1.

## TROUBLESHOOTING

Here are some common issue that could be ran into as you work with the Ridgeback.

1. Losing connection from the controller to the Ridgeback.

    -Open the right side panel (opposite panel from the power inverter) and connect the controller directly to an open USB port. Press the controller power button and options at the same time, and it should connect once again. 

    -If this fails, connect a monitor to the Ridgeback using a VGA connection. Make sure to have a keyboard and mouse on hand to operate on the Ridgeback. Once connected, use the keyboard to type in the Ridgeback password, then connect the mouse. With the mouse, go to the bluetooth devices on the Ridgeback and manually connect the PS4 controller by setting the controller to the pairing mode by again holding both the options and home button at the same time for a few seconds. 

2. Bumblebee camera fails to work or gives error messages when the launchfile is ran. 

    -Ensure the camera is not operating by trying to view a camera's view through a topic using RQT_IMAGE_VIEW. If the camera is indeed not working, disconnect the camera by unplugging the USB from the back of the camera then connecting it again.
