# DevliveryHelper
By Marco Morales

## Startup

To start the Ridgeback and arm follow the instructions below.

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
2. `con_sawback`
    - bash function to complete the three commands at once. The ROS_IP is set the HOSTNAME to the laptop, yours will be different than the one below.
        - `export ROS_MASTER_URI=http://192.168.131.40:11311`
        - `export ROS_IP=10.42.0.171`
        - `unset ROS_HOSTNAME`
3. `sudo ip route add 192.168.131.0/24 via 10.42.0.1`	
4. `traceroute 192.168.131.40`	
5. `ssh administrator@192.168.131.1`	
6. `systemctl stop ridgeback.service`	
7. `consawyer`	
    - bash function to complete the following three commands at once. There will be no difference between laptops/users.
        - `export ROS_MASTER_URI='http://192.168.131.40:11311'`
        - `export ROS_IP='192.168.131.1'`
        - `unset ROS_HOSTNAME`
8. `delrefs`
    - bash functions to kill saywer's ref state publishers and TFs
        - `rosnode kill /robot_ref_publisher`
        - `rosnode kill /ref_base_to_world`
        - `rosnode kill /base_to_world`
9. `cd ~/sawback_ws`	
10. `source ~test_dir/Deliveryhelper/devel/setup.bash`	
11. `rosrun intera_interface enable_robot.py -e`	

    -Enable the robot arm to be used.
12. `roslaunch deliveryhelper launch_all.launch`
    - Launches all the neccessary launch files

On the laptop, opens RVIZ
1. `con_saywer`
2. `cd ridge_ws`
3. `roslaunch nuridgeback_robot visualization.launch viz_lab:=true`

To send a package or node to the Ridgeback, use below as an example.

`
rsync -av /home/mmoral45/DeliveryHelper/src/deliveryhelper/nodes/ administrator@192.168.131.1:/home/administrator/test_dir/DeliveryHelper/src/deliveryhelper/nodes/
`

To save a map 

`rosrun map_server map_saver -f marco_map  map:=/ridgeback/rtabmap/grid_map1`

To open a map 

`rosrun map_server map_server marco_lab.yaml /map:=/ridgeback/rtabmap/grid_map1`

- Make sure to change the grid map on RVIZ to the new topic.

## TROUBLESHOOTING

Here are some common issue that could be ran into as you work with the Ridgeback.

1. Losing connection from the controller to the Ridgeback.

    -Open the right side panel (opposite panel from the power inverter) and connect the controller directly to an open USB port. Press the controller power button and options at the same time, and it should connect once again. 

    -If this fails, connect a monitor to the Ridgeback using a VGA connection. Make sure to have a keyboard and mouse on hand to operate on the Ridgeback. Once connected, use the keyboard to type in the Ridgeback password, then connect the mouse. With the mouse, go to the bluetooth devices on the Ridgeback and manually connect the PS4 controller by setting the controller to the pairing mode by again holding both the options and home button at the same time for a few seconds. 

2. Bumblebee camera fails to work or gives error messages when the launchfile is ran. 

    -Ensure the camera is not operating by trying to view a camera's view through a topic using RQT_IMAGE_VIEW. If the camera is indeed not working, disconnect the camera by unplugging the USB from the back of the camera then connecting it again.
