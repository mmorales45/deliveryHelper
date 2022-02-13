# DevliveryHelper
By Marco Morales

Instructions to run
For the ridgeback
1. `Connect to the HOSTNAME network on the ridgeback`
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
10. `source ~/Deliveryhelper/devel/setup.bash`	
11. `rosrun intera_interface enable_robot.py -e`	
12. `roslaunch deliveryhelper launch_all.launch`
    - Launches all the neccessary launch files

On the laptop, opens RVIZ
1. `con_saywer`
2. `cd ridge_ws`
3. `roslaunch nuridgeback_robot visualization.launch viz_lab:=true`

To send a package or node to the ridgeback

```
rsync -av /home/mmoral45/DeliveryHelper/src/deliveryhelper/nodes/ administrator@192.168.131.1:/home/administrator/test_dir/DeliveryHelper/src/deliveryhelper/nodes/
```

To save a map 

`rosrun map_server map_saver -f marco_map  map:=/ridgeback/rtabmap/grid_map`

To open a map 

`rosrun map_server map_server marco_lab.yaml /map:=/ridgeback/rtabmap/grid_map1`

- Make sure to change the grid map on RVIZ to the new topic.

