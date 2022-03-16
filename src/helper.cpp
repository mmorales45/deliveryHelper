/// \file
/// \brief Makes the ridgeback follow a person
///
#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <std_msgs/String.h>

// for this node
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

//from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

class helper
{
    public:
        helper() 
        {
            __lin_x = 0;
            __lin_y = 0;
            __lin_z = 0;
            __ang_x = 0;
            __ang_y = 0;
            __ang_z = 0;
            rotate_z = 0;

            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            ridgeback_forward = nh.advertiseService("ridgeback_forward", &helper::forward_callback, this);
            ridgeback_backward = nh.advertiseService("ridgeback_backward", &helper::backward_callback, this);
            ridgeback_pause = nh.advertiseService("ridgeback_pause", &helper::pause_callback, this);
            move_to_tag = nh.advertiseService("move_to_tag", &helper::move_to_tag_callback, this);
            follow_ridgeback = nh.advertiseService("follow_ridgeback", &helper::follow_ridgeback_callback, this);
            delivery_sub = nh.subscribe("/delivery_state",10,&helper::delivery_callback,this);


            ros::Duration(2).sleep();
            ROS_INFO("Ready!");
            timer = nh.createTimer(ros::Duration(1/100), &helper::main_loop, this);
            
        }
        /// \brief subscribe to delivery state to get the current state of the robot.
        ///
        void delivery_callback(const std_msgs::String & msg)
        {
            mode = msg.data;
        }

        /// \brief Makes the robot go forward using cmd_vel
        ///
        bool forward_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            __lin_x = 0.5;
            return true;
        }

        /// \brief Makes the robot go backward using cmd_vel
        ///
        bool backward_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            __lin_x = -0.5;
            return true;
        }

        /// \brief Makes the robot stop using cmd_vel
        ///
        bool pause_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            __lin_x = 0;
            return true;
        }

        /// \brief Makes the robot go to an april tag
        ///
        bool move_to_tag_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {            
            MoveBaseClient ac("ridgeback/move_base", true);

            while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            ROS_INFO("Pass the wait for server while loop");

            goal.target_pose.header.frame_id = "map";
            // goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = april_x - 1.5;
            // goal.target_pose.pose.position.y = april_y;
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            ROS_INFO("Sent goal");
            ac.waitForResult(); 
            ROS_INFO("Waited for result");
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray, the base moved 1 meter forward");
            }
            else {
                ROS_INFO("The base failed to move forward 1 meter for some reason");
            }
            return true;
        }
        /// \brief Service to make the robot follow person
        ///
        bool follow_ridgeback_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {            
            mode = "Follow";
            return true;
        }
        /// \brief Stop the robot from following
        ///
        bool stop_follow_ridgeback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {            
            mode = "Empty";
            return true;
        }
        /// \brief Find tfs from base to tag1
        ///
        void find_tf()
        {
            static tf2_ros::Buffer tfBuffer;
            static geometry_msgs::TransformStamped transformStamped,transformStamped1;
            transformStamped1 = tfBuffer.lookupTransform("base_link", "tag_1", ros::Time(0));
            base_x = transformStamped1.transform.translation.x;
            base_y = transformStamped1.transform.translation.y;
            base_z = transformStamped1.transform.translation.z;
            base_x_orien = transformStamped1.transform.rotation.x;
            base_y_orien = transformStamped1.transform.rotation.y;
            base_z_orien = transformStamped1.transform.rotation.z;
            base_w_orien = transformStamped1.transform.rotation.w;
        }

        /// \brief Main Loop
        ///
        void main_loop(const ros::TimerEvent &)
        {
            velocity.linear.x = __lin_x;
            velocity.linear.y = __lin_y;
            velocity.linear.z = __lin_z;
            velocity.angular.x = __ang_x;
            velocity.angular.x = __ang_y;
            velocity.angular.x = __ang_z;

            rotate_twist.linear.x = 0;
            rotate_twist.linear.y = 0;
            rotate_twist.linear.z = 0;
            rotate_twist.angular.x = 0;
            rotate_twist.angular.x = 0;
            rotate_twist.angular.x = __ang_z;
            

            static tf2_ros::Buffer tfBuffer;
            static geometry_msgs::TransformStamped transformStamped,transformStamped1,transformStamped2;
            static tf2_ros::TransformListener tfListener(tfBuffer);

            try{
                MoveBaseClient ac("ridgeback/move_base", true);
                while(!ac.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
                ROS_INFO("Pass the wait for server while loop");

                transformStamped = tfBuffer.lookupTransform("map", "tag_1", ros::Time(0));
                // q.setRPY(0, 0, target.orientation);
                // ROS_INFO("The x translation %f", transformStamped.transform.translation.x);
                // transform between frames to be used for finding transforms between robot and human.
                april_x = transformStamped.transform.translation.x;
                april_y = transformStamped.transform.translation.y;
                april_z = transformStamped.transform.translation.z;
                april_x_orien = transformStamped.transform.rotation.x;
                april_y_orien = transformStamped.transform.rotation.y;
                april_z_orien = transformStamped.transform.rotation.z;
                april_w_orien = transformStamped.transform.rotation.w;
                
                transformStamped1 = tfBuffer.lookupTransform("base_link", "tag_1", ros::Time(0));
                base_x = transformStamped1.transform.translation.x;
                base_y = transformStamped1.transform.translation.y;
                base_z = transformStamped1.transform.translation.z;
                base_x_orien = transformStamped1.transform.rotation.x;
                base_y_orien = transformStamped1.transform.rotation.y;
                base_z_orien = transformStamped1.transform.rotation.z;
                base_w_orien = transformStamped1.transform.rotation.w;

                transformStamped2 = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
                world_base_x = transformStamped2.transform.translation.x;
                world_base_y = transformStamped2.transform.translation.y;
                world_base_z = transformStamped2.transform.translation.z;
                world_base_x_orien = transformStamped2.transform.rotation.x;
                world_base_y_orien = transformStamped2.transform.rotation.y;
                world_base_z_orien = transformStamped2.transform.rotation.z;
                world_base_w_orien = transformStamped2.transform.rotation.w;

                double theta_base_tag = atan2(base_y,base_x);

                if (base_x<4.0){
                //Create quaternions and place in a 3x3 matrix.
                tf::Quaternion q(base_x_orien, base_y_orien, base_z_orien, base_w_orien); //original 
                tf::Matrix3x3 m(q);

                tf::Quaternion q1(april_x_orien, april_y_orien, april_z_orien, april_w_orien); //original 
                tf::Matrix3x3 m1(q1);

                tf::Quaternion q2(world_base_x_orien, world_base_y_orien, world_base_z_orien, world_base_w_orien); //original 
                tf::Matrix3x3 m2(q2);
                //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
                //create RPY from quaternion
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                double roll1, pitch1, yaw1;
                m1.getRPY(roll1, pitch1, yaw1);
                
                double roll2, pitch2, yaw2;
                m2.getRPY(roll2, pitch2, yaw2);


                // ROS_WARN("[x: %f, y: %f, z: %f, x_orien: %f,y_orien: %f,z_orien: %f,w_orien: %f, roll: %f, pitch: %f,yaw: %f] ", 
                //     april_x,april_y,april_z,april_x_orien, april_y_orien, april_z_orien, april_w_orien,roll,pitch,yaw);
                ROS_WARN("[x: %f, y: %f, z: %f", 
                    base_x,base_y,base_z);
                //maybe use ROS_INFO_STREAM?
                // ROS_ERROR("The z translation %f", transformStamped.transform.translation.z);

                ///new as of 1/27/2021
                //enable robot to continously follow a person.
                if (mode == "Follow")
                {
                    goal.target_pose.header.frame_id = "map";
                    // goal.target_pose.header.frame_id = "base_link";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal.target_pose.pose.position.x = april_x - 0.25;
                    goal.target_pose.pose.position.y = april_y;
                    // goal.target_pose.pose.orientation.w = 1.0;
                    yaw1 = yaw1 + 3.14/2;

                    tf2::Quaternion q_test;
                    q_test.setRPY(0, 0, yaw1);
                    q_test.normalize();
                    geometry_msgs::Quaternion quad_msg;
                    tf2::convert(q_test, goal.target_pose.pose.orientation);
                    // goal.target_pose.pose.orientation = quad_msg;

                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);
                    ROS_INFO("Sent goal");
                    ros::Duration(1.0).sleep();


                }
                else{
                    //if not follow, cancel all the goals.
                    ac.cancelAllGoals();
                }
                }
            }

            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_INFO("Cant find transform!");
                ros::Duration(1.0).sleep();
            }
            
            
        }
        
    private:
    //private variables to be used in the class.
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::ServiceServer ridgeback_forward;
    ros::ServiceServer ridgeback_backward;
    ros::ServiceServer ridgeback_pause;
    ros::ServiceServer move_to_tag;
    ros::ServiceServer follow_ridgeback;
    ros::Subscriber delivery_sub;

    geometry_msgs::Twist velocity;
    geometry_msgs::Twist rotate_twist;

    ros::Timer timer;   
    move_base_msgs::MoveBaseGoal goal;
    double __lin_x;
    double __lin_y;
    double __lin_z;
    double __ang_x;
    double __ang_y;
    double __ang_z;
    
    double april_x;
    double april_y;
    double april_z;
    tf2::Quaternion q;
    std::string mode = "Empty";

    double april_x_orien;
    double april_y_orien;
    double april_z_orien;
    double april_w_orien;

    double base_x;
    double base_y;
    double base_z;
    double base_x_orien;
    double base_y_orien;
    double base_z_orien;
    double base_w_orien;

    double world_base_x;
    double world_base_y;
    double world_base_z;
    double world_base_x_orien;
    double world_base_y_orien;
    double world_base_z_orien;
    double world_base_w_orien;

    double rotate_z;
    
};




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "helper");
    helper node;
    ros::spin();
    return 0;
}