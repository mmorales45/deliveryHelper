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

// for this node
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
            // MoveBaseClient ac("ridgeback/move_base", true);

            // while(!ac.waitForServer(ros::Duration(5.0))){
            //     ROS_INFO("Waiting for the move_base action server to come up");
            // }
            // ROS_INFO("Pass the wait for server while loop");

            ////works

            // goal.target_pose.header.frame_id = "map";
            // // goal.target_pose.header.frame_id = "base_link";
            // goal.target_pose.header.stamp = ros::Time::now();

            // goal.target_pose.pose.position.x = 3.0;
            // // goal.target_pose.pose.position.y = april_y;
            // goal.target_pose.pose.orientation.w = 1.0;

            // ROS_INFO("Sending goal");
            // ac.sendGoal(goal);
            // ROS_INFO("Sent goal");
            // ac.waitForResult(); 
            // ROS_INFO("Waited for result");
            // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //     ROS_INFO("Hooray, the base moved 1 meter forward");
            // }
            // else {
            //     ROS_INFO("The base failed to move forward 1 meter for some reason");
            // }

            ros::Duration(2).sleep();
            ROS_INFO("Ready!");
            timer = nh.createTimer(ros::Duration(1/100), &helper::main_loop, this);
            
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

        bool follow_ridgeback_callback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {            
            mode = "Follow";
            return true;
        }
        bool stop_follow_ridgeback(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {            
            mode = "Empty";
            return true;
        }

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
            static geometry_msgs::TransformStamped transformStamped,transformStamped1;
            static tf2_ros::TransformListener tfListener(tfBuffer);

            try{
                transformStamped = tfBuffer.lookupTransform("map", "tag_1", ros::Time(0));
                // q.setRPY(0, 0, target.orientation);
                // ROS_INFO("The x translation %f", transformStamped.transform.translation.x);
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

                tf::Quaternion q(base_x_orien, base_y_orien, base_z_orien, base_w_orien); //original 
                tf::Matrix3x3 m(q);
                //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                ROS_WARN("[x: %f, y: %f, z: %f, x_orien: %f,y_orien: %f,z_orien: %f,w_orien: %f, roll: %f, pitch: %f,yaw: %f] ", 
                    april_x,april_y,april_z,april_x_orien, april_y_orien, april_z_orien, april_w_orien,roll,pitch,yaw);
                ROS_WARN("[x: %f, y: %f, z: %f", 
                    base_x,base_y,base_z);
                //maybe use ROS_INFO_STREAM?
                // ROS_ERROR("The z translation %f", transformStamped.transform.translation.z);

                ///new as of 1/27/2021
                if (mode == "Follow")
                {
                    MoveBaseClient ac("ridgeback/move_base", true);
                    if(base_y<-0.2)
                    {
                        ac.cancelAllGoals();
                        
                        rotate_twist.angular.z = -0.2;
                        ROS_WARN("BEfore PUBLISHER");
                        cmd_vel_pub.publish(rotate_twist);
                        ROS_WARN("after PUBLISHER");
                        ros::Duration(0.5).sleep();
                        rotate_twist.angular.z = 0.0;
                        cmd_vel_pub.publish(rotate_twist);
                    }

                    if(base_y>0.2)
                    {
                        ac.cancelAllGoals();
                        rotate_twist.angular.z = 0.2;
                        ROS_WARN("BEfore PUBLISHER");
                        cmd_vel_pub.publish(rotate_twist);
                        ROS_WARN("after PUBLISHER");
                        ros::Duration(0.5).sleep();
                        rotate_twist.angular.z = 0.0;
                        cmd_vel_pub.publish(rotate_twist);
                    }


                    while(!ac.waitForServer(ros::Duration(5.0))){
                        ROS_INFO("Waiting for the move_base action server to come up");
                    }
                    ROS_INFO("Pass the wait for server while loop");

                    goal.target_pose.header.frame_id = "map";
                    // goal.target_pose.header.frame_id = "base_link";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal.target_pose.pose.position.x = april_x - 1.0;
                    // goal.target_pose.pose.position.y = april_y;
                    // goal.target_pose.pose.orientation.w = 1.0;
                    yaw = yaw + 3.14/2;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    q.normalize();
                    geometry_msgs::Quaternion quad_msg;
                    quad_msg = tf2::toMsg(q);
                    goal.target_pose.pose.orientation = quad_msg;

                    ROS_INFO("Sending goal");
                    ac.sendGoal(goal);
                    ROS_INFO("Sent goal");
                    ros::Duration(1.0).sleep();


                }
            }

            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_INFO("Cant find transform!");
                ros::Duration(1.0).sleep();
            }
            
            
        }
        
    private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::ServiceServer ridgeback_forward;
    ros::ServiceServer ridgeback_backward;
    ros::ServiceServer ridgeback_pause;
    ros::ServiceServer move_to_tag;
    ros::ServiceServer follow_ridgeback;

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

    double rotate_z;
    
};




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "helper");
    helper node;
    ros::spin();
    return 0;
}