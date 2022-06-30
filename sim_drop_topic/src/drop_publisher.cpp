#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sim_drop_topic/coorMsg.h>
#include <nav_msgs/Odometry.h>

ros::Time current_time, last_time;
ros::Publisher pub;
double delta_odom = 0;
double delta_time = 0;

void doMsg(const nav_msgs::Odometry::ConstPtr& msg_odom){
    current_time = ros::Time::now();
    delta_time = (current_time - last_time).toSec();
    // add deltaO = v * deltaT
    if(delta_time > 0.01){
        double vx = msg_odom->twist.twist.linear.x;
        delta_odom += vx * delta_time;
    }
    

    
    if(delta_odom > 2.0){
        sim_drop_topic::coorMsg msg;
        msg.x = msg_odom->pose.pose.position.x;
        msg.y = msg_odom->pose.pose.position.y;
        pub.publish(msg);
        ROS_INFO("the new drop is in (%f, %f)", msg.x, msg.y);
        delta_odom = 0;
    }

    last_time = current_time;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    current_time = ros::Time::now();
    last_time = current_time;
    // ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    pub = nh.advertise<sim_drop_topic::coorMsg>("chatter", 10);

    ros::Duration(1.0).sleep();
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, doMsg);
    ros::spin();
    
    return 0;
}
