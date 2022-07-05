#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sim_drop_topic/coorMsg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

ros::Time current_time, last_time, prev_time;
ros::Publisher pub;
double delta_odom = 0;
bool drop_flag;
double pose_x;
double pose_y;
double check_forward_dist = 0.3;
double check_side_dist = 0.25;

void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg_odom){
    pose_x = msg_odom->pose.pose.position.x;
    pose_y = msg_odom->pose.pose.position.y;
    current_time = ros::Time::now();

    double delta_time = (current_time - last_time).toSec();
    if(delta_time > 0.1){
        double vx = msg_odom->twist.twist.linear.x;
        delta_odom += vx * delta_time;
        last_time = current_time;
    }
    
    if(delta_odom > 2.0){
        sim_drop_topic::coorMsg msg;
        msg.type = "ethanol";
        msg.x = pose_x;
        msg.y = pose_y;
        
        pub.publish(msg);
        ROS_INFO("The new ethanol drop is in (%f, %f)", msg.x, msg.y);
        delta_odom = 0;
    }
    
}

void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg){
    current_time = ros::Time::now();
    if(current_time - prev_time > ros::Duration(2)){
        drop_flag = true;
        prev_time = current_time;
    }

    int scan_angle[3] = {0, 30, 330};
    double scan_data[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++)
    {
        if(std::isinf(msg->ranges.at(scan_angle[i]))){
            scan_data[i] = msg->range_max;
        }else{
            scan_data[i] = msg->ranges.at(scan_angle[i]);
        }
    }

    if(drop_flag && (scan_data[0] < check_side_dist || scan_data[1] < check_forward_dist || scan_data[2] < check_side_dist)){
        sim_drop_topic::coorMsg cmsg;
        cmsg.type = "ammonia";
        cmsg.x = pose_x;
        cmsg.y = pose_y;
        pub.publish(cmsg);
        ROS_INFO("The new ammonia drop is in (%f, %f)", cmsg.x, cmsg.y);
        drop_flag = false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dispenser");
    ros::NodeHandle nh;
    current_time = ros::Time::now();
    last_time = current_time;
    pub = nh.advertise<sim_drop_topic::coorMsg>("drop_coor", 10);

    ros::Duration(1.0).sleep();
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, odomCallBack);
    ros::Subscriber laser_scan_sub = nh.subscribe("scan", 10, laserScanMsgCallBack);
    ros::spin();
    
    return 0;
}
