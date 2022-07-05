#ifndef OBS_AVOID_H_
#define OBS_AVOID_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sim_drop_topic/coorMsg.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT 1
#define RIGHT 2

#define LINEAR_VELOCITY 0.25
#define ANGULAR_VELOCITY 1.5

#define GET_DIRECTION 0
#define DRIVE_FORWARD 1
#define RIGHT_TURN 2
#define LEFT_TURN 3

class ObstacleAvoidanceDrive{
public:
    ObstacleAvoidanceDrive();
    ~ObstacleAvoidanceDrive();
    void controlLoop();

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_scan_sub;
    ros::Subscriber odom_sub;
    
    double pose_x;
    double pose_y;
    double escape_range;
    double check_forward_dist;
    double check_side_dist;
    double current_pose;
    double prev_pose;
    double scan_data[3] = {0.0, 0.0, 0.0};

    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void updateCommandVel(double lin, double ang);
};
#endif