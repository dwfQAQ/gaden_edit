#include "obstacle_avoidance/obstacle_avoidance.h"

ObstacleAvoidanceDrive::ObstacleAvoidanceDrive(){
    // initialize parameters
    escape_range = 60.0 * DEG2RAD;
    check_forward_dist = 0.3;
    check_side_dist = 0.25;

    current_pose = 0.0;
    prev_pose = 0.0;

    // motion command
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // subscribe laser and odometry information
    laser_scan_sub = nh.subscribe("scan", 10, &ObstacleAvoidanceDrive::laserScanMsgCallBack, this);
    odom_sub = nh.subscribe("odom", 10, &ObstacleAvoidanceDrive::odomMsgCallBack, this);
}

ObstacleAvoidanceDrive::~ObstacleAvoidanceDrive(){}

// update current robot pose
void ObstacleAvoidanceDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg){
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);

	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
                            msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
	current_pose = atan2(siny, cosy);
}

// update laser range data in 0 30 330
void ObstacleAvoidanceDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg){
    int scan_angle[3] = {0, 30, 330};
    for (int i = 0; i < 3; i++)
    {
        if(std::isinf(msg->ranges.at(scan_angle[i]))){
            scan_data[i] = msg->range_max;
        }else{
            scan_data[i] = msg->ranges.at(scan_angle[i]);
        }
    }
}

// publish moving command
void ObstacleAvoidanceDrive::updateCommandVel(double lin, double ang){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = lin;
    cmd_vel.angular.z = ang;
    cmd_vel_pub.publish(cmd_vel);
}

// state machine for obstacle avoidance
void ObstacleAvoidanceDrive::controlLoop(){
    static int state_num = 0;
    switch (state_num)
    {
    // get the next moving direction from laser data
    case GET_DIRECTION:
        if (scan_data[CENTER] > check_forward_dist)
        {
            if (scan_data[LEFT] < check_side_dist)
            {
                prev_pose = current_pose;
                state_num = RIGHT_TURN;
            }else if (scan_data[RIGHT] < check_side_dist)
            {
                prev_pose = current_pose;
                state_num = LEFT_TURN;
            }else{
                state_num = DRIVE_FORWARD;
            }
        }else{

            prev_pose = current_pose;
            state_num = RIGHT_TURN;
        }
        break;

    // go forward when no obstacle detected
    case DRIVE_FORWARD:
        updateCommandVel(LINEAR_VELOCITY, 0.0);
        state_num = GET_DIRECTION;
        break;

    // turn left or right regarding the obstacle position
    case RIGHT_TURN:
        if(fabs(prev_pose - current_pose) >= escape_range){
            
            state_num = GET_DIRECTION;
        }else
            updateCommandVel(0.0, -1 * ANGULAR_VELOCITY);
        break;

    case LEFT_TURN:
        if(fabs(prev_pose - current_pose) >= escape_range){
            state_num = GET_DIRECTION;
        }else
            updateCommandVel(0.0, ANGULAR_VELOCITY);
        break;
    
    default:
        state_num = GET_DIRECTION;
        break;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_avoidance");
    ObstacleAvoidanceDrive obs_avo;
    ros::Rate loop_rate(125);

    while(ros::ok()){
        obs_avo.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

