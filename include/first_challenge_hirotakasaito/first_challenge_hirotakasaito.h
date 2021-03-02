#ifndef FIRST_CHALLENGE_HIROTAKASAITO_H
#define FIRST_CHALLENGE_HIROTAKASAITO_H
#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

class RoombaController
{
public:
    RoombaController();
    void process();
    void move_roomba();

private:
    void pose_callback(const nav_msgs::Odometry::ConstPtr &);
    void pose_callback_lider(const sensor_msgs::LaserScan::ConstPtr &);

    int hz;
    int straight_count = 0;
    int scan_cout = 0.0;
    float terminal_vel_z;
    float goal;
    float init_pose_x = 0.0;
    float init_pose_y = 0.0;
    float abs_current_theta = 0.0;
    float distance = 0.0;
    ros::Publisher pub_cmd_vel;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_pose;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    nav_msgs::Odometry current_pose;
    nav_msgs::Odometry quaternion;
    sensor_msgs::LaserScan laserscan;
};

#endif


