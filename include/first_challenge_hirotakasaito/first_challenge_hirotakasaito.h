#ifndef FIRST_CHALLENGE_HIROTAKASAITO_H
#define FIRST_CHALLENGE_HIROTAKASAITO_H
#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"

class RoombaController
{
public:
    RoombaController();
    void process();
    void go_straight();

private:
    void pose_callback(const nav_msgs::Odometry::ConstPtr &);

    int hz;
    float init_pose_x = 0.0;
    float init_pose_y = 0.0;
    float distance = 0.0;
    ros::Publisher pub_cmd_vel;
    ros::Subscriber sub_pose;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    nav_msgs::Odometry current_pose;
    nav_msgs::Odometry quaternion;
};

#endif


