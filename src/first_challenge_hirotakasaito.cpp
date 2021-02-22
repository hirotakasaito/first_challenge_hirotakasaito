#include "first_challenge_hirotakasaito/first_challenge_hirotakasaito.h"

RoombaController::RoombaController():private_nh("~")
{
    private_nh.param("hz",hz,{10});

    sub_pose = nh.subscribe("roomba/odometry",10,&RoombaController::pose_callback,this);
    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control",1);
}

void RoombaController::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = *msg;

}


void RoombaController::go_straight()
{
    if(init_pose_x == 0)
    {
        init_pose_x = current_pose.pose.pose.position.x;
        init_pose_y = current_pose.pose.pose.position.y;
    }
    if(distance >= 1)
    {
        cmd_vel.cntl.linear.x = 0;
        cmd_vel.cntl.angular.z = 1.0;
    }
    else
    {
        cmd_vel.cntl.linear.x = 0.3;
    }

    distance = sqrt((current_pose.pose.pose.position.x - init_pose_x) * (current_pose.pose.pose.position.x - init_pose_x) + (current_pose.pose.pose.position.y - init_pose_y) * (current_pose.pose.pose.position.y - init_pose_y));
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaController::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        go_straight();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"first_challenge_hirotakasaito");
    RoombaController roomba_controller;
    roomba_controller.process();
    return 0;
}

