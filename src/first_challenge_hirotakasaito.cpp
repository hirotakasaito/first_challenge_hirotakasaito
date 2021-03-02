#include "first_challenge_hirotakasaito/first_challenge_hirotakasaito.h"
#include <tf/transform_datatypes.h>
#include <stdio.h>

RoombaController::RoombaController():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("goal",goal,{1});
    private_nh.param("terminal_vel_z",terminal_vel_z,{0.1});

    sub_pose = nh.subscribe("roomba/odometry",10,&RoombaController::pose_callback,this);
    sub_scan = nh.subscribe("scan",10,&RoombaController::pose_callback_lider,this);

    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control",1);
}

void RoombaController::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose = *msg;
}

void RoombaController::pose_callback_lider(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laserscan = *msg;
}

void RoombaController::move_roomba()
{
    tf::Quaternion q(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(init_pose_x == 0)
    {
        init_pose_x = current_pose.pose.pose.position.x;
        init_pose_y = current_pose.pose.pose.position.y;
    }
    if(distance >= goal)
    {
        if(abs_current_theta >= (2*M_PI - 0.1))
        {
            cmd_vel.cntl.angular.z = 0.0;
            cmd_vel.cntl.linear.x = 0.3;
            if(straight_count == 1)
            {
                distance = 0.0;
                init_pose_x = current_pose.pose.pose.position.x;
                init_pose_y = current_pose.pose.pose.position.y;
                straight_count = 2;
            }
            if(distance >= goal)
            {
                //ここでlidarの処理
                cmd_vel.cntl.linear.x = 0;
            }
            distance = sqrt((current_pose.pose.pose.position.x - init_pose_x) * (current_pose.pose.pose.position.x - init_pose_x) + (current_pose.pose.pose.position.y - init_pose_y) * (current_pose.pose.pose.position.y - init_pose_y));
        cmd_vel.cntl.linear.x = 0;
        }
        else
        {
            //回転速度を調整する、最初は早く、目標に近づくほど遅くする
            cmd_vel.cntl.linear.x = 0.0;
            cmd_vel.cntl.angular.z = 0.2*round((2*M_PI - 0.01) - abs_current_theta);
            //0にならないようにする
            if(cmd_vel.cntl.angular.z == 0)
            {
                cmd_vel.cntl.linear.x = 0.0;
                cmd_vel.cntl.angular.z = terminal_vel_z;
            }
            //0から2πの範囲に変換する
            if(abs_current_theta >= (M_PI - 0.1))
            {
                abs_current_theta = 2*M_PI - fabs(yaw);
            }
            else
            {
                abs_current_theta = fabs(yaw);
            }
        }
    }
    else
    {
        cmd_vel.cntl.linear.x = 0.3;

        if(init_pose_x == 0)
        {
            init_pose_x = current_pose.pose.pose.position.x;
            init_pose_y = current_pose.pose.pose.position.y;
            straight_count = 1;
        }
        distance = sqrt((current_pose.pose.pose.position.x - init_pose_x) * (current_pose.pose.pose.position.x - init_pose_x) + (current_pose.pose.pose.position.y - init_pose_y) * (current_pose.pose.pose.position.y - init_pose_y));
    }
    ROS_INFO_STREAM(laserscan.range_min);
    ROS_INFO_STREAM(laserscan.range_max);
    ROS_INFO_STREAM(laserscan.scan_time);
    ROS_INFO_STREAM(laserscan.range_min);
    ROS_INFO_STREAM(laserscan.angle_min);
    ROS_INFO_STREAM(laserscan.angle_max);
    cmd_vel.mode = 11;//ルンバのモード
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaController::process()
{
    ros::Rate loop_rate(hz);

    while(ros::ok())
    {
        move_roomba();
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

