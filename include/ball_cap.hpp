#include <ros/ros.h>
#include <angles/angles.h>

#include <std_msgs/Float32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>
#include "pid.h"

// #define STATIC_TARGET

mavros_msgs::State current_state;
geometry_msgs::Point curr_pos, target_pos;
float target_rel_vel = 0;
float Fs = 3;

bool detect_flag = false;
ros::Time last_det;

// PID pid_Vx = PID(1/Fs, 10, -10, 0.3, 0, 0);
PID pid_Vy = PID(1.0/Fs, 10, -10, 0.3, 0.0, 0.0);
PID pid_Vz = PID(1.0/Fs, 7, -7, 0.35, 0, 0);
PID yaw_rt = PID(1.0/Fs, 3, -3, 0.35, 0.02, 0);


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = (*msg).pose.position;
}

void target_vel_sub_cb(const std_msgs::Float32::ConstPtr& msg){
    target_rel_vel = (*msg).data;
}

void target_pos_sub_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    target_pos = (*msg).point;
    detect_flag = true;
    last_det = (*msg).header.stamp;
}

void go_to_next_pos(const ros::Publisher &pos_pub, geometry_msgs::Point go_pos){
    mavros_msgs::PositionTarget nav_cmd;
    nav_cmd.coordinate_frame= nav_cmd.FRAME_LOCAL_NED;
    nav_cmd.type_mask = 0b100111111000;
    nav_cmd.position = go_pos;
    nav_cmd.yaw = 0;
    pos_pub.publish(nav_cmd);
    ros::Rate rt(20);
    while(abs(go_pos.x-curr_pos.x) > 0.1 || abs(go_pos.y-curr_pos.y) > 0.1 || abs(go_pos.z-curr_pos.z) > 0.1){
        ros::spinOnce();
        rt.sleep();
    }
}

void search_func(const ros::Publisher &pos_pub, geometry_msgs::Point &search_pos){
    ROS_INFO("Searching Started");
    ros::Rate rt(10);
    detect_flag = false;
    go_to_next_pos(pos_pub, search_pos);
    mavros_msgs::PositionTarget nav_cmd;
    nav_cmd.coordinate_frame= nav_cmd.FRAME_BODY_OFFSET_NED;
    nav_cmd.type_mask = 0b010111111000;
    nav_cmd.position.x = 0;
    nav_cmd.position.y = 0;
    nav_cmd.position.z = 0;
    ros::Time start_tm = ros::Time::now();
    ros::Time turn_tm = ros::Time::now();
    float yrt = 0.31416;
    ros::spinOnce();
    while(!detect_flag && !(ros::Time::now() - start_tm > ros::Duration(40.0))){
        if(ros::Time::now() - turn_tm >= ros::Duration(10.0)){
             yrt = -1.0*yrt;
             turn_tm = ros::Time::now();
        }
        nav_cmd.yaw_rate = yrt;
        pos_pub.publish(nav_cmd);
        ros::spinOnce();
        rt.sleep();
    }
    nav_cmd.yaw_rate =0;
    pos_pub.publish(nav_cmd);
    ros::spinOnce();
    ROS_INFO("Searching Done");
}


void cap_ball(const ros::Publisher &pos_pub){
    ros::Rate rt(3);
    mavros_msgs::PositionTarget Tar_Pos;
    Tar_Pos.coordinate_frame= Tar_Pos.FRAME_BODY_OFFSET_NED;
    Tar_Pos.type_mask = 0b010111000111;
    Tar_Pos.velocity.x = 0.0;
    ros::spinOnce();
    while((ros::Time::now() - last_det) < ros::Duration(1)){
        // Tar_Pos.velocity.x = pid_Vz.calculate2(target_pos.x);
        if(detect_flag){
            detect_flag = false;
            Tar_Pos.velocity.y = pid_Vy.calculate2(target_pos.y);
            Tar_Pos.velocity.z = pid_Vz.calculate2(target_pos.z);
            // auto y_ang = atan2(target_pos.y, target_pos.x);
            // auto y_ang = target_pos.x;
            // Tar_Pos.yaw_rate = yaw_rt.calculate(y_ang, angles::from_degrees(90));
            Tar_Pos.yaw_rate = yaw_rt.calculate2(target_pos.x);

            #ifdef STATIC_TARGET
                pos_pub.publish(Tar_Pos);
            #endif
            rt.sleep();
        }
        ros::spinOnce();
    }
}
