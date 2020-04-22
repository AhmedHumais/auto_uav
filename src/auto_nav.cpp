
#include <ros/ros.h>
#include <angles/angles.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/StreamRate.h>
#include "pid.h"

mavros_msgs::State current_state;
geometry_msgs::Point curr_pos, target_pos;
float Fs = 10;

bool detect_flag = false;
ros::Time last_det;

PID pid_Vx = PID(1/Fs, 10, -10, 0.3, 0, 0);
PID pid_Vy = PID(1/Fs, 10, -10, 0.5, 0, 0);
PID pid_Vz = PID(1/Fs, 7, -7, 0.4, 0, 0);
PID yaw_rt = PID(1/Fs, 3, -3, 0.3, 0, 0);


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = (*msg).pose.position;
}

void target_sub_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    last_det = (*msg).header.stamp;
    target_pos = (*msg).point;
    detect_flag = true;
}

void search_func(const ros::Publisher &pos_pub){
    ROS_INFO("Searching Started");
    ros::Rate rt(10);
    detect_flag = false;
    mavros_msgs::PositionTarget nav_cmd;
    nav_cmd.coordinate_frame= nav_cmd.FRAME_BODY_OFFSET_NED;
    nav_cmd.type_mask = 0b010111000111;
    nav_cmd.velocity.x = 0;
    nav_cmd.velocity.y = 0;
    nav_cmd.velocity.z = 0;
    ros::Time start_tm = ros::Time::now();
    ros::spinOnce();
    while(!detect_flag && !(ros::Time::now() - start_tm > ros::Duration(20.0))){
        nav_cmd.yaw_rate = 0.3;
        pos_pub.publish(nav_cmd);
        ros::spinOnce();
        rt.sleep();
    }
    nav_cmd.yaw_rate =0;
    pos_pub.publish(nav_cmd);
    ros::spinOnce();
    ROS_INFO("Searching Done");
}

void go_to_next_pos(const ros::Publisher &pos_pub, geometry_msgs::Point target_pos){
    mavros_msgs::PositionTarget nav_cmd;
    nav_cmd.coordinate_frame= nav_cmd.FRAME_LOCAL_NED;
    nav_cmd.type_mask = 0b110111111000;
    nav_cmd.position = target_pos;
    ros::Rate rt(10);
    float dist = 100;
    while(dist > 0.08 && !detect_flag){
        dist = sqrt(pow((target_pos.x-curr_pos.x), 2) + pow((target_pos.y-curr_pos.y), 2) + pow((target_pos.z-curr_pos.z), 2));
        ros::spinOnce();
    }
}

bool pop_balloon(const ros::Publisher &pos_pub){
    ros::Rate rt(10);
    mavros_msgs::PositionTarget Tar_Pos;
    Tar_Pos.coordinate_frame= Tar_Pos.FRAME_BODY_OFFSET_NED;
    Tar_Pos.type_mask = 0b010111000111;
    Tar_Pos.yaw_rate = 0.0;
    pid_Vy.change_P(0.2);
    while((ros::Time::now() - last_det) < ros::Duration(10)){
        if (detect_flag && target_pos.y < 5){
            Tar_Pos.velocity.x = pid_Vx.calculate2(target_pos.x);
            Tar_Pos.velocity.y = pid_Vy.calculate2(target_pos.y);
            Tar_Pos.velocity.z = pid_Vz.calculate2(target_pos.z);
            pos_pub.publish(Tar_Pos);
            detect_flag = false;
        }
        else{
            Tar_Pos.velocity.x = 0;
            Tar_Pos.velocity.y = 0;
            Tar_Pos.velocity.z = 0;
            pos_pub.publish(Tar_Pos);
        }
        if(target_pos.y > 5){
            ros::Time err_time = ros::Time::now();
           while(target_pos.y > 5){
               if (ros::Time::now() - err_time > ros::Duration(4)){
                   return false;
               }
               detect_flag = false;
               rt.sleep();
               ros::spinOnce();
           }
        }
        if ((ros::Time::now() - last_det) > ros::Duration(4)){
            Tar_Pos.velocity.x = 0;
            Tar_Pos.velocity.y = -0.3;
            Tar_Pos.velocity.z = 0;
            pos_pub.publish(Tar_Pos);
        }
        detect_flag = false;
        rt.sleep();
        ros::spinOnce();
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_nav_node");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    // ros::Subscriber target_body = nh.subscribe<geometry_msgs::PointStamped>
    //         ("target/body", 10, target_sub_cb);

    ros::Subscriber target_body = nh.subscribe<geometry_msgs::PointStamped>
            ("zed/objpos", 10, target_sub_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>
            ("mavros/set_stream_rate");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(Fs);
    float takeoff_height = 2.0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = takeoff_height;

    mavros_msgs::StreamRate stream_rate_cmd;
    stream_rate_cmd.request.stream_id = 6;
    stream_rate_cmd.request.message_rate = 10;
    stream_rate_cmd.request.on_off = 1;

    mavros_msgs::PositionTarget TarPos;
    TarPos.coordinate_frame= TarPos.FRAME_BODY_OFFSET_NED;
    TarPos.type_mask = 0b010111000111;

    geometry_msgs::Vector3 Vel;

    ros::Time last_request = ros::Time::now();
    bool takeoff_comp = false;

    stream_rate_client.call(stream_rate_cmd);
    while(ros::ok() && !takeoff_comp){
        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else if ( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
                if( takeoff_client.call(takeoff_cmd) && 
                    takeoff_cmd.response.success){
                        ROS_INFO("Taking Off");
                    }
            }
            last_request = ros::Time::now();
        } 
        else{
            if (curr_pos.z > (takeoff_height-0.05)){
                takeoff_comp = true;
            }

        }
        ros::spinOnce();
        rate.sleep();

    }
    while(ros::ok()){
        search_func(local_pos_pub);
        if (detect_flag){
            while((ros::Time::now() - last_det) < ros::Duration(2)){
                float x = target_pos.x;
                float y = target_pos.y;
                float y_ang = atan2(y, x);
                if ((abs(x) + abs(y)) < 3){
                    pop_balloon(local_pos_pub);
                }
                else{
                    if ( abs(angles::to_degrees(y_ang) - 90) < 10 ){
                        Vel.y = pid_Vy.calculate2(y);                                
                    }
                    else {                        
                        Vel.x = 0;
                        Vel.y = 0;
                        Vel.z = 0;
                    }
                    if (detect_flag){
                        TarPos.yaw_rate = yaw_rt.calculate(y_ang, angles::from_degrees(90));
                        Vel.z = pid_Vz.calculate2(target_pos.z);
                    }
                    TarPos.velocity = Vel;
                    local_pos_pub.publish(TarPos);
                    ROS_INFO("Published velocity");
                }
                detect_flag = false;
                ros::spinOnce();
                rate.sleep();
            }
        }
        else{
            mavros_msgs::CommandTOL land_cmd;
            land_client.call(land_cmd);
            break;
        }
    }
    return 0;
}
