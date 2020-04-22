#include <ros/ros.h>
#include "ball_cap.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_cap_node");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("blob/pos", 10, target_pos_sub_cb);
    // ros::Subscriber target_vel_sub = nh.subscribe<std_msgs::Float32>
    //         ("target/rel_vel", 10, target_vel_sub_cb);


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
    float takeoff_height = 5.0;

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
            if (curr_pos.z > (takeoff_height-0.15)){
                takeoff_comp = true;
            }

        }
        ros::spinOnce();
        rate.sleep();

    }
    while(ros::ok()){
        geometry_msgs::Point search_pos;
        search_pos.x = 0; search_pos.y = 1; search_pos.z = 2.0;
        search_func(local_pos_pub, search_pos);
        if (detect_flag){
            while((ros::Time::now() - last_det) < ros::Duration(2)){
                float y_ang = atan2( target_pos.y, target_pos.x);
                if ( abs(angles::to_degrees(y_ang) - 90) > 40 ){
                    TarPos.yaw_rate = yaw_rt.calculate(y_ang, angles::from_degrees(90));
                    Vel.x = 0;
                    Vel.y = 0;
                    Vel.z = pid_Vz.calculate2(target_pos.z);
                    
                    TarPos.velocity = Vel;
                    local_pos_pub.publish(TarPos);
                    ROS_INFO("Published velocity");
                }
                else{
                    cap_ball(local_pos_pub);
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
