

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool flag =false, vel_flag=false;

tf2_ros::Buffer tfBuffer;
geometry_msgs::PointStamped prev_pos, cur_pos, pred_pos, pred_pos_body;
std_msgs::Float32 vel_msg;

geometry_msgs::Point get_predicted_pos(const geometry_msgs::PointStamped &new_pos, const geometry_msgs::PointStamped &old_pos){
    geometry_msgs::Point pred_pos_;
    geometry_msgs::PointStamped relative_vel, rel_vel;

    auto t_diff = new_pos.header.stamp - old_pos.header.stamp;
    double dt = t_diff.toSec();
    float dx = new_pos.point.x - old_pos.point.x; 
    float dy = new_pos.point.y - old_pos.point.y; 
    float dz = new_pos.point.z - old_pos.point.z;
    relative_vel = new_pos;
    relative_vel.point.x = dx/dt;
    relative_vel.point.y = dy/dt;
    relative_vel.point.z = dz/dt;
    pred_pos_.x = new_pos.point.x + (relative_vel.point.x)*0.1f; // for 10Hz control loop freq
    pred_pos_.y = new_pos.point.y + (relative_vel.point.y)*0.1f; // for 10Hz control loop freq
    pred_pos_.z = new_pos.point.z + (relative_vel.point.z)*0.1f; // for 10Hz control loop freq
    try {
        tfBuffer.transform(relative_vel, rel_vel, "base_link", ros::Duration(0.4));
        vel_msg.data = rel_vel.point.y;
        vel_flag = true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    return pred_pos_;
}

void blob_sub_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
        try {
            tfBuffer.transform((*msg), cur_pos, "map", ros::Duration(0.4));
            cur_pos.header.stamp = ros::Time::now();
            if ( (cur_pos.header.stamp - prev_pos.header.stamp) < ros::Duration(0.5) ){
                auto pred_point = get_predicted_pos(cur_pos, prev_pos);
                pred_pos.header.stamp = cur_pos.header.stamp;
                pred_pos.point = pred_point;
                try{
                    tfBuffer.transform(pred_pos, pred_pos_body, "base_link", ros::Duration(0.2));
                    flag = true;
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                }
            }
            prev_pos = cur_pos;
            
        }
        catch (tf2::TransformException &ex) {
           ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_finder_node");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PointStamped>
            ("target/pred_pos", 10);
    ros::Publisher target_vel = nh.advertise<std_msgs::Float32>
            ("target/rel_vel", 10);
    ros::Rate rate(1000.0);
    pred_pos.header.frame_id = "map";

    prev_pos.header.frame_id = "map";
    prev_pos.header.stamp = ros::Time::now();
    
    ros::Subscriber blob_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("blob/pos", 10, blob_sub_cb);

    while(ros::ok()){
        if(flag){
            pred_pos_body.header.stamp = ros::Time::now();
            target_pub.publish(pred_pos_body);
            flag = false;
        }
        if(vel_flag){
            target_vel.publish(vel_msg);
            vel_flag = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
