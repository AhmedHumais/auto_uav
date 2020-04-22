#include <ros/ros.h>
#include "detector_tracker.hpp"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <signal.h>
#include <thread>

void body_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    body_pos = (*msg).pose.position;
}

void intrpt_handler(int signum) {
    ROS_INFO("Interrupted, exiting");
    exit_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_info_node");
    ros::NodeHandle nh;

    std::string cfg_file;
    std::string weights_file;
    float clearence;
    int im_dim;
    nh.param<std::string>("cfg_path", cfg_file,"~/trained-nets/balloon/balloon_tiny_30jan.cfg");
    nh.param<std::string>("weights_path",weights_file, "~/trained-nets/balloon/balloon_tiny_30jan_final.weights");
    nh.param<float>("clearence", clearence, 0.3);
    nh.param<int>("net_dim", im_dim, 416);
    ros::Publisher obj_pos_pub = nh.advertise<geometry_msgs::PointStamped>
            ("zed/objpos", 10);
    ros::Subscriber body_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, body_pos_cb);


    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_minimum_distance = 0.15;
    init_params.depth_maximum_distance = 20;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    auto err = zed.open(init_params);
    while (err != sl::ERROR_CODE::SUCCESS){
        ROS_INFO("Can't connect to camera");
        ROS_ERROR("Camera not connected");
        ros::spinOnce();
        err = zed.open(init_params);
        exit(1);
    }
    signal(SIGINT, intrpt_handler);
    zed.grab();

    float const thresh = 0.75;

    sl::Mat left;
    geometry_msgs::PointStamped ObjPos;
    ObjPos.header.frame_id = "camera";

    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
    slMat2cvMat(left).copyTo(cur_frame);
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh, im_dim);
    sl::sleep_ms(5000);

    std::thread tracker_thread(trackerThread, obj_pos_pub, clearence);
    int frame_count = 0;
    while (ros::ok() && (!exit_flag)) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left);

            frame_lock.lock();
            frame = slMat2cvMat(left);
            frame_lock.unlock();

            if(frame_count > 5){
                new_frame = true;
                frame_count = 0;
                ROS_INFO("Detecting");
                if( zed.getSensorsData( rot_data, sl::TIME_REFERENCE::IMAGE ) != sl::ERROR_CODE::SUCCESS ) {
                    ROS_WARN("Sensor data not available");
                }
            }
            else{
                frame_count +=1;
            }

            cloud_lock.lock();
            zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
            cloud_lock.unlock();
            new_cloud = true;

        }
        else{
            ROS_ERROR("frame_not_available");
        }
    }

    tracker_thread.join();
    detect_thread.join();
    zed.close();
    return 0;
}
