#include <ros/ros.h>
#include "zed_helper.hpp"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <thread>

bool new_frame = false, exit_flag = false;
cv::Mat frame;
std::mutex frame_lock, cloud_lock;
sl::Mat cur_cloud; 

void intrpt_handler(int signum) {
    ROS_INFO("Interrupted, exiting");
    exit_flag = true;
}

void detectorThread(const ros::Puiblisher &pub) {
    cv::SimpleBlobDetector::Params params;
 
    //  # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    // # Filter by Area.
    params.filterByArea = True
    params.minArea = 250;
    params.maxArea = 250000;
    
    // # Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.7;
    params.maxCircularity = 1;
    
    // # Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.7;
    params.maxConvexity = 1;
    
    // # Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.55;
    params.maxInertiaRatio = 1;

    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    geometry_msgs::PointStamped target_pos;
    target_pos.header.frame_id = "body";
    cv::Mat blob_frame, mask, dst;

    while (!exit_flag) {
        if (new_frame) {
            std::vector<cv::KeyPoint> keypoints;
            new_frame = false;
            frame_lock.lock();
            cv::cvtColor(frame, blob_frame, cv::COLOR_BGRA2LAB);
            frame_lock.unlock();
            cv::inRange(blob_frame, cv::Scalar(15, 120, 165), cv::Scalar(255, 150, 210), mask);
            cv::medianBlur(mask, dst, 5);
            cv::bitwise_not(dst, mask);
            detector->detect(mask, keypoints); 

            if(!keypoints.empty()){
                auto kp = get_largest_blob(keypoints);
                cloud_lock.lock();
                auto pos_3d = get_xyz_from_blob(kp.pt, cur_cloud);
                cloud_lock.unlock();
                target_pos.header.stamp = ros::Time::now();
                target_pos.point.x = pos_3d.x;
                target_pos.point.y = pos_3d.y;
                target_pos.point.z = pos_3d.z - 0.15;
                pub.publish(target_pos);
                ROS_INFO("published position");
            }
            
        } else sl::sleep_ms(1);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_detect_node");
    ros::NodeHandle nh;

    ros::Publisher obj_pos_pub = nh.advertise<geometry_msgs::PointStamped>
            ("blob/pos", 10);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_minimum_distance = 0.10;
    init_params.depth_maximum_distance = 20;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    auto err = zed.open(init_params);
    while (err != sl::ERROR_CODE::SUCCESS){
        ROS_ERROR("Camera not connected");
        ros::spinOnce();
        err = zed.open(init_params);
        exit(1);
    }
    signal(SIGINT, intrpt_handler);
    zed.grab();

    sl::Mat left;
    geometry_msgs::PointStamped ObjPos;
    ObjPos.header.frame_id = "camera";

    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
    slMat2cvMat(left).copyTo(frame);
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(blobDetectorThread, obj_pos_pub);

    while (ros::ok() && (!exit_flag)) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left);

            frame_lock.lock();
            frame = slMat2cvMat(left);
            new_frame = true;
            frame_lock.unlock();

            cloud_lock.lock();
            zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
            cloud_lock.unlock();
        }
        else{
            ROS_ERROR("frame_not_available");
            exit_flag = true;
        }
    }

    detect_thread.join();
    zed.close();
    return 0;
}
