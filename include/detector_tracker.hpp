#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>

#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "zed_helper.hpp"

geometry_msgs::Point body_pos;
std::mutex frame_lock, bb_lock, cloud_lock, data_lock;
cv::Mat frame, cur_frame;
sl::Mat cur_cloud; 
sl::SensorsData rot_data;

std::vector<cv::Rect> result_vect;
std::atomic<bool> exit_flag, new_frame, start_tracking, new_cloud, new_detect, new_data;
cv::Rect det_bb;

std::vector<cv::Rect> get_detections(std::vector<cv::Mat> &detections, float confidence_threshold, float nms_threshold=0.5, int num_classes = 1){
        std::vector<cv::Rect> boxes;
        std::vector<cv::Rect> ret_boxes;
        std::vector<float> scores;
        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (size_t i = 0; i < num_boxes; i++)
            {
                auto itr = std::max_element(output.ptr<float>(i, 5), output.ptr<float>(i, 5 + num_classes));
                auto confidence = *itr;
                if (confidence >= confidence_threshold)
                {
                    auto x = output.at<float>(i, 0) * frame.cols;
                    auto y = output.at<float>(i, 1) * frame.rows;
                    auto width = output.at<float>(i, 2) * frame.cols;
                    auto height = output.at<float>(i, 3) * frame.rows;
                    cv::Rect rect(x - width/2, y - height/2, width, height);
                    scores.push_back(confidence);
                    boxes.push_back(rect);
                }
            }
        }
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, 0.0, nms_threshold, indices);

        for (size_t i = 0; i < indices.size(); ++i)
        {
            auto idx = indices[i];
            ret_boxes.push_back(boxes[idx]);
        }
    return ret_boxes;
}

std::vector<cv::Rect> get_verified(std::vector<cv::Rect> &dets){
    std::vector<cv::Rect> verified_boxes;
    auto cam_orient = rot_data.imu.pose.getRotationMatrix();
    sl::Mat depth_map;
    
    cloud_lock.lock();
    depth_map = cur_cloud;
    cloud_lock.unlock();

    for( auto &det : dets){
        auto obj_coord = get_3d_BB(det, depth_map);
        float coordmat[9] = {obj_coord.x, 0, 0, obj_coord.y, 0, 0, obj_coord.z, 0, 0};
        sl::Matrix3f cMat(coordmat);
        cMat = cam_orient*cMat;
        float ht = cMat(2,0) + body_pos.z;
        if (ht > 1.5 && ht <3){
            verified_boxes.push_back(det);
        }
    }
    return verified_boxes;
}

void detectorThread(std::string cfg_file, std::string weights_file, float thresh, int net_size = 416) {
    // initializing opencv dnn yolov3
    auto net = cv::dnn::readNetFromDarknet(cfg_file,weights_file);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    auto output_names = net.getUnconnectedOutLayersNames();

    std::vector<cv::Mat> detections;
    cv::Mat blob;
    std::vector<cv::Rect> det_vect;
    cv::Size const frame_size = frame.size();

    while (!exit_flag) {
        if (new_frame) {
            frame_lock.lock();
            cv::dnn::blobFromImage(frame, blob, (1/255.0), cv::Size(net_size, net_size), cv::Scalar(), true, false, CV_32F);
            frame_lock.unlock();
            net.setInput(blob);
            net.forward(detections, output_names);
            det_vect = get_detections(detections, thresh);
            det_vect = get_verified(det_vect);                   
            new_frame = false;
            if(!det_vect.empty()){
                bb_lock.lock();
                det_bb = get_largest_bb(det_vect);
                if((det_bb.width * det_bb.height) > 50.0){
                    new_detect = true;
                    start_tracking = true;
                    ROS_INFO("Detected");
                }
                bb_lock.unlock();
//                ROS_INFO("Detected");
            }
            
        } else sl::sleep_ms(1);
    }
}

void trackerThread(const ros::Publisher &pub, float clearence) {
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d BB_track2d;
    bool ok = false;
    geometry_msgs::PointStamped target_pos;
    target_pos.header.frame_id = "camera";
    cv::Mat track_frame;
    ros::Time last_detection; 

    while(!exit_flag){
        if(start_tracking){
            frame_lock.lock();
            cv::cvtColor(frame, track_frame, CV_RGBA2RGB);
            frame_lock.unlock();
            if (new_detect){
                last_detection = ros::Time::now();
                bb_lock.lock();
                BB_track2d = det_bb;  
                bb_lock.unlock();
//                tracker->clear();
                tracker = cv::TrackerCSRT::create();
                if(!(tracker->init(track_frame, BB_track2d))){
                    ROS_INFO("Tracker init failed");
                }
                new_detect = false;

                ok = true;
            }
            else{
                if (ros::Time::now() - last_detection > ros::Duration(2)){
                    ok = false;
                }
                else{
                    ok = tracker->update(track_frame, BB_track2d);
                }
            }

            if (ok){
                while(!new_cloud){
                    sl::sleep_ms(1);
                }
                cloud_lock.lock();
                auto BB_object3d = get_3d_BB(BB_track2d, cur_cloud);
                cloud_lock.unlock();
                new_cloud = false;
                target_pos.header.stamp = ros::Time::now();
                target_pos.point.x = BB_object3d.x;
                target_pos.point.y = BB_object3d.y - clearence;
                target_pos.point.z = BB_object3d.z;
                pub.publish(target_pos);
                ROS_INFO("published position");

                cv::rectangle(track_frame, BB_track2d, cv::Scalar(255, 0, 0), 2, 1);
                cv::imshow("Tracking", track_frame);
                cv::waitKey(1);
            }
            else{
                new_frame = true;
                sl::sleep_ms(10);

                ROS_INFO("Lost Tracking");
                start_tracking = false;
            }
        }
        else{
            new_frame = true;
            sl::sleep_ms(10);
            ROS_INFO("Nothing to track");
        }
        sl::sleep_ms(5);
    }
}