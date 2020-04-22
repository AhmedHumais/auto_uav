#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <ros/ros.h>

#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>

class bbox_t_3d {
public:
    cv::Rect bbox;
    sl::float3 coord;

    bbox_t_3d(cv::Rect bbox_, sl::float3 coord_) {
        bbox = bbox_;
        coord = coord_;
    }
};

float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t_3d> getObjectDepth(std::vector<cv::Rect> &bbox_vect, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;

    std::vector<bbox_t_3d> bbox3d_vect;

    for (auto &it : bbox_vect) {

        int center_i = it.x + it.width * 0.5f, center_j = it.y + it.height * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure) {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        }

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
            float x_med = getMedian(x_vect);
            float y_med = getMedian(y_vect);
            float z_med = getMedian(z_vect);

            bbox3d_vect.emplace_back(it, sl::float3(x_med, y_med, z_med));
        }
    }

    return bbox3d_vect;
}

sl::float3 get_3d_BB(cv::Rect bbox, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;
    sl::float3 coords_3d = sl::float3(0,0,0);

    int center_i = bbox.x + bbox.width * 0.5f, center_j = bbox.y + bbox.height * 0.5f;

    std::vector<float> x_vect, y_vect, z_vect;
    for (int R = 0; R < R_max; R++) {
        for (int y = -R; y <= R; y++) {
            for (int x = -R; x <= R; x++) {
                i = center_i + x;
                j = center_j + y;
                xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
                valid_measure = std::isfinite(out.z);
                if (valid_measure) {
                    x_vect.push_back(out.x);
                    y_vect.push_back(out.y);
                    z_vect.push_back(out.z);
                }
            }
        }
    }

    if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
        coords_3d.x = getMedian(x_vect);
        coords_3d.y = getMedian(y_vect);
        coords_3d.z = getMedian(z_vect);

    }

    return coords_3d;
}

sl::float3 get_xyz_from_blob(cv::Point2f point, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 5;
    sl::float3 coords_3d = sl::float3(0,0,0);

    int center_i = point.x, center_j = point.y;

    std::vector<float> x_vect, y_vect, z_vect;
    for (int R = 0; R < R_max; R++) {
        for (int y = -R; y <= R; y++) {
            for (int x = -R; x <= R; x++) {
                i = center_i + x;
                j = center_j + y;
                xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
                valid_measure = std::isfinite(out.y);
                if (valid_measure) {
                    x_vect.push_back(out.x);
                    y_vect.push_back(out.y);
                    z_vect.push_back(out.z);
                }
            }
        }
    }

    if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
        coords_3d.x = getMedian(x_vect);
        coords_3d.y = getMedian(y_vect);
        coords_3d.z = getMedian(z_vect);

    }

    return coords_3d;
}


cv::Rect get_largest_bb(std::vector<cv::Rect> &bbox_vect){
    float largest;
    int i = 0;
    cv::Rect BB;
    for (auto &it : bbox_vect) {

        float center_i = it.x + it.width * 0.5f, center_j = it.y + it.height * 0.5f;
        float area = it.width*it.height;
        if (i==0){
            i++;
            largest = area;
            BB = it;
        }
        else{
            if (largest < area){
                largest = area;
                BB = it;
            }
        }
    }
    return BB;
}

cv::KeyPoint get_largest_blob(std::vector<cv::KeyPoint> &keyp_vect){
    cv::KeyPoint largest_keyp;
    float largest = 0;
    for (auto &it : keyp_vect) {
        if(it.size > largest){
            largest = it.size;
            largest_keyp = it;
        }
    }
    return largest_keyp;
}


std::vector<sl::float3> get_3d_coords(std::vector<bbox_t_3d> result_vec){
    std::vector<sl::float3> obj_coords;
    for (auto &i : result_vec) {
        obj_coords.push_back(i.coord);
    }
    return obj_coords;
}


cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}




