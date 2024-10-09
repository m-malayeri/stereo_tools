#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
namespace fs = boost::filesystem;

sensor_msgs::CameraInfo loadCameraInfo(const std::string &calib_file) {
    
    YAML::Node calib_data = YAML::LoadFile(calib_file);

    sensor_msgs::CameraInfo camera_info_msg;
    
    camera_info_msg.width = calib_data["image_width"].as<int>();
    camera_info_msg.height = calib_data["image_height"].as<int>();
    
    // Check and parse YAML data safely
    std::vector<double> K, D, R, P;

    try {
        K = calib_data["camera_matrix"]["data"].as<std::vector<double>>();
        D = calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();
        R = calib_data["rectification_matrix"]["data"].as<std::vector<double>>();
        P = calib_data["projection_matrix"]["data"].as<std::vector<double>>();
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Error parsing YAML: %s", e.what());
    }

    // Ensure the sizes of the vectors match the required size for CameraInfo fields
    if (K.size() != 9) {
        ROS_ERROR("Invalid size for camera matrix K, expected 9 but got %lu", K.size());
    }
    if (R.size() != 9) {
        ROS_ERROR("Invalid size for rectification matrix R, expected 9 but got %lu", R.size());
    }
    if (P.size() != 12) {
        ROS_ERROR("Invalid size for projection matrix P, expected 12 but got %lu", P.size());
    }

    // Remove the check for the size of D, as D can have different sizes
    camera_info_msg.D = D;  // Direct assignment to the D vector

    // Now safely copy the values into the CameraInfo message
    std::copy(K.begin(), K.end(), camera_info_msg.K.begin());
    std::copy(R.begin(), R.end(), camera_info_msg.R.begin());
    std::copy(P.begin(), P.end(), camera_info_msg.P.begin());

    /**/
    camera_info_msg.distortion_model = calib_data["distortion_model"].as<std::string>();
    
    return camera_info_msg;
    
}

std::vector<std::string> getFiles(const std::string& dir_path, const std::string& extension) {
    std::vector<std::string> files;
    for (fs::directory_iterator itr(dir_path); itr != fs::directory_iterator(); ++itr) {
        if (fs::is_regular_file(itr->path()) && itr->path().extension() == extension) {
            files.push_back(itr->path().string());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_publisher");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 10);
    ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 10);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_image_pub = it.advertise("/stereo/left/image_raw", 10);
    image_transport::Publisher right_image_pub = it.advertise("/stereo/right/image_raw", 10);

    // Load camera calibration data
    sensor_msgs::CameraInfo left_camera_info_msg = loadCameraInfo("/home/cam0.yaml");
    sensor_msgs::CameraInfo right_camera_info_msg = loadCameraInfo("/home/cam1.yaml");

    // Get image file paths
    std::string left_path = "/root/Downloads/dataset/store/cam0";
    std::string right_path = "/root/Downloads/dataset/store/cam1";
    
    std::vector<std::string> left_files = getFiles(left_path, ".png");
    std::vector<std::string> right_files = getFiles(right_path, ".png");

    
    ros::Rate rate(15);  // 1 Hz

    for (size_t i = 0; i < left_files.size() && ros::ok(); ++i) {
        // Read images
        cv::Mat left_img = cv::imread(left_files[i], cv::IMREAD_COLOR);
        cv::Mat right_img = cv::imread(right_files[i], cv::IMREAD_COLOR);

        // Convert images to ROS messages
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_img).toImageMsg();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_img).toImageMsg();

        // Set frame_id
        left_msg->header.frame_id = "cam0";
        right_msg->header.frame_id = "cam1";

        // Set the timestamp
        ros::Time current_time = ros::Time::now();
        left_msg->header.stamp = current_time;
        right_msg->header.stamp = current_time;

        // Publish images and camera info
        left_image_pub.publish(left_msg);
        right_image_pub.publish(right_msg);

        left_camera_info_msg.header = left_msg->header;
        right_camera_info_msg.header = left_msg->header;

        left_cam_info_pub.publish(left_camera_info_msg);
        right_cam_info_pub.publish(right_camera_info_msg);

        rate.sleep();
    }/**/

    return 0;
}
