#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher pointcloud_pub;

void disparityToPointCloud(const cv::Mat& disparity, const std_msgs::Header& header, float focal_length, float baseline) {
    int width = disparity.cols;
    int height = disparity.rows;

    // Create the PointCloud2 message
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header = header;
    cloud_msg.width = width;
    cloud_msg.height = height;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u, ++iter_x, ++iter_y, ++iter_z) {
            float d = disparity.at<float>(v, u);
            if (d <= 0.0) {
                *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();  // Ignore invalid disparity
                continue;
            }
            
            // Reconstruct 3D point
            float Z = focal_length * baseline / d;
            float X = (u - width / 2.0) * Z / focal_length;
            float Y = (v - height / 2.0) * Z / focal_length;
            
            *iter_x = X;
            *iter_y = Y;
            *iter_z = Z;
        }
    }

    pointcloud_pub.publish(cloud_msg);
}

void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg) {
    try {
        // Convert disparity to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, msg->image.encoding);
        cv::Mat disparity = cv_ptr->image;

        // Camera parameters (focal length, baseline)
        float focal_length = 480.0;  // Example focal length (adjust for your camera)
        float baseline = 0.1;        // Example baseline (adjust for your camera)

        // Convert disparity to PointCloud2
        disparityToPointCloud(disparity, msg->header, focal_length, baseline);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "disparity_to_pointcloud_node");
    ros::NodeHandle nh;

    // Subscribe to disparity topic
    ros::Subscriber disparity_sub = nh.subscribe("/disparity", 1, disparityCallback);

    // Publisher for point cloud
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/stereo/pointcloud", 1);

    ros::spin();

    return 0;
}
