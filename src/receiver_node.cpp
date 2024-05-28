#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <vector>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include "utils.cpp"

std::vector<double> time_vec;

void message_cbk(const sensor_msgs::PointCloud2::Ptr& msg)
{
    static int count = 0;
    auto receTime = ros::Time::now();
    auto sendTime = msg->header.stamp;
    double duration = (receTime - sendTime).toNSec() / 1000000.0;
    time_vec.push_back(duration);
    
    // NOTE ==============================================================
    std::string compress_data_str(msg->data.begin(), msg->data.end());
    std::stringstream ss(compress_data_str);
    pcl::io::OctreePointCloudCompression<PointType> pointCloudEncoder;
    pcl::PointCloud<PointType>::Ptr unCompress_cloud(new pcl::PointCloud<PointType>);
    auto startDecode = std::chrono::high_resolution_clock::now();
    pointCloudEncoder.decodePointCloud(ss, unCompress_cloud);
    auto endDecode = std::chrono::high_resolution_clock::now();
    double decodeTimes = std::chrono::duration_cast<std::chrono::milliseconds>(endDecode - startDecode).count();
    ROS_INFO("UnCompress time: %f", decodeTimes);
    ROS_INFO("Received message NO.%d frame data. Transmission dely: %f ms", count++, duration);
    // NOTE ==============================================================
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "receiver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/testDemo", 200000, message_cbk, ros::TransportHints().tcp());

    // ros::Rate rate(0.5);
    
    while (ros::ok())
    {
        ros::spinOnce();
        // rate.sleep();
    }

    return 0;
}
