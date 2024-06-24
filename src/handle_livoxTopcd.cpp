#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <stdlib.h>
#include "utils.cpp"

int scan_cnt = 0;
// pcl 格式
PointCloudVector pc_full;
PointCloudVector pc_surf;
int blind = 4;

void compress_pointclouds()
{
    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::OctreePointCloudCompression<PointType>* pointCloudEncoder = 
    //             new pcl::io::OctreePointCloudCompression<PointType>(compressionProfile, true);
    // pcl::io::OctreePointCloudCompression<PointType> pointCloudEncoder(compressionProfile, true);
    pcl::io::OctreePointCloudCompression<PointType> pointCloudEncoder;

    std::stringstream compressed_data;
    // 压缩
    auto start = std::chrono::high_resolution_clock::now();
    pointCloudEncoder.encodePointCloud(pc_surf.makeShared(), compressed_data);
    auto end = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000000.0;

    // string prefix = "/home/uav/dataset/avia/compressed_data/";
    // string suffix = ".bin";
    // string fileName = prefix + std::to_string(scan_cnt) + suffix; 
    // std::ofstream outfile(fileName, std::ios::binary);
    // outfile.write(reinterpret_cast<const char*>(compressed_data.str().c_str()), compressed_data.str().size());
    // outfile.close();
    ROS_INFO("Compress need time: %f ms", dt);
    // ROS_INFO("save %s sucessfully.", fileName.c_str());
    // 解压
    // pcl::PointCloud<PointType>::Ptr decompressedCloud(new pcl::PointCloud<PointType>);
    // decompressedCloud->decompressedCloud(compressedData, decompressedCloud);
}

void livox_to_pcl(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    pc_full.clear();
    pc_surf.clear();
    int plsize = msg->point_num;
    // std::cout << "plsize size: " << plsize << std::endl;
    pc_full.resize(plsize);
    for (int i = 1; i < plsize; i++)
    {
        if ((msg->points[i].line < 6) && ((msg->points[i].tag & 0x30) == 0x10 ||
            (msg->points[i].tag & 0x30) == 0x00))
        {
            pc_full[i].x = msg->points[i].x;
            pc_full[i].y = msg->points[i].y;
            pc_full[i].z = msg->points[i].z;
            // pc_full[i].intensity = msg->points[i].reflectivity;
            // pc_full[i].curvature = msg->points[i].offset_time / float(1000000);
            // pc_full.push_back(point);
            // std::cout << "x: " << pc_full[i].x << " y: " << pc_full[i].y << " z: " << pc_full[i].z << std::endl;
        }
        if (((abs(pc_full[i].x - pc_full[i-1].x) > 1e-7) 
              || (abs(pc_full[i].y - pc_full[i-1].y) > 1e-7)
              || (abs(pc_full[i].z - pc_full[i-1].z) > 1e-7))
              && (pc_full[i].x * pc_full[i].x + pc_full[i].y * pc_full[i].y + pc_full[i].z * pc_full[i].z > (blind * blind)))
        {
            pc_surf.push_back(pc_full[i]);
        }
    }
    // std::cout << pc_surf[0].x << pc_surf[0].y << pc_surf[0].z << std::endl;
    // std::cout << "pc_surf size :" << pc_surf.size() << std::endl;
}

void livox_to_pcd()
{
    string prefix = "/home/uav/dataset/avia/cmsg_pcd/";
    string suffix = ".pcd";
    string fileName = prefix + std::to_string(scan_cnt) + suffix; 
    pcl::io::savePCDFileASCII(fileName, pc_surf);
    ROS_INFO("save %s sucessfully.", fileName.c_str());
}

void livox_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    scan_cnt++;
    livox_to_pcl(msg);
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sender_node");
    ros::NodeHandle nh;

    // 订阅rosbag /livox/lidar
    ros::Subscriber sub_lidar = nh.subscribe("/livox/lidar", 200000, livox_cbk);

    ros::Rate rate(10); // 调整速率 1Hz
    // int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (!pc_surf.empty())
        {
            // livox_to_pcd();
            compress_pointclouds();
        }
        rate.sleep();
    }

    return 0;
}
