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

int scan_count = 0;
// pcl 格式
PointCloudVector pc_full;
PointCloudVector pc_surf;
std::vector<double> time_vec;
int blind = 4;


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

void publish_pointcloud(const ros::Publisher& pub)
{
    static int cnt = 0;

    sensor_msgs::PointCloud2 laserCloudMsg;
    // pcl::toROSMsg(pc_surf, laserCloudMsg);

    // pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    // pcl::io::OctreePointCloudCompression<PointType>* pointCloudEncoder = 
    //             new pcl::io::OctreePointCloudCompression<PointType>(compressionProfile, true);
    pcl::io::OctreePointCloudCompression<PointType> pointCloudEncoder;
    std::stringstream compressed_data;
    auto startCompress = std::chrono::high_resolution_clock::now();
    pointCloudEncoder.encodePointCloud(pc_surf.makeShared(), compressed_data);
    auto endCompress = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::milliseconds>(endCompress - startCompress).count();

    std::string compressed_data_str = compressed_data.str();
    laserCloudMsg.data = std::move(std::vector<uint8_t>(compressed_data_str.begin(), compressed_data_str.end()));
    laserCloudMsg.height = 1;
    laserCloudMsg.width = compressed_data_str.size();
    laserCloudMsg.header.frame_id = "camera_init";
    laserCloudMsg.header.stamp = ros::Time().now();
    pub.publish(laserCloudMsg);
    pc_surf.clear();

    ROS_INFO("Compress time: %f", duration);
    ROS_INFO("The %d frame point cloud is published. ", cnt++);
}

void livox_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    scan_count++;
    livox_to_pcl(msg);
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg_in) {}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sender_node");
    ros::NodeHandle nh;
    
    // 订阅rosbag /lidar/imu
    // ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 200000, imu_cbk);

    // 订阅rosbag /livox/lidar
    ros::Subscriber sub_lidar = nh.subscribe("/livox/lidar", 200000, livox_cbk);
    ros::Publisher        pub = nh.advertise<sensor_msgs::PointCloud2>("/testDemo", 100000);

    ros::Rate rate(10); // 调整速率 1Hz
    // int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        if (!pc_surf.empty())
        {
            publish_pointcloud(pub);
        }
        rate.sleep();
    }

    return 0;
}
