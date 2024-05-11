#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>

int frame_count_node1 = 0;
int frame_count_node2 = 0;
std::string prefix;
std::string suffix = ".pcd";
std::string suffix_bin = ".bin";

// void check(sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     for (auto i = 0; i < msg->fields.size(); i++)
//     {
//         std::cout << "Field " << i << ": "
//               << "name=" << msg->fields[i].name << ", "
//               << "offset=" << msg->fields[i].offset << ", "
//               << "datatype=" << msg->fields[i].datatype << ", "
//               << "count=" << msg->fields[i].count << std::endl;
//     }
// }

bool fileExists(const std::string &filename)
{
    std::ifstream infile(filename);
    return infile.good();
}

inline void savePCD(std::string topic, pcl::PointCloud<pcl::PointXYZI>::Ptr &pclPointCloud)
{
    std::stringstream file_name_ss;
    if (topic == "/os1_cloud_node1/points")
    {
        frame_count_node1++;
        prefix = "/home/uav/NTU-VIRAL_ws/src/dataSeparate/dataset/eee_01/lidar_horz/";
        file_name_ss << prefix << std::setfill('0') << std::setw(4) << frame_count_node1 << suffix;
    }
    else
    {
        frame_count_node2++;
        prefix = "/home/uav/NTU-VIRAL_ws/src/dataSeparate/dataset/eee_01/lidar_vert/";
        file_name_ss << prefix << std::setfill('0') << std::setw(4) << frame_count_node2 << suffix;
    }
    
    std::string file_name = file_name_ss.str();
    pcl::io::savePCDFileASCII(file_name, *pclPointCloud);
    // pcl::io::savePCDFileBinary(file_name, *pclPointCloud);
    // pcl::io::savePCDFileBinaryCompressed(file_name, *pclPointCloud);
    
    
    if (fileExists(file_name))
    {
        std::cout << "成功保存文件：" << file_name << std::endl;
    }
    else
    {
        std::cout << "没有找到文件：" << file_name << std::endl;
    }

}

inline void saveBIN(std::string topic, pcl::PointCloud<pcl::PointXYZI>::Ptr &pclPointCloud)
{
    std::stringstream file_name_ss;
    if (topic == "/os1_cloud_node1/points")
    {
        frame_count_node1++;
        prefix = "/home/uav/NTU-VIRAL_ws/src/dataSeparate/dataset/eee_01/lidar_horz_bin/";
        file_name_ss << prefix << std::setfill('0') << std::setw(4) << frame_count_node1 << suffix_bin;
    }
    else
    {
        frame_count_node2++;
        prefix = "/home/uav/NTU-VIRAL_ws/src/dataSeparate/dataset/eee_01/lidar_vert_bin/";
        file_name_ss << prefix << std::setfill('0') << std::setw(4) << frame_count_node2 << suffix_bin;
    }
    
    std::string file_name = file_name_ss.str();
    std::ofstream out;
    out.open(file_name, std::ios::out | std::ios::binary);
    int cloudSize = pclPointCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        float point_x = pclPointCloud->points[i].x;
        float point_y = pclPointCloud->points[i].y;
        float point_z = pclPointCloud->points[i].z;
        out.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
        out.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
        out.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    }
    out.close();

    if (fileExists(file_name))
    {
        std::cout << "成功保存文件：" << file_name << std::endl;
    }
    else
    {
        std::cout << "没有找到文件：" << file_name << std::endl;
    }
}

int main(int argc, char** argv)
{
    // 创建ROS bag对象并打开需要读取的bag文件
    rosbag::Bag bag;
    bag.open("/home/uav/NTU-VIRAL_ws/src/dataSeparate/dataset/eee_01/eee_01.bag", rosbag::bagmode::Read);

    // 设置需要读取的topic列表
    std::vector<std::string> topics;
    topics.push_back("/os1_cloud_node1/points"); // Message Type: sensor_msgs/PointCloud2
    topics.push_back("/os1_cloud_node2/points");

    // 创建一个rosnag::View对象,并设置需要读取的topic
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // 遍历bag文件中的消息
    for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
    {
        // 获取当前消息的topic名称
        std::string topic = it->getTopic();

        // 根据topic类型进行相应的处理
        if (topic == "/os1_cloud_node1/points")
        {
            sensor_msgs::PointCloud2::ConstPtr msg = it->instantiate<sensor_msgs::PointCloud2>();
            if (msg != nullptr)
            {
                // check(msg);
                pcl::PointCloud<pcl::PointXYZI>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(*msg, *pclPointCloud);
                // std::cout << "话题: " << topic << std::endl;
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                // for (auto it = pclPointCloud->begin(); it != pclPointCloud->end(); it++)
                // {
                //     std::cout << "Point XYZ: " << it->x << ", " << it->y << ", " << it->z << std::endl;
                // }
                savePCD(topic, pclPointCloud);
                // saveBIN(topic, pclPointCloud);
            }
            else
            {
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                std::cout << "msg is empty!" << std::endl;
            }
            
        }
        else if (topic == "/os1_cloud_node2/points")
        {
            sensor_msgs::PointCloud2::ConstPtr msg = it->instantiate<sensor_msgs::PointCloud2>();
            if (msg != nullptr)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(*msg, *pclPointCloud);
                // std::cout << "话题: " << topic << std::endl;
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                // for (auto it = pclPointCloud->begin(); it != pclPointCloud->end(); it++)
                // {
                //     std::cout << "Point XYZ: " << it->x << ", " << it->y << ", " << it->z << std::endl;
                // }
                savePCD(topic, pclPointCloud);
                // saveBIN(topic, pclPointCloud);
            }
            else
            {
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                std::cout << "msg is empty!" << std::endl;
            }   
        }
    }

    return 0;
}
