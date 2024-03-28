#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <unordered_map>
#include <pcl/io/pcd_io.h>

using namespace std;

typedef pcl::PointXYZI PointType;

class ConfigLoader
{
public:
  ConfigLoader(const std::string& config_path) : config_path(config_path) {}

  void loader(std::unordered_map<std::string, std::string>& configMap)
  {
    std::ifstream configFile(config_path);
    if (!configFile.is_open())
    {
      std::cout << "Error opening config file!" << std::endl;
      return;
    }
    
    std::string line;
    while (std::getline(configFile, line))
    {
      size_t index = line.find("=");
      if(index != std::string::npos)
      {
        std::string key = line.substr(0, index);
        std::string value = line.substr(index + 1);
        configMap[key] = value;
      }
    }

    configFile.close();
  }

private:
  std::string config_path;  
};

int main(int argc, char const *argv[])
{   
    std::string config_path = "/home/uav/exp_tool_ws/config/config.txt";
    std::unordered_map<std::string, std::string> configMap;
    ConfigLoader configloader(config_path);
    configloader.loader(configMap);

    // 读取参数
    double down_samping_v1_ = std::stod(configMap["down_samping_v1"]);
    double down_samping_v2_ = std::stod(configMap["down_samping_v2"]);
    std::string pcd_path_    = configMap["pcd_path"];
    std::string save_filter1_path_ = configMap["save_filter1_path"];
    std::string save_filter2_path_ = configMap["save_filter2_path"];


    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_v1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_v2(new pcl::PointCloud<PointType>);

    pcl::ApproximateVoxelGrid<PointType> voxelgrid;
    // pcl::VoxelGrid<PointType> voxelgrid;
    
    // 第一次滤波
    voxelgrid.setLeafSize(down_samping_v1_, down_samping_v1_, down_samping_v1_);
    if (pcl::io::loadPCDFile<PointType>(pcd_path_, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file!\n");
        return -1;
    }
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*cloud_v1);
    pcl::io::savePCDFileBinary(save_filter1_path_, *cloud_v1);

    // 第二次滤波
    voxelgrid.setLeafSize(down_samping_v2_, down_samping_v2_, down_samping_v2_);
    voxelgrid.setInputCloud(cloud_v1);
    voxelgrid.filter(*cloud_v2);
    pcl::io::savePCDFileBinary(save_filter2_path_, *cloud_v2);
    

    return 0;
}
