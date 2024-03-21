#include <iostream>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <unordered_map>
#include <pcl/io/pcd_io.h>

using namespace std;

typedef pcl::PointXYZI PointType;

class KittiLoader 
{
public:
    // 构造函数，记录文件个数
    KittiLoader(const std::string& dataset_path) : dataset_path(dataset_path) 
    {
        for (num_frames = 0;; num_frames++) 
        {
            std::string filename = (boost::format("%s/%06d.bin") % dataset_path % num_frames).str();
            if (!boost::filesystem::exists(filename)) break;
        }

        if (num_frames == 0) 
        {
            std::cerr << "error: no files in " << dataset_path << std::endl;
        }
    }

    ~KittiLoader() {}

    size_t size() const { return num_frames; }

    pcl::PointCloud<PointType>::ConstPtr cloud(size_t i) const 
    {
        std::string filename = (boost::format("%s/%06d.bin") % dataset_path % i).str();
        FILE* file = fopen(filename.c_str(), "rb");
        if (!file) 
        {
            std::cerr << "error: failed to load " << filename << std::endl;
            return nullptr;
        }

        std::vector<float> buffer(1000000);
        // 点云个数
        size_t num_points = fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        cloud->resize(num_points);

        for (int i = 0; i < num_points; i++) 
        {
            auto& pt = cloud->at(i);
            pt.x = buffer[i * 4];
            pt.y = buffer[i * 4 + 1];
            pt.z = buffer[i * 4 + 2];
            // pt.intensity = buffer[i * 4 + 3];
        }

        return cloud;
    }

private:
    int num_frames;
    std::string dataset_path;
};

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
    std::string config_path = "/home/uav/gicp_exp_tool_ws/config/config.txt";
    std::unordered_map<std::string, std::string> configMap;
    ConfigLoader configloader(config_path);
    configloader.loader(configMap);

    std::string dataset_path = configMap["dataset_path"];
    KittiLoader kitti(dataset_path);

    std::string prefix = configMap["prefix"];

    for (int i = 0; i < kitti.size(); i++)
    {
        std::string file_name = std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(prefix + file_name, *kitti.cloud(i));
        std::cout << "成功保存" << prefix + file_name << std::endl;
    }

    return 0;
}
