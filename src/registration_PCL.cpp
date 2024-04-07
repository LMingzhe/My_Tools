#include <iostream>
#include <chrono>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/circular_buffer.hpp>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef pcl::PointXYZ PointType;

/**
 * @brief 读取KITTI数据
 */
class KittiLoader 
{
public:
  KittiLoader(const std::string& dataset_path) : dataset_path(dataset_path) 
  {
    for (num_frames = 0;; num_frames++) 
    {
      std::string filename = (boost::format("%s/%06d.bin") % dataset_path % num_frames).str();
      if (!boost::filesystem::exists(filename)) 
      {
        break;
      }
    }

    if (num_frames == 0) {
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

/**
 * @brief 读取配置文件信息
 */
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
    std::string dataset_path;  // 数据集路径
    std::string save_path;     // 轨迹保存路径
    bool if_save_pose;         // 是否保存轨迹

    std::string config_path = "/home/uav/exp_tool_ws/config/config.txt";
    ConfigLoader configloader(config_path);
    std::unordered_map<std::string, std::string> configMap;
    configloader.loader(configMap);

    dataset_path =  configMap["dataset_path"];
    save_path    =  configMap["save_path"];
    if_save_pose =  (std::stoi(configMap["if_save_pose"]) != 0);

    std::cout << "本次配准信息:"             << std::endl;
    std::cout << "data_path: "             << dataset_path             << std::endl;
    std::cout << "save_path: "             << save_path                << std::endl;
    std::cout << "if_save_pose: "          << if_save_pose             << std::endl;
    sleep(2);
    
    KittiLoader kitti(dataset_path);

    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(0.25, 0.25, 0.25);

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> method;
    // pcl::IterativeClosestPoint<PointType, PointType> method;
    // pcl::NormalDistributionsTransform<PointType, PointType> method;
    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses(kitti.size());
    poses[0].setIdentity();

    boost::circular_buffer<std::chrono::high_resolution_clock::time_point> stamps(30);
    stamps.push_back(std::chrono::high_resolution_clock::now());

    std::vector<double> fps_avger;

    for (int i = 0; i < kitti.size() - 1; i++)
    {   
        // 初始化
        source.reset(new pcl::PointCloud<PointType>);
        target.reset(new pcl::PointCloud<PointType>);
        aligned.reset(new pcl::PointCloud<PointType>);

        // 处理输入点云
        voxelgrid.setInputCloud(kitti.cloud(i + 1));
        voxelgrid.filter(*source);
        method.setInputSource(source);

        // 处理输出点云
        voxelgrid.setInputCloud(kitti.cloud(i));
        voxelgrid.filter(*target);
        method.setInputTarget(target);

        // 配准
        method.align(*aligned);
        Eigen::Matrix<float, 4, 4> F_T = method.getFinalTransformation();
        poses[i + 1] = poses[i] * F_T.cast<double>();

        stamps.push_back(std::chrono::high_resolution_clock::now());
        double fps = stamps.size() / (std::chrono::duration_cast<std::chrono::nanoseconds>(stamps.back() - stamps.front()).count() / 1e9);
        std::cout << fps << "fps" << std::endl;
        fps_avger.push_back(fps);

    }

    double sum = 0;
    for (auto &fps : fps_avger)
    {
        sum += fps;
    }
    double fps_avg = sum / fps_avger.size();
    std::cout << "平均fps: " << fps_avg << std::endl;

    if (if_save_pose)
    {
        std::ofstream ofs(save_path);
        for (const auto& pose : poses) 
        {
            for (int i = 0; i < 3; i++) 
            {
                for (int j = 0; j < 4; j++) 
                {
                if (i || j) 
                {
                    ofs << " ";
                }

                ofs << pose(i, j);
                }
            }
            ofs << std::endl;
        }
    }


    return 0;
}
