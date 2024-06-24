#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <fstream>


/**
 * @brief 从文件中加载带有时间戳的位姿数据
 */
void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec) 
{
  // 清空之前的数据
  times_vec.clear();
  poses_vec.clear();

  // 逐行读取文件内容
  std::ifstream fin(pose_file);
  std::string line;
  Eigen::Matrix<double, 1, 7> temp_matrix; // 存储文件中的七个量，前三个是平移向量，后四个是四元数
  while (getline(fin, line)) 
  {
    std::istringstream sin(line);
    std::vector<std::string> Waypoints;
    std::string info;
    int number = 0;
    // 根据空格分割字符串
    while (getline(sin, info, ' ')) 
    {
      if (number == 0) 
      {
        // 存储时间戳 double
        double time;
        std::stringstream data;
        data << info;
        data >> time;
        times_vec.push_back(time);
        number++;
      } 
      else 
      {
        // 存储位姿 double
        double p;
        std::stringstream data;
        data << info;
        data >> p;
        temp_matrix[number - 1] = p;
        if (number == 7) {
          Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1],
                                      temp_matrix[2]);
          Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4], // w x y z
                               temp_matrix[5]);
          std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
          single_pose.first = translation;
          single_pose.second = q.toRotationMatrix();
          poses_vec.push_back(single_pose);
        }
        number++;
      }
    }
  }
}

int main(int argc, char** argv)
{

    std::string pose_path = "/home/uav/第五章/位姿数据/iG-LIO_240526_2.txt";

    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_vec;
    std::vector<double> times_vec;

    load_pose_with_time(pose_path, pose_vec, times_vec);

    std::cout << "pose number: " << pose_vec.size() << std::endl;
    std::cout << "time number: " << times_vec.size() << std::endl; 

    Eigen::Vector3d poseStart = pose_vec[0].first;
    std::cout << poseStart[0] << " " << poseStart[1] << " " << poseStart[2] << std::endl;
    Eigen::Vector3d poseEnd = pose_vec[pose_vec.size() - 1].first;
    std::cout << poseEnd[0] << " " << poseEnd[1] << " " << poseEnd[2] << std::endl;

    double diff = (poseEnd - poseStart).norm();
    
    std::cout << diff << std::endl;
    
    return 0;
}