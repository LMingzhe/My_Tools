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
      } else 
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
          Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
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
  // std::string lidar_path = "";
  std::string pose_path = "/home/uav/STD_dataset/KITTI_dataset/KITTI_poses/kitti05.txt";

  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_vec;
  std::vector<double> times_vec;

  load_pose_with_time(pose_path, pose_vec, times_vec);

  std::cout << "pose number: " << pose_vec.size() << std::endl;
  std::cout << "time number: " << times_vec.size() << std::endl; 


  int loop_num = 0; 
  int current_frame_id = 0;
  bool loop_flag = false;
  std::vector<std::pair<int, int>> match_id;
  std::vector<double> diff_vec;

  for (int i = 0; i < pose_vec.size(); i++)
  { 
    Eigen::Vector3d current_trans = pose_vec[i].first;
    loop_flag = false;
    // std::cout << "current_trans: " << current_trans.transpose() << std::endl;
    for (int j = i + 1; j < pose_vec.size(); j++)
    {
      if((j - i) > 300) // 相距最少50帧
      {
        Eigen::Vector3d loop_trans = pose_vec[j].first;
        double diff = (loop_trans - current_trans).norm();
        std::cout << diff << std::endl;
        if (diff < 3) // 距离阈值
        {
          loop_flag = true;
          loop_num++;
          std::pair<int, int> single_match_id;
          single_match_id.first = i / 10;
          single_match_id.second = j / 10;
          match_id.push_back(single_match_id);
          break;
        }
      }
    }
  }

  for (auto pair : match_id)
  {
    std::cout << "发生回环的一对点云帧id: <" << pair.first << ", " << pair.second << ">" << std::endl; 
  }

  std::cout << "回环真阳性个数: " << loop_num << std::endl;
    


    return 0;
}