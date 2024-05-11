**做实验时写的一些小工具**

- binTpcd.cpp 实现成批地将kitti点云数据集的格式由".bin"改为".pcd"
- searchLoop.cpp 根据一定规则寻找kitti数据集中的回环帧
- double_filter.cpp 点云双层体素将采样Demo
- registration_PCL.cpp PCL库中的算云配准算法对KITTI数据集进行配准实验
- extract_pointclouds.cpp 从rosbag文件中提取点云数据，保存为bin/pcd格式（无法单独编译，需要配合ros系统使用）
- extract_imu_msgs.cpp 从rosbag文件中提取IMU数据，保存为txt格式文件（无法单独编译，需要配合ros系统使用）