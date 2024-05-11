#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <vector>
#include <string>

using std::vector;
using std::cout;
using std::endl;
using std::string;

bool fileExists(const std::string &fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

void saveData(const string& fileName, const vector<double>& dataVec)
{
    std::ofstream outFile(fileName);
    for (auto& data : dataVec)
    {
        outFile << data << endl;
    }
    outFile.close();

    if (fileExists(fileName))
    {
        cout << "成功保存文件：" << fileName << endl;
    }
    else 
    {
        cout << "没有找到文件：" << fileName << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extract_imu_msgs");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("/home/uav/imu_data/imu_data.bag", rosbag::bagmode::Read);
    
    rosbag::View view(bag, rosbag::TopicQuery("/livox/imu"));

    vector<double> timeStamp;

    vector<double> acc_x;
    vector<double> acc_y;
    vector<double> acc_z;

    vector<double> gry_x;
    vector<double> gry_y;
    vector<double> gry_z;

    for (const rosbag::MessageInstance& msg : view)
    {
        sensor_msgs::Imu::ConstPtr imuMsg = msg.instantiate<sensor_msgs::Imu>();
        if (imuMsg != nullptr)
        {
            timeStamp.push_back(imuMsg->header.stamp.toSec());
            acc_x.push_back(imuMsg->linear_acceleration.x);
            acc_y.push_back(imuMsg->linear_acceleration.y);
            acc_z.push_back(imuMsg->linear_acceleration.z);
            gry_x.push_back(imuMsg->angular_velocity.x);
            gry_y.push_back(imuMsg->angular_velocity.y);
            gry_z.push_back(imuMsg->angular_velocity.z);
        }
    }

    bag.close();

    std::ofstream outFile("/home/uav/imu_data/timestamp.txt");
    for (auto& data : timeStamp)
    {
        outFile << data << endl;
    }
    outFile.close();

    saveData("/home/uav/imu_data/timestamp.txt", timeStamp);
    saveData("/home/uav/imu_data/acc_x.txt", acc_x);
    saveData("/home/uav/imu_data/acc_y.txt", acc_y);
    saveData("/home/uav/imu_data/acc_z.txt", acc_z);
    saveData("/home/uav/imu_data/gry_x.txt", gry_x);
    saveData("/home/uav/imu_data/gry_y.txt", gry_y);
    saveData("/home/uav/imu_data/gry_z.txt", gry_z);

    return 0;
}