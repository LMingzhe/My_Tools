#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

using namespace std;

int main(int argc, char const *argv[])
{
    string fileName = "/home/uav/System_Communication/result/50m.txt";
    vector<double> datas;
    std::ifstream file(fileName);
    if (file.is_open())
    {
        string line;
        while (getline(file, line))
        {
            double data = stod(line);
            datas.push_back(data);
        }
    }
    file.close();
    
    double sum = std::accumulate(datas.begin(), datas.end(), 0.0);
    double avg = sum / datas.size();
    cout << "size: " << datas.size() << endl;
    cout << "avg: " << avg << endl;


    int cnt = 0;
    double filtered_sum = 0.0;
    for (const auto& data : datas)
    {
        if (data <= 800)
        {
            filtered_sum += data;
            cnt++;
        }
    }
    double filtered_avg = filtered_sum / cnt;
    cout << "filtered_size: " << cnt << endl;
    cout << "filtered_avg: " << filtered_avg << endl;

    string filterName = "/home/uav/System_Communication/result/filtered_50m.txt";
    std::ofstream filtered(filterName);
    if (filtered.is_open())
    {
        for (const auto& data : datas)
        {
            if (data <= 800)
            {
                filtered << data << endl;
            }
        }
        cout << "文件写入成功" << endl;
    }
    else cout << "打开文件失败" << endl;
    filtered.close();

    return 0;
}
