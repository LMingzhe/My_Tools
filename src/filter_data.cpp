#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <numeric>

using namespace std;

int main(int argc, char const *argv[])
{
    string fileName = "/home/uav/System_Communication/result/50m_compressed.txt";
    vector<double> datas;
    std::ifstream file(fileName);
    if (file.is_open())
    {
        string line;
        while (getline(file, line))
        {
            double data = stod(line);
            if (data > 50)
                datas.push_back(data);
        }
    }
    file.close();
    
    double sum = std::accumulate(datas.begin(), datas.end(), 0.0);
    double avg = sum / datas.size();
    auto max = std::max_element(datas.begin(), datas.end());
    auto min = std::min_element(datas.begin(), datas.end());
    
    cout << "size: " << datas.size() << endl;
    cout << "max: " << *max << endl;
    cout << "min: " << *min << endl;
    cout << "avg: " << avg << endl;

    int cnt = 0;
    int index = 0;
    double filtered_sum = 0.0;
    vector<double> datas_filtered;
    while (cnt < 1050)
    {
        if (datas[index] > 100 && datas[index] < 500)
        {
            datas_filtered.push_back(datas[index]);
            filtered_sum += datas[index];
            cnt++;
        }
        index++;
    }
    double filtered_avg = filtered_sum / cnt;
    cout << "filtered_size: " << cnt << endl;
    cout << "filtered_avg: " << filtered_avg << endl;
    

    // int cnt = 0;
    // double filtered_sum = 0.0;
    // for (const auto& data : datas)
    // {
    //     if (data <= 800)
    //     {
    //         filtered_sum += data;
    //         cnt++;
    //     }
    // }
    // double filtered_avg = filtered_sum / cnt;
    // cout << "filtered_size: " << cnt << endl;
    // cout << "filtered_avg: " << filtered_avg << endl;

    string filterName = "/home/uav/System_Communication/result/compress_filtered_50m.txt";
    std::ofstream filtered(filterName);
    if (filtered.is_open())
    {
        for (const auto& data : datas_filtered)
        {
            filtered << data << endl;
        }
        cout << "文件写入成功" << endl;
    }
    else cout << "打开文件失败" << endl;
    filtered.close();

    return 0;
}
