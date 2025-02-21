//导入c++自带头文件
#include <iostream>
#include <math.h>
#include <vector>
#include <limits>
//自定义文件
using namespace std;

//纯跟踪算法
class PurePursuit
{
    public:
        //构造函数
        PurePursuit();
        //析构函数
        ~PurePursuit(){};

    private:
        //查找最近路径点下标
        int findCloesetPointIndex(const double x, const double y, const double v, 
            const std::vector<std::vector<double>> &path);
        //计算前轮转角
        double calSteeringAngle(int index, const std::vector<std::vector<double>> &path, 
    double vehicle_x, double vehicle_y, double vehicle_theta,double vehicle_v);
        double getSteeringAngle(const std::vector<std::vector<double>> &path, const std::vector<double> &vehicle_state);
        //控制指令话题
    

     
        //轴距
        double l = 2.0;
        //起始预瞄距离
        double ld0 = 0.5;
        //预瞄系数
        double kv = 0.1;
        //最大转向角
        double max_steer_angle = 35 / 180 *3.1415926;

        //全局路径点
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;

};