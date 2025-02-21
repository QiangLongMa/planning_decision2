#include"pure_pursuit.h"

PurePursuit::PurePursuit(){}


//加载路径点信息

//查找最近路径点下标
int PurePursuit::findCloesetPointIndex(const double x, const double y, const double v, 
    const std::vector<std::vector<double>> &path)
{   
    //位置索引
    int index;
    //与路径点距离
    vector<double> dist;
    //获取当前速度
    //计算全局路径点到小车位置的距离
    double x_diff, y_diff, mindistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); i++) {
        x_diff = x - path[i][0];
        y_diff = y - path[i][1];
       double distance =  x_diff * x_diff + y_diff * y_diff;
       if (distance < mindistance) {
            mindistance  = distance;
            index = i;
       }
    }
    // for (int i =0;i<(int)xr.size();i++)
    // {
    //     double dist_temp = pow(xr[i]-x,2) + pow(yr[i]-y,2);
    //     dist.push_back(dist_temp);
    // }
    // //获得最小值
    // auto smallest = min_element(dist.begin(),dist.end());
    // //获得最小值对应的下标
    // index = distance(dist.begin(),smallest);
    //计算预瞄距离
    double ld = kv*v + ld0;
    double ld_now = 0;
    //遍历全局路径点
    //找到符合在预瞄距离上且不在最终点的位置下标
    //比较的是下一个路径点与当前最近路径点的距离
    while(ld_now<ld && index<(int)(path.size() - 1))
    {
        double dx_ref = xr[index+1] - xr[index];
        double dy_ref = yr[index+1] - yr[index];
        ld_now += sqrt(pow(dx_ref,2)+pow(dy_ref,2));
        index++;
    }
    //向前两个预瞄点
    return index;
}

//计算前轮转角
double PurePursuit::calSteeringAngle(int index, const std::vector<std::vector<double>> &path, 
    double vehicle_x, double vehicle_y, double vehicle_theta, double vehicle_v)
{
    //设置转向限制
    double prev_x = path[index][0];
    double prev_y = path[index][1];
    //计算alpha角（小车与预瞄点间的夹角，需要减去小车航向角进行纠正）
    double alpha = atan2(prev_y - vehicle_y, prev_x - vehicle_x) - vehicle_theta;
    //计算预瞄距离
    double ld = kv*vehicle_v+ld0;
    double error = ld * sin(alpha);
    std::cout<<"pure_pursuit error: "<<error<<std::endl;
    double steer = atan2(2*l*sin(alpha),ld);
    if (steer > max_steer_angle)
    {
        steer = max_steer_angle;
    }
    else if (steer < -max_steer_angle)
    {
        steer = -max_steer_angle;
    }

    //返回转向角
    return steer;
}

//局部路径 车辆状态
double PurePursuit::getSteeringAngle(const std::vector<std::vector<double>> &path, const std::vector<double> &vehicle_state) {
    double vechicle_x = vehicle_state[0];
    double vechicle_y = vehicle_state[1];
    double vechicle_speed = vehicle_state[2];
    double vechicle_theta = vehicle_state[3];
    int minindex = findCloesetPointIndex(vechicle_x, vechicle_y, vechicle_speed, path);
    return calSteeringAngle(minindex, path, vechicle_x, vechicle_y,vechicle_theta, vechicle_speed);
}
