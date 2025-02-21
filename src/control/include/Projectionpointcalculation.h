#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
namespace Projectionpoint{
    double normalizeAngle(double angle) {
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
    }

    std::array<double, 5> InterpolateUsingLinearApproximation(Eigen::VectorXd p0,Eigen::VectorXd p1,const double s){
        double s0 = p0(5);
        double s1 = p1(5);
        double weight = (s - s0) / (s1 - s0);  // 权重
        //std::cout<<"weight: "<<weight<<std::endl;
        // 公式: x = p0.x + weight * (p1.x - p0.x),  下面用的是合并同类项之后的,都一样
        //std::cout<<"p0: "<<p0(0)<<" "<<p0(1)<<std::endl;
        //std::cout<<"p1: "<<p1(0)<<" "<<p1(1)<<std::endl;
        double x = (1 - weight) * p0(0) + weight * p1(0);
        double y = (1 - weight) * p0(1) + weight * p1(1);
        //std::cout<<"xy: "<<x<<" "<<y<<std::endl;
        // 对两个heading角处理使得在[-pi,pi)之间
        const double a0_n = normalizeAngle(p0(3));
        const double a1_n = normalizeAngle(p1(3));
        // 得到delta(heading)
        double d = a1_n - a0_n;
        if (d > M_PI) {
            d = d - 2 * M_PI;
        } else if (d < -M_PI) {
            d = d + 2 * M_PI;
        }
        const double a = a0_n + d * weight;
       // std::cout<<"a: "<<a<<std::endl;
        double theta= normalizeAngle(a);
        //std::cout<<"theta: "<<theta<<std::endl;
        double kappa = (1 - weight) * p0(4) + weight * p1(4);
        std::array<double,5>result{x,y,p0(2),theta,kappa};
        return result;

        
    }
    // 在参考线上计算匹配点, 入参:参考线,起点x,起点y, 相当于说在参考线上找到一点,该点离我当前轨迹
    // 上车辆所在点的距离是最短的,即frenet公式推导中的 l 最短
    // 为什么投影用的 向量start->vehicle_position 在 向量start->end 上的投影而不用
    // 向量index_min->vehicle_position 在 向量start->end 上的投影呢?
    // 参考:https://www.bilibili.com/video/BV1rM4y1g71p?spm_id_from=333.999.0.0
    // x y  v thea dk 
    std::array<double, 5> FindProjectionPoint(Eigen::VectorXd p0,Eigen::VectorXd p1,double x,double y){
        double v0x = x - p0(0);
        double v0y = y - p0(1);

        double v1x = p1(0) - p0(0);
        double v1y = p1(1) - p0(1);

        double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
        double dot = v0x * v1x + v0y * v1y;
        // v0 · v1 = ||v0||*||v1||*cos(theta)    theta为v0和v1的夹角
        // (v0 · v1) / ||v1|| = ||v0||*cos(theta)
        // ||v0||*cos(theta) 即为v0在v1上的投影
        double delta_s = dot/v1_norm;
        // 这个函数主要使用线性插值的办法来计算投影得到的点在参考线上的坐标,heading等一系列值,p0.s(),
        // 为从参考线的起点开始到这个点的距离,p0.s() + delta_s就是v0在参考线上的投影到参考线起点的距离
        return InterpolateUsingLinearApproximation(p0, p1, p0(5) + delta_s);
        
    }
    std::array<double, 5> GetProjectionpoint(int &closestIndex,const Eigen::MatrixXd &optTrajxy,double x,double y){
        if (closestIndex==0 || closestIndex == optTrajxy.cols() - 1){
            std::array<double,5>result{optTrajxy(0,closestIndex),optTrajxy(1,closestIndex),
                            optTrajxy(2,closestIndex),optTrajxy(3,closestIndex),optTrajxy(4,closestIndex)};
            return result;
        }
        // index_min == 0,意味着最近的点是referenceLine的起点,不等于0就 index_min - 1,
        std::size_t index_start = closestIndex-1;
        std::size_t index_end = closestIndex+1;
        // 做投影,向量A(index_start -> traj上的车辆当前位置) 点乘 向量B(index_start -> index_end)
        return FindProjectionPoint(optTrajxy.col(index_start), optTrajxy.col(index_end), x,  y);
    }  
}