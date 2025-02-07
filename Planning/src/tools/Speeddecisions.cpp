#include"Speeddecisions.h"
void SpeedDecisions::GetSpeed(double car_s,double start_v,double end_v,
                                Eigen::MatrixXd &optTrajxy,double safetydistance,bool flag){
    //路径中无障碍物 
    double weight ;
    double v;
    double a;
    size_t optsize = optTrajxy.cols();
    if (!flag){ //加速 
        a = (end_v/3.6-start_v)/T;//加速度 
        double start_s = optTrajxy(5,0);
        for (size_t i = 0; i < optsize; ++i){
            double v0_pow = start_v * start_v + 2 * a * (optTrajxy(5,i) - start_s); // now speed * speed 
            if(a >= 0){//acc 
                if(v0_pow >= end_v / 3.6 * end_v / 3.6){
                    v = end_v / 3.6;
                    a = 0.0;
                }else{
                    v = std::sqrt(std::abs(v0_pow));
                }
            }else{ // dece 
                if(v0_pow <= end_v / 3.6 * end_v / 3.6){
                    v = end_v / 3.6;
                    a = 0.0;
                }else{
                    v = std::sqrt(std::abs(v0_pow));
                }
            }
            optTrajxy(2,i) = v;//向上取整 
            optTrajxy(6,i) = a;
        }
    }
    else {//路径中有障碍物 减速操作 减速  
        a = -start_v*start_v/(2*safetydistance+1e-3);//加速度 速度 m/s
        for (size_t i = 0; i < optsize; ++i){
            double X = optTrajxy(5,i) - optTrajxy(5,0);
            double v_pow =  2 * a * X + start_v * start_v;
            if (v_pow < 0) {
                optTrajxy(2,i) = 0; //四舍五入 m/s
                optTrajxy(6,i) = 0;           
            } else {
                optTrajxy(2,i) = std::sqrt(v_pow); //四舍五入 m/s
                optTrajxy(6,i) = a;           
            }
        }
    }
}

void SpeedDecisions::closeobsgetspeed(Eigen::MatrixXd &optTrajxy){
    size_t size = optTrajxy.cols();
    for (size_t i = 0; i < size; ++i){
        optTrajxy(2,i) =0;
    }
    
}
