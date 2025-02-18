#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Eigen>
#include<vector>
#include"local_dp_qp.h"
//#include"QP.h"
#include"omp.h"
#include"Calobs.h"

namespace obs{
    void obsfindClosestPoint(double x, double y, Eigen::MatrixXd &globalPath, int& minIndex, const int carIndex){
        int startIndex = std::max(carIndex - 40, 0);
        int endIndex = std::min(carIndex+ 160,static_cast<int>(globalPath.cols()));
        double distance, d_min = std::numeric_limits<double>::max();
        double x_diff, y_diff;
        for (int i= startIndex; i < endIndex; ++i){
            x_diff = globalPath(0,i) - x;
            y_diff = globalPath(1,i) - y;
            distance = x_diff * x_diff + y_diff * y_diff;
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;	
            }
        }
    }
    void findClosestPoint(const Eigen::VectorXd& car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& path, int& minIndex) {
        int startIndex = std::max(minIndex - 40,0);
        int end = std::min(minIndex + 100,static_cast<int>(path.cols()));
        double distance;
        double d_min = std::numeric_limits<double>::max();
        int closestIndex = minIndex;
        for (int i = startIndex; i < end; ++i) {
            Eigen::VectorXd diff = path.block<2,1>(0,i)-car.block<2,1>(0,0);
            distance = diff.squaredNorm();
            if (distance < d_min) {
                closestIndex = i;
                d_min = distance;
            }
        }
        minIndex = closestIndex;
    }
    int cartesianToFrenet(const Eigen::VectorXd& car, const Eigen::Matrix<double, 7, Eigen::Dynamic>& path, frentPoint& carFrentPoint, int& index) {
        findClosestPoint(car, path, index);
        double dx = car(0) - path(0,index);
        double dy = car(1) - path(1,index);

        double cos_theta_r = std::cos(path(3,index));
        double sin_theta_r = std::sin(path(3,index));
        double ref_s = path(6, index);
        double path_s = dx * cos_theta_r + dy * sin_theta_r + ref_s;
        double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
        // carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
        carFrentPoint.d = std::copysign(cross_rd_nd, cross_rd_nd);
        carFrentPoint.s = std::abs(path_s);
        //index = minIndex;
        return 0;
    }

    //计算障碍物的四个顶点和中心点的sl
    void lidar_obs_Frenet(const Eigen::VectorXd& realPosition,  Eigen::MatrixXd &globalPath,  obses_sd &obsfrent,int carindex)
        {
        /*
            2-------3
            |       |
            1-------4
        */
        Eigen::MatrixXd obs(4,2);
        //添加障碍物的四个点 
        int minIndex;
        obs<<   realPosition(0),realPosition(1),
                realPosition(0),realPosition(3),
                realPosition(2),realPosition(3),
                realPosition(2),realPosition(1);
        double dx;
        double dy ;
        double cos_theta_r;
        double sin_theta_r ;
        double cross_rd_nd ;
        // 用于存储每个顶点的 SL 值和最近点的索引
        std::vector<std::array<double,2>> SL(obs.rows());
        std::vector<int> minIndices(obs.rows());
        for (size_t i = 0; i < obs.rows(); i++){
            obsfindClosestPoint(obs(i,0), obs(i,1),globalPath, minIndex,carindex); //寻找距离最近的点 
            SL[i] = FindSL(globalPath,minIndex,carindex,obs(i,0),obs(i,1));//寻找每一点的SL 利用线性插值法 
            minIndices[i]=minIndex;
        }
         // 计算中心点的 SL 值
        double sum_s = 0.0;
        double sum_l = 0.0;
        for (size_t i = 0; i < obs.rows(); i++){
            sum_s+=SL[i][0];
            sum_l+=SL[i][1];
            switch (i){
            case 0:
                obsfrent.point1.l = SL[i][1];
                obsfrent.point1.s = SL[i][0];
                obsfrent.point1.index = minIndices[i];
                break;
            case 1:
                obsfrent.point2.l = SL[i][1];
                obsfrent.point2.s = SL[i][0];
                obsfrent.point2.index = minIndices[i];
                break;
            case 2:
                obsfrent.point3.l = SL[i][1];
                obsfrent.point3.s = SL[i][0];
                obsfrent.point3.index = minIndices[i];
                break;
            case 3:
                obsfrent.point4.l = SL[i][1];
                obsfrent.point4.s = SL[i][0];
                obsfrent.point4.index = minIndices[i];
                break;
            default:
                break;
            }

        }
         // 计算中心点的 SL 值
        obsfrent.centre_points.s = sum_s / 4;
        obsfrent.centre_points.l = sum_l / 4;
    }

    std::array<double, 2> FindSL(Eigen::MatrixXd &globalPath,int minIndex,int carindex, double x, double y){
        std::array<double,2>result;
        if(minIndex==0||minIndex==globalPath.cols()-1){
            double dx = x- globalPath(0, minIndex);
            double dy = y - globalPath(1, minIndex);
            double cos_theta_r = std::cos(globalPath(3, minIndex));
            double sin_theta_r = std::sin(globalPath(3, minIndex));
            double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            result[0] = globalPath(6, minIndex);
            result[1] = std::copysign(cross_rd_nd, cross_rd_nd);
        }else{
            double v0x = x - globalPath(0,minIndex-1);
            double v0y = y - globalPath(1,minIndex-1);

            double v1x = globalPath(0,minIndex+1)-globalPath(0,minIndex-1);
            double v1y = globalPath(1,minIndex+1)-globalPath(1,minIndex-1);

            double v0_norm = std::sqrt(v0x*v0x+v0y*v0y);
            double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
            double dot = v0x * v1x + v0y * v1y;
            // v0 · v1 = ||v0||*||v1||*cos(theta)    theta为v0和v1的夹角
            // (v0 · v1) / ||v1|| = ||v0||*cos(theta)
            // ||v0||*cos(theta) 即为v0在v1上的投影
            double delta_s = dot/v1_norm;
            double s = delta_s + globalPath(6,minIndex-1);
            double l = std::sqrt(v0_norm*v0_norm-delta_s*delta_s);

            //判别l的正负号 
            double dx = x- globalPath(0, minIndex);
            double dy = y - globalPath(1, minIndex);
            double cos_theta_r = std::cos(globalPath(3, minIndex));
            double sin_theta_r = std::sin(globalPath(3, minIndex));
            double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            l = std::copysign(l,cross_rd_nd);
            result[0] = std::abs(s);
            result[1] = l;
        }
        return result;
    }

    void CalculateobsesSD(frentPoint vehicle_position, Eigen::MatrixXd& globalPath_,Eigen::MatrixXd &obs,std::vector<obses_sd> &sd,
                        std::vector<Eigen::VectorXd> &obses_limit,Eigen::MatrixXd &obses_base_lidar,std::vector<Eigen::VectorXd> &obs_limits_distance,
                        int carindex){
                                     //清空传输进来的上一次的数据 
        sd.clear();
        obses_limit.clear();
        obs_limits_distance.clear();   
        //std::cout<<"检测到障碍物的个数"<<obs.cols()<<std::endl;
        int obsIndex; //障碍物Frenet位置, 
        obses_sd sd_;
        double min_s;
        double max_s;
        double min_l;
        for (size_t i = 0; i < obs.cols(); ++i){
            lidar_obs_Frenet(obs.col(i),globalPath_,sd_,carindex);
            //将符合距离的障碍物加入到其中 
            /***
             *   长度 60m
             *    -2.9~2.9 对每一个点进行判断   
             * 
             * **/
            min_s = std::min({sd_.point1.s, sd_.point2.s, sd_.point3.s, sd_.point4.s});
            max_s = std::max({sd_.point1.s, sd_.point2.s, sd_.point3.s, sd_.point4.s});
            double weigth = std::abs(sd_.point4.l - sd_.point1.l);
            if((min_s-vehicle_position.s) <= 30){
                if(decide_obs_true_false(sd_,2.9,-2.9)){ //之前时 2.5 ～ -2.9
                    sd_.min_s = min_s;
                    sd_.max_s = max_s;
                    sd_.wigth = weigth;
                    sd.emplace_back(sd_);
                    obses_limit.emplace_back(obses_base_lidar.col(i)); //这里的障碍物和sd里的是一一对应的  原始的障碍物 基于雷达坐标系的
                    obs_limits_distance.emplace_back(obs.col(i));//全局坐标系 
                }
            }
        }
        
        // std::sort(sd.begin(), sd.end(),
        //     [](obses_sd &a, obses_sd&b){
        //         return a.centre_points.s < b.centre_points.s; 
        //     });
    }

    //障碍物横向范围 -2.9 ～ 2.9  
    int decide_obs_true_false(obses_sd &sd,double left_bound,double right_bound){
        for (size_t i = 1; i <=4; ++i){
            double l_value=0.0;
            switch (i){
                case 1:
                    l_value = sd.point1.l;
                    break;
                case 2:
                    l_value = sd.point2.l;
                    break;
                case 3:
                    l_value = sd.point3.l;
                    break;
                case 4:
                    l_value = sd.point4.l;
                    break;
                default:
                    break;
            }
             // Check if the l_value is within the bounds
            if (l_value <= left_bound && l_value >= right_bound) {
                //std::cout << "point" << i << " : " << l_value << std::endl;
                return 1;  // Found a valid point, return true
            }
        }
        if (sd.centre_points.l <= left_bound && sd.centre_points.l >= right_bound ) {
            return 1;
        }
        return 0;
    }


    void get_car_fourpoint(double length,double wigth,Eigen::VectorXd car_center,Eigen::MatrixXd& car_Fpoint)
	{
		/*
            2-------3
            |       |
            1-------4
			车子的四个顶点 
        */
		car_Fpoint.resize(2,4);
		double thea = car_center[3]; //转换
        double zhouju = 1.308;
        double cos_thea = cos(thea);
        double sin_thea = sin(thea);
        double term0 = car_center[0] + zhouju * cos_thea;
        double term1 = car_center[1] + zhouju * sin_thea;//恢复到车体的中心 
		//1
		car_Fpoint(0,0) = (-length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		car_Fpoint(1,0) = (-length/2)*sin_thea+(-wigth/2)*cos_thea+term1;

		//2
		car_Fpoint(0,1) = (-length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		car_Fpoint(1,1) = (-length/2)*sin_thea+(wigth/2)*cos_thea+term1;

		//3
		car_Fpoint(0,2) = (length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		car_Fpoint(1,2) = (length/2)*sin_thea+(wigth/2)*cos_thea+term1;

		//4
		car_Fpoint(0,3) = (length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		car_Fpoint(1,3) = (length/2)*sin_thea+(-wigth/2)*cos_thea+term1;
	}
    
    //每一个点与所有障碍物进行比较 这样如果有碰撞 速度会加快 
    bool HasOverlap(Eigen::MatrixXd &car_point,std::vector<Eigen::VectorXd> &obses, double thea){
        if(obses.size()==0){
            return 0;
        }
        double car_x_min,car_y_min,car_x_max,car_y_max,car_centre_x,car_centre_y,car_half_length,car_half_width;
        car_half_length = 4.532/2; car_half_width=1.814/2;
        double obs_x_min,obs_y_min,obs_x_max,obs_y_max,obs_centre_x,obs_centre_y,obs_half_length,obs_half_width;    
        //计算车辆的相关信息 最大x y 最小 x y 中心点坐标 
        car_x_min = std::min({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
        car_y_min = std::min({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
        car_x_max = std::max({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
        car_y_max = std::max({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
        car_centre_x = (car_point(0,0)+car_point(0,1)+car_point(0,2)+car_point(0,3))/4;
        car_centre_y = (car_point(1,0)+car_point(1,1)+car_point(1,2)+car_point(1,3))/4;
        double cos_heading_ = std::cos(thea);    
        double sin_heading_ = std::sin(thea);
        for (const auto &obs : obses){
            double obs_1_x = obs(0);
            double obs_1_y = obs(1);

            double obs_2_x = obs(0);
            double obs_2_y = obs(3);

            double obs_3_x = obs(2);
            double obs_3_y = obs(3);

            double obs_4_x = obs(2);
            double obs_4_y = obs(1);

            obs_x_min = std::min({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
            obs_y_min = std::min({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
            obs_x_max = std::max({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
            obs_y_max = std::max({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
            obs_centre_x = (obs_1_x+obs_2_x+obs_3_x+obs_4_x)/4;
            obs_centre_y = (obs_1_y+obs_2_y+obs_3_y+obs_4_y)/4;
            obs_half_length = std::abs(obs_4_x-obs_1_x)/2;
            obs_half_width  = std::abs(obs_2_y-obs_1_y)/2;
            double obs_cos_heading = std::cos(0);
            double obs_sin_heading = std::sin(0);
            if(car_x_max<obs_x_min||car_x_min>obs_x_max||car_y_max<obs_y_min||car_y_min>obs_y_max){
                continue;
            }else{
                const double shift_x = obs_centre_x - car_centre_x;
                const double shift_y = obs_centre_y - car_centre_y;
                const double dx1 = cos_heading_ * car_half_length;
                const double dy1 = sin_heading_ * car_half_length;
                const double dx2 = sin_heading_ * car_half_width;
                const double dy2 = -cos_heading_ * car_half_width;

                const double dx3 = obs_cos_heading * obs_half_length;
                const double dy3 = obs_sin_heading * obs_half_length;
                const double dx4 = obs_sin_heading * obs_half_width;
                const double dy4 = -obs_cos_heading* obs_half_width;

                //f3
                if(std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
                            std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                                std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                                car_half_length &&
                //f4
                        std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
                            std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                                std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                                car_half_width &&
                //f5
                        std::abs(shift_x * obs_cos_heading + shift_y * obs_sin_heading) <=
                            std::abs(dx1 *obs_cos_heading + dy1 * obs_sin_heading) +
                                std::abs(dx2 * obs_cos_heading + dy2 * obs_sin_heading) +
                                obs_half_length &&
                //f6
                        std::abs(shift_x * obs_sin_heading - shift_y * obs_cos_heading) <=
                            std::abs(dx1 * obs_sin_heading - dy1 * obs_cos_heading) +
                                std::abs(dx2 * obs_sin_heading - dy2 * obs_cos_heading) +
                                obs_half_width){
                    return true;
                }else{
                    continue;
                }
            }
        }
        return false;
    }
    
    bool HasOverlap(std::vector<Eigen::MatrixXd> &carpoint_points,std::vector<Eigen::VectorXd> &obses,Eigen::MatrixXd &opty){
        if(obses.size()==0||carpoint_points.size()==0||carpoint_points.size()!=opty.cols()){
            return false;
        }
        double car_x_min,car_y_min,car_x_max,car_y_max,car_centre_x,car_centre_y,car_half_length,car_half_width;
        car_half_length = 4.575/2; car_half_width=1.825/2;
        double obs_x_min,obs_y_min,obs_x_max,obs_y_max,obs_centre_x,obs_centre_y,obs_half_length,obs_half_width;    
        //计算车辆的相关信息 最大x y 最小 x y 中心点坐标 
        int size=0;
        for(const auto &car_point : carpoint_points){
            car_x_min = std::min({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
            car_y_min = std::min({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
            car_x_max = std::max({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
            car_y_max = std::max({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
            car_centre_x = (car_point(0,0)+car_point(0,1)+car_point(0,2)+car_point(0,3))/4;
            car_centre_y = (car_point(1,0)+car_point(1,1)+car_point(1,2)+car_point(1,3))/4;
            double cos_heading_ = std::cos(opty(3,size));    
            double sin_heading_ = std::sin(opty(3,size));
            size++;
            for (const auto &obs : obses){
                double obs_1_x = obs(0);
                double obs_1_y = obs(1);

                double obs_2_x = obs(0);
                double obs_2_y = obs(3);

                double obs_3_x = obs(2);
                double obs_3_y = obs(3);

                double obs_4_x = obs(2);
                double obs_4_y = obs(1);

                obs_x_min = std::min({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
                obs_y_min = std::min({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
                obs_x_max = std::max({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
                obs_y_max = std::max({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
                obs_centre_x = (obs_1_x+obs_2_x+obs_3_x+obs_4_x)/4;
                obs_centre_y = (obs_1_y+obs_2_y+obs_3_y+obs_4_y)/4;
                obs_half_length = std::abs(obs_4_x-obs_1_x)/2;
                obs_half_width  = std::abs(obs_2_y-obs_1_y)/2;
                double obs_cos_heading = std::cos(0);
                double obs_sin_heading = std::sin(0);
                if(car_x_max<obs_x_min||car_x_min>obs_x_max||car_y_max<obs_y_min||car_y_min>obs_y_max){
                    continue;
                }else{
                    const double shift_x = obs_centre_x - car_centre_x;
                    const double shift_y = obs_centre_y - car_centre_y;
                    const double dx1 = cos_heading_ * car_half_length;
                    const double dy1 = sin_heading_ * car_half_length;
                    const double dx2 = sin_heading_ * car_half_width;
                    const double dy2 = -cos_heading_ * car_half_width;

                    const double dx3 = obs_cos_heading * obs_half_length;
                    const double dy3 = obs_sin_heading * obs_half_length;
                    const double dx4 = obs_sin_heading * obs_half_width;
                    const double dy4 = -obs_cos_heading* obs_half_width;

                    //f3
                    if(std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
                                std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                                    std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                                    car_half_length &&
                    //f4
                            std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
                                std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                                    std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                                    car_half_width &&
                    //f5
                            std::abs(shift_x * obs_cos_heading + shift_y * obs_sin_heading) <=
                                std::abs(dx1 *obs_cos_heading + dy1 * obs_sin_heading) +
                                    std::abs(dx2 * obs_cos_heading + dy2 * obs_sin_heading) +
                                    obs_half_length &&
                    //f6
                            std::abs(shift_x * obs_sin_heading - shift_y * obs_cos_heading) <=
                                std::abs(dx1 * obs_sin_heading - dy1 * obs_cos_heading) +
                                    std::abs(dx2 * obs_sin_heading - dy2 * obs_cos_heading) +
                                    obs_half_width){
                        return true;
                    }else{
                        continue;
                    }
                }
            }
        }
        return false;
    }

    bool HasOverlap(std::vector<Eigen::MatrixXd> &carpoint_points,Eigen::VectorXd &obs, Eigen::MatrixXd &opty, size_t &index){
        if(carpoint_points.size() ==0 ||carpoint_points.size() != opty.cols()){
            return false;
        }
        double car_x_min,car_y_min,car_x_max,car_y_max,car_centre_x,car_centre_y,car_half_length,car_half_width;
        car_half_length = 4.575/2; car_half_width=1.825/2;
        double obs_x_min,obs_y_min,obs_x_max,obs_y_max,obs_centre_x,obs_centre_y,obs_half_length,obs_half_width;    
        //计算车辆的相关信息 最大x y 最小 x y 中心点坐标 
        int size=0;
        for(size_t i = 0; i< carpoint_points.size(); ++i){
            auto &car_point =  carpoint_points[i];
            std::array<double, 4> x_list = {car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)};
            std::array<double, 4> y_list = {car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)};
            // 计算最小和最大值
            double car_x_min = *std::min_element(x_list.begin(), x_list.end());
            double car_y_min = *std::min_element(y_list.begin(), y_list.end());
            double car_x_max = *std::max_element(x_list.begin(), x_list.end());
            double car_y_max = *std::max_element(y_list.begin(), y_list.end());
            car_centre_x = (car_point(0,0)+ car_point(0,1) + car_point(0,2) + car_point(0,3)) / 4;
            car_centre_y = (car_point(1,0)+ car_point(1,1) + car_point(1,2) + car_point(1,3)) / 4;
            double cos_heading_ = std::cos(opty(3, i));    
            double sin_heading_ = std::sin(opty(3, i));
            //size++;
            double obs_1_x = obs(0);
            double obs_1_y = obs(1);

            double obs_2_x = obs(0);
            double obs_2_y = obs(3);

            double obs_3_x = obs(2);
            double obs_3_y = obs(3);

            double obs_4_x = obs(2);
            double obs_4_y = obs(1);

            obs_x_min = std::min({obs_1_x, obs_2_x, obs_3_x, obs_4_x});
            obs_y_min = std::min({obs_1_y, obs_2_y, obs_3_y, obs_4_y});
            obs_x_max = std::max({obs_1_x, obs_2_x, obs_3_x, obs_4_x});
            obs_y_max = std::max({obs_1_y, obs_2_y, obs_3_y, obs_4_y});
            
            obs_centre_x = (obs_1_x + obs_2_x + obs_3_x + obs_4_x) / 4;
            obs_centre_y = (obs_1_y + obs_2_y + obs_3_y + obs_4_y) / 4;
            obs_half_length = std::abs(obs_4_x - obs_1_x) / 2;
            obs_half_width  = std::abs(obs_2_y - obs_1_y) / 2;
            double obs_cos_heading = std::cos(0);
            double obs_sin_heading = std::sin(0);
            if(car_x_max < obs_x_min || car_x_min  >obs_x_max ||
                car_y_max < obs_y_min || car_y_min > obs_y_max){
                continue;
            }else{
                const double shift_x = obs_centre_x - car_centre_x;
                const double shift_y = obs_centre_y - car_centre_y;
                const double dx1 = cos_heading_ * car_half_length;
                const double dy1 = sin_heading_ * car_half_length;
                const double dx2 = sin_heading_ * car_half_width;
                const double dy2 = -cos_heading_ * car_half_width;

                const double dx3 = obs_cos_heading * obs_half_length;
                const double dy3 = obs_sin_heading * obs_half_length;
                const double dx4 = obs_sin_heading * obs_half_width;
                const double dy4 = -obs_cos_heading* obs_half_width;

                //f3
                if(std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
                            std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                                std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                                car_half_length &&
                //f4
                        std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
                            std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                                std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                                car_half_width &&
                //f5
                        std::abs(shift_x * obs_cos_heading + shift_y * obs_sin_heading) <=
                            std::abs(dx1 *obs_cos_heading + dy1 * obs_sin_heading) +
                                std::abs(dx2 * obs_cos_heading + dy2 * obs_sin_heading) +
                                obs_half_length &&
                //f6
                        std::abs(shift_x * obs_sin_heading - shift_y * obs_cos_heading) <=
                            std::abs(dx1 * obs_sin_heading - dy1 * obs_cos_heading) +
                                std::abs(dx2 * obs_sin_heading - dy2 * obs_cos_heading) +
                                obs_half_width){
                    index = i ;
                    return true;
                } else {
                    continue;
                }
            }
            
        }
        return false;
    }

    //面积比较的方法进行碰撞检测  carpoint 为局部路径上的点 obses为障碍物上的坐标 
	int Area_Comparison(std::vector<Eigen::MatrixXd> &carpoint,std::vector<Eigen::VectorXd> &obses)
	{
        // std::cout<<"carpoint: "<<carpoint.size()<<std::endl;
        // std::cout<<"obses: "<<obses.size()<<std::endl;
        if(obses.size()==0){
            return 0;
        }
		for (size_t i = 0; i < carpoint.size(); ++i){
			for (size_t j = 0; j < obses.size(); ++j){
				// 获取障碍物包围盒的坐标 min_x min_y   max_x  max_y 四个顶点坐标
				/*  3-------2
					|       |
					4-------1*/
				double obs_1_x = obses[j](0);
				double obs_1_y = obses[j](1);

				double obs_2_x = obses[j](0);
				double obs_2_y = obses[j](3);

				double obs_3_x = obses[j](2);
				double obs_3_y = obses[j](3);

				double obs_4_x = obses[j](2);
				double obs_4_y = obses[j](1);
				
				double area = std::abs(obs_2_y-obs_1_y)*std::abs(obs_3_x-obs_2_x);//障碍物面积
				double l1_2 = std::sqrt(std::pow(obs_2_x-obs_1_x,2)+std::pow(obs_2_y-obs_1_y,2));
				double l2_3 = std::sqrt(std::pow(obs_3_x-obs_2_x,2)+std::pow(obs_3_y-obs_2_y,2));
				double l3_4 = std::sqrt(std::pow(obs_4_x-obs_3_x,2)+std::pow(obs_4_y-obs_3_y,2));
				double l4_1 = std::sqrt(std::pow(obs_1_x-obs_4_x,2)+std::pow(obs_1_y-obs_4_y,2));
				//对车辆的四个点进行判断 是否在障碍物相交 
				for (size_t k = 0; k < 4; k++){
					double L1 = std::sqrt(std::pow(carpoint[i](0,k)-obs_1_x,2)+std::pow(carpoint[i](1,k)-obs_1_y,2));
					double L2 = std::sqrt(std::pow(carpoint[i](0,k)-obs_2_x,2)+std::pow(carpoint[i](1,k)-obs_2_y,2));
					double L3 = std::sqrt(std::pow(carpoint[i](0,k)-obs_3_x,2)+std::pow(carpoint[i](1,k)-obs_3_y,2));
					double L4 = std::sqrt(std::pow(carpoint[i](0,k)-obs_4_x,2)+std::pow(carpoint[i](1,k)-obs_4_y,2));

					//计算三角形的面积之和 
					double p12 = (L1+L2+l1_2)/2;
					double s12 = std::sqrt(p12*(p12-L1)*(p12-L2)*(p12-l1_2));

					double p23 = (L2+L3+l2_3)/2;
					double s23 = std::sqrt(p23*(p23-L3)*(p23-L2)*(p23-l2_3));

					double p34 = (L3+L4+l3_4)/2;
					double s34 = std::sqrt(p34*(p34-L3)*(p34-L4)*(p34-l3_4));

					double p41 = (L1+L4+l4_1)/2;
					double s41= std::sqrt(p41*(p41-L1)*(p41-L4)*(p41-l4_1));
                    //没有碰撞 
					if ((s12+s23+s34+s41)>area){
						continue;
					}
					else
					{return 1;}					
				}				
			}
		}
        return 0;
    }


}




