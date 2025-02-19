#include "localFuns.h"
#include "omp.h"
#include <iostream>
#include <fstream>
#include <string>
namespace local{
    int getOptimalLocalTraj(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, Eigen::MatrixXd& obs1, 
        tool::controlParams& cps, Eigen::MatrixXd& cartesianTrajt, Eigen::MatrixXd& frenetTraj, int &minIndex, bool fre,
        std::vector<Eigen::MatrixXd> &all_local_points,std::vector<Eigen::VectorXd> &obsess,Eigen::MatrixXd obs_lidar){
        int optimal;//最优局部路径下标
        // int minIndex;//车辆与全局路径最近点的下标
        //controlParams cps; //控制参数定义
        std::vector<Eigen::MatrixXd> localTrajs;//存储最优局部路径
        optimal = genFrenetLocalTrajs(carRealPosition, globalPath, obs1, cps, localTrajs, minIndex,all_local_points,obsess,obs_lidar);
        // optimal = -1;
        if (optimal < 0) {
                std::cout << "optimal<0!!" << std::endl;
                return 1;
        }  
        if(fre){
            frenetTraj.resize(2, localTrajs[optimal].cols()); 
            for (int i = 0; i < localTrajs[optimal].cols(); i++) {		
                frenetTraj(0, i) = localTrajs[optimal](0, i);
                frenetTraj(1, i) = localTrajs[optimal](1, i);
            }
        }
        Eigen::MatrixXd cartesianXY; //cartesian坐标系下xy
        // optimal=0;
        genCartesianLocalTrajs(localTrajs, globalPath, optimal, minIndex, cps, cartesianXY); //获取cartesian坐标系下xy
        referencePath(cartesianXY, cps.vEnd, cartesianTrajt);//由xy计算参考路径参数cartesian坐标系下
        return 0;
    }

    int showCartesianLocalTrajs(const std::vector<Eigen::MatrixXd>& frenetLocalTrajs, const Eigen::MatrixXd& globalPath, const int& optimal, const int& initMinIndex, const tool::controlParams cps, std::vector<Eigen::MatrixXd>& allCartesianXY){
        int endIndex = (initMinIndex + cps.lp)> globalPath.cols()? globalPath.cols(): (initMinIndex + cps.lp);
        int minIndex = -1;
        Eigen::MatrixXd cartesianXY;
        cartesianXY.resize(2, frenetLocalTrajs[optimal].cols());
        Eigen::Vector2d XY;
        for (int k = 0; k < cps.n; k++) {
            //std::cout << "k: " << k << std::endl;
            for (int i = 0; i < frenetLocalTrajs[k].cols(); i++) {
                //std::cout << "i: " << i << std::endl;
                double distance, d_min = 999999;
                for (int j = initMinIndex; j < endIndex; j++) {
                    distance = std::fabs(globalPath(6, j) - frenetLocalTrajs[k](0, i));
                    //std::cout << "distance: " << distance << std::endl;
                    if (distance < d_min) {
                        minIndex = j;
                        d_min = distance;
                    }
                }
                //std::cout << "最近点下标： " << minIndex << std::endl;
                frenet2Cartesian(globalPath, frenetLocalTrajs[k](0, i), frenetLocalTrajs[k](1, i), minIndex, XY);
                cartesianXY.col(i) = XY;
            }
            allCartesianXY.push_back(cartesianXY);
        }
        return 0;
    }

    int genCartesianLocalTrajs(std::vector<Eigen::MatrixXd>& frenetLocalTrajs, const Eigen::MatrixXd& globalPath, const int& optimal, const int& initMinIndex, const tool::controlParams cps, Eigen::MatrixXd& cartesianXY){
        int endIndex = (initMinIndex + cps.lp) > globalPath.cols() ? globalPath.cols() -1: (initMinIndex + cps.lp);
        
        int minIndex = -1;
        int trajLon=0;
        Eigen::MatrixXd TcartesianXY;
        TcartesianXY.resize(2, frenetLocalTrajs[optimal].cols());
        Eigen::Vector2d XY;
        Eigen::Vector2d old_XY;
        int DEL=0;
        for (int i = 0; i < frenetLocalTrajs[optimal].cols(); i++) {
            if(globalPath(6,globalPath.cols()-1)>frenetLocalTrajs[optimal](0, i)){
                trajLon=i;
                // break;
            }
            // std::cout << "i: " << i << std::endl;
            double distance, d_min = 999999;
            for (int j = initMinIndex; j < endIndex; j++) {
                
                distance = std::fabs(globalPath(6, j) - frenetLocalTrajs[optimal](0, i));
                
                if (distance < d_min) {
                    minIndex = j;
                    d_min = distance;
                }
            }         
            frenet2Cartesian(globalPath, frenetLocalTrajs[optimal](0, i), frenetLocalTrajs[optimal](1, i), minIndex, XY);
            //??? 
            /*使用一些条件判断，对于相邻的两个笛卡尔坐标点，如果它们之间的距离小于某个阈值（0.3），则跳过一个点，以减少轨迹中的点的数量*/
            if(i!=0){
                double DELTA=std::fabs(XY(0)-old_XY(0))+std::fabs(XY(1)-old_XY(1));
                if(DELTA>0.3){
                    TcartesianXY.col(i-DEL) = XY;
                }else{
                    DEL+=1;
                    // TcartesianXY.col(i) = XY;
                }
            }
            else{
                TcartesianXY.col(i) = XY;
            }
            old_XY = XY;
            // TcartesianXY.col(i) = XY;
        }
        cartesianXY=TcartesianXY.block(0,0,2,trajLon-DEL);
        return 0;
    }
    
    int genFrenetLocalTrajs(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, const Eigen::MatrixXd& obs, tool::controlParams& cps, 
        std::vector<Eigen::MatrixXd>& localTrajs, int& minIndex, std::vector<Eigen::MatrixXd> &all_local_points, std::vector<Eigen::VectorXd> &obsess,
        Eigen::MatrixXd obs_lidar){
        frentPoint carFrentPoint; //车辆Frenet坐标位置
        //int minIndex; //车辆与全局路径的最近点下标作为参数传递到函数外
        int carIndex;
        cartesian2Frenet(carRealPosition, globalPath, carFrentPoint, minIndex,carIndex,0);//车辆坐标变换cartesian2frenet  s d 
        
        // cps.tT =(globalPath(5,globalPath.cols()-1)-globalPath(5,minIndex)<cps.vEnd*cps.tT)?(globalPath(5,globalPath.cols()-1)-globalPath(5,minIndex))/cps.vEnd:cps.tT; 
    
        std::vector<Eigen::VectorXd> obses;//障碍物 的xy 
        std::vector<Eigen::VectorXd> obses_sd;//障碍物的 sd 
        std::vector<frentPoint > obs1FrentPoint; 
        int obsIndex; //障碍物Frenet位置, 
        std::cout<<"检测到障碍物的个数"<<obs.cols()<<std::endl;
        for (int i = 0; i < obs.cols(); i++) {
            lidar_obs_Frenet(obs.col(i), globalPath, obs1FrentPoint, obsIndex,carIndex);  //将障碍物坐标转化为 sd 
            double s_min = 99999;
            double d_min = 99999;
            for (size_t j = 0; j < obs1FrentPoint.size(); j++)
            {
                if(obs1FrentPoint[j].s<s_min){s_min=obs1FrentPoint[j].s;} //寻找距离车辆最近点的 s 
                if (std::abs(obs1FrentPoint[j].d)<d_min){d_min = std::abs(obs1FrentPoint[j].d);}//寻找距离车道线最近的d 
            }
            obs1FrentPoint.clear();
            //只有同时满足这两个条件 才能视为阻碍车辆行进的障碍物 
            if ((s_min-carFrentPoint.s)<=50&&d_min<6)
            {
                obses.push_back(obs.col(i));//将基于全局坐标系的在范围内的障碍物加入其中
                obsess.push_back(obs_lidar.col(i));//将基于激光雷达坐标系的在范围内的障碍物加入其中
            }
          
        }
        std::cout<<"在范围内障碍物的个数"<<obses.size()<<std::endl;

        Eigen::Vector4d startPoint; //车辆初始状态
        // startPoint << carFrentPoint.s, carFrentPoint.d, cps.vEnd, tool::normalizeAngle(tool::d2r(carRealPosition(3)) - globalPath(3, minIndex));
        startPoint << carFrentPoint.s, carFrentPoint.d, cps.vEnd, tool::normalizeAngle(tool::d2r(carRealPosition(3)));
        Eigen::MatrixXd endPointSeries;
        endPointSeries.resize(cps.n, 4);  //采样点终点序列

        tool::states xyStates;
        Eigen::MatrixXd A, B; //n条路径的纵向(x)、横向（y）三次多项式系数矩阵
        A.resize(4, cps.n), B.resize(4, cps.n);
        Eigen::Vector4d Ai, Bi; //第i条路径的系数向量

        Eigen::MatrixXd localTraj;//存储每一条局部路径
        localTraj.resize(8, cps.lp); //局部路径存储

        int optimal = -1; //最优路径标号
        Eigen::VectorXd cost_one_Traj;
        // std::cout << "cps.n" << cps.n << std::endl;
        // std::cout << "obs.cols()" << obs.cols() << std::endl;    
        // Eigen::MatrixXd all_trajs_cost;
        // all_trajs_cost.resize(cps.n, obs.cols());

        Eigen::VectorXd all_traj_max;
        all_traj_max.resize(cps.n);

        for (int i = 0; i < cps.n; i++) {
            endPointSeries.row(i) << startPoint(0) + cps.l, cps.d - i * 2, cps.vEnd, 0;          
            xyStates.x0 = startPoint(0);
            xyStates.y0 = startPoint(1);    
            xyStates.vx0 = startPoint(2)*cos(startPoint(3));
            xyStates.vy0 = startPoint(2)*sin(startPoint(3));
            xyStates.xT = endPointSeries(i, 0);
            xyStates.yT = endPointSeries(i, 1);
            xyStates.vxT = endPointSeries(i, 2)*cos(endPointSeries(i, 3));
            xyStates.vyT = endPointSeries(i, 2)*sin(endPointSeries(i, 3));

            double one_traj_max_cost = evalfun(cps, xyStates, obses, Ai, Bi, cost_one_Traj, localTraj,globalPath,all_local_points);	         
            A.col(i) = Ai;
            B.col(i) = Bi;
            all_traj_max(i) = one_traj_max_cost;
            localTrajs.push_back(localTraj); //每条局部路径压入vector
        }
        Eigen::VectorXd::Index min_index_trajs;
        double min_val_trajs = all_traj_max.minCoeff(&min_index_trajs);
        if (min_val_trajs < 99999){
            optimal = min_index_trajs;
        }
        return optimal;
    }

    double evalfun(const tool::controlParams& cps, const tool::states& xyStates, const std::vector<Eigen::VectorXd> & obstructs, Eigen::Vector4d& Ai, Eigen::Vector4d& Bi, 
    Eigen::VectorXd& cost_one_Traj, Eigen::MatrixXd& localTraj,const Eigen::MatrixXd globalPath, std::vector<Eigen::MatrixXd> &all_local_points){
        /*评价函数：*/
        Ai = cubicCoffec(cps.tT, xyStates.x0, xyStates.vx0, xyStates.xT, xyStates.vxT);
        Bi = cubicCoffec(cps.tT, xyStates.y0, xyStates.vy0, xyStates.yT, xyStates.vyT);
        //计算障碍物到各轨迹从起点到终点的直线距离选出避障轨迹
        double A = xyStates.yT - xyStates.y0;
        double B = xyStates.x0 - xyStates.xT;
        double C = xyStates.y0 * xyStates.xT - xyStates.x0 * xyStates.yT;
        double jerkmaxY = 0;
        for (int i = 0; i < cps.lp; i++) {
            double t = i * cps.deltaT;
            Eigen::Vector4d positionCoffectTime, velocityCoffectTime, accelertCoffectTime, jerkCoffectTime;
            positionCoffectTime <<     pow(t, 3),  pow(t, 2), t, 1;
            velocityCoffectTime << 3 * pow(t, 2),      2 * t, 1, 0;
            accelertCoffectTime <<         6 * t,          2, 0, 0;
                jerkCoffectTime <<             6,          0, 0, 0;
            localTraj(0, i) = positionCoffectTime.transpose() * Ai; //s,Frenet纵向位移
            localTraj(1, i) = positionCoffectTime.transpose() * Bi; //d,Frenet横向位移
            localTraj(2, i) = velocityCoffectTime.transpose() * Ai; //dot_s,Frenet纵向速度
            localTraj(3, i) = velocityCoffectTime.transpose() * Bi; //dot_d,Frenet横向速度
            localTraj(4, i) = accelertCoffectTime.transpose() * Ai; //ddot_s,Frenet纵向加速度
            localTraj(5, i) = accelertCoffectTime.transpose() * Bi; //ddot_d,Frenet横向加速度
            localTraj(6, i) =     jerkCoffectTime.transpose() * Ai; //tdot_s,Frenet纵向jerk
            localTraj(7, i) =     jerkCoffectTime.transpose() * Bi; //tdot_d,Frenet横向jerk
            if (jerkmaxY < localTraj(7, i)) {
                jerkmaxY = localTraj(7, i); //每一条轨迹的最大jerk值
            }
        }
        //先将sd的局部路径转化为xy路径进行碰撞检测
        Eigen::MatrixXd  linshi_xy,linshi_path;
        linshi_xy.resize(2,localTraj.cols());
        for (size_t i = 0; i < localTraj.cols(); i++)
        {
            double distance, d_min = 999999; 
            int min_dex = 0;
            for (int j = 0; j < globalPath.cols(); j++) {
                // std::cout<<globalPath(5, j)<<std::endl;
                // std::cout<<optimal<<"   "<<i<<std::endl;
                distance = std::fabs(globalPath(6, j) - localTraj(0,i));
                // std::cout << "distance: " << distance << std::endl;
                if (distance < d_min) {
                    min_dex = j;
                    d_min = distance;
                }
            }
            Eigen::Vector2d cartesianXY;
            frenet2Cartesian(globalPath,localTraj(0,i),localTraj(1,i),min_dex,cartesianXY);
            linshi_xy.col(i) =cartesianXY;
        }
        linshi_path.resize(3,linshi_xy.cols());
        calthea(linshi_xy,linshi_path);//计算局部路径的x y  thea

        //碰撞检测 
        double length = 4.572;
        double wigth = 1.825;
        Eigen::MatrixXd car_Fpoint;
        std::vector<Eigen::MatrixXd> all_local_pointss;
        for (size_t i = 0; i < linshi_path.cols(); i++)
        {
            linshi_path(0,i) = linshi_path(0,i) + 2.647/2*cos(linshi_path(2,i));
            linshi_path(1,i) = linshi_path(1,i) + 2.647/2*sin(linshi_path(2,i));
            tool::get_car_fourpoint(length,wigth,linshi_path.col(i),car_Fpoint);//由车辆的中心点 计算局部路径上的每个店对应的车辆的四个顶点
            all_local_pointss.push_back(car_Fpoint);
        }
        // std::ofstream outFile;
        // static size_t  SIZE=1;
        // std::string filename = "/home/mm/v2x/FILE/file_"+std::to_string(SIZE)+".txt";
        // std::string filename_obs = "/home/mm/v2x/FILE/obs_file_"+std::to_string(SIZE)+".txt";
        // outFile.open(filename);
        // for (size_t i = 0; i < all_local_pointss.size(); i++)
        // {
        //     outFile<<all_local_pointss[i](0,0)<<" "<<all_local_pointss[i](1,0)<<" "<<
        //             all_local_pointss[i](0,1)<<" "<<all_local_pointss[i](1,1)<<" "<<
        //             all_local_pointss[i](0,2)<<" "<<all_local_pointss[i](1,2)<<" "<<
        //             all_local_pointss[i](0,3)<<" "<<all_local_pointss[i](1,3)<<std::endl;
        // }
        // outFile.close();
        // outFile.open(filename_obs);
        // for (size_t i = 0; i < obstructs.size(); i++)
        // { 
        //     outFile<<obstructs[i](0)<<" "<<obstructs[i](1)<<" "<<
        //         obstructs[i](0)<<" "<<obstructs[i](3)<<" "<<
        //         obstructs[i](2)<<" "<<obstructs[i](3)<<" "<<
        //             obstructs[i](2)<<" "<<obstructs[i](1)<<" "<<std::endl;
        // }
        // outFile.close();
        // SIZE++;
        //利用面积比较法对路径上的点和障碍物的点进行碰撞检测 
        double cost_coeff_distance=1.0, cost_coeff_righted=2;
        if (tool::Area_Comparison(all_local_pointss,obstructs)==1) //有碰撞 
        {
           std::cout<<"attention"<<std::endl;
           //靠右侧的局部路径会有碰撞
        //    if (xyStates.yT==-1.5){
        //         cost_coeff_distance=2;
        //         cost_coeff_righted=0.5;
        //    };
           return 99999;//返回比较大的代价 
        }
        else{
            //惩罚函数中加入轨迹到障碍物的距离 
            double all_distance ;
            for (size_t i = 0; i < obstructs.size(); i++)
            {
                double distance, d_min = 999999; 
                double center_x = (obstructs[i](0) + obstructs[i](2)) / 2;
                double center_y = (obstructs[i](1) + obstructs[i](3)) / 2;
                for (size_t j = 0; j < linshi_xy.cols(); j++)
                {
                    distance = std::sqrt(std::pow(center_x-linshi_xy(0,j),2)+std::pow(center_y-linshi_xy(1,j),2));
                    if (distance<d_min)
                    {
                        d_min=distance;
                    }
                }
                all_distance += 1/d_min;

            }
            double end_distance;
            for (size_t j = 0; j < localTraj.cols(); j+=2)
            {
                end_distance = std::sqrt(pow((localTraj(1,j)-(-1.5)), 2));
            }

            double cost_coeff_jerk = 0.2;
            return all_distance*cost_coeff_distance+ cost_coeff_righted * end_distance;  //距离障碍物的距离和靠右行驶作为损失函数 
        }
        //????
        // double distance;// 一条轨迹到其中一个障碍物的距离
        // cost_one_Traj.resize(obstructs.cols());
        // double cost_coeff_jerk = 0.8, cost_coeff_distance = 1.0, cost_coeff_righted = 1.5;
        // // std::cout << "obstructs.cols():" << obstructs.cols() << std::endl;  
        // // std::cout << "obstructs:" << obstructs << std::endl;        
        // for (int j = 0; j < obstructs.cols(); j++) {
        //     distance = sqrt(pow((A * obstructs(0, j) + B * obstructs(1, j) + C), 2) / (A * A + B * B));
        //     // distance = sqrt(pow((A * obstructs(1, j) + B * obstructs(0, j) + C), 2) / (A * A + B * B));
        //     // std::cout<<"obstructs(1, j): " << obstructs(1, j)<<" " <<obstructs(0, j)<<std::endl;
        //     // std::cout<<"distance: "<<distance<<std::endl;
        //     if (distance < obstructs(4, j)){
        //         cost_one_Traj(j) = 99999;
        //     } 
        //     else{
        //         cost_one_Traj(j) =  1 / distance;
        //     }
        //}
        // std::cout << "cost_one_Traj.maxCoeff():" << cost_one_Traj.maxCoeff() <<std::endl;
        // std::cout << "cost_coeff_jerk * jerkmaxY:" << cost_coeff_jerk * jerkmaxY << " cost_coeff_righted * pow((xyStates.yT - (-1)), 2): "<<cost_coeff_righted * pow((xyStates.yT - (-1)), 2)<<std::endl;
        // return cost_coeff_distance*cost_one_Traj.maxCoeff() + cost_coeff_jerk * jerkmaxY + cost_coeff_righted * pow((xyStates.yT - (-1)), 2);
        // return cost_one_Traj.maxCoeff() + cost_coeff_jerk * jerkmaxY + cost_coeff_righted * pow((xyStates.yT - (-1)), 2);
        // return cost_one_Traj.maxCoeff();
    }
    void findClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex){
        int end = globalPath.cols();
        double distance, d_min = 999999;
        for (int i = 0; i < end; ++i) {
            distance = std::pow(globalPath(0, i) - realPosition(0),2) + std::pow(globalPath(1, i) - realPosition(1),2);
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;
            }
        }
    }

    void carfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex){
        static int startIndex = 0;
        static int end = globalPath.cols();
        double distance, d_min = 999999;
        int i = startIndex;
        for (; i < end; i++) {
            distance = std::fabs(globalPath(0, i) - realPosition(0)) + std::fabs(globalPath(1, i) - realPosition(1));
            // std::cout << "distance: "<<distance<<std::endl;
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;
                startIndex = i;
            }
        }
        startIndex = startIndex - 10 < 0 ? 0 : startIndex;
        end = startIndex + 40;//30
        end = end - globalPath.cols() < 0 ? end : globalPath.cols();
    }

    void obsfindClosestPoint(double x, double y, Eigen::MatrixXd globalPath, int& minIndex, const int& carIndex){
        int startIndex = (carIndex - 10)>0 ? (carIndex - 10):0;
        int endIndex = (carIndex + 50)>globalPath.cols()? globalPath.cols(): (carIndex + 50);
        double distance, d_min = 999999;
        for (int i= startIndex; i < endIndex; i++)
        {
            distance = std::fabs(globalPath(0, i) - x) + std::fabs(globalPath(1, i) - y);
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;	
            }
        }
    }
    //车辆在sd坐标系下的位置   /***************************************************************/
    int cartesian2Frenet(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, frentPoint & carFrentPoint, int& minIndex,int &carIndex, const int flag){
        if (flag == 0) {
            carfindClosestPoint(realPosition, globalPath, minIndex);
            carIndex = minIndex;
            const double dx = realPosition(0) - globalPath(0, minIndex);
            const double dy = realPosition(1) - globalPath(1, minIndex);
            const double cos_theta_r = std::cos(globalPath(3, minIndex));
            const double sin_theta_r = std::sin(globalPath(3, minIndex));
            const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            carFrentPoint.d = std::copysign(cross_rd_nd, cross_rd_nd);
            carFrentPoint.s = globalPath(6, minIndex);
        }
        return 0;
    }
   
    void lidar_obs_Frenet(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, std::vector<frentPoint>& obsFrentPoint, 
                        int& minIndex,int &carIndex)
    {
        /*
            3-------2
            |       |
            4-------1
        */
        Eigen::MatrixXd obs;
        obs.resize(4,2);
        //添加障碍物的四个点 
        obs<<   realPosition(0),realPosition(1),
                realPosition(0),realPosition(3),
                realPosition(2),realPosition(3),
                realPosition(2),realPosition(1);
        for (size_t i = 0; i < obs.rows(); i++)
        {
            obsfindClosestPoint(obs.row(i)[0], obs.row(i)[1],globalPath, minIndex,carIndex);
            frentPoint obsfrent;
            const double dx = obs.row(i)[0] - globalPath(0, minIndex);
            const double dy = obs.row(i)[1] - globalPath(1, minIndex);
            const double cos_theta_r = std::cos(globalPath(3, minIndex));
            const double sin_theta_r = std::sin(globalPath(3, minIndex));
            const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            obsfrent.d = std::copysign(cross_rd_nd, cross_rd_nd);
            obsfrent.s = globalPath(6, minIndex);
            obsFrentPoint.push_back(obsfrent);
        }
    }

    int frenet2Cartesian(const Eigen::MatrixXd& globalPath, const double& s, const double& d, const int& minIndex, Eigen::Vector2d& cartesianXY){
        double cos_theta_r = cos(globalPath(3, minIndex));
        double sin_theta_r = sin(globalPath(3, minIndex));
        cartesianXY(0) = globalPath(0, minIndex) - sin_theta_r * d;
        cartesianXY(1) = globalPath(1, minIndex) + cos_theta_r * d;	
        return 0;
    }

    Eigen::Vector4d cubicCoffec(double tT, double pStart, double vStart, double pEnd, double vEnd){
        Eigen::Vector4d b, x;
        b << pStart, vStart, pEnd, vEnd;
        Eigen::Matrix4d T;
        T << 0, 0, 0, 1,
             0, 0, 1, 0,
             pow(tT, 3), pow(tT, 2), tT, 1,
             3 * pow(tT, 2), 2 * tT, 1, 0;
        x = T.householderQr().solve(b);
        return x;
    }

    int referencePath(const Eigen::MatrixXd& xy, const double v, Eigen::MatrixXd& path) {
        path.resize(5, xy.cols());
        double dx, dy, ddx, ddy;
        path.setZero();
        path.row(0) = xy.row(0);//道路x
        path.row(1) = xy.row(1);//道路y
        for (int i = 0; i < path.cols (); i++) {
            path(2, i) = v; //v
            if (i == 0) {
                dx = path(0, i + 1) - path(0, i);
                dy = path(1, i + 1) - path(1, i);
                ddx = path(0, 2) + path(0, 0) - 2 * path(0, 1);
                ddy = path(1, 2) + path(1, 0) - 2 * path(1, 1);
            }
            else if (i == path.cols() - 1) {
                dx = path(0, i) - path(0, i - 1);
                dy = path(1, i) - path(1, i - 1);
                ddx = path(0, i) + path(0, i - 2) - 2 * path(0, i - 1);
                ddy = path(1, i) + path(1, i - 2) - 2 * path(1, i - 1);
            }
            else {
                dx = path(0, i + 1) - path(0, i);
                dy = path(1, i + 1) - path(1, i);
                ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
                ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
            }
            // std::cout << "i: " << i << " dx: " << dx << " dy: " << dy << " ddx: " << ddx << " ddy: " << ddy << std::endl; 
            path(3, i) = atan2(dy, dx); // //道路yaw
            double kappa = (ddy * dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), (1.5));
            path(4, i) = kappa;// 道路y曲率k计算
        }
        return 0;
    }

    void calthea(const Eigen::MatrixXd& xy,Eigen::MatrixXd& path)
    {
        double dx, dy;
        path.row(0) = xy.row(0);//道路x
        path.row(1) = xy.row(1);//道路y
        for (int i = 0; i < path.cols (); i++) 
        {
            if (i == 0) {
                dx = path(0, i + 1) - path(0, i);
                dy = path(1, i + 1) - path(1, i);
               
            }
            else if (i == path.cols() - 1) {
                dx = path(0, i) - path(0, i - 1);
                dy = path(1, i) - path(1, i - 1);
               
            }
            else {
                dx = path(0, i + 1) - path(0, i);
                dy = path(1, i + 1) - path(1, i);
    
            }
            path(2, i) = atan2(dy, dx); // //道路yaw

        }
    }
}

