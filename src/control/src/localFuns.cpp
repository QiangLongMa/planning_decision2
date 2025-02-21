#include "localFuns.h"

namespace local{
    int getOptimalLocalTraj(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, Eigen::MatrixXd& obs1, tool::controlParams& cps, Eigen::MatrixXd& cartesianTrajt, Eigen::MatrixXd& frenetTraj, int &minIndex, bool fre) {
        int optimal;//最优局部路径下标
        // int minIndex;//车辆与全局路径最近点的下标
        //controlParams cps; //控制参数定义
        std::vector<Eigen::MatrixXd> localTrajs;//存储最优局部路径
        // std::cout << "sssssssssssssss" << std::endl;
        optimal = genFrenetLocalTrajs(carRealPosition, globalPath, obs1, cps, localTrajs, minIndex);
        // std::cout << "eeeeeeeeeeeeeeee" << std::endl;
        // optimal = -1;
        if (optimal < 0) {
                std::cout << "optimal<0!!" << std::endl;
                return 1;
        }
        // std::cout << "车辆与全局路径的最近点下标:" << std::endl;
        // std::cout << minIndex << std::endl;
        std::cout << "optimal index: " << std::endl;
        std::cout << optimal << std::endl;
        // std::cout << "localTrajs:" << std::endl;
        // for (int i = 0; i < localTrajs.size(); i++){
        //     std::cout << "localTrajs" << i << ":"<<std::endl;
        //     std::cout << localTrajs[i].transpose() << std::endl; 
        // }
        // std::cout << "localTrajs[optimal]:" << std::endl;
        // std::cout << localTrajs[optimal].transpose() << std::endl;    
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
        // std::cout << "optimal cartesianXY: " << std::endl;
        // std::cout << cartesianXY << std::endl;
        referencePath(cartesianXY, cps.vEnd, cartesianTrajt);//由xy计算参考路径参数cartesian坐标系下
        // std::cout << "cartesianTrajtLon: " << std::endl;
        // std::cout << cartesianTrajt.cols() << std::endl;

        //std::vector<Eigen::MatrixXd> allCartesianXY;
        //showCartesianLocalTrajs(localTrajs, globalPath, optimal, minIndex, cps, allCartesianXY);
        //std::cout << "cartesian所有局部路径信息显示(localPath1(x,y)): " << std::endl;
        //for (int i = 0; i < cps.n; i++)
        //	std::cout << allCartesianXY[i].transpose() << std::endl;
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
                    distance = abs(globalPath(6, j) - frenetLocalTrajs[k](0, i));
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
        // std::cout << "initMinIndex:" << initMinIndex << std::endl;
        // std::cout << "cps.lp:" << cps.lp << std::endl;
        // std::cout<<globalPath.cols()<<std::endl;
        // std::cout << "endIndex:" << endIndex << std::endl;
        int minIndex = -1;
        int trajLon=0;
        Eigen::MatrixXd TcartesianXY;
        TcartesianXY.resize(2, frenetLocalTrajs[optimal].cols());
        Eigen::Vector2d XY;
        // std::cout<< "------trajLon: " << std::endl;
        // std::cout << frenetLocalTrajs[optimal].transpose() << std::endl;
        for (int i = 0; i < frenetLocalTrajs[optimal].cols(); i++) {
            // std::cout<<globalPath(5,globalPath.cols()-1)<<" "<<frenetLocalTrajs[optimal](0, i)<<std::endl;
            if(globalPath(6,globalPath.cols()-1)>frenetLocalTrajs[optimal](0, i)){
                trajLon=i;
                // std::cout<< "------trajLon: " << trajLon << std::endl;
                // break;
            }
            // std::cout << "i: " << i << std::endl;
            double distance, d_min = 999999;
            for (int j = initMinIndex; j < endIndex; j++) {
                // std::cout<<globalPath(5, j)<<std::endl;
                // std::cout<<optimal<<"   "<<i<<std::endl;
                distance = abs(globalPath(6, j) - frenetLocalTrajs[optimal](0, i));
                // std::cout << "distance: " << distance << std::endl;
                if (distance < d_min) {
                    minIndex = j;
                    d_min = distance;
                }
            }
            // std::cout<< "最近点下标： " << minIndex << std::endl;
            frenet2Cartesian(globalPath, frenetLocalTrajs[optimal](0, i), frenetLocalTrajs[optimal](1, i), minIndex, XY);
            TcartesianXY.col(i) = XY;
        }
        // std::cout<< "trajLon: " << frenetLocalTrajs[optimal].cols()<<"  "<<trajLon << std::endl;
        cartesianXY=TcartesianXY.block(0,0,2,trajLon);
        // std::cout<<"frenetLocalTrajs.clear(); "<<frenetLocalTrajs.size()<<std::endl;
        // std::cout << "cartesianXY :" << cartesianXY << std::endl;
        return 0;
    }
    
    int genFrenetLocalTrajs(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, const Eigen::MatrixXd& obs, tool::controlParams& cps, 
        std::vector<Eigen::MatrixXd>& localTrajs, int& minIndex){
        tool::frentPoint carFrentPoint; //车辆Frenet坐标位置
        //int minIndex; //车辆与全局路径的最近点下标作为参数传递到函数外
        int carIndex;
        cartesian2Frenet(carRealPosition, globalPath, carFrentPoint, minIndex,carIndex, 0);//车辆坐标变换cartesian2frenet
        // std::cout<<"FrenetLocalTrajs carRealPosition:"<<std::endl;
        // std::cout<<carRealPosition<<std::endl;
        // std::cout<< "FrenetLocalTrajs carIndex:" <<std::endl;
        // std::cout<< carIndex <<std::endl;
        // std::cout<< "FrenetLocalTrajs carFrentPoint:" <<std::endl;
        // std::cout<< carFrentPoint.s << ", " << carFrentPoint.d <<std::endl;
        // std::cout<< "FrenetLocalTrajs cartesian obs:" <<std::endl;
        // std::cout<< obs <<std::endl;
        // cps.tT =(globalPath(5,globalPath.cols()-1)-globalPath(5,minIndex)<cps.vEnd*cps.tT)?(globalPath(5,globalPath.cols()-1)-globalPath(5,minIndex))/cps.vEnd:cps.tT; 
        Eigen::MatrixXd obses; //障碍物
        obses.resize(obs.rows(), obs.cols());
        tool::frentPoint obs1FrentPoint; 
        int obsIndex; //障碍物Frenet位置, 
        for (int i = 0; i < obs.cols(); i++) {
            cartesian2Frenet(obs.col(i), globalPath, obs1FrentPoint, obsIndex,carIndex, 1);
            //  cartesian2Frenet(carRealPosition, globalPath, obs1FrentPoint, obsIndex,carIndex, 1);
            obses(0, i) = obs1FrentPoint.s > 20 ? 0 : obs1FrentPoint.s;
            obses(1, i) = obs1FrentPoint.s > 20 ? 0 : obs1FrentPoint.d;
            obses(2, i) = obs(2);
            obses(3, i) = obs(3); 
            obses(4, i) = obs1FrentPoint.s > 20 ? 0 : obs(4);
        }
        // std::cout<<"FrenetLocalTrajs Frenet obses:"<<std::endl;
        // std::cout<<obses<<std::endl;
        
        // std::cout << "globalPath: " << std::endl;
        // std::cout << globalPath.transpose() << std::endl;

        Eigen::Vector4d startPoint; //车辆初始状态
        startPoint << carFrentPoint.s, carFrentPoint.d, cps.vEnd, tool::normalizeAngle(tool::d2r(carRealPosition(3)) - globalPath(3, minIndex));
        // std::cout << "startPoint :" << startPoint  << std::endl;

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
            // std::cout <<"startState:" << xyStates.x0 << "      " << xyStates.y0 << "       "<<  xyStates.vx0 <<"      "<<  xyStates.vy0 << std::endl;
            xyStates.xT = endPointSeries(i, 0);
            xyStates.yT = endPointSeries(i, 1);
            xyStates.vxT = endPointSeries(i, 2)*cos(endPointSeries(i, 3));
            xyStates.vyT = endPointSeries(i, 2)*sin(endPointSeries(i, 3));
            // std::cout <<"endState:" << xyStates.xT << "      " << xyStates.yT << "       "<<  xyStates.vxT <<"      "<<  xyStates.vyT << std::endl;
            double one_traj_max_cost = evalfun(cps, xyStates, obses, Ai, Bi, cost_one_Traj, localTraj);	         
            A.col(i) = Ai;
            B.col(i) = Bi;
            all_traj_max(i) = one_traj_max_cost;
            localTrajs.push_back(localTraj); //每条局部路径压入vector
        }
        std::cout<<"all_traj_max: "<<all_traj_max<<std::endl;
        Eigen::VectorXd::Index min_index_trajs;
        double min_val_trajs = all_traj_max.minCoeff(&min_index_trajs);
        if (min_val_trajs < 99999){
            optimal = min_index_trajs;
        }
        // std::cout << "Frenet路径:" << std::endl;
        // for (int i = 0; i < cps.n; i++) {
        //     std::cout << "Frenet路径: "  << i << std::endl;
        //     std::cout << localTrajs[i].transpose() << std::endl;
        // }     
        // std::cout << cartesianXY << std::endl;
        // std::cout<<"opt: "<< optimal <<std::endl;
        return optimal;
    }

    double evalfun(const tool::controlParams& cps, const tool::states& xyStates, const Eigen::MatrixXd& obstructs, Eigen::Vector4d& Ai, Eigen::Vector4d& Bi, Eigen::VectorXd& cost_one_Traj, Eigen::MatrixXd& localTraj){
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
        double distance;// 一条轨迹到其中一个障碍物的距离
        cost_one_Traj.resize(obstructs.cols());
        double cost_coeff_jerk = 1.0, cost_coeff_distance = 0.5, cost_coeff_righted = 0.5;
        std::cout << "obstructs.cols():" << obstructs.cols() << std::endl;        
        for (int j = 0; j < obstructs.cols(); j++) {
            distance = sqrt(pow((A * obstructs(0, j) + B * obstructs(1, j) + C), 2) / (A * A + B * B));
            if (distance < obstructs(4, j)){
                cost_one_Traj(j) = 99999;
            }
            else{
                cost_one_Traj(j) = cost_coeff_distance / distance;
            }
        }
        std::cout << "cost_one_Traj.maxCoeff():" << cost_one_Traj.maxCoeff() <<std::endl;
        std::cout << "cost_coeff_jerk * jerkmaxY:" << cost_coeff_jerk * jerkmaxY << " cost_coeff_righted * pow((xyStates.yT - (-1)), 2): "<<cost_coeff_righted * pow((xyStates.yT - (-1)), 2)<<std::endl;
        return cost_one_Traj.maxCoeff() + cost_coeff_jerk * jerkmaxY + cost_coeff_righted * pow((xyStates.yT - (-2)), 2);
    }

    void findClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex){
        int startIndex = 0;
        int end = globalPath.cols();
        double distance, d_min = 999999;
        int i = startIndex;
        for (; i < end; i++) {
            distance = std::pow(globalPath(0, i) - realPosition(0), 2) + std::pow(globalPath(1, i) - realPosition(1), 2);
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;
                startIndex = i;
            }
        }
    }

    void carfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex){
        static int startIndex = 0;
        static int end = globalPath.cols();
        double distance, d_min = 999999;
        int i = startIndex;
        for (; i < end; i++) {
            distance = std::pow(globalPath(0, i) - realPosition(0), 2) + std::pow(globalPath(1, i) - realPosition(1), 2);
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;
                startIndex = i;
            }
        }
        startIndex = startIndex - 10 < 0 ? 0 : startIndex;
        end = startIndex + 30;
        end = end - globalPath.cols() < 0 ? end : globalPath.cols();
    }

    void obsfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex, const int& carIndex){
        int startIndex = (carIndex - 10)>0 ? (carIndex - 10):0;
        int endIndex = (carIndex + 50)>globalPath.cols()? globalPath.cols(): (carIndex + 50);
        double distance, d_min = 999999;
        for (int i= startIndex; i < endIndex; i++) {
            distance = abs(globalPath(0, i) - realPosition(0)) + abs(globalPath(1, i) - realPosition(1));
            if (distance < d_min) {
                minIndex = i;
                d_min = distance;	
            }
        }
    }

    int cartesian2Frenet(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& minIndex,int &carIndex, const int flag){
        if (flag == 0) {
            carfindClosestPoint(realPosition, globalPath, minIndex);
            carIndex = minIndex;
        }
        else {
            obsfindClosestPoint(realPosition, globalPath, minIndex,carIndex);
        }
        const double dx = realPosition(0) - globalPath(0, minIndex);
        const double dy = realPosition(1) - globalPath(1, minIndex);
        const double cos_theta_r = std::cos(globalPath(3, minIndex));
        const double sin_theta_r = std::sin(globalPath(3, minIndex));
        const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
        carFrentPoint.d = std::copysign(cross_rd_nd, cross_rd_nd);
        carFrentPoint.s = globalPath(6, minIndex);
        return 0;
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
            path(3, i) = atan2(dy, dx); // //道路yaw
            double kappa = (ddy * dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), (1.5));
            path(4, i) = kappa;// 道路y曲率k计算
        }
        return 0;
    }

}

