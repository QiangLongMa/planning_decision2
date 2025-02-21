	#include "map.h"
	
	bool down;
	int que[20]; // 存储最优路径节点标号
	int tot = 0; // que[]的容量
	/*初始道路关键点共18个点*/
	// Eigen::Matrix<double, 18, 2> initKeyPoint;  //初始关键点实际坐标（X,Y）, 单位：m
	int keyPointInit(Eigen::Matrix<double, 18, 2> &initKeyPoint){
		// initKeyPoint << -14.7704, 247.134, 170.692, 247.134, 463.371, 251.573, -14.7704, 116.943, 173.181, 117.937,
		// 								244.84, 118.044, 359.294, 119.82, 465.792, 121.228, -14.7704, 2.0351, 174.38, 4.2551, 247.193, 5.4791,
		// 								310.452, 6.3547, 360.907, 7.0226, 467.595, 8.8481, 187.435, -86.5555, 311.545, -84.182, 188.042, -116.024, 312.021, -130.645;
		
		initKeyPoint <<  -14.7704,247.134,  170.9204,247.4605,  463.5712,251.0059,    
		-14.7704,116.943,  172.7404,117.0002,  245.2492,118.1097,  358.9128,119.8489,  465.8112,121.4845,
        -14.7704,3.889,  174.3142,4.1968,  246.9904,5.20749,  310.996,6.7247,  359.4990,6.7721,  467.7586,8.27767, 
		187.435,-83.6042,  310.996,-84.182, 
        188.042,-116.024,  312.021,-130.645;
		
		std::cout << "Successfully set initial key points on the map!" << std::endl;
		return 0;
	}

	/*
	*初始道路连接关系,19*19 Matrix
	*包含投影点但投影点未加入全局连接
	*/
	// Eigen::MatrixXd initMap;  //初始连接地图
	int mapInit(Eigen::MatrixXd &initMap){
		Eigen::Matrix<double, 19, 19> mapTemp;
		mapTemp.setZero();
		for (int i = 0; i < mapTemp.rows(); i++)
			for (int j = 0; j < mapTemp.cols(); j++)
				if (i < j)
					mapTemp(i, j) = -1;
		mapTemp(0, 1) = 1;
		mapTemp(3, 8) = 1;
		mapTemp(6, 12) = 1;
		mapTemp(11, 15) = 1;
		mapTemp(4, 5) = 1;
		mapTemp(7, 13) = 1;
		mapTemp(12, 13) = 1;
		mapTemp(1, 2) = 1;
		mapTemp(4, 9) = 1;
		mapTemp(8, 9) = 1;
		mapTemp(14, 15) = 1;
		mapTemp(1, 4) = 1;
		mapTemp(5, 6) = 1;
		mapTemp(9, 10) = 1;
		mapTemp(14, 16) = 1;
		mapTemp(2, 7) = 1;
		mapTemp(5, 10) = 1;
		mapTemp(10, 11) = 1;
		mapTemp(15, 17) = 1;
		mapTemp(6, 7) = 1;
		mapTemp(11, 12) = 1;
		initMap = mapTemp + mapTemp.transpose();
		std::cout << "Successfully set the initial connection relationship of the map!" << std::endl;
		return 0;
	}

	/*
	* 找与目标点最近的关键点
	* point: 目标点
	* index: 与目标点最近的关键点的序号
	*/
	int findClosestPoint(const Eigen::MatrixXd &pointSeries,
											const Eigen::Matrix<double, 1, 2> &currentPoint, int &minIndex){
		double d_min = 999999;
		double distance;
		for (int i = 0; i < pointSeries.rows(); i++){
			distance = abs(currentPoint(0, 0) - pointSeries(i, 0)) + abs(currentPoint(0, 1) - pointSeries(i, 1));
			if (distance < d_min){
				minIndex = i;
				d_min = distance;
			}
		}
		return 0;
	}

	int findClosestPointArray(const Eigen::MatrixXd &pointSeries,
											const Eigen::Matrix<double, 1, 2> &currentPoint, int &minIndex){
		Eigen::MatrixXd distance;
		Eigen::MatrixXd::Index minRowIndex, minColIndex;
		distance.resize(1, pointSeries.cols());
		distance = abs(pointSeries.row(0).array() -currentPoint(0, 0)) +  abs(pointSeries.row(1).array() - currentPoint(0, 1)); 
		distance.minCoeff(&minRowIndex, &minColIndex);
		minIndex = minColIndex;
		return 0;
	}

	int findProjPoint(const Eigen::Matrix<double, 1, 2> &currentPoint, const Eigen::MatrixXd &pointSeries,
										const Eigen::MatrixXd &map, const int &minIndex, Eigen::Matrix<double, 1, 2> &projPoint){
		Eigen::Vector2d current_min, connect_min, target_connect_min;
		double projScalar = 0.0;
		double theta = 0.0;
		double thetaTh = 999;
		int connect_min_index;
		// std::cout << "pointSeries: " << pointSeries << std::endl;
		current_min << currentPoint(0)- pointSeries(minIndex,0),currentPoint(1)- pointSeries(minIndex,1);
		// std::cout << "minIndex: " << minIndex << std::endl;
		// std::cout << "ap: " << current_min << std::endl;
		// std::cout << "pointSeries_minIndex: " << pointSeries(minIndex,0) << ", " << pointSeries(minIndex,1) << std::endl;
		for (int i = 0; i < map.rows(); i++){
			if (map(i, minIndex) > 0){
				connect_min << pointSeries(i,0) - pointSeries(minIndex,0), pointSeries(i,1) - pointSeries(minIndex,1);
				// std::cout << "connect_min: " << connect_min << std::endl;
				projScalar = current_min.dot(connect_min);
				// std::cout << "all_projScalar: " << projScalar << std::endl;
				theta = std::acos(projScalar/(connect_min.dot(connect_min)));
				// std::cout << "all_theta: " << theta << std::endl;
				if (theta < thetaTh){
					connect_min_index = i;
					thetaTh = theta;
					// minConnectProjScalar = projScalar;
				}
			}
		}

		// std::cout << "connect_min_index_theta: " << thetaTh << std::endl;
		// std::cout << "connect_min_index: " << connect_min_index << std::endl;

		target_connect_min << pointSeries(minIndex,0)-pointSeries(connect_min_index,0), pointSeries(minIndex,1)-pointSeries(connect_min_index,1);
		const double targetProjDis = current_min.dot(target_connect_min);
		const double normDis = target_connect_min.dot(target_connect_min);
		const double minConnectProjScalar = targetProjDis/normDis;
		projPoint(0,0) = pointSeries(minIndex,0)+ minConnectProjScalar*target_connect_min(0);
		projPoint(0,1) = pointSeries(minIndex,1)+ minConnectProjScalar*target_connect_min(1);

		// std::cout << "ab: " << target_connect_min << std::endl;
		// std::cout << "minConnectProjScalar: " << minConnectProjScalar << std::endl;
		// std::cout << "projPoint: " << projPoint << std::endl;
		return 0;
	}

	/*
	*第一次修改全局连接关系，加入投影点
	*判断投影点的x坐标是否在最近点x_min的左右范围内，
	*y坐标是否在最近点y_min的上下范围内
	*将地图重置为20*20大小准备加入车辆
	*与现有点的连接关系
	* projPoint: 投影点
	* min: 最近点序号
	* finalMap: 最终地图
	*/
	int keyPointSet(const Eigen::MatrixXd &pointSeries, const Eigen::Matrix<double, 1, 2> &currentPoint,
									std::vector<cv::Point2d> &realPoint){
		for (int i = 0; i < pointSeries.rows(); i++)
			realPoint.push_back(cv::Point2d(pointSeries(i, 0), pointSeries(i, 1)));
		realPoint.push_back(cv::Point2d(currentPoint(0), currentPoint(1)));
		return 0;
	}

	int modificMap(const Eigen::Matrix<double, 1, 2> &projPoint, const Eigen::MatrixXd &pointSeries,
				const int &minIndex, Eigen::MatrixXd &map){
		for (int i = 0; i < map.rows() - 1; i++){
			if (map(i, minIndex) > 0){
				bool condx1 = (pointSeries(i, 0) <= projPoint(0, 0) && projPoint(0, 0) <= pointSeries(minIndex, 0));
				bool condx2 = (pointSeries(i, 0) >= projPoint(0, 0) && projPoint(0, 0) >= pointSeries(minIndex, 0));
				bool condy1 = (pointSeries(i, 1) <= projPoint(0, 1) && projPoint(0, 1) <= pointSeries(minIndex, 1));
				bool condy2 = (pointSeries(i, 1) >= projPoint(0, 1) && projPoint(0, 1) >= pointSeries(minIndex, 1));
				if ((condx1 || condx2) && (condy1 || condy2)){
					map(i, minIndex) = -1;
					map(minIndex, i) = -1;
					map(i, map.cols() - 1) = 1;
					map(minIndex, map.cols() - 1) = 1;
					map(map.cols() - 1, i) = 1;
					map(map.cols() - 1, minIndex) = 1;
				}
			}
		}
		return 0;
	}

	/*
	*计算代价地图
	*最终连接关系地图，全部关键点
	*return: 最终代价地图
	*/
	int costMapSet(const Eigen::Matrix<double, 20, 20> &map,
				const Eigen::Matrix<double, 20, 2> &keyPoint,Eigen::Matrix<double, 20, 20> &CostMap){
		CostMap << map;
		for (int i = 0; i < map.rows(); i++)
			for (int j = 0; j < map.cols(); j++){
				bool cond = (i == 4 || i == 5 || i == 6 || i == 7 || i == 10 || i == 11) && (j == 4 || j == 5 || j == 6 || j == 7 || j == 10 || j == 11);
				if (cond){
					if (map(i, j) >= 0)
						CostMap(i, j) = tool::distance(keyPoint, i, j) * 1.5;
					else
						CostMap(i, j) = 9999999;
				}
				else{
					if (map(i, j) >= 0)
						CostMap(i, j) = tool::distance(keyPoint, i, j);
					else
						CostMap(i, j) = 9999999;
				}
			}
		return 0;
	}

	int InitGlobalMap(const double gpsX, const double gpsY, const double heading, const double speed, const Eigen::Matrix<double, 1, 2> endPoint, Eigen::Matrix<double, 7, Eigen::Dynamic> &globalPath, Eigen::VectorXd& car){
		// cv::Mat image;								 // 地图背景
		Eigen::Matrix<double, 18, 2> initKeyPoint;	 // 初始关键点实际坐标, 单位：m
		Eigen::MatrixXd initMap;					 // 初始连接地图
		Eigen::Matrix<double, 1, 2> endPointProj;	 // 实际终点投影点
		Eigen::Matrix<double, 1, 2> carProj;		 // 车投影点
		Eigen::Matrix<double, 1, 2> carRealPosition; // 车辆实际位置
		Eigen::Matrix<double, 1, 3> carRealPosition_theta; // 车辆实际位置与车头朝向（与x轴夹角）
		Eigen::MatrixXd finalMap;					 // 最终地图
		Eigen::Matrix<double, 20, 20> finalCostMap;
		Eigen::Matrix<double, 19, 2> end_point_key_point; // 第一次调用加入目标点投影点的关键点实际坐标, 单位：m
		Eigen::Matrix<double, 20, 2> finalKeyPoint;		  // 第二次调用加入车辆投影点的关键点实际坐标, 单位：m
		std::vector<cv::Point2d> realPoint; // 第一次调用加入目标点的投影点，第二次调用加入车辆投影点,to全局关键点

		// 获取车辆位置(x, y), 转换为以西门为原点的(x, y)单位:m
		tool::getCarPosition(gpsX, gpsY, heading, speed, car);	
		carRealPosition << car(0), car(1);
		// 原始关键点初始化
		keyPointInit(initKeyPoint);
		// 原始地图初始化
		// int res = mapInit(initMap);
		mapInit(initMap);
		// 距离终点最近的关键点下标
		int endPoint_InitKeyPoint_minIndex; // 终点距离初始关键点最近的点的下标			
		findClosestPoint(initKeyPoint, endPoint, endPoint_InitKeyPoint_minIndex);	
		// 求终点的投影点
		findProjPoint(endPoint, initKeyPoint, initMap, endPoint_InitKeyPoint_minIndex, endPointProj);
		// 第一次修改地图
		modificMap(endPointProj, initKeyPoint, endPoint_InitKeyPoint_minIndex, initMap);
		// 关键点重置,加入目标点的投影点到全局关键点
		end_point_key_point << initKeyPoint, endPointProj;
		// 距离车最近的关键点下标
		int carRealPosition_InitKeyPoint_minIndex;
		findClosestPoint(end_point_key_point, carRealPosition, carRealPosition_InitKeyPoint_minIndex);
		// 求车的投影点
		findProjPoint(carRealPosition, end_point_key_point, initMap, carRealPosition_InitKeyPoint_minIndex, carProj);
		finalMap.resize(20, 20);
		finalMap.setOnes() *= (-1);
		for (int i = 0; i < initMap.rows(); i++)
			for (int j = 0; j < initMap.cols(); j++)
				finalMap(i, j) = initMap(i, j);
		finalMap(finalMap.rows() - 1, finalMap.cols() - 1) = 0;
		// 第二次修改地图
		modificMap(carProj, end_point_key_point, carRealPosition_InitKeyPoint_minIndex, finalMap);
		// 关键点重置,加入车的投影点到全局关键点
		finalKeyPoint << end_point_key_point, carProj;
		keyPointSet(end_point_key_point, carProj, realPoint);
		costMapSet(finalMap, finalKeyPoint, finalCostMap);
		int headIndex;
		carRealPosition_theta <<  car(0), car(1), car(3);
		getHeadingPoint(carRealPosition_theta, finalMap, carProj, finalKeyPoint,carRealPosition_InitKeyPoint_minIndex, headIndex);
		Dijkstra(headIndex, 19, finalCostMap);
		std::cout << "Turn condition:" << std::endl;
		genGlobalPath(que, tot, realPoint, globalPath);
		que[20]={0};
		return 0;
	}

	// Dijkstra算法
	void Dijkstra(int begin, int head, const Eigen::Matrix<double, 20, 20> &map){
		int num = map.cols();
		const int MAX_NUM = 20;
		int path[MAX_NUM];
		int dis[MAX_NUM]; // 保存到其他点的距离
		bool visit[MAX_NUM];
		for (int i = 0; i < num; i++){
			if (map(begin, i) >= 0 && map(begin, i) < 9999999){ // 设置为原本的距离
				dis[i] = map(begin, i);
				path[i] = begin;
			}
			else{ // 不存在就设置距离为无穷大
				dis[i] = 9999999;
				path[i] = -1;
			}
		}
		// 初始化访问数组全部为未访问
		for (int i = 0; i < num; i++)
			visit[i] = false;
		visit[begin] = true; // 起始结点已被访问
		visit[head] = true; // 车头朝向结点已被访问
		// 已经添加了一个begin，还需要将剩下num-1个点加入
		for (int i = 0; i < num - 1; i++){
			int min_index = -1;
			int min_dis = 9999999;
			// 找到当前最小的一条边
			for (int j = 0; j < num; j++){
				if (min_dis > dis[j] && visit[j] == false){
					min_dis = dis[j];
					min_index = j;
				}
			}
			if (min_dis == 9999999) // 存在最短距离
				continue;
			visit[min_index] = true; // 该点加入到目标点集中
			// 更新距离
			for (int j = 0; j < num; j++){
				if (map(min_index, j) != 0 && dis[j] > dis[min_index] + map(min_index, j)){
					dis[j] = dis[min_index] + map(min_index, j);
					path[j] = min_index;
				}
			}
		}
		searchPath(path, begin, 18);
	}

	int searchPath(int *pre, int v, int u){
		que[tot] = u;
		tot++;
		int tmp = pre[u];
		while (tmp != v){
			que[tot] = tmp;
			tot++;
			tmp = pre[tmp];
		}
		que[tot] = v;
		que[++tot] = 19;
		std::cout << "Optimal key point sequence:"<< std::endl;
		for (int i = tot; i >= 0; i--){
			if (i != 0)
				std::cout << que[i] << "->";
			else
				std::cout << que[i] << std::endl;
		}
		// que[20]={0};
		return 0;
	}

	void getHeadingPoint(const Eigen::MatrixXd& carPosition, const Eigen::Matrix<double, 20, 20>& finalMap, const Eigen::Matrix<double, 1, 2> carProj, const Eigen::Matrix<double, 20, 2> finalKeyPoint, const int& minIndex, int& head){
		Eigen::Matrix<double, 1, 2> V_car;
		int pointIndex;
		// std:: cout << "carPosition(2)" << carPosition(2) << std::endl;
		V_car << cos(tool::d2r(carPosition(2))), sin(tool::d2r(carPosition(2)));
		double flag = (carProj(0) - finalKeyPoint.row(minIndex)(0)) * V_car(0) + (carProj(1) - finalKeyPoint.row(minIndex)(1)) * V_car(1);
		for (int j = 0; j < finalMap.rows() - 1; j++) {
			if (finalMap(j, 19) > 0 && j != minIndex ) {
				pointIndex = j;
			}
		}
		if (flag <= 0)
			head = minIndex;
		else
			head = pointIndex;

	}


	int genGlobalPath(int *que, int tot, const std::vector<cv::Point2d> realPoint, Eigen::Matrix<double, 7, Eigen::Dynamic> &globalPath){
		// 全局路径规划预处理
		std::vector<Eigen::Matrix<double, 5, Eigen::Dynamic>> trajt; // 全部路径
		Eigen::Matrix<double, 5, Eigen::Dynamic> path;				 // 段路径信息缓存
		double R, speed = 5, deltaT = 0.05;						 // 半径，速度，控制指令周期
		double cirstartx, cirstarty, cirendx, cirendy;
		Eigen::Vector4d cirStart;
		Eigen::Vector3d cirEnd, oldCirEnd;
		oldCirEnd << realPoint[que[tot]].x, realPoint[que[tot]].y, speed;
		if (tot < 2){ // 路径关键点只有2个，生成直线
			R = 0.0;
			cirstartx = realPoint[que[tot - 1]].x - R;
			cirstarty = realPoint[que[tot - 1]].y;
			cirendx = realPoint[que[tot - 1]].x;
			cirendy = realPoint[que[tot - 1]].y - R;
			if (cirstartx == cirendx && cirstarty == cirendy){
				cirEnd << cirstartx, cirstarty, speed;
				// 生成直线
			}
			tool::straight(oldCirEnd, cirEnd, deltaT, path);
			trajt.push_back(path);
		}
		else{ // 路径关键点有3个或以上
			for (int i = tot; i >= 2; i--){
				double deltaX1 = realPoint[que[i - 1]].x - realPoint[que[i]].x;
				double deltaY1 = realPoint[que[i - 1]].y - realPoint[que[i]].y;
				double deltaX2 = realPoint[que[i - 2]].x - realPoint[que[i - 1]].x;
				double deltaY2 = realPoint[que[i - 2]].y - realPoint[que[i - 1]].y;
				double th = fabs(cos(atan2(deltaY1, deltaX1) - atan2(deltaY2, deltaX2)));
				// 判定两段是否可认为是直线
				if (th > 0.7){
					R = 0.0;
					cirstartx = realPoint[que[i - 1]].x - R;
					cirstarty = realPoint[que[i - 1]].y;
					cirendx = realPoint[que[i - 1]].x;
					cirendy = realPoint[que[i - 1]].y - R;
					cirEnd << cirstartx, cirstarty, speed;
					tool::straight(oldCirEnd, cirEnd, deltaT, path);
					trajt.push_back(path);
				}
				else{ // 转90度的8个条件
					R = 10.0;
					bool dircond1 = realPoint[que[i]].x < realPoint[que[i - 1]].x && realPoint[que[i - 1]].y > realPoint[que[i - 2]].y && realPoint[que[i]].x < realPoint[que[i - 2]].x && realPoint[que[i]].y > realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) < 0);
					bool dircond2 = realPoint[que[i]].x > realPoint[que[i - 1]].x && realPoint[que[i - 1]].y > realPoint[que[i - 2]].y && realPoint[que[i]].x > realPoint[que[i - 2]].x && realPoint[que[i]].y > realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) > 0);
					bool dircond3 = realPoint[que[i]].x < realPoint[que[i - 1]].x && realPoint[que[i - 1]].y < realPoint[que[i - 2]].y && realPoint[que[i]].x < realPoint[que[i - 2]].x && realPoint[que[i]].y < realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) > 0);
					bool dircond4 = realPoint[que[i]].x > realPoint[que[i - 1]].x && realPoint[que[i - 1]].y < realPoint[que[i - 2]].y && realPoint[que[i]].x > realPoint[que[i - 2]].x && realPoint[que[i]].y < realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) < 0);
					bool dircond5 = realPoint[que[i]].y < realPoint[que[i - 1]].y && realPoint[que[i - 1]].x > realPoint[que[i - 2]].x && realPoint[que[i]].x > realPoint[que[i - 2]].x && realPoint[que[i]].y < realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) > 0);
					bool dircond6 = realPoint[que[i]].y < realPoint[que[i - 1]].y && realPoint[que[i - 1]].x < realPoint[que[i - 2]].x && realPoint[que[i]].x < realPoint[que[i - 2]].x && realPoint[que[i]].y < realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) < 0);
					bool dircond7 = realPoint[que[i]].y > realPoint[que[i - 1]].y && realPoint[que[i - 1]].x > realPoint[que[i - 2]].x && realPoint[que[i]].x > realPoint[que[i - 2]].x && realPoint[que[i]].y > realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) < 0);
					bool dircond8 = realPoint[que[i]].y > realPoint[que[i - 1]].y && realPoint[que[i - 1]].x < realPoint[que[i - 2]].x && realPoint[que[i]].x < realPoint[que[i - 2]].x && realPoint[que[i]].y > realPoint[que[i - 2]].y && (tool::lineEq(realPoint[que[i]], realPoint[que[i - 2]], realPoint[que[i - 1]]) > 0);
					if (dircond1){
						cirstartx = realPoint[que[i - 1]].x - R;
						cirstarty = realPoint[que[i - 1]].y;
						cirendx = realPoint[que[i - 1]].x;
						cirendy = realPoint[que[i - 1]].y - R;
						std ::cout << "dir1" << std::endl;
					}
					else if (dircond2){
						cirstartx = realPoint[que[i - 1]].x + R;
						cirstarty = realPoint[que[i - 1]].y;
						cirendx = realPoint[que[i - 1]].x;
						cirendy = realPoint[que[i - 1]].y - R;
						std ::cout << "dir2" << std::endl;
					}
					else if (dircond3){
						cirstartx = realPoint[que[i - 1]].x - R;
						cirstarty = realPoint[que[i - 1]].y;
						cirendx = realPoint[que[i - 1]].x;
						cirendy = realPoint[que[i - 1]].y + R;
						std ::cout << "dir3" << std::endl;
					}
					else if (dircond4){
						cirstartx = realPoint[que[i - 1]].x + R;
						cirstarty = realPoint[que[i - 1]].y;
						cirendx = realPoint[que[i - 1]].x;
						cirendy = realPoint[que[i - 1]].y + R;
						std ::cout << "dir4" << std::endl;
					}
					else if (dircond5){
						cirstartx = realPoint[que[i - 1]].x;
						cirstarty = realPoint[que[i - 1]].y - R;
						cirendx = realPoint[que[i - 1]].x - R;
						cirendy = realPoint[que[i - 1]].y;
						std ::cout << "dir5" << std::endl;
					}
					else if (dircond6){
						cirstartx = realPoint[que[i - 1]].x;
						cirstarty = realPoint[que[i - 1]].y - R;
						cirendx = realPoint[que[i - 1]].x + R;
						cirendy = realPoint[que[i - 1]].y;
						std ::cout << "dir6" << std::endl;
					}
					else if (dircond7){
						cirstartx = realPoint[que[i - 1]].x;
						cirstarty = realPoint[que[i - 1]].y + R;
						cirendx = realPoint[que[i - 1]].x - R;
						cirendy = realPoint[que[i - 1]].y;
						std ::cout << "dir7" << std::endl;
					}
					else if (dircond8){
						cirstartx = realPoint[que[i - 1]].x;
						cirstarty = realPoint[que[i - 1]].y + R;
						cirendx = realPoint[que[i - 1]].x + R;
						cirendy = realPoint[que[i - 1]].y;
						std ::cout << "dir8" << std::endl;
					}
					cirStart << cirstartx, cirstarty, speed, atan2(cirstartx - oldCirEnd(0), cirstarty - oldCirEnd(1));
					cirEnd << cirendx, cirendy, speed;
					tool::straight(oldCirEnd, cirStart.head(3), deltaT, path);
					trajt.push_back(path);
					bool cond1 = cirStart(0) < cirEnd(0) && cirStart(1) > cirEnd(1) && dircond1;
					bool cond2 = cirStart(0) > cirEnd(0) && cirStart(1) > cirEnd(1) && dircond2;
					bool cond3 = cirStart(0) < cirEnd(0) && cirStart(1) < cirEnd(1) && dircond3;
					bool cond4 = cirStart(0) > cirEnd(0) && cirStart(1) < cirEnd(1) && dircond4;
					bool cond5 = cirStart(0) > cirEnd(0) && cirStart(1) < cirEnd(1) && dircond5;
					bool cond6 = cirStart(0) < cirEnd(0) && cirStart(1) < cirEnd(1) && dircond6;
					bool cond7 = cirStart(0) > cirEnd(0) && cirStart(1) > cirEnd(1) && dircond7;
					bool cond8 = cirStart(0) < cirEnd(0) && cirStart(1) > cirEnd(1) && dircond8;
					if (cond1){
						tool::arc(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond1" << std::endl;
					}
					else if (cond2){
						tool::arc2(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond2" << std::endl;
					}
					else if (cond3){
						tool::arc3(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond3" << std::endl;
					}
					else if (cond4){
						tool::arc4(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond4" << std::endl;
					}
					else if (cond5){
						tool::arc5(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond5" << std::endl;
					}
					else if (cond6){
						tool::arc6(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond6" << std::endl;
					}
					else if (cond7){
						tool::arc7(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond7" << std::endl;
					}
					else if (cond8){
						tool::arc8(cirStart, R, 90.0, deltaT, path);
						trajt.push_back(path);
						std ::cout << "cond8" << std::endl;
					}
				}
				oldCirEnd << cirEnd;
			}
		}
		// 最后一个点加入并生成轨迹压入vector
		Eigen::Vector3d allEnd;
		allEnd << realPoint[que[0]].x, realPoint[que[0]].y, speed;
		tool::straight(cirEnd, allEnd, deltaT, path);
		trajt.push_back(path);
		double s = 0;
		std::vector<Eigen::Matrix<double, 5, Eigen::Dynamic>>::iterator it_trajt = trajt.begin();
		Eigen::Matrix<double, 3, Eigen::Dynamic> xyv;
		int reTrajtCols = 0;
		for (; it_trajt < trajt.end(); ++it_trajt)
			reTrajtCols += (*it_trajt).cols();
		xyv.resize(3, reTrajtCols);
		it_trajt = trajt.begin();
		int sumcols = 0;
		for (unsigned int i = 0; i < trajt.size(); ++i){
			sumcols += (i == 0 ? 0 : trajt[i - 1].cols());
			for (int j = 0; j < trajt[i].cols(); ++j){
				xyv(0, sumcols + j) = trajt[i](0, j);
				xyv(1, sumcols + j) = trajt[i](1, j);
				xyv(2, sumcols + j) = trajt[i](2, j);
			}
		}
		std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> reTrajt; // 全部路径
		tool::referencePath(xyv, reTrajt);
		int allTrajtCols = 0;
		std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>>::iterator it_reTrajt = reTrajt.begin();
		for (; it_reTrajt < reTrajt.end(); ++it_reTrajt)
			allTrajtCols += (*it_reTrajt).cols();
		globalPath.resize(7,allTrajtCols);
		int subcols = 0;
		for(unsigned int i = 0; i < reTrajt.size(); ++i){
			subcols += (i == 0 ? 0 : reTrajt[i - 1].cols());
			for(unsigned int j = 0; j < reTrajt[i].cols(); ++j){
				globalPath(0,subcols+j) = reTrajt[i](0,j);  //x
				globalPath(1,subcols+j) = reTrajt[i](1,j);  //y
				globalPath(2,subcols+j) = reTrajt[i](2,j);  //v
				globalPath(3,subcols+j) = reTrajt[i](3,j);  //theta
				globalPath(4,subcols+j) = reTrajt[i](4,j);  //kappa
				globalPath(5,subcols+j) = reTrajt[i](5,j);  //dkappa
				if (j < 1){
					globalPath(6,subcols+j) = s;
				}else{
					s += sqrt(pow(reTrajt[i](0,j) - reTrajt[i](0,j-1), 2) + pow(reTrajt[i](1,j) - reTrajt[i](1,j-1), 2));
					globalPath(6,subcols+j) = s;
				}

			}
		}
		return 0;
	}
