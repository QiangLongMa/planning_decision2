#pragma once
#include"toolKits.h"
#include <vector>
#include <fstream>

/**获取车辆位置（gpsX, gpsY），转换为以西门为原点的（x, y）单位:m
*车辆gps信息: gpsX, gpsY
*以西门为原点的相对世界坐标x, y
*/
// int getCarPosition(const double gpsX, const double gpsY, const double heading, const double speed, Eigen::VectorXd& car);

/**读地图背景
*读取地图背景图片
*/
// int mapImageRead(cv::Mat& image);

/**原始关键点初始化
 * 园区路口相对西门口的世界坐标
*/
int keyPointInit(Eigen::Matrix<double, 18, 2>& initKeyPoint);

/**原始地图初始化
 * 未加入车辆位置和目标点位置
 * 的关键点连接关系
*/
int mapInit(Eigen::MatrixXd& initMap);
//找最近点

int findClosestPoint(const Eigen::MatrixXd& pointSeries, const Eigen::Matrix<double, 1, 2>& currentPoint, 
                                           int& minIndex);

int findClosestPointArray(const Eigen::MatrixXd& pointSeries, const Eigen::Matrix<double, 1, 2>& currentPoint, 
                                           int& minIndex);
//求投影点
int findProjPoint(const Eigen::Matrix<double, 1, 2>& currentPoint, const Eigen::MatrixXd& pointSeries,
                                const Eigen::MatrixXd& map, const int& minIndex, Eigen::Matrix<double, 1, 2>& projPoint);
//关键点重置
int keyPointSet(const Eigen::MatrixXd& pointSeries, const Eigen::Matrix<double, 1, 2>& currentPoint,
                                std::vector<cv::Point2d>& realPoint);   
//修改地图
int modificMap(const Eigen::Matrix<double, 1, 2>& projPoint, const Eigen::MatrixXd& pointSeries, 
                                const int& minIndex, Eigen::MatrixXd& map);
//计算代价地图
int costMapSet(const Eigen::Matrix<double, 20, 20>& map,
                            const Eigen::Matrix<double, 20, 2>& keyPoint,
                            Eigen::Matrix<double, 20, 20>& CostMap );
//获取目标点像素坐标并转换成实际位置
// void onMouse(int event, int x, int y, int flags, void* ustc);
// Dijkstra算法
void Dijkstra(int begin, int head, const Eigen::Matrix<double, 20, 20> &map);
// 最优路径显式化
int searchPath(int *pre, int v, int u);    

void getHeadingPoint(const Eigen::MatrixXd& carPosition, const Eigen::Matrix<double, 20, 20>& finalMap, const Eigen::Matrix<double, 1, 2> carProj, const Eigen::Matrix<double, 20, 2> finalKeyPoint, const int& minIndex, int& head);
//
int genGlobalPath(int* que, int tot, const std::vector<cv::Point2d>  realPoint, Eigen::Matrix<double, 7, Eigen::Dynamic>&  globalPath);

// int InitGlobalMap(const double x, const double y, const double heading, const double speed, Eigen::Matrix<double, 7, Eigen::Dynamic>&  globalPath,  cv::Mat& retImage,  Eigen::VectorXd& car); 
int InitGlobalMap(const double gpsX, const double gpsY, const double heading, const double speed, const Eigen::Matrix<double, 1, 2> endPoint, Eigen::Matrix<double, 7, Eigen::Dynamic> &globalPath, Eigen::VectorXd& car);
