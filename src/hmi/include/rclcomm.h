#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <iostream>
#include <QDebug>
#include <QObject>
#include <QThread>
#include <QPoint>
#include <QVector> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include"toolKits.h"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include "target_bbox_msgs/msg/bounding_box.hpp"
// #include "target_bbox_msgs/msg/bounding_boxes.hpp"

class rclcomm : public QThread {
  Q_OBJECT
  
  public:
    rclcomm();
    void run() override;
  
 private:
   void gps_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void global_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void cam_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void local_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void local_callback_1(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void lidar_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
   void stopline_callback(const std_msgs::msg::Float64::SharedPtr msg);
  //  void lidar_callback(const target_bbox_msgs::msg::BoundingBoxes::SharedPtr msg);
  //  void pointCloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg);
  
 private:

    std::shared_ptr<rclcpp::Node> node;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_gps;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_dest;
    
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_speed_gears;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_global;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cam;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_lidar;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_local;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_local_1;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_stopline;



    // rclcpp::Subscription<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr sub_lidar;

    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointCloud;

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_other;

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_pointCloud;


    // rclcpp::executors::MultiThreadedExecutor executor;//不可以定义在这里

  
  signals:
    void emitGpsTopicData(qreal, qreal, QString, QString); 
    void emitGlobalData(QVector<qreal>); 
    void emitLocalData(QVector<qreal>); 
    void emitLocalData1(QVector<qreal>); 
    void emitCamTopicData(QVector<qreal>);
    void emitLidarTopicData(QVector<qreal>);
    void emitstopline(qreal);
    // void emitLidarTopicData(QVector<qreal>); 
    // void emitPointCloudData(QVector<qreal>);

  private slots:
    void recvDestCallback(QPointF);
    void recvVelocityAndGearsData(QVector<qint64>);
};

#endif  // RCLCOMM_H