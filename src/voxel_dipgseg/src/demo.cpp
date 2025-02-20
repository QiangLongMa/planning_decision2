#include "time_utils.h"
#include "evaluate.h"
#include "dipgseg.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <chrono>
using Clock = std::chrono::high_resolution_clock;

float voxel_leaf_size_x_ = 0.1;
float voxel_leaf_size_y_ = 0.1;
float voxel_leaf_size_z_ = 0.1; 


// ROI boundaries
// The velodyne is mounted on the car with a bad orientation, so the x is the y axis. 
double roi_max_x_ = 80.0; //FRONT THE CAR
double roi_max_y_ = 7.0;  //LEFT THE CAR
double roi_max_z_ = 0.5; //UP THE VELODYNE

double roi_min_x_ = 1.5; //RIGHT THE CAR 
double roi_min_y_ = -7.0; //BACK THE CAR
double roi_min_z_ = -1.9; //DOWN THE VELODYNE
// Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
Eigen::Vector4f ROI_MAX_POINT = Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1);
Eigen::Vector4f ROI_MIN_POINT = Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1);

class Groundremove:public rclcpp::Node
{
public:  
    Groundremove(std::string name):Node(name) 
    {   RCLCPP_INFO(this->get_logger(), "%s node start!", name.c_str());
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("rslidar_points", 1, std::bind(&Groundremove::callback, this, std::placeholders::_1));
        non_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground", 1);
        lidar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_to_myrviz", 10);

     
    }  

    void callback(const sensor_msgs::msg::PointCloud2::ConstPtr cloud_msg){
            
            Clock::time_point m_sync_start_time_= Clock::now();
            DIPGSEG::Dipgseg dipgseg;

            pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_noi(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud3(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloudmyrviz(new pcl::PointCloud<pcl::PointXYZI>);

            pcl::fromROSMsg(*cloud_msg, *input_cloud2);
            // 定义旋转角度（60度转换为弧度）
            double angle = 4.5* M_PI/180.0;
             // 计算旋转矩阵
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
             // 执行点云旋转
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*input_cloud2, *input_cloud, transform);
        
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*input_cloud,msg);
            msg.header.frame_id = cloud_msg->header.frame_id;
            msg.header.stamp = rclcpp::Clock().now();
            lidar_pub->publish(msg);

            pcl::CropBox<pcl::PointXYZI> roi_filter;
            roi_filter.setInputCloud(input_cloud);
            roi_filter.setMax(ROI_MAX_POINT);
            roi_filter.setMin(ROI_MIN_POINT);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
            roi_filter.filter(*cloud_roi);
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(cloud_roi);
            vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

            pcl::PointCloud<pcl::PointXYZI> filtered_cloud;


            vg.filter(filtered_cloud);




            pcl::PointCloud<pcl::PointXYZI> cloud_ground, cloud_non_ground;

            dipgseg.segment_ground(filtered_cloud, cloud_ground, cloud_non_ground);
            double time_all = dipgseg.get_whole_time();
            double time_seg = dipgseg.get_seg_time();

            
            //printf("cloud size: %ld, cloud_ground size: %ld, cloud_non_ground size: %ld\n", filtered_cloud.size(), cloud_ground.size(), cloud_non_ground.size());




            pcl::toROSMsg(cloud_non_ground, non_ground_msg);
            non_ground_msg.header.frame_id = cloud_msg->header.frame_id;
            non_ground_msg.header.stamp = rclcpp::Clock().now();
            non_ground_pub->publish(non_ground_msg);

            double sync_duration_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - m_sync_start_time_).count() / 1e6;
            //std::cout<<" --------------dipgseg callback-------------------------- :   "<<sync_duration_ms<<std::endl;

    }
private:  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    sensor_msgs::msg::PointCloud2 ground_msg, non_ground_msg;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Groundremove>("Groundremove");
   

    rclcpp::spin(node);
    rclcpp::shutdown();
    

    return 0;
}