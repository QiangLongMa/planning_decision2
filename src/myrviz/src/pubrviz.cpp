#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/float64_multi_array.hpp>
#include<std_msgs/msg/float64.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include<visualization_msgs/msg/marker_array.hpp>
#include<nav_msgs/msg/path.hpp>
#define x0 629985.3117 // 园区西门口世界坐标，单位：m
#define y0 3481694.8199
class PointsAndLinesPublisher : public rclcpp::Node {
public:

    PointsAndLinesPublisher(std::string name)
        : Node(name) {
        
        RCLCPP_INFO(this->get_logger(), "%s start!", name.c_str());
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("globallines", 1);
        publisherlocal_ = this->create_publisher<visualization_msgs::msg::Marker>("locallines", 1);
        publishercar_ = this->create_publisher<visualization_msgs::msg::Marker>("carlines", 1);
        publisherleftlane_ = this->create_publisher<visualization_msgs::msg::Marker>("leftines", 1);
        publisherrightlane_ = this->create_publisher<visualization_msgs::msg::Marker>("rightlines", 1);
        publisherrslidar_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubrslidar", 1);
        publisherbox_ =  this->create_publisher<visualization_msgs::msg::MarkerArray>("pubbox", 2);
        publishercarpath_ = this->create_publisher<nav_msgs::msg::Path>("pubcarpath",2);
        publisherglobalbox_ =  this->create_publisher<visualization_msgs::msg::MarkerArray>("pubglobalbox", 1);
        publisherspeed_ = this->create_publisher<visualization_msgs::msg::Marker>("pubspeed", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        gps_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("gps", 1, std::bind(&PointsAndLinesPublisher::gps_callback, this, std::placeholders::_1));
        subglobal_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("global_local_topic",1,std::bind(&PointsAndLinesPublisher::pubglobal,this, std::placeholders::_1));
        sublocal_  = this->create_subscription<std_msgs::msg::Float64MultiArray>("local_publisher",10,std::bind(&PointsAndLinesPublisher::publocal,this, std::placeholders::_1));
        rslidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("lidar_to_myrviz",1,std::bind(&PointsAndLinesPublisher::lidar_callback,this, std::placeholders::_1));
        box_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("visualization_marker_array",1,std::bind(&PointsAndLinesPublisher::box_callback,this, std::placeholders::_1));
        subglobalobses_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("global_obses",1,std::bind(&PointsAndLinesPublisher::subglobalobeses,this, std::placeholders::_1));
    }

    void box_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg){
        if (!receive_gps_flag){
            return;
        }
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < msg->markers.size(); ++i){//障碍物的个数 
            visualization_msgs::msg::Marker connecting_lines_marker;
            connecting_lines_marker.header.frame_id = "rslidar";
            connecting_lines_marker.header.stamp = rclcpp::Clock().now();
            connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
            connecting_lines_marker.id=i;
            connecting_lines_marker.pose = msg->markers[i].pose;
            connecting_lines_marker.scale.x = 0.1;
            connecting_lines_marker.color.r = 0.0f;
            connecting_lines_marker.color.g = 1.0f;
            connecting_lines_marker.color.b = 0.0f;
            connecting_lines_marker.color.a = 0.8;
            geometry_msgs::msg::Point point;
            for (size_t j = 0; j < msg->markers[i].points.size(); j++){
                double thea = normalizeAngle((gpsD)*M_PI/180); //转换
                double costhea = std::cos(thea);
                double sinthea = std::sin(thea);
                double zhouju = 1.308;
                double term0 = gpsx + zhouju*costhea;
                double term1 =gpsy + zhouju*sinthea;//恢复到车体的中心 
                point.x =  msg->markers[i].points[j].x*costhea -  msg->markers[i].points[j].y*sinthea +term0 ;
                point.y =  msg->markers[i].points[j].x*sinthea +  msg->markers[i].points[j].y*costhea +term1;
                point.z =  msg->markers[i].points[j].z;
                connecting_lines_marker.points.push_back(point);
            }
            marker_array.markers.push_back(connecting_lines_marker);
        }
        publisherbox_->publish(marker_array);
    }
    void subglobalobeses(std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if (!receive_gps_flag){
            return;
        }
        geometry_msgs::msg::Point point1;
        geometry_msgs::msg::Point point2;
        geometry_msgs::msg::Point point3;
        geometry_msgs::msg::Point point4;
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < msg->data.size() / 5; ++i){
            visualization_msgs::msg::Marker connecting_lines_marker;
            connecting_lines_marker.header.frame_id = "rslidar";
            connecting_lines_marker.header.stamp = rclcpp::Clock().now();
            connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
            connecting_lines_marker.scale.x = 0.3;
            connecting_lines_marker.color.r = 255;
            connecting_lines_marker.color.g = 97;
            connecting_lines_marker.color.b = 3;
            connecting_lines_marker.color.a = 1;
            connecting_lines_marker.id =i;
            point1.x = msg->data[0 + i*5];
            point1.y = msg->data[1 + i*5];
            point1.z = -1.0;

            point2.x = msg->data[0 + i*5];
            point2.y = msg->data[3 + i*5];
            point2.z = -1.0;

            point3.x = msg->data[2 + i*5];
            point3.y = msg->data[3 + i*5];
            point3.z = -1.0;

            point4.x = msg->data[2 + i*5];
            point4.y = msg->data[1 + i*5];
            point4.z = -1.0;
            connecting_lines_marker.points.push_back(point1); 
            connecting_lines_marker.points.push_back(point2); 
            connecting_lines_marker.points.push_back(point3); 
            connecting_lines_marker.points.push_back(point4); 
            connecting_lines_marker.points.push_back(point1);
            marker_array.markers.push_back(connecting_lines_marker);
        }
        publisherglobalbox_->publish(marker_array);
    }
    void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg){
        /*
            2-------3
            |       |
            1-------4
			车子的四个顶点 
        */
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI points;
        pcl::fromROSMsg(*msg, *input_cloud);
        if(receive_gps_flag){
           // std::cout<<"gpsD: "<<gpsD<<std::endl;
            double thea = normalizeAngle((gpsD)*M_PI/180); //转换
            double costhea = std::cos(thea);
            double sinthea = std::sin(thea);
            double zhouju = 1.308;
            double term0 = gpsx + zhouju*costhea;
            double term1 =gpsy + zhouju*sinthea;//恢复到车体的中心 
            for (size_t i = 0; i < input_cloud->points.size(); ++i){
                points.x = input_cloud->points[i].x*costhea - input_cloud->points[i].y*sinthea + term0;
                points.y = input_cloud->points[i].x*sinthea + input_cloud->points[i].y*costhea + term1;
                points.z = input_cloud->points[i].z;
                points.intensity =  input_cloud->points[i].intensity; 
                out_cloud->points.push_back(points);           
            }
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*out_cloud,cloud_msg);
            cloud_msg.header.stamp =rclcpp::Clock().now();
            cloud_msg.header.frame_id = msg->header.frame_id;
            publisherrslidar_->publish(cloud_msg);

            
        }
    }
    void gps_callback(std_msgs::msg::Float64MultiArray::SharedPtr msg){
        //std::cout<<"rviz receive gps"<<std::endl;
        static int id =0;
        if (msg->data.size()==0){
           return;
        }
        receive_gps_flag = true;
        gpsx = msg->data[0] - x0; 
        gpsy = msg->data[1] - y0;
        gpsD = 90.0-msg->data[2];
        gpsS = msg->data[3];  
        //std::cout<<"gps_msg: "<<msg->data.size()<<std::endl;
        /*
            2-------3
            |       |
            1-------4
			车子的四个顶点 
        */
		double thea = normalizeAngle(gpsD*M_PI/180); //转换
        double zhouju = 1.308;
        double cos_thea = cos(thea);
        double sin_thea = sin(thea);
        double term0 = gpsx + zhouju * cos_thea;
        double term1 = gpsy + zhouju * sin_thea;//恢复到车体的中心 

        geometry_msgs::msg::Point point1,point2,point3,point4;

		//1
		point1.x = (-length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		point1.y = (-length/2)*sin_thea+(-wigth/2)*cos_thea+term1;
        point1.z = -1.5;

		//2
		point2.x = (-length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		point2.y = (-length/2)*sin_thea+(wigth/2)*cos_thea+term1;
        point2.z = -1.5;

		//3
		point3.x = (length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		point3.y = (length/2)*sin_thea+(wigth/2)*cos_thea+term1;
        point3.z = -1.5;

		//4
		point4.x = (length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		point4.y = (length/2)*sin_thea+(-wigth/2)*cos_thea+term1;
        point4.z = -1.5;

        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.frame_id = "rslidar";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "points_and_lines";
        marker_msg.id = id;
        marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.3; // 线的宽度
          // 设置点的颜色为红色
        marker_msg.color.b = 1;
        marker_msg.color.g = 1; 
        marker_msg.color.a = 1; // 完全不透明
        marker_msg.points.push_back(point1);
        marker_msg.points.push_back(point2);
        marker_msg.points.push_back(point3);
        marker_msg.points.push_back(point4);
        marker_msg.points.push_back(point1);
        publishercar_->publish(marker_msg);

        path_msg.header.frame_id = "rslidar";  // 设置路径的坐标系
        path_msg.header.stamp = rclcpp::Clock().now();
        geometry_msgs::msg::PoseStamped pose1;
        pose1.pose.orientation.w = 1.0;
        pose1.pose.position.x = term0;
        pose1.pose.position.y = term1;
        pose1.pose.position.z = -1.5;
        path_msg.poses.push_back(pose1);
        publishercarpath_->publish(path_msg);
        geometry_msgs::msg::TransformStamped transformStamped;
            //Set the header
        transformStamped.header.stamp = rclcpp::Clock().now();
        transformStamped.header.frame_id = "rslidar";
        transformStamped.child_frame_id = "map";

        // Set the transform translation
        transformStamped.transform.translation.x = term0;  // Example translation in X direction
        transformStamped.transform.translation.y = term1;
        transformStamped.transform.translation.z = 0.0;

        // Convert Euler angles (roll, pitch, yaw) to quaternion
        double roll = 0.0;   // Roll angle (around X axis)
        double pitch = 0.0;  // Pitch angle (around Y axis)
        double yaw = thea;   // Yaw angle (around Z axis)
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        // Set the transform rotation (Quaternion)
        // Set rotation as quaternion
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();
        //Broadcast the transform
        tf_broadcaster_->sendTransform(transformStamped);
        // id ++;
        //发布车辆速度信息 
        visualization_msgs::msg::Marker marker_; // 用于显示速度的 Marker
         // 初始化 Marker
        marker_.header.frame_id = "map"; // 参考坐标系
         marker_.header.stamp = rclcpp::Clock().now();
        marker_.ns = "speed_marker";
        marker_.id = 0;  // Marker 的唯一 ID
        marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // 文本类型
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.pose.position.x = 100; // 位置（可以根据需要调整）
        marker_.pose.position.y = 100;
        marker_.pose.position.z = 1; // 高度
        // 设置文本样式
        marker_.scale.z = 4;  // 设置文本大小
        marker_.color.a = 1.0;  // 不透明
        marker_.color.r = 1.0;  // 红色（可以根据需要修改颜色）

        // std::stringstream speed_text;
        // speed_text << "Speed: " << gpsS * 3.6 << "km/h";
        // marker_.text = speed_text.str();
        // publisherspeed_->publish(marker_);
    }
    void publocal(std_msgs::msg::Float64MultiArray::SharedPtr msg){
        if(msg->data.size()==1){
            return;
        }
        //std::cout<<"rviz receive local"<<std::endl;
        static int localid =0;
        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.frame_id = "rslidar";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "points_and_lines";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.9; // 线的宽度
          // 设置点的颜色为红色
        marker_msg.color.g = 1;
        marker_msg.color.a = 1.0; // 完全不透明
        msg->data.pop_back();
        msg->data.pop_back();
        geometry_msgs::msg::Point POINT;
        for (size_t i = 0; i < msg->data.size() / 9; ++i){
            POINT.x = msg->data[i * 9];
            POINT.y = msg->data[i * 9 + 1];
            POINT.z = -1.5;
            marker_msg.points.push_back(POINT);
        }
        publisherlocal_->publish(marker_msg);
        //localid++;
    }
    void pubglobal(std_msgs::msg::Float64MultiArray::SharedPtr msg){
        //std::cout<<"rviz receive global"<<std::endl;
        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.frame_id = "rslidar";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "points_and_lines";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.1; // 线的宽度
          // 设置点的颜色为红色
        marker_msg.color.r = 1.0;
        marker_msg.color.a = 1.0; // 完全不透明
        msg->data.pop_back();
        geometry_msgs::msg::Point POINT;
        for (size_t i = 0; i < msg->data.size() / 7; ++i){
            POINT.x = msg->data[i * 7];
            POINT.y = msg->data[i * 7 + 1];
            POINT.z = -1.5;
            marker_msg.points.push_back(POINT);
        }
        publisher_->publish(marker_msg);

        auto marker_msgleft = visualization_msgs::msg::Marker();
        marker_msgleft.header.frame_id = "rslidar";
        marker_msgleft.header.stamp = rclcpp::Clock().now();
        marker_msgleft.ns = "points_and_lines";
        marker_msgleft.id = 0;
        marker_msgleft.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msgleft.action = visualization_msgs::msg::Marker::ADD;
        marker_msgleft.pose.orientation.w = 1.0;
        marker_msgleft.scale.x = 0.1; // 线的宽度
          // 设置点的颜色为红色
        marker_msgleft.color.r = 1.0;
        marker_msgleft.color.a = 1.0; // 完全不透明
        geometry_msgs::msg::Point left_point,right_point;
        for (size_t i = 0; i < msg->data.size()/7; ++i){
            double cos_theta_r = std::cos(msg->data[i*7+3]);
            double sin_theta_r = std::sin(msg->data[i*7+3]);
            left_point.x = msg->data[i*7] - sin_theta_r * left_bound;
            left_point.y = msg->data[i*7+1] + cos_theta_r * left_bound;	
            left_point.z=-1.5;
            marker_msgleft.points.push_back(left_point);
        }
        publisherleftlane_->publish(marker_msgleft);

        auto marker_msgright= visualization_msgs::msg::Marker();
        marker_msgright.header.frame_id = "rslidar";
        marker_msgright.header.stamp = rclcpp::Clock().now();
        marker_msgright.ns = "points_and_lines";
        marker_msgright.id = 0;
        marker_msgright.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msgright.action = visualization_msgs::msg::Marker::ADD;
        marker_msgright.pose.orientation.w = 1.0;
        marker_msgright.scale.x = 0.1; // 线的宽度
          // 设置点的颜色为红色
        marker_msgright.color.r = 1.0;
        marker_msgright.color.a = 1.0; // 完全不透明
        for (size_t i = 0; i < msg->data.size()/7; ++i){
            double cos_theta_r = std::cos(msg->data[i*7+3]);
            double sin_theta_r = std::sin(msg->data[i*7+3]);
            right_point.x = msg->data[i*7] - sin_theta_r * right_bound;
            right_point.y = msg->data[i*7+1] + cos_theta_r * right_bound;
            right_point.z = -1.5; 	
            marker_msgright.points.push_back(right_point);
        }
        publisherrightlane_->publish(marker_msgright);
    }

    double normalizeAngle(double angle) {
		angle = std::fmod(angle, 2 * M_PI);
		if (angle > M_PI)
			angle -= 2 * M_PI;
		else if (angle < -M_PI)
			angle += 2 * M_PI;
		return angle;
    }
private:
    // 实施发布gps的位置 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisherlocal_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publishercar_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisherleftlane_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisherrightlane_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   publisherrslidar_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr   publisherbox_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr   publisherglobalbox_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr   publishercarpath_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisherspeed_; // Marker 发布器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subglobal_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sublocal_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rslidar_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr box_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subglobalobses_;


    nav_msgs::msg::Path path_msg ;
    rclcpp::TimerBase::SharedPtr timer_;
    double wigth = 1.825;//车辆的宽度 
    double length = 4.572;
    double left_bound = 3.0;
    double right_bound = -3.0;
    bool receive_gps_flag=false;
    double gpsx; 
    double gpsy;
    double gpsD;
    double gpsS;  
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointsAndLinesPublisher>("points_and_lines_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
