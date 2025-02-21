#include <rclcpp/rclcpp.hpp>
// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include<cstdio>
// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

// OpenCV and ROS
#include <image_geometry/pinhole_camera_model.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include "obstacle_detector.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using Clock = std::chrono::high_resolution_clock;

class Processing_lidar : public rclcpp::Node
{
private:

    float CLUSTER_THRESH = 0.6;//0.5
    int CLUSTER_MAX_SIZE = 5000; //2000
    int CLUSTER_MIN_SIZE = 10;
    size_t obstacle_id_;
    int obs =1;


    bool USE_PCA_BOX = 0;
    float DISPLACEMENT_THRESH = 1.0;
    float IOU_THRESH = 1.0;
    bool USE_TRACKING = 1;


    std::string bbox_target_frame_;
    std::string bbox_source_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<Box> curr_boxes_;   
    std::vector<Box> prev_boxes_;


    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;

    void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header);

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  lidar_pub;



public:
    Processing_lidar(/* args */);
    ~Processing_lidar();
    void initialize();
};


Processing_lidar::Processing_lidar(/* args */) : Node("Lidar_Processing_node")

{
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/non_ground", 1, std::bind(&Processing_lidar::pointCloudCallback, this, std::placeholders::_1));

    marker_pub =this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);

    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();
    lidar_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("lidar_pub",1);
    obstacle_id_ = 0;
    RCLCPP_INFO(this->get_logger(), "Lidar_Processing_node initialized");
}

Processing_lidar::~Processing_lidar()
{
}

void Processing_lidar::initialize()
{
    tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer, shared_from_this(), false);
}


void Processing_lidar::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{   
    Clock::time_point m_sync_start_time_= Clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *input_cloud);
    // Convert your PointXYZI cloud to PointXYZ cloud if necessary.
    pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud, *converted_cloud);
    auto cloud_clusters = obstacle_detector->clustering(converted_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
    // Publish the detected objects
    publishDetectedObjects(std::move(cloud_clusters), msg->header);
    // std::cout << "Number of clustersasdasd: " << cloud_clusters.size() << std::endl;
    double sync_duration_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - m_sync_start_time_).count() / 1e6;
    //std::cout<<" --------------clustering-------------------------- :   "<<sync_duration_ms<<std::endl;

}


void Processing_lidar::publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&& cloud_clusters, const std_msgs::msg::Header& header)
{
    curr_boxes_.clear(); // Clear the current boxes at the beginning
    for (auto& cluster : cloud_clusters){
        // Create Bounding Boxes
        Box box = USE_PCA_BOX ?
          obstacle_detector->pcaBoundingBox(cluster, obstacle_id_) :
          obstacle_detector->axisAlignedBoundingBox(cluster, obstacle_id_);
        if (obstacle_id_ < SIZE_MAX) {
            ++obstacle_id_;
        } else {
            obstacle_id_ = 0;
        }
        curr_boxes_.emplace_back(box);
    }

    // Additional code for Box ID re-assignment and frame transform...
    if (USE_TRACKING)
        obstacle_detector->obstacleTracking(prev_boxes_, curr_boxes_, DISPLACEMENT_THRESH, IOU_THRESH);

    //==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = header;


    // std::cout << "Number of boxes: " << curr_boxes_.size() << std::endl;

    Eigen::MatrixXd obs_lidar(5,curr_boxes_.size());
    Eigen::MatrixXd obs_lidar_;
    obs_lidar_.resize(5,curr_boxes_.size());
    std_msgs::msg::Float64MultiArray lidar_array_msg;

    for (const auto& box : curr_boxes_){

        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.lifetime=rclcpp::Duration::from_seconds(0.1);
        connecting_lines_marker.pose.orientation.x= box.quaternion.x();
        connecting_lines_marker.pose.orientation.y= box.quaternion.y();
        connecting_lines_marker.pose.orientation.z= box.quaternion.z();
        connecting_lines_marker.pose.orientation.w= box.quaternion.w();
        connecting_lines_marker.scale.x = 0.1;
        connecting_lines_marker.color.r = 0.0f;
        connecting_lines_marker.color.g = 1.0f;
        connecting_lines_marker.color.b = 0.0f;
        connecting_lines_marker.color.a = 0.8;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p[8];
        p[0].x =  box.dimension(0)/2; p[0].y =  box.dimension(1)/2;   p[0].z =  box.dimension(2)/2;
        p[1].x =  box.dimension(0)/2; p[1].y = -box.dimension(1)/2;   p[1].z =  box.dimension(2)/2;
        p[2].x =  box.dimension(0)/2; p[2].y = -box.dimension(1)/2;   p[2].z = -box.dimension(2)/2;
        p[3].x =  box.dimension(0)/2; p[3].y =  box.dimension(1)/2;   p[3].z = -box.dimension(2)/2;
        p[4].x = -box.dimension(0)/2; p[4].y =  box.dimension(1)/2;   p[4].z = -box.dimension(2)/2;
        p[5].x = -box.dimension(0)/2; p[5].y = -box.dimension(1)/2;   p[5].z = -box.dimension(2)/2;
        p[6].x = -box.dimension(0)/2; p[6].y = -box.dimension(1)/2;   p[6].z =  box.dimension(2)/2;
        p[7].x = -box.dimension(0)/2; p[7].y =  box.dimension(1)/2;   p[7].z =  box.dimension(2)/2;

        float RT[12] = {
        1 - 2 * box.quaternion.y() * box.quaternion.y() - 2 * box.quaternion.z() * box.quaternion.z(), 2 * box.quaternion.x() * box.quaternion.y() - 2 * box.quaternion.z() * box.quaternion.w(),
        2 * box.quaternion.x() * box.quaternion.z() + 2 * box.quaternion.y() * box.quaternion.w(),     box.position(0),
        2 * box.quaternion.x() * box.quaternion.y() + 2 * box.quaternion.z() * box.quaternion.w(),     1 - 2 * box.quaternion.x() * box.quaternion.x() - 2 * box.quaternion.z() * box.quaternion.z(),
        2 * box.quaternion.y() * box.quaternion.z() - 2 * box.quaternion.x() * box.quaternion.w(),     box.position(1),
        2 * box.quaternion.x() * box.quaternion.z() - 2 * box.quaternion.y() * box.quaternion.w(),     2 * box.quaternion.y() * box.quaternion.z() + 2 * box.quaternion.x() * box.quaternion.w(),
        1 - 2 * box.quaternion.x() * box.quaternion.x() - 2 * box.quaternion.y() * box.quaternion.y(), box.position(2)};
        
        for (int i = 0; i < 8; i++) {
            float x =
                RT[0] * p[i].x + RT[1] * p[i].y + RT[2] * p[i].z + RT[3];
            float y =
                RT[4] * p[i].x + RT[5] * p[i].y + RT[6] * p[i].z + RT[7];
            float z =
                RT[8] * p[i].x + RT[9] * p[i].y + RT[10] * p[i].z + RT[11];
            p[i].x = x;
            p[i].y = y;
            p[i].z = z; 
        }
        for(size_t i=0;i<8;i++)
        {
            connecting_lines_marker.points.push_back(p[i]);
        }
        connecting_lines_marker.points.push_back(p[0]);
        connecting_lines_marker.points.push_back(p[3]);
        connecting_lines_marker.points.push_back(p[2]);
        connecting_lines_marker.points.push_back(p[5]);
        connecting_lines_marker.points.push_back(p[6]);
        connecting_lines_marker.points.push_back(p[1]);
        connecting_lines_marker.points.push_back(p[0]);
        connecting_lines_marker.points.push_back(p[7]);
        connecting_lines_marker.points.push_back(p[4]);


       
        marker_array.markers.push_back(connecting_lines_marker);


        double x_min=std::min({p[0].x,p[1].x,p[6].x,p[7].x});
        double y_min=std::min({p[0].y,p[1].y,p[6].y,p[7].y});
        double x_max = std::max({p[0].x,p[1].x,p[6].x,p[7].x});
        double y_max = std::max({p[0].y,p[1].y,p[6].y,p[7].y});
      

        double thea =90;
        obs_lidar(0,id)=x_min;
        obs_lidar(1,id)=y_min;
        obs_lidar(2,id)=x_max;
        obs_lidar(3,id)=y_max;
        obs_lidar(4,id)=thea;

        obs_lidar_(0,id)=x_min;
        obs_lidar_(1,id)=y_min;
        obs_lidar_(2,id)=x_max;
        obs_lidar_(3,id)=y_max;
        obs_lidar_(4,id)=thea;
        std::vector<double> ldiar_array(&obs_lidar(0), obs_lidar.data() + obs_lidar.size());
        lidar_array_msg.data = ldiar_array;    
        id++;

  
    }
    //std::ofstream outputFile;
    // std::string fileName = "/home/nvidia/v2x6.20/obs1/" + std::to_string(obs) + ".txt";
    // std::remove(fileName.c_str());
    // std::ofstream outFile;
    // outFile.open(fileName);
    // for (size_t i = 0; i < obs_lidar_.cols(); ++i){
    //     outFile<< obs_lidar_(0,i)<<" "<< obs_lidar_(1,i)<<" "<< obs_lidar_(2,i)<<" "<< obs_lidar_(3,i)<<std::endl;
    // }
    // outFile.close(); 
    // obs++;  

    marker_pub->publish(marker_array);
    lidar_pub->publish(lidar_array_msg);  

}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Processing_lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}