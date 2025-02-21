#include "rclcomm.h"

rclcomm::rclcomm() {
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("hmi");
  //是一个回调组（callback group），它被用来控制订阅者的回调函数的执行。
  callback_group_sub_other = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  // callback_group_sub_pointCloud = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto sub_other_opt = rclcpp::SubscriptionOptions();
  // auto sub_pointCloud_opt = rclcpp::SubscriptionOptions();
  //是用来设置订阅者选项的对象，它指定了订阅者的回调组。
  sub_other_opt.callback_group = callback_group_sub_other;
  // sub_pointCloud_opt.callback_group = callback_group_sub_pointCloud;

  pub_dest = node->create_publisher<std_msgs::msg::Float64MultiArray>("dest_pub", 10);
  pub_speed_gears = node->create_publisher<std_msgs::msg::Int64MultiArray>("speed_gears_pub", 10);
  sub_gps = node->create_subscription<std_msgs::msg::Float64MultiArray>("gps", 1, std::bind(&rclcomm::gps_callback, this, std::placeholders::_1), sub_other_opt); 
  sub_global = node->create_subscription<std_msgs::msg::Float64MultiArray>("global_hmi_topic", 10,  std::bind(&rclcomm::global_callback, this, std::placeholders::_1), sub_other_opt); 
  sub_cam = node->create_subscription<std_msgs::msg::Float64MultiArray>("cam_pub", 10,  std::bind(&rclcomm::cam_callback, this, std::placeholders::_1), sub_other_opt); 
  sub_lidar=node->create_subscription<std_msgs::msg::Float64MultiArray>("pub_hmi_obses", 1,  std::bind(&rclcomm::lidar_callback, this, std::placeholders::_1), sub_other_opt); 
  sub_local = node->create_subscription<std_msgs::msg::Float64MultiArray>("local_publisher", 10,  std::bind(&rclcomm::local_callback, this, std::placeholders::_1), sub_other_opt); 
  sub_local_1 = node->create_subscription<std_msgs::msg::Float64MultiArray>("local_to_control_pub", 10,  std::bind(&rclcomm::local_callback_1, this, std::placeholders::_1), sub_other_opt); 
  // sub_lidar = node->create_subscription<target_bbox_msgs::msg::BoundingBoxes>("boundingbox_objects", 10, std::bind(&rclcomm::lidar_callback, this, std::placeholders::_1), sub_other_opt);
  // sub_pointCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>("rslidar_points",10,std::bind(&rclcomm::pointCloud_callback, this, std::placeholders::_1), sub_pointCloud_opt);
  sub_stopline = node->create_subscription<std_msgs::msg::Float64>("pub_stop_line", 1,  std::bind(&rclcomm::stopline_callback, this, std::placeholders::_1), sub_other_opt);
}

void rclcomm::run() {
  rclcpp::WallRate loop_rate(20); //创建了一个循环频率为20Hz的WallRate对象，用来控制循环的速率。
  rclcpp::executors::MultiThreadedExecutor executor; //创建了一个多线程执行器executor，用于执行多个节点的回调函数。
  executor.add_node(node);//
  while (rclcpp::ok()) {
    executor.spin();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
}

void rclcomm::gps_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if(msg->data.size()>0){
    // emit emitGpsTopicData(qreal(msg->data[0]), qreal(msg->data[1]), QString::fromStdString(std::to_string(msg->data[2])), QString::fromStdString(std::to_string(msg->data[3])));
    emit emitGpsTopicData(qreal(msg->data[0]), qreal(msg->data[1]), QString::number((msg->data[2]), 'f', 2), QString::number((msg->data[3]), 'f', 2));
    //   x = msg->data[0];  //   y = msg->data[1];  // Direction = msg->data[2];  //  Speed = msg->data[3];
  }else{
    std::cout<<"hmi dont get gps!!!"<<std::endl;
  }
}

void rclcomm::global_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  QVector<qreal> globalPathArray;
  for (unsigned int i = 0; i < msg->data.size(); ++i){
    globalPathArray.push_back(msg->data[i]);
  }
  emit emitGlobalData(QVector<qreal> (globalPathArray));
}

void rclcomm::cam_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  QVector<qreal> camData;
  // std::cout<<"size:" << msg->data.size() << std::endl;
  for (unsigned int i = 0; i < msg->data.size(); i++){
    camData.push_back(msg->data[i]);
    // std::cout<<"cam_data:" << msg->data[i] << std::endl;
  }
  emit emitCamTopicData(QVector<qreal> (camData));
}

void rclcomm::local_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  QVector<qreal> localPathArray;
  for (unsigned int i = 0; i < msg->data.size(); ++i){
    localPathArray.push_back(msg->data[i]);
  }
  emit emitLocalData(QVector<qreal> (localPathArray));
}

void rclcomm::local_callback_1(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  // std::cout << "conrrrrrrrrrrrrrrrrrrr!" << std::endl;
  QVector<qreal> localPathArray;
  // int ML=msg->data.size();
  // std::cout << "msg->data.size():" << msg->data.size() << std::endl;
  // std::cout << "commNode rec local_1 : " << std::endl;
  if(msg->data.size() > 2){
      msg->data.pop_back();
      msg->data.pop_back();
  }
  // std::cout << "msg->data.size() after pop :" << msg->data.size() << std::endl;
  for (unsigned int i = 0; i < msg->data.size(); i++){
    localPathArray.push_back(msg->data[i]);
    // std::cout << msg->data[i] << std::endl;
  }
  emit emitLocalData1(QVector<qreal> (localPathArray));
  // std::cout << "concommNode rec local_1 ok!" << std::endl;
}
void rclcomm::lidar_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
  QVector<qreal>  lidararray;
  if (msg->data.size()==0)
  {
    emit emitLidarTopicData(QVector<qreal> (lidararray));
  }
  else{
    //std::cout<<"hmi收到障碍物信息，数量为："<<msg->data.size()/5<<std::endl;
    for (size_t i = 0; i < msg->data.size(); i++)
    {
      lidararray.push_back(msg->data[i]);
    }
    emit emitLidarTopicData(QVector<qreal> (lidararray));
  }
}

void rclcomm::stopline_callback(const std_msgs::msg::Float64::SharedPtr msg){
  qreal stopline;
  stopline = msg->data;
  emit emitstopline(stopline);
}

//发布选择的点 
void rclcomm::recvDestCallback(QPointF selecDest){
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(selecDest.x());
  msg.data.push_back(selecDest.y());
  pub_dest->publish(msg);
}

void rclcomm::recvVelocityAndGearsData(QVector<qint64> speed_gears_flag){
  // std::cout << "Qt connect OK!" << std::endl;
  std_msgs::msg::Int64MultiArray msg;
  for(int i = 0; i < speed_gears_flag.size(); i++){
    msg.data.push_back(speed_gears_flag[i]);
  }
  pub_speed_gears->publish(msg);
}
