#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <gnss/msg/imu.hpp>
#include <gnss/msg/ins.hpp>
#include <math.h>

namespace gnss {

class SyncImuGnss : public rclcpp::Node
{
public:
  SyncImuGnss(const char *name) : Node(name), sync_policy_(10), sync(sync_policy_)
  {
    syncPub = this->create_publisher<std_msgs::msg::Float64MultiArray>("gps", 10);
    subINS.subscribe(this, "/sensor/gnss/inspvax");
    subIMU.subscribe(this, "/sensor/gnss/corrimu");
    sync.connectInput(subINS, subIMU);
    sync.registerCallback(std::bind(&SyncImuGnss::SyncCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<gnss::msg::Ins, gnss::msg::Imu> sync_policy;
  sync_policy sync_policy_;
  message_filters::Subscriber<gnss::msg::Ins> subINS;
  message_filters::Subscriber<gnss::msg::Imu> subIMU;
  message_filters::Synchronizer<sync_policy> sync;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr syncPub;

  void SyncCallback(const gnss::msg::Ins::ConstSharedPtr& gnss,
                    const gnss::msg::Imu::ConstSharedPtr& imu)
  {
    // std::cout << "-SyncCallback-" << std::endl;
    // std::cout << "gnss-- sec: " << gnss->header.stamp.sec << " nono_sec: " << gnss->header.stamp.nanosec << std::endl;
    // std::cout << "imu-- sec: " << imu->header.stamp.sec << " nono_sec: " << imu->header.stamp.nanosec << std::endl;

    std_msgs::msg::Float64MultiArray msg;
    double x, y, speed;
    BLToGauss(gnss->longitude, gnss->latitude, x, y);
    speed = std::sqrt(std::pow(gnss->north_velocity, 2) + std::pow(gnss->east_velocity, 2));

    msg.data.push_back(y);
    msg.data.push_back(x);
    msg.data.push_back(gnss->azimuth);
    msg.data.push_back(speed);
    msg.data.push_back(gnss->east_velocity);
    msg.data.push_back(gnss->north_velocity);
    msg.data.push_back(imu->x_linear_acceleration);
    msg.data.push_back(imu->y_linear_acceleration);


    syncPub->publish(msg);
  }

  void BLToGauss(double longitude, double latitude, double& x, double& y)
  {
    int ProjNo = 0;

    // 带宽
    // int ZoneWide = 6;
    // int ZoneWide = 3;  //3度带宽

    double longitude1, latitude1, longitude0, X0, Y0, xval, yval;
    double a, f, e2, ee, NN, T, C, A, M, iPI;

    // 3.1415926535898/180.0;
    iPI = 0.0174532925199433;

    // 84年北京坐标系参数
    a = 6378137;
    f = 1.0 / 298.257223563;

    ProjNo = 0;
    // ProjNo = (int)(longitude / ZoneWide+0.5) ;//3度带
    // ProjNo = (int)(longitude / ZoneWide) ;      //6度带
    // longitude0 = ProjNo * ZoneWide + ZoneWide / 2; //6度带
    // longitude0 = ProjNo * ZoneWide ;//3度带
    longitude0 = 117 * iPI;

    // 经度转换为弧度
    longitude1 = longitude * iPI;

    // 纬度转换为弧度
    latitude1 = latitude * iPI;

    e2 = 2 * f - f * f;
    ee = e2 * (1.0 - e2);
    NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
    T = tan(latitude1) * tan(latitude1);
    C = ee * cos(latitude1) * cos(latitude1);
    A = (longitude1 - longitude0) * cos(latitude1);
    M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * latitude1) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) - (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));
    xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
    yval = M + NN * tan(latitude1) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
    X0 = 1000000 * (ProjNo) + 500000;
    Y0 = 0;
    xval = xval + X0;
    yval = yval + Y0;
    x = yval;
    y = xval;
  }
};
} // namespace gnss

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto gnss_sync_node = std::make_shared<gnss::SyncImuGnss>("gnss_sync_node");
  rclcpp::spin(gnss_sync_node);
  rclcpp::shutdown();

  return 0;
}
