#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using std::placeholders::_1;

class MapSaver : public rclcpp::Node {
public:
  MapSaver() : Node("map_saver") {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Laser_map", 1, std::bind(&MapSaver::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), ">>> 监听 /Laser_map 中... 将持续更新 PCD 文件！");
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 【核心修复】：如果是空地图，直接跳过，不要去惹 PCL 报错！
    if (cloud.empty()) {
        RCLCPP_WARN(this->get_logger(), "收到空地图帧（0个点），安全跳过...");
        return; 
    }

    // 只要有数据，就执行保存
    pcl::io::savePCDFileBinary("/root/fastlio_ws/PCD_FILES/R2_Field_Map_Latest.pcd", cloud);
    RCLCPP_INFO(this->get_logger(), "地图已更新！当前点云规模: %d 个点。请继续播放数据包...", (int)cloud.size());
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaver>());
  rclcpp::shutdown();
  return 0;
}
