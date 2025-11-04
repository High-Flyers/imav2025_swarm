#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

class PositionPublisher : public rclcpp::Node {
public:
  PositionPublisher()
  : Node("position_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    std::string ns = this->get_namespace();
    drone_id_ = ns.substr(ns.find_last_of('/') + 1);
    id_ = drone_id_.back();
    ns_prefix_ = ns.substr(0, ns.size() - 2);

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/imav/swarm_positions", 10);

    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "fmu/out/vehicle_local_position", qos_profile_sub,
      std::bind(&PositionPublisher::local_position_cb, this, _1));

    reference_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      ns_prefix_ + "_1/fmu/out/vehicle_local_position", qos_profile_sub,
      std::bind(&PositionPublisher::reference_position_cb, this, _1));

    reference_frame_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    current_position_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    should_send_reference_transform_ = (id_ != '1');
  }

private:
  std::string drone_id_;
  char id_;
  std::string ns_prefix_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr reference_position_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> reference_frame_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> current_position_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  px4_msgs::msg::VehicleLocalPosition::SharedPtr reference_local_position_;
  bool should_send_reference_transform_;

  static std::array<double, 3> ned_to_enu(double n, double e, double d) {
    return {e, n, -d};
  }

  static std::array<double, 3> lla_to_ecef(double lat, double lon, double alt) {
    constexpr double a = 6378137.0; // semi-major axis
    constexpr double f = 1.0 / 298.257223563; // flattening
    constexpr double e_sq = f * (2 - f);
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double N = a / sqrt(1 - e_sq * pow(sin(lat_rad), 2));
    double x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    double y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    double z = (N * (1 - e_sq) + alt) * sin(lat_rad);
    return {x, y, z};
  }

  static std::array<double, 3> ecef_to_enu(double x, double y, double z, double lat_ref, double lon_ref, double alt_ref) {
    auto ref = lla_to_ecef(lat_ref, lon_ref, alt_ref);
    double lat_rad = lat_ref * M_PI / 180.0;
    double lon_rad = lon_ref * M_PI / 180.0;
    double dx = x - ref[0];
    double dy = y - ref[1];
    double dz = z - ref[2];
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    double t[3];
    t[0] = -sin_lon * dx + cos_lon * dy;
    t[1] = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    t[2] = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    return {t[0], t[1], t[2]}; // East, North, Up
  }

  static std::array<double, 3> lla_to_enu(double lat, double lon, double alt, double lat_ref, double lon_ref, double alt_ref) {
    auto ecef = lla_to_ecef(lat, lon, alt);
    return ecef_to_enu(ecef[0], ecef[1], ecef[2], lat_ref, lon_ref, alt_ref);
  }

  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    if (!reference_local_position_) return;
    if (should_send_reference_transform_) {
      make_reference_transform(msg);
      should_send_reference_transform_ = false;
      return;
    } else {
      make_current_transform(msg);
    }
    try {
      auto trans = tf_buffer_.lookupTransform(
        ns_prefix_.substr(1) + "_1_ref",
        drone_id_,
        tf2::TimePointZero);
      auto odom_msg = nav_msgs::msg::Odometry();
      odom_msg.header.stamp = this->get_clock()->now();
      odom_msg.header.frame_id = drone_id_;
      odom_msg.pose.pose.position.x = trans.transform.translation.x;
      odom_msg.pose.pose.position.y = trans.transform.translation.y;
      odom_msg.pose.pose.position.z = trans.transform.translation.z;
      auto vel_enu = ned_to_enu(msg->vx, msg->vy, msg->vz);
      odom_msg.twist.twist.linear.x = vel_enu[0];
      odom_msg.twist.twist.linear.y = vel_enu[1];
      odom_msg.twist.twist.linear.z = vel_enu[2];
      publisher_->publish(odom_msg);
      RCLCPP_DEBUG(this->get_logger(), "Published odometry from tf lookup for %s", odom_msg.header.frame_id.c_str());
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", e.what());
    }
  }

  void reference_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    reference_local_position_ = msg;
  }

  void make_reference_transform(const px4_msgs::msg::VehicleLocalPosition::SharedPtr local_position) {
    if (!reference_local_position_) return;
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->get_clock()->now();
    ts.header.frame_id = ns_prefix_ + "_1_ref";
    ts.child_frame_id = drone_id_ + "_ref";
    auto enu = lla_to_enu(
      local_position->ref_lat,
      local_position->ref_lon,
      local_position->ref_alt,
      reference_local_position_->ref_lat,
      reference_local_position_->ref_lon,
      reference_local_position_->ref_alt
    );
    ts.transform.translation.x = enu[0];
    ts.transform.translation.y = enu[1];
    ts.transform.translation.z = enu[2];
    reference_frame_static_broadcaster_->sendTransform(ts);
  }

  void make_current_transform(const px4_msgs::msg::VehicleLocalPosition::SharedPtr local_position) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->get_clock()->now();
    ts.header.frame_id = drone_id_ + "_ref";
    ts.child_frame_id = drone_id_;
    auto enu = ned_to_enu(local_position->x, local_position->y, local_position->z);
    ts.transform.translation.x = enu[0];
    ts.transform.translation.y = enu[1];
    ts.transform.translation.z = enu[2];
    current_position_broadcaster_->sendTransform(ts);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
