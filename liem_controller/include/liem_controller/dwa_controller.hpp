// #ifndef DWA_CONTROLLER__DWA_CONTROLLER_HPP_
// #define DWA_CONTROLLER__DWA_CONTROLLER_HPP_

// #include <string>
// #include <vector>
// #include <memory>
// #include <algorithm>
// #include <mutex>
// #include <tuple>

// // ROS 2 Core
// #include "rclcpp/rclcpp.hpp"
// #include "pluginlib/class_loader.hpp"
// #include "pluginlib/class_list_macros.hpp"

// // Nav2 Interfaces & Utils
// #include "nav2_core/controller.hpp"
// #include "nav2_util/lifecycle_node.hpp"
// #include "nav2_util/robot_utils.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "nav2_costmap_2d/footprint_collision_checker.hpp"

// // TF2 (Biến đổi tọa độ)
// #include "tf2_ros/buffer.h"

// namespace dwa_controller
// {

// /**
//  * @class DWA_Controller
//  * @brief Plugin điều khiển robot bám đường sử dụng thuật toán Dynamic Window Approach (DWA) thủ công.
//  */
// class DWA_Controller : public nav2_core::Controller
// {
// public:
//   DWA_Controller() = default;
//   ~DWA_Controller() override = default;

//   // --- INTERFACE BẮT BUỘC CỦA NAV2 ---

//   /**
//    * @brief Cấu hình controller, load tham số từ file yaml
//    */
//   void configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name, 
//     std::shared_ptr<tf2_ros::Buffer> tf,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

//   /**
//    * @brief Kích hoạt controller (bắt đầu publish/subscribe)
//    */
//   void activate() override;

//   /**
//    * @brief Ngưng kích hoạt controller
//    */
//   void deactivate() override;

//   /**
//    * @brief Dọn dẹp tài nguyên
//    */
//   void cleanup() override;

//   /**
//    * @brief Cập nhật giới hạn tốc độ (Interface bắt buộc, hiện tại chưa dùng)
//    */
//   void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

//   /**
//    * @brief Nhận đường dẫn toàn cục từ Planner
//    */
//   void setPlan(const nav_msgs::msg::Path & path) override;

//   /**
//    * @brief HÀM CHÍNH: Tính toán vận tốc (v, w) gửi xuống robot
//    * Chạy liên tục theo tần số controller (vd: 20Hz)
//    */
//   geometry_msgs::msg::TwistStamped computeVelocityCommands(
//     const geometry_msgs::msg::PoseStamped & pose,
//     const geometry_msgs::msg::Twist & velocity,
//     nav2_core::GoalChecker * goal_checker) override;

// protected:
//   // --- CÁC HÀM LOGIC DWA (HELPER FUNCTIONS) ---

//   /**
//    * @brief Chuyển đổi và cắt tỉa Global Plan về Local Frame (Odom/Base)
//    */
//   std::vector<geometry_msgs::msg::PoseStamped> transformGlobalPlan(
//     const geometry_msgs::msg::PoseStamped & pose);

//   /**
//    * @brief Tính toán "Cửa sổ động": Giới hạn vận tốc dựa trên gia tốc
//    * @return tuple (min_v, max_v, min_w, max_w)
//    */
//   std::tuple<double, double, double, double> computeDynamicWindow(
//     const geometry_msgs::msg::Twist & current_vel);

//   /**
//    * @brief Mô phỏng vị trí tương lai của robot dựa trên model Unicycle
//    */
//   std::vector<geometry_msgs::msg::PoseStamped> simulateTrajectory(
//     const geometry_msgs::msg::PoseStamped & current_pose,
//     const double v,
//     const double w);

//   /**
//    * @brief Kiểm tra va chạm cho toàn bộ đường dẫn mô phỏng
//    * @return Cost lớn nhất trên đường đi (-1.0 nếu va chạm)
//    */
//   double checkTrajectoryCollision(
//     const std::vector<geometry_msgs::msg::PoseStamped> & trajectory);

//   /**
//    * @brief Tính điểm cho một quỹ đạo
//    * @return Total Cost (Càng nhỏ càng tốt)
//    */
//   double scoreTrajectory(
//     const double v, 
//     const double w, 
//     const geometry_msgs::msg::Twist & current_vel,
//     const std::vector<geometry_msgs::msg::PoseStamped> & local_plan);

//   // --- BIẾN THÀNH VIÊN ---

//   // Pointer tới hệ thống ROS
//   rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
//   std::shared_ptr<tf2_ros::Buffer> tf_;
//   std::string plugin_name_;
//   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//   nav2_costmap_2d::Costmap2D * costmap_; // Truy cập dữ liệu map thô
//   rclcpp::Logger logger_ {rclcpp::get_logger("DWA_Controller")};

//   // Công cụ check va chạm (Footprint)
//   std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker> collision_checker_;

//   // Visualization Publishers
//   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
//   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

//   // Lưu trữ Global Plan
//   nav_msgs::msg::Path global_plan_;

//   // Tham số cấu hình (Load từ YAML)
//   double max_vel_x_, min_vel_x_;
//   double max_vel_theta_, min_vel_theta_;
//   double acc_lim_x_, acc_lim_theta_;
  
//   double sim_time_;
//   double sim_granularity_;
//   double controller_frequency_;

//   // Trọng số (Weights)
//   double scale_heading_; 
//   double scale_obs_;     
//   double scale_vel_;     
//   double scale_align_;   

//   std::string robot_base_frame_;
//   std::string global_frame_;
// };

// } // namespace dwa_controller

// #endif // DWA_CONTROLLER__DWA_CONTROLLER_HPP_



#ifndef DWA_CONTROLLER__DWA_CONTROLLER_HPP_
#define DWA_CONTROLLER__DWA_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <tuple>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

// Nav2 Interfaces & Utils
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

// TF2
#include "tf2_ros/buffer.h"

namespace dwa_controller
{

class DWA_Controller : public nav2_core::Controller
{
public:
  DWA_Controller() = default;
  ~DWA_Controller() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  std::vector<geometry_msgs::msg::PoseStamped> transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  std::tuple<double, double, double, double> computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_vel);

  std::vector<geometry_msgs::msg::PoseStamped> simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const double v,
    const double w);

  double checkTrajectoryCollision(
    const std::vector<geometry_msgs::msg::PoseStamped> & trajectory);

  double scoreTrajectory(
    const double v, 
    const double w, 
    const std::vector<geometry_msgs::msg::PoseStamped> & local_plan);

  // --- BIẾN THÀNH VIÊN ---
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("DWA_Controller")};

  // --- [FIX 1] SỬA LỖI TEMPLATE CHO COLLISION CHECKER ---
  // FootprintCollisionChecker trong Humble là template class
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> predict_traj_pub_;

  nav_msgs::msg::Path global_plan_;

  double max_vel_x_, min_vel_x_;
  double max_vel_theta_, min_vel_theta_;
  double acc_lim_x_, acc_lim_theta_;
  
  double sim_time_;
  double sim_granularity_;
  double controller_frequency_;

  double scale_heading_; 
  double scale_obs_;     
  double scale_vel_;     
  double scale_align_;   

  std::string robot_base_frame_;
  std::string global_frame_;
};

} // namespace dwa_controller

#endif // DWA_CONTROLLER__DWA_CONTROLLER_HPP_