#include "liem_controller/dwa_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;

namespace dwa_controller
{

void DWA_Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();

  // Khai báo tham số
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.7));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_granularity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".controller_frequency", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_heading", rclcpp::ParameterValue(32.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_obs", rclcpp::ParameterValue(24.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_align", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".robot_base_frame", rclcpp::ParameterValue("base_link"));

  // Lấy giá trị tham số
  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity", sim_granularity_);
  node->get_parameter(plugin_name_ + ".controller_frequency", controller_frequency_);
  node->get_parameter(plugin_name_ + ".scale_heading", scale_heading_);
  node->get_parameter(plugin_name_ + ".scale_obs", scale_obs_);
  node->get_parameter(plugin_name_ + ".scale_vel", scale_vel_);
  node->get_parameter(plugin_name_ + ".scale_align", scale_align_);
  node->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);
  
  global_frame_ = costmap_ros_->getGlobalFrameID();

  // Khởi tạo công cụ check va chạm (Template chính xác cho Humble)
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  predict_traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predict_trajectory", 1);
  
  RCLCPP_INFO(logger_, "DWA Controller Configured!");
}

void DWA_Controller::activate()
{
  local_plan_pub_->on_activate();
  global_pub_->on_activate();
  predict_traj_pub_->on_activate();
}

void DWA_Controller::deactivate()
{
  local_plan_pub_->on_deactivate();
  global_pub_->on_deactivate();
  predict_traj_pub_->on_deactivate();
}

void DWA_Controller::cleanup()
{
  local_plan_pub_.reset();
  global_pub_.reset();
  predict_traj_pub_.reset();
  collision_checker_.reset();
}

void DWA_Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  (void)speed_limit; (void)percentage;
}

void DWA_Controller::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped DWA_Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // 1. Khóa Costmap để an toàn luồng
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.header.stamp = rclcpp::Clock().now();


  double raw_obs;
  double norm_obs;
  double raw_goal;
  double raw_align;
  double norm_vel;


  // 2. Cắt và Biến đổi đường dẫn toàn cục về Local Frame
  std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  try {
    local_plan = transformGlobalPlan(pose);
  } catch (const nav2_core::PlannerException & e) {
    RCLCPP_ERROR(logger_, "Could not transform plan: %s", e.what());
    return cmd_vel; 
  }

  // Visualize Local Plan
  if (local_plan_pub_->get_subscription_count() > 0 && !local_plan.empty()) {
    nav_msgs::msg::Path local_path;
    local_path.header = local_plan.front().header;
    local_path.poses = local_plan;
    local_plan_pub_->publish(local_path);
  }

  // 3. Kiểm tra đến đích
  if (global_plan_.poses.empty()) return cmd_vel;
  
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    RCLCPP_INFO(logger_, "GOAL REACHED!");
    return cmd_vel;
  }

  // 4. Tính toán Cửa sổ động (Dynamic Window)
  auto [min_v, max_v, min_w, max_w] = computeDynamicWindow(velocity);

  // 5. Lấy mẫu và Tìm kiếm (Sampling & Search)
  double best_score = 1e9;
  double best_v = 0.0;
  double best_w = 0.0;
  bool found_valid_traj = false;

  double v_sample_step = 0.01; 
  double w_sample_step = 0.01;

  for (double v = min_v; v <= max_v; v += v_sample_step) {
    for (double w = min_w; w <= max_w; w += w_sample_step) {
      
      // Chấm điểm
      double score = scoreTrajectory(v, w, local_plan);

      if (score == -1.0) continue; // Va chạm -> Bỏ qua

      if (score < best_score) {
        best_score = score;
        best_v = v;
        best_w = w;
        found_valid_traj = true;
      }
    }
  }


  RCLCPP_INFO_THROTTLE(logger_, *node_.lock()->get_clock(), 1000,
            "\n>>> BEST (v=%.2f, w=%.2f) TOTAL: %.3f\n"
            "   + OBS:   %.3f (Raw: %.0f, Norm: %.2f * Scale %.1f)\n"
            "   + VEL:   %.3f (Raw: %.2f, Norm: %.2f * Scale %.1f)\n"
            "   + ALIGN: %.3f (Raw: %.2fm * Scale %.1f)\n"
            "   + GOAL:  %.3f (Raw: %.2fm * Scale %.1f)",
            best_v, best_w, best_score,
            (norm_obs * scale_obs_), raw_obs, norm_obs, scale_obs_,
            (norm_vel * scale_vel_), best_v, norm_vel, scale_vel_,
            (raw_align * scale_align_), raw_align, scale_align_,
            (raw_goal * scale_heading_), raw_goal, scale_heading_
        );

  

  // 6. Xử lý kết quả
  if (found_valid_traj) {
    cmd_vel.twist.linear.x = best_v;
    cmd_vel.twist.angular.z = best_w;

    // --- [VISUALIZE] VẼ QUỸ ĐẠO DỰ ĐOÁN LÊN RVIZ ---
    if (predict_traj_pub_->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseStamped current_pose_vis;
        costmap_ros_->getRobotPose(current_pose_vis);
        auto best_traj = simulateTrajectory(current_pose_vis, best_v, best_w);

        nav_msgs::msg::Path traj_msg;
        traj_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
        traj_msg.header.stamp = rclcpp::Clock().now();
        traj_msg.poses = best_traj;
        predict_traj_pub_->publish(traj_msg);
    }

    // --- [DEBUG LOG] IN RA CHI TIẾT ĐIỂM SỐ (1s/lần) ---
    static rclcpp::Time last_print = rclcpp::Clock().now();
    if ((rclcpp::Clock().now() - last_print).seconds() > 1.0) {
        last_print = rclcpp::Clock().now();

        // Tái tạo lại các thông số để in log (Tính xuôi, không tính ngược)
        geometry_msgs::msg::PoseStamped current_pose_debug;
        costmap_ros_->getRobotPose(current_pose_debug);
        auto traj_debug = simulateTrajectory(current_pose_debug, best_v, best_w);

        // a. Obs Cost
        raw_obs = checkTrajectoryCollision(traj_debug);
        norm_obs = raw_obs / 254.0; // Normalize
        
        // b. Align Cost
        auto last_pose = traj_debug.back();
        raw_align = 1e9;
        for (const auto & p : local_plan) {
            double d = std::hypot(last_pose.pose.position.x - p.pose.position.x,
                                  last_pose.pose.position.y - p.pose.position.y);
            if(d < raw_align) raw_align = d;
        }
        
        // c. Goal Cost
        raw_goal = 0.0;
        if(!local_plan.empty()){
            raw_goal = std::hypot(last_pose.pose.position.x - local_plan.back().pose.position.x,
                                  last_pose.pose.position.y - local_plan.back().pose.position.y);
        }

        // d. Vel Cost
        norm_vel = (max_vel_x_ - best_v) / max_vel_x_;
        if(norm_vel < 0) norm_vel = 0;


    }
    // ---------------------------------------------------

  } else {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000, 
      "DWA failed to find a valid trajectory! Robot stops.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  return cmd_vel;
}
std::vector<geometry_msgs::msg::PoseStamped> DWA_Controller::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // Lấy tên frame của costmap (thường là "odom")
  std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  
  // Lấy transform từ Plan Frame -> Costmap Frame
  geometry_msgs::msg::TransformStamped plan_to_costmap_transform;
  try {
    plan_to_costmap_transform = tf_->lookupTransform(
      costmap_frame,
      global_plan_.header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    throw nav2_core::PlannerException("Could not transform plan: " + std::string(ex.what()));
  }

  // [FIX] Tự thực hiện transform và prune thủ công để tránh lỗi thư viện
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  
  // 1. Transform toàn bộ points
  for (const auto & global_pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped local_pose;
    tf2::doTransform(global_pose, local_pose, plan_to_costmap_transform);
    transformed_plan.push_back(local_pose);
  }

  // 2. Tìm điểm gần robot nhất để cắt bỏ phần đường cũ (Pruning)
  double min_dist = 1e9;
  size_t closest_index = 0;
  for (size_t i = 0; i < transformed_plan.size(); ++i) {
    double d = std::hypot(
      pose.pose.position.x - transformed_plan[i].pose.position.x,
      pose.pose.position.y - transformed_plan[i].pose.position.y);
    if (d < min_dist) {
      min_dist = d;
      closest_index = i;
    }
  }

  // 3. Chỉ lấy các điểm từ robot trở đi (cộng thêm một khoảng dự phòng)
  std::vector<geometry_msgs::msg::PoseStamped> final_plan;
  for (size_t i = closest_index; i < transformed_plan.size(); ++i) {
    final_plan.push_back(transformed_plan[i]);
    // Giới hạn độ dài plan cục bộ (ví dụ: lấy 50 điểm hoặc 2-3 mét tới)
    if (final_plan.size() > 100) break; 
  }

  if (final_plan.empty()) return transformed_plan; // Fallback
  return final_plan;
}

std::tuple<double, double, double, double> DWA_Controller::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_vel)
{
  double dt = 1.0 / controller_frequency_;

  double min_v_acc = current_vel.linear.x - acc_lim_x_ * dt;
  double max_v_acc = current_vel.linear.x + acc_lim_x_ * dt;
  double min_v = std::max(min_vel_x_, min_v_acc);
  double max_v = std::min(max_vel_x_, max_v_acc); 

  double min_w_acc = current_vel.angular.z - acc_lim_theta_ * dt;
  double max_w_acc = current_vel.angular.z + acc_lim_theta_ * dt;
  double min_w = std::max(min_vel_theta_, min_w_acc);
  double max_w = std::min(max_vel_theta_, max_w_acc);

  return std::make_tuple(min_v, max_v, min_w, max_w);
}

std::vector<geometry_msgs::msg::PoseStamped> DWA_Controller::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const double v,
  const double w)
{
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  double dt = sim_granularity_;
  
  for (double time = 0.0; time < sim_time_; time += dt) {
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    theta += w * dt;

    geometry_msgs::msg::PoseStamped p;
    p.header = current_pose.header;
    p.pose.position.x = x;
    p.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    p.pose.orientation = tf2::toMsg(q);

    trajectory.push_back(p);
  }
  return trajectory;
}

double DWA_Controller::checkTrajectoryCollision(
  const std::vector<geometry_msgs::msg::PoseStamped> & trajectory)
{
  if (trajectory.empty()) return 0.0;
  
  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
  double max_cost = 0.0;

  for (const auto & pose : trajectory) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);

    double cost = collision_checker_->footprintCostAtPose(x, y, theta, footprint);

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || 
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || 
        cost == nav2_costmap_2d::NO_INFORMATION) {
      return -1.0; 
    }
    if (cost > max_cost) max_cost = cost;
  }
  return max_cost;
}

double DWA_Controller::scoreTrajectory(
  const double v, 
  const double w, 
  const std::vector<geometry_msgs::msg::PoseStamped> & local_plan)
{
  // 1. Mô phỏng quỹ đạo
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) return -1.0;
  
  auto trajectory = simulateTrajectory(current_pose, v, w);

  // 2. Kiểm tra va chạm
  double obs_raw_cost = checkTrajectoryCollision(trajectory);
  if (obs_raw_cost < 0.0) return -1.0; // Va chạm -> Loại bỏ ngay

  // --- [FIX QUAN TRỌNG] CHUẨN HÓA CÁC GIÁ TRỊ VỀ 0.0 -> 1.0 ---

  // a. Chuẩn hóa Obstacle Cost (Chia cho giá trị Max của Costmap là 254)
  // Giá trị giờ đây sẽ từ 0.0 (an toàn) -> 1.0 (sát vật cản)
  double normalized_obs_cost = obs_raw_cost / 254.0;

  // b. Tính Alignment Cost (Khoảng cách lệch khỏi đường dẫn)
  auto last_pose = trajectory.back();
  double min_dist_to_path = 1e9;
  for (const auto & path_pose : local_plan) {
    double d = std::hypot(
      last_pose.pose.position.x - path_pose.pose.position.x,
      last_pose.pose.position.y - path_pose.pose.position.y);
    if (d < min_dist_to_path) min_dist_to_path = d;
  }
  // Alignment không có giới hạn trên rõ ràng, nhưng ta có thể giữ nguyên
  // hoặc chia cho một ngưỡng chấp nhận được (ví dụ 1.0 mét)
  double normalized_align_cost = min_dist_to_path; 

  // c. Tính Goal Cost (Khoảng cách tới đích cục bộ)
  double dist_to_goal = 0.0;
  if (!local_plan.empty()) {
    dist_to_goal = std::hypot(
      last_pose.pose.position.x - local_plan.back().pose.position.x,
      last_pose.pose.position.y - local_plan.back().pose.position.y);
  }
  // Goal cost cũng có thể giữ nguyên vì đơn vị là mét (tương đồng với align)

  // d. Chuẩn hóa Velocity Cost (Chia cho Vận tốc tối đa)
  // v càng gần max_vel thì cost càng gần 0. v=0 thì cost=1.0
  double normalized_vel_cost = (max_vel_x_ - v) / max_vel_x_;
  if (normalized_vel_cost < 0) normalized_vel_cost = 0; // Đề phòng v vượt max

  // 3. TÍNH TỔNG CHI PHÍ
  // Bây giờ các thành phần đều có trọng số tương đương nhau
  return (scale_obs_ * normalized_obs_cost) + 
         (scale_align_ * normalized_align_cost) + 
         (scale_heading_ * dist_to_goal) + 
         (scale_vel_ * normalized_vel_cost);
}


} // namespace dwa_controller

PLUGINLIB_EXPORT_CLASS(dwa_controller::DWA_Controller, nav2_core::Controller)