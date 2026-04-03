#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import math
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, inverse_matrix, concatenate_matrices, euler_from_quaternion



# Định nghĩa các trạng thái cho dễ đọc
STATE_IDLE = 0        # Chờ path
STATE_ALIGN_START = 1 # Xoay đầu lúc bắt đầu
STATE_TRACKING = 2    # Đang chạy bám đường
STATE_ALIGN_GOAL = 3  # Xoay hướng lúc về đích


class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner_node")

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        # self.declare_parameter("kp", 2.0)
        # self.declare_parameter("kd", 0.1)
        # self.declare_parameter("step_size", 0.2)
        # self.declare_parameter("max_linear_velocity", 0.3)
        # self.declare_parameter("max_angular_velocity", 1.0)


        # Thay vì 'kp', 'kd'
        self.declare_parameter("kp_linear", 1.5)  # 1.5
        self.declare_parameter("kd_linear", 0.01) # 0.02
        self.declare_parameter("kp_angular", 1.5)
        self.declare_parameter("kd_angular", 0.01)

        self.declare_parameter("step_size", 0.2)
        self.declare_parameter("max_linear_velocity", 0.2)
        self.declare_parameter("max_angular_velocity", 0.3)



        self.declare_parameter("xy_goal_tolerance", 0.05)  # 15cm
        self.declare_parameter("yaw_goal_tolerance", 0.15) # ~3 độ

        # Get values
        self.kp_lin = self.get_parameter("kp_linear").value
        self.kd_lin = self.get_parameter("kd_linear").value
        self.kp_ang = self.get_parameter("kp_angular").value
        self.kd_ang = self.get_parameter("kd_angular").value

        self.step_size = self.get_parameter("step_size").value
        self.max_v = self.get_parameter("max_linear_velocity").value
        self.max_w = self.get_parameter("max_angular_velocity").value

        
        self.xy_tol = self.get_parameter("xy_goal_tolerance").value
        self.yaw_tol = self.get_parameter("yaw_goal_tolerance").value


        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, "/plan", self.path_callback, 10) 

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.global_plan = None

        self.current_state = STATE_IDLE

        self.prev_angular_error = 0.0
        self.prev_linear_error = 0.0
        self.last_cycle_time = self.get_clock().now()

    def path_callback(self, path: Path):
        self.get_logger().info("New path received")
        self.global_plan = path
        self.current_state = STATE_ALIGN_START
        self.prev_angular_error = 0.0
        self.prev_linear_error = 0.0
        self.last_cycle_time = self.get_clock().now()
        # self.get_logger().info(f"End point: {self.global_plan.poses[-1].pose.position.x}, {self.global_plan.poses[-1].pose.position.y}") 

    # def goal_callback(self, msg: PoseStamped):
    #         try:
    #             # 1. Lấy Transform từ frame của Goal (map) -> odom
    #             # Dùng rclpy.time.Time() để lấy transform mới nhất
    #             trans = self.tf_buffer.lookup_transform(
    #                 'odom', 
    #                 msg.header.frame_id, 
    #                 rclpy.time.Time())
    #             self.goal = PoseStamped()
    #             self.goal.pose.orientation = trans.transform.rotation  # Lấy orientation từ transform
    #             self.calc_orientation_error(self.goal, self.goal)  # Chỉ để log góc

    #         except Exception as ex:
    #             self.get_logger().warn(f"Could not transform goal pose: {ex}")

    def goal_callback(self, msg: PoseStamped):
        try:
            # 1. Lấy Transform từ frame của Goal (thường là 'map') -> 'odom'
            trans = self.tf_buffer.lookup_transform(
                'odom', 
                msg.header.frame_id, 
                rclpy.time.Time())

            # 2. CHUYỂN ĐỔI: Phải nhân ma trận để đưa Goal về hệ Odom
            # (Bạn đã xóa mất đoạn này trong code vừa gửi!)
            
            # Quaternion của Goal
            goal_quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
                         msg.pose.orientation.z, msg.pose.orientation.w]
            goal_mat = quaternion_matrix(goal_quat)

            # Quaternion của Transform
            trans_quat = [trans.transform.rotation.x, trans.transform.rotation.y, 
                          trans.transform.rotation.z, trans.transform.rotation.w]
            trans_mat = quaternion_matrix(trans_quat)

            # Nhân ma trận: Odom_Pose = Transform * Map_Pose
            final_mat = concatenate_matrices(trans_mat, goal_mat)

            # 3. Trích xuất Yaw từ kết quả cuối cùng
            _, _, final_yaw = euler_from_quaternion(quaternion_from_matrix(final_mat))
            
            # 4. Lưu vào biến class (để dùng ở STATE_ALIGN_GOAL)
            self.final_goal_yaw = final_yaw
            
            # Log ra để kiểm tra
            self.get_logger().info(f"Goal received in {msg.header.frame_id}, converted to Odom Yaw: {final_yaw:.3f} rad")

        except Exception as ex:
            self.get_logger().warn(f"Could not transform goal pose: {ex}")


    def control_loop(self):
        if not self.global_plan or not self.global_plan.poses:
            return

        # Get the robot's current pose in the odom frame
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time())    
        except Exception as ex:
            self.get_logger().warn(f"Could not transform: {ex}")
            return

        # Transform plan to robot's frame
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform Plan in robot's frame")
            return

        robot_pose = PoseStamped()
        robot_pose.header.frame_id  = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x  = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y  = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        next_pose: PoseStamped = self.get_next_pose(robot_pose)
        dx = next_pose.pose.position.x - robot_pose.pose.position.x
        dy = next_pose.pose.position.y - robot_pose.pose.position.y
        dist_to_goal = math.sqrt(dx ** 2 + dy ** 2)
        cmd_vel = Twist()

        self.next_pose_pub.publish(next_pose)

        # Calculate the PDMotionPlanner command
        # Transform robot pose and next pose into matrices
        robot_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w,
        ])
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y
 
        next_pose_tf = quaternion_matrix([
            next_pose.pose.orientation.x,
            next_pose.pose.orientation.y,
            next_pose.pose.orientation.z,
            next_pose.pose.orientation.w,
        ])

        next_pose_tf[0][3] = next_pose.pose.position.x
        next_pose_tf[1][3] = next_pose.pose.position.y

        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)
        y = next_pose_robot_tf[1, 3]
        x = next_pose_robot_tf[0, 3] 


        heading_error = math.atan2(y, x)  
        linear_error = math.sqrt(y**2 + x**2)


        dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9
        if dt == 0: dt = 0.1

        if self.current_state == STATE_ALIGN_START:
            # orient_error = self.calc_orientation_error(robot_pose, next_pose)
            self.get_logger().info(f"Heading Error at Start: {heading_error:.3f} rad")
            # self.get_logger().info(f"Orientation Error at Start: {orient_error:.3f} rad")
               
            if abs(heading_error) < self.yaw_tol:
                self.get_logger().info("Start Aligned! -> Switch to TRACKING")
                self.current_state = STATE_TRACKING
                self.prev_angular_error = 0.0  # Reset PID terms
            else:
                # Chỉ xoay (Linear = 0)
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.calc_pid_angular(heading_error, dt)
                # self.get_logger().info(f"Aligning Start... Err: {heading_error:.2f}")

        elif self.current_state == STATE_TRACKING:
            # self.get_logger().info(f"Tracking... X: {x:.3f}, y: {y:.3f}")
            if dist_to_goal < self.xy_tol:
                self.get_logger().info("XY Goal Reached! -> Switch to ALIGN_GOAL")
                self.current_state = STATE_ALIGN_GOAL

            else:

                # self.get_logger().info(f"Tracking... Linear Error: {linear_error:.3f}, Heading Error: {heading_error:.3f}")


                cmd_vel.linear.x = self.calc_pid_linear(x, dt)
                cmd_vel.angular.z = self.calc_pid_angular(y, dt)
                
                # Publish next pose để debug visual
                self.next_pose_pub.publish(next_pose)

        elif self.current_state == STATE_ALIGN_GOAL:
            # 1. Lấy điểm đích cuối cùng từ Plan (lúc này đã ở frame Odom do hàm transform_plan phía trên)

            
            # 2. Tính góc lệch (Orientation Error)
            # Hàm này sẽ lấy Yaw của robot (odom) và Yaw của final_pose (odom) để trừ nhau
            yaw_error = self.final_goal_yaw - self.get_yaw(robot_pose)
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize to [-pi, pi]
            # 3. Kiểm tra điều kiện dừng
            if abs(yaw_error) < self.yaw_tol:
                self.get_logger().info("Goal Reached.")
                self.get_logger().info(f"Error... Linear: {dist_to_goal:.3f}, Angular: {yaw_error:.3f}")
                self.global_plan = None # Xóa plan để dừng loop hoặc về IDLE
                self.current_state = STATE_IDLE
                cmd_vel = Twist() # Gửi lệnh vận tốc 0 để dừng hẳn xe
            else:
                # 4. Điều khiển xoay tại chỗ (Linear = 0)
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = self.calc_pid_angular(yaw_error, dt)
                
                # Log để debug
                # self.get_logger().info(f"Aligning Goal... Error: {math.degrees(yaw_error):.2f} deg")

        self.get_logger().info(f"In State: {self.current_state}. Errors... Linear: {x:.3f}, Heading: {y:.3f}, Cmd Vel - Linear: {cmd_vel.linear.x:.3f}, Angular: {cmd_vel.angular.z:.3f}")
        self.cmd_pub.publish(cmd_vel)
        self.last_cycle_time = self.get_clock().now()


    def calc_orientation_error(self, robot_pose, target_pose):
        _, _, robot_yaw = euler_from_quaternion([
            robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z, robot_pose.pose.orientation.w
        ])   # Lấy yaw của robot
        _, _, target_yaw = euler_from_quaternion([
            target_pose.pose.orientation.x, target_pose.pose.orientation.y,
            target_pose.pose.orientation.z, target_pose.pose.orientation.w
        ])   # Lấy yaw của target goal
        
        delta = target_yaw - robot_yaw
        # Chuẩn hóa về [-pi, pi]
        self.get_logger().info(f"Robot Yaw: {robot_yaw:.3f}, Target Yaw: {target_yaw:.3f}, Delta: {delta:.3f}")
        return math.atan2(math.sin(delta), math.cos(delta))
    
    def get_yaw(self, target_pose):

        _, _, target_yaw = euler_from_quaternion([
            target_pose.pose.orientation.x, target_pose.pose.orientation.y,
            target_pose.pose.orientation.z, target_pose.pose.orientation.w
        ])   # Lấy yaw của target goal
        
        return target_yaw
    
    def calc_pid_angular(self, error, dt):
        derivative = (error - self.prev_angular_error) / dt
        output = self.kp_ang * error + self.kd_ang * derivative
        self.prev_angular_error = error
        # Clamp
        return max(-self.max_w, min(output, self.max_w))

    def calc_pid_linear(self, error, dt):
        derivative = (error - self.prev_linear_error) / dt
        output = self.kp_lin * error + self.kd_lin * derivative
        self.prev_linear_error = error
        # Clamp
        return max(-self.max_v, min(output, self.max_v))

    def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        next_pose = self.global_plan.poses[-1]  
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > self.step_size:
                next_pose = pose
            else:
                break
        return next_pose

    def transform_plan(self, frame):
        if self.global_plan.header.frame_id == frame:
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                frame, self.global_plan.header.frame_id, rclpy.time.Time())  
        except Exception as ex:
            self.get_logger().error(
                f"Couldn't transform plan from frame {self.global_plan.header.frame_id} to {frame}: {ex}")
            return False
        # Biến transform của ROS thành ma trận đồng nhất 4x4 (rotation + translation), để tiện tính toán:
        transform_matrix = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y

        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y

            # transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)
            transformed_pose = concatenate_matrices(transform_matrix, pose_matrix)  
            
            [pose.pose.orientation.x,pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] = translation_from_matrix(transformed_pose)

        self.global_plan.header.frame_id = frame
        return True


def main(args=None):
    rclpy.init(args=args)
    pd_motion_planner = PDMotionPlanner()
    rclpy.spin(pd_motion_planner)
    pd_motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Path
# from tf2_ros import Buffer, TransformListener
# import math
# from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, inverse_matrix, concatenate_matrices, euler_from_quaternion



# # Định nghĩa các trạng thái cho dễ đọc
# STATE_IDLE = 0        # Chờ path
# STATE_ALIGN_START = 1 # Xoay đầu lúc bắt đầu
# STATE_TRACKING = 2    # Đang chạy bám đường
# STATE_ALIGN_GOAL = 3  # Xoay hướng lúc về đích


# class PDMotionPlanner(Node):
#     def __init__(self):
#         super().__init__("pd_motion_planner_node")

#         # TF buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # Parameters
#         # self.declare_parameter("kp", 2.0)
#         # self.declare_parameter("kd", 0.1)
#         # self.declare_parameter("step_size", 0.2)
#         # self.declare_parameter("max_linear_velocity", 0.3)
#         # self.declare_parameter("max_angular_velocity", 1.0)


#         # Thay vì 'kp', 'kd'
#         self.declare_parameter("kp_linear", 1.5)  # 1.5
#         self.declare_parameter("kd_linear", 0.01) # 0.02
#         self.declare_parameter("kp_angular", 1.5)
#         self.declare_parameter("kd_angular", 0.01)

#         self.declare_parameter("step_size", 0.3)
#         self.declare_parameter("max_linear_velocity", 0.2)
#         self.declare_parameter("max_angular_velocity", 0.3)



#         self.declare_parameter("xy_goal_tolerance", 0.10)  # 15cm
#         self.declare_parameter("yaw_goal_tolerance", 0.15) # ~3 độ

#         # Get values
#         self.kp_lin = self.get_parameter("kp_linear").value
#         self.kd_lin = self.get_parameter("kd_linear").value
#         self.kp_ang = self.get_parameter("kp_angular").value
#         self.kd_ang = self.get_parameter("kd_angular").value

#         self.step_size = self.get_parameter("step_size").value
#         self.max_v = self.get_parameter("max_linear_velocity").value
#         self.max_w = self.get_parameter("max_angular_velocity").value

        
#         self.xy_tol = self.get_parameter("xy_goal_tolerance").value
#         self.yaw_tol = self.get_parameter("yaw_goal_tolerance").value


#         # Subscribers and publishers
#         self.path_sub = self.create_subscription(Path, "/plan", self.path_callback, 10) 

#         self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, "/nav/cmd_vel", 10)
#         self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

#         # Control loop
#         self.timer = self.create_timer(0.1, self.control_loop)
#         self.global_plan = None

#         self.current_state = STATE_IDLE

#         self.prev_angular_error = 0.0
#         self.prev_linear_error = 0.0
#         self.last_cycle_time = self.get_clock().now()

#     def path_callback(self, path: Path):
#         self.get_logger().info("New path received")
#         self.global_plan = path
#         self.current_state = STATE_ALIGN_START
#         self.prev_angular_error = 0.0
#         self.prev_linear_error = 0.0
#         self.last_cycle_time = self.get_clock().now()

#     def goal_callback(self, msg: PoseStamped):
#         try:
#             trans = self.tf_buffer.lookup_transform(
#                 'odom', 
#                 msg.header.frame_id, 
#                 rclpy.time.Time())
        
#             # Quaternion của Goal
#             goal_quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
#                          msg.pose.orientation.z, msg.pose.orientation.w]
#             goal_mat = quaternion_matrix(goal_quat)

#             # Quaternion của Transform
#             trans_quat = [trans.transform.rotation.x, trans.transform.rotation.y, 
#                           trans.transform.rotation.z, trans.transform.rotation.w]
#             trans_mat = quaternion_matrix(trans_quat)

#             # Nhân ma trận: Odom_Pose = Transform * Map_Pose
#             final_mat = concatenate_matrices(trans_mat, goal_mat)

#             _, _, final_yaw = euler_from_quaternion(quaternion_from_matrix(final_mat))
#             self.final_goal_yaw = final_yaw
#             self.get_logger().info(f"Goal received in {msg.header.frame_id}, converted to Odom Yaw: {self.final_goal_yaw:.3f} rad")
            

#             #reformulate to use get_yaw function
#             # self.final_goal_yaw = self.get_yaw(quaternion_from_matrix(final_mat))
#             # self.get_logger().info(f"Final Goal Yaw in Odom: {self.final_goal_yaw:.3f} rad")

#         except Exception as ex:
#             self.get_logger().warn(f"Could not transform goal pose: {ex}")


#     def control_loop(self):
#         if not self.global_plan or not self.global_plan.poses:
#             return

#         # Get the robot's current pose in the odom frame
#         try:
#             robot_pose_transform = self.tf_buffer.lookup_transform(
#                 "odom", "base_link", rclpy.time.Time())    
#         except Exception as ex:
#             self.get_logger().warn(f"Could not transform: {ex}")
#             return

#         # Transform plan to robot's frame
#         if not self.transform_plan(robot_pose_transform.header.frame_id):
#             self.get_logger().error("Unable to transform Plan in robot's frame")
#             return

#         robot_pose = PoseStamped()
#         robot_pose.header.frame_id  = robot_pose_transform.header.frame_id
#         robot_pose.pose.position.x  = robot_pose_transform.transform.translation.x
#         robot_pose.pose.position.y  = robot_pose_transform.transform.translation.y
#         robot_pose.pose.orientation = robot_pose_transform.transform.rotation

#         next_pose: PoseStamped = self.get_next_pose(robot_pose)
#         dx = next_pose.pose.position.x - robot_pose.pose.position.x
#         dy = next_pose.pose.position.y - robot_pose.pose.position.y
#         dist_to_goal = math.sqrt(dx ** 2 + dy ** 2)
#         cmd_vel = Twist()

#         self.next_pose_pub.publish(next_pose)

#         # Calculate the PDMotionPlanner command
#         # Transform robot pose and next pose into matrices
#         robot_tf = quaternion_matrix([
#             robot_pose.pose.orientation.x,
#             robot_pose.pose.orientation.y,
#             robot_pose.pose.orientation.z,
#             robot_pose.pose.orientation.w,
#         ])
#         robot_tf[0][3] = robot_pose.pose.position.x
#         robot_tf[1][3] = robot_pose.pose.position.y
 
#         next_pose_tf = quaternion_matrix([
#             next_pose.pose.orientation.x,
#             next_pose.pose.orientation.y,
#             next_pose.pose.orientation.z,
#             next_pose.pose.orientation.w,
#         ])

#         next_pose_tf[0][3] = next_pose.pose.position.x
#         next_pose_tf[1][3] = next_pose.pose.position.y

#         next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)
#         y = next_pose_robot_tf[1, 3]
#         x = next_pose_robot_tf[0, 3] 

#         heading_error = math.atan2(y, x)  
#         linear_error = math.sqrt(y**2 + x**2)

#         dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9
#         if dt == 0: dt = 0.1

#         if self.current_state == STATE_ALIGN_START:
               
#             if abs(heading_error) < self.yaw_tol:
#                 self.get_logger().info("Start Aligned! -> Switch to TRACKING")
#                 self.current_state = STATE_TRACKING
#                 self.prev_angular_error = 0.0  # Reset PID terms
#             else:
#                 # Chỉ xoay (Linear = 0)
#                 cmd_vel.linear.x = 0.0
#                 cmd_vel.angular.z = self.calc_pid_angular(heading_error, dt)

#         elif self.current_state == STATE_TRACKING:
#             # self.get_logger().info(f"Tracking... X: {x:.3f}, y: {y:.3f}")
#             if dist_to_goal < self.xy_tol:
#                 self.get_logger().info("XY Goal Reached! -> Switch to ALIGN_GOAL")
#                 self.current_state = STATE_ALIGN_GOAL

#             else:

#                 cmd_vel.linear.x = self.calc_pid_linear(x, dt)
#                 cmd_vel.angular.z = self.calc_pid_angular(y, dt)
                
#                 self.next_pose_pub.publish(next_pose)

#         elif self.current_state == STATE_ALIGN_GOAL:

#             yaw_error = self.final_goal_yaw - self.get_yaw(robot_pose)
#             yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize to [-pi, pi]
#             # 3. Kiểm tra điều kiện dừng
#             if abs(yaw_error) < self.yaw_tol:
#                 self.get_logger().info("MISSION COMPLETE! Goal Reached. Final position: x={:.3f}, y={:.3f}".format(robot_pose.pose.position.x, robot_pose.pose.position.y))
#                 self.global_plan = None 
#                 self.current_state = STATE_IDLE
#                 cmd_vel = Twist() # Gửi lệnh vận tốc 0 để dừng hẳn xe
#             else:
#                 cmd_vel.linear.x = 0.0
#                 cmd_vel.angular.z = self.calc_pid_angular(yaw_error, dt)
        
#         self.get_logger().info(f"Cmd Vel - Linear: {cmd_vel.linear.x:.3f}, Angular: {cmd_vel.angular.z:.3f}")
#         self.cmd_pub.publish(cmd_vel)
#         self.last_cycle_time = self.get_clock().now()


#     def get_yaw(self, target_pose):

#         _, _, target_yaw = euler_from_quaternion([
#             target_pose.pose.orientation.x, target_pose.pose.orientation.y,
#             target_pose.pose.orientation.z, target_pose.pose.orientation.w
#         ])   # Lấy yaw của target goal
        
#         return target_yaw
    
#     def calc_pid_angular(self, error, dt):
#         derivative = (error - self.prev_angular_error) / dt
#         output = self.kp_ang * error + self.kd_ang * derivative
#         self.prev_angular_error = error
#         # Clamp
#         return max(-self.max_w, min(output, self.max_w))

#     def calc_pid_linear(self, error, dt):
#         derivative = (error - self.prev_linear_error) / dt
#         output = self.kp_lin * error + self.kd_lin * derivative
#         self.prev_linear_error = error
#         # Clamp
#         return max(-self.max_v, min(output, self.max_v))

#     def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
#         next_pose = self.global_plan.poses[-1]  
#         for pose in reversed(self.global_plan.poses):
#             dx = pose.pose.position.x - robot_pose.pose.position.x
#             dy = pose.pose.position.y - robot_pose.pose.position.y
#             distance = math.sqrt(dx * dx + dy * dy)
#             if distance > self.step_size:
#                 next_pose = pose
#             else:
#                 break
#         return next_pose

#     def transform_plan(self, frame):
#         if self.global_plan.header.frame_id == frame:
#             return True

#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 frame, self.global_plan.header.frame_id, rclpy.time.Time())  
#         except Exception as ex:
#             self.get_logger().error(
#                 f"Couldn't transform plan from frame {self.global_plan.header.frame_id} to {frame}: {ex}")
#             return False
#         # Biến transform của ROS thành ma trận đồng nhất 4x4 (rotation + translation), để tiện tính toán:
#         transform_matrix = quaternion_matrix([
#                 transform.transform.rotation.x,
#                 transform.transform.rotation.y,
#                 transform.transform.rotation.z,
#                 transform.transform.rotation.w,
#             ])
#         transform_matrix[0][3] = transform.transform.translation.x
#         transform_matrix[1][3] = transform.transform.translation.y

#         for pose in self.global_plan.poses:
#             pose_matrix = quaternion_matrix([
#                 pose.pose.orientation.x,
#                 pose.pose.orientation.y,
#                 pose.pose.orientation.z,
#                 pose.pose.orientation.w,
#             ])
#             pose_matrix[0][3] = pose.pose.position.x
#             pose_matrix[1][3] = pose.pose.position.y

#             # transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)
#             transformed_pose = concatenate_matrices(transform_matrix, pose_matrix)  
            
#             [pose.pose.orientation.x,pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
#             [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] = translation_from_matrix(transformed_pose)

#         self.global_plan.header.frame_id = frame
#         return True


# def main(args=None):
#     rclpy.init(args=args)
#     pd_motion_planner = PDMotionPlanner()
#     rclpy.spin(pd_motion_planner)
#     pd_motion_planner.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()