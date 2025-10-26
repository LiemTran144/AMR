#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
import rclpy.time
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue
import math


class GraphNode:
    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev
    
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y)) 


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("a_star_node")
        self.get_logger().info("A Star Path Planning Node has started")
        
        map_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # Keep last map
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PointStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/a_star/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

        self.map_ = None 
        self.visited_map = OccupancyGrid()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, map_msg: OccupancyGrid):
        self.get_logger().info("Map received")
        self.map_ = map_msg
        self.visited_map.header.frame_id = map_msg.header.frame_id
        self.visited_map.info = map_msg.info
        self.visited_map.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def goal_callback(self, pose: PointStamped):
        self.get_logger().info(f"Goal pose received at ({pose.point.x}, {pose.point.y})")
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return
            
        self.visited_map.data = [-1] * (self.map_.info.height * self.map_.info.width)
        
        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, 
                "base_link", 
                rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform map to base_footprint")
            return 
        
        start_pose = Pose()
        start_pose.position.x = map_to_base_tf.transform.translation.x
        start_pose.position.y = map_to_base_tf.transform.translation.y
        start_pose.orientation = map_to_base_tf.transform.rotation

        goal_pose = Pose()
        goal_pose.position.x = pose.point.x
        goal_pose.position.y = pose.point.y
        goal_pose.position.z = pose.point.z

        path = self.plan(start_pose, goal_pose)

        if path.poses: 
            self.get_logger().info(f"Shortest path found with {len(path.poses)} poses")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):
        explore_directions = [(-1, 0), (1, 0), (0, 1), (0, -1)] # 4-connected
        # explore_directions = [(-1, 0), (1, 0), (0, 1), (0, -1), 
        #                       (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8-connected
        pending_nodes = PriorityQueue()  
        visited_nodes = set()          
        
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        
        if not self.pose_on_map(start_node) or not self.pose_on_map(goal_node):
            self.get_logger().error("Start or goal position outside map boundaries!")
            return Path()
            
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        # start_node.heuristic = self.euclidean_distance(start_node, goal_node)
        pending_nodes.put(start_node)
        
        path_found = False
        active_node = None
        
        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()
            
            if active_node == goal_node:
                path_found = True
                break

            visited_nodes.add((active_node.x, active_node.y))
            self.visited_map.data[self.pose_to_cell(active_node)] = 100

            for dir_x, dir_y in explore_directions:
                new_node = GraphNode(active_node.x + dir_x, active_node.y + dir_y)  #new_node : GraphNode = active_node + (dir_x, dir_y)
                
                if (new_node.x, new_node.y) in visited_nodes:
                    continue
                    
                if not self.pose_on_map(new_node):
                    continue
                    
                cell_index = self.pose_to_cell(new_node)
                if cell_index >= len(self.map_.data):
                    continue
                    
                # Kiểm tra ô có thể đi được (0 = free, -1 = unknown, >0 = occupied)
                if self.map_.data[cell_index] > 0:  # Occupied
                    continue

                new_node.cost = active_node.cost + 1
                new_node.heuristic = self.manhattan_distance(new_node, goal_node)
                # new_node.heuristic = self.euclidean_distance(new_node, goal_node)
                new_node.prev = active_node
                pending_nodes.put(new_node)

            # Publish visited map occasionally
            if pending_nodes.qsize() % 50 == 0:  
                
                self.map_pub.publish(self.visited_map)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        if path_found and active_node:
            # Reconstruct path
            path_points = []
            current_node = active_node
            
            while current_node is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.map_.header.frame_id
                pose_stamped.header.stamp = path.header.stamp
                pose_stamped.pose = self.grid_to_world(current_node)
                path_points.append(pose_stamped)
                current_node = current_node.prev
            
            path_points.reverse()
            path.poses = path_points
            self.get_logger().info(f"Path reconstructed with {len(path.poses)} points")

        # Publish final visited map
        self.map_pub.publish(self.visited_map)
        return path

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0  # Default orientation
        return pose

    def world_to_grid(self, pose: Pose) -> GraphNode: 
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def pose_on_map(self, node: GraphNode):
        return (0 <= node.x < self.map_.info.width and 
                0 <= node.y < self.map_.info.height)

    def pose_to_cell(self, node: GraphNode):
        return int(node.y * self.map_.info.width + node.x)

    def manhattan_distance(self, node: GraphNode, goal_node: GraphNode):
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)
    
    def euclidean_distance(self, node: GraphNode, goal_node: GraphNode):
        return math.sqrt((node.x - goal_node.x) ** 2 + (node.y - goal_node.y) ** 2)


def main():
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()