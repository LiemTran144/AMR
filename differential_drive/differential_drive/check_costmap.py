#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

class CostmapCheckNode(Node):
    """
    Một node đơn giản để đăng ký vào costmap và một điểm được click.
    Nó sẽ in ra giá trị costmap tại điểm được click.
    """
    def __init__(self):
        super().__init__('costmap_check_node')
        
        # Biến để lưu trữ thông tin và dữ liệu costmap
        self.costmap_data = None
        self.costmap_info = None
        self.costmap_frame = ""

        # Thiết lập QoS để khớp với costmap (thường là transient local)
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- CÓ THỂ BẠN CẦN THAY ĐỔI DÒNG NÀY ---
        # Đăng ký vào chủ đề costmap (global hoặc local)
        # Kiểm tra chủ đề của bạn bằng `ros2 topic list`
        costmap_topic = '/global_costmap/costmap' 
        # costmap_topic = '/local_costmap/costmap' 
        # -------------------------------------

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.costmap_callback,
            costmap_qos)

        # Đăng ký vào chủ đề /clicked_point
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10)

    def costmap_callback(self, msg):
        """Lưu lại dữ liệu và thông tin costmap khi nhận được."""
        if self.costmap_data is None:
            self.get_logger().info("Đã nhận được Costmap!")
        
        self.costmap_data = msg.data
        self.costmap_info = msg.info
        self.costmap_frame = msg.header.frame_id

    def click_callback(self, msg: PointStamped):
        """Xử lý sự kiện click chuột và kiểm tra giá trị cost."""
        
        # Kiểm tra xem đã nhận được costmap chưa
        if self.costmap_info is None or self.costmap_data is None:
            self.get_logger().warn("Chưa nhận được costmap, không thể kiểm tra điểm.")
            return

        # Kiểm tra xem frame của điểm click có khớp với frame của costmap không
        if msg.header.frame_id != self.costmap_frame:
            self.get_logger().error(
                f"Frame không khớp! Costmap ở frame '{self.costmap_frame}' "
                f"nhưng điểm click ở frame '{msg.header.frame_id}'.\n"
                f"Trong RViz, hãy chắc chắn 'Fixed Frame' của bạn được đặt là '{self.costmap_frame}'."
            )
            return

        # Lấy thông tin bản đồ
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        width = self.costmap_info.width
        height = self.costmap_info.height

        # Lấy tọa độ thế giới (world coordinates) từ điểm click
        world_x = msg.point.x
        world_y = msg.point.y

        # Chuyển đổi tọa độ thế giới sang tọa độ lưới (grid coordinates)
        # Đây là logic tương tự như hàm worldToGrid của bạn
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)

        self.get_logger().info(f"--- Đang kiểm tra điểm ---")
        self.get_logger().info(f"Tọa độ thế giới (x, y): ({world_x:.3f}, {world_y:.3f})")

        # Kiểm tra xem điểm có nằm trong bản đồ không (tương tự poseOnMap)
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Chuyển đổi tọa độ lưới 2D sang chỉ số mảng 1D
            # (Costmap được lưu trữ dưới dạng mảng 1D theo thứ tự row-major)
            index = grid_y * width + grid_x
            
            # Lấy giá trị cost
            cost_value = self.costmap_data[index]

            self.get_logger().info(f"Tọa độ lưới (x, y): ({grid_x}, {grid_y})")
            self.get_logger().info(f"Giá trị Costmap tại điểm: {cost_value}")

            # Giải thích giá trị (dựa trên Nav2)
            if cost_value == -1:
                self.get_logger().info("Ý nghĩa: UNKNOWN (Chưa khám phá)")
            elif cost_value == 0:
                self.get_logger().info("Ý nghĩa: FREE SPACE (Không gian trống)")
            elif 1 <= cost_value <= 98:
                self.get_logger().info("Ý nghĩa: INSCRIBED (Vùng đệm/Lạm phát)")
            elif cost_value == 99:
                self.get_logger().info("Ý nghĩa: LETHAL OBSTACLE (Vật cản chết)")
            elif cost_value == 100:
                self.get_logger().warn("Ý nghĩa: LETHAL OBSTACLE (Giá trị 100 theo OccupancyGrid, Nav2 dùng 99)")
            else:
                 self.get_logger().warn(f"Ý nghĩa: Giá trị không xác định theo tiêu chuẩn Nav2/OccupancyGrid.")

        else:
            self.get_logger().warn(
                f"Tọa độ lưới ({grid_x}, {grid_y}) nằm ngoài giới hạn bản đồ "
                f"(Kích thước: {width}x{height})."
            )

def main(args=None):
    rclpy.init(args=args)
    node = CostmapCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()