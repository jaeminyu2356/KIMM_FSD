#!/isaac-sim/kit/python/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32
import math
from ament_index_python.packages import get_package_share_directory
import yaml
import tf_transformations

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Create publisher for path
        self.publisher_ = self.create_publisher(Path, 'path_drone', 10)

        # Subscribe to /clock topic and new float topic
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Int32, '/Control/mod', self.status_callback, 10)

        # 경로 상태를 추적하기 위한 변수 추가
        self.current_path_index = 0
        self.should_publish = True

        # Load configuration for start and end points
        package_name = 'isaac_ml'
        package_share_directory = get_package_share_directory(package_name)

        config_file_path = package_share_directory + "/config/isaac_sim_settings.yaml"
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)

        self.drone_pose = tuple(config["Drone"]["drone_translation"])
        self.drone_orientation = tuple(config["Drone"]["drone_orientation"])
        
        # Calculate start point 3m behind the drone based on orientation
        yaw = self.drone_orientation[2]  # Get yaw angle
        yaw = yaw * math.pi / 180.0
        
        # Calculate point 3m behind drone
        self.middle_point = (
            self.drone_pose[0] - 3.0 * math.cos(yaw),  # x coordinate
            self.drone_pose[1] - 3.0 * math.sin(yaw)   # y coordinate
        )
        self.start_point = (0.0, 0.0)
        self.end_point = (self.drone_pose[0], self.drone_pose[1])
        
        self.increment = 0.05  # 5cm
        self.path1 = self.generate_path1()
        self.path2 = self.generate_path2()
        
        self.mod_cnt = 0

    def status_callback(self, msg):
        """새로운 콜백 함수: path_status 토픽을 처리합니다."""
        if msg.data == 3:  # Int32 메시지가 3일 때
            self.mod_cnt += 1
            if self.mod_cnt % 10 == 0:
                self.current_path_index += 1  # 다음 경로로 이동
                self.should_publish = True    # 발행 허용
                self.get_logger().info(f'Moving to next path: {self.current_path_index}')
                self.mod_cnt = 0

    def clock_callback(self, msg):
        """Callback function triggered whenever a message is received on the /clock topic."""
        if self.should_publish:
            if self.current_path_index == 1:
                self.publisher_.publish(self.path1)
            elif self.current_path_index == 2:
                self.publisher_.publish(self.path2)

    def generate_path1(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Generate points for first line (start to middle)
        start_x, start_y = self.start_point
        middle_x, middle_y = self.middle_point
        total_distance1 = math.sqrt((middle_x - start_x) ** 2 + (middle_y - start_y) ** 2)
        num_points1 = int(total_distance1 / self.increment)

        # Calculate orientations
        yaw1 = math.atan2(middle_y - start_y, middle_x - start_x)
        quaternion1 = tf_transformations.quaternion_from_euler(0, 0, yaw1)
        

        # First line
        for i in range(num_points1 + 1):
            ratio = i / num_points1
            x = start_x + ratio * (middle_x - start_x)
            y = start_y + ratio * (middle_y - start_y)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = quaternion1[0]
            pose.pose.orientation.y = quaternion1[1]
            pose.pose.orientation.z = quaternion1[2]
            pose.pose.orientation.w = quaternion1[3]
            
            path_msg.poses.append(pose)

        return path_msg

    def generate_path2(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        middle_x, middle_y = self.middle_point

        # Generate points for second line (middle to end)
        end_x, end_y = self.end_point
        total_distance2 = math.sqrt((end_x - middle_x) ** 2 + (end_y - middle_y) ** 2)
        num_points2 = int(total_distance2 / self.increment)
        
        yaw2 = self.drone_orientation[2]
        quaternion2 = tf_transformations.quaternion_from_euler(0, 0, yaw2)

        # Second line
        for i in range(num_points2 + 1):
            ratio = i / num_points2
            x = middle_x + ratio * (end_x - middle_x)
            y = middle_y + ratio * (end_y - middle_y)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = quaternion2[0]
            pose.pose.orientation.y = quaternion2[1]
            pose.pose.orientation.z = quaternion2[2]
            pose.pose.orientation.w = quaternion2[3]
            
            path_msg.poses.append(pose)

        return path_msg
    
def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
