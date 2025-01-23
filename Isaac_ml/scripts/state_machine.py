#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
import subprocess
import threading

class StateListener(Node):
    def __init__(self):
        super().__init__('topic_listener')

        self.sim_subscription = self.create_subscription(
            JointState,
            '/isaac_joint_commands',
            self.sim_callback,
            10
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # 최신 10개의 메시지만 유지
        )

        self.lio_subscription = self.create_subscription(
            Odometry,
            '/odometry/imu',
            self.lio_callback,
            qos_profile
        )

        self.current_state = 0
        self.is_sim_ready = False
        self.is_lio_ready = False
        self.is_nav2_ready = False

        self.sim_sub_count = 0
        self.lio_sub_count = 0

        # SIM 스크립트 실행
        self.run_sim()
    
    def sim_callback(self, msg):
        self.sim_sub_count += 1

        if self.sim_sub_count >= 100 and not self.is_sim_ready:
            self.is_sim_ready = True
            self.get_logger().info('SIM ready, starting LIO')
            self.run_lio()

    def lio_callback(self, msg):
        if self.is_sim_ready:
            self.lio_sub_count += 1

            if self.lio_sub_count >= 100 and not self.is_lio_ready:
                self.is_lio_ready = True
                self.get_logger().info('LIO ready, starting GUI')
                self.run_gui()

    def run_sim(self):
        process = subprocess.Popen(['python3', '/root/ros2_ws/src/Isaac_ml/scripts/start_sim.py'], 
                                   stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE,
                                   universal_newlines=True)
        threading.Thread(target=self.read_process_output, args=(process,)).start()

    def run_lio(self):
        process = subprocess.Popen(['ros2', 'launch', 'lio_sam', 'run_loc.launch.py'], 
                                   stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE,
                                   universal_newlines=True)
        threading.Thread(target=self.read_process_output, args=(process,)).start()

    def run_gui(self):
        process = subprocess.Popen(['ros2', 'run', 'robotics_gui', 'gui'], 
                                   stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE,
                                   universal_newlines=True)
        threading.Thread(target=self.read_process_output, args=(process,)).start()

    def read_process_output(self, process):
        # 프로세스의 출력을 실시간으로 읽어오기, ROS logger로 출력
        for stdout_line in iter(process.stdout.readline, ""):
            # 출력 텍스트 색상 변경
            formatted_line = self.format_log_line(stdout_line.strip())
            self.get_logger().info(formatted_line)  # 로그 출력
        process.stdout.close()
        process.wait()

    def format_log_line(self, line):
        # 'WARN' 또는 'ERROR'가 들어간 로그에 색상 추가
        if 'WARN' in line:
            return f"\033[93m{line}\033[0m"  # 노란색
        elif 'ERROR' in line:
            return f"\033[91m{line}\033[0m"  # 빨간색
        else:
            return line  # 기본 색상

def main(args=None):
    rclpy.init(args=args)
    topic_listener = StateListener()
    rclpy.spin(topic_listener)
    topic_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
