#!/usr/bin/python3
import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import numpy as np

vel_msg = Twist()  # 로봇의 속도 메시지
mode_selection = 0  # 1: 반대 위상, 2: 동위상, 3: 회전, 4: 정지 없음

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        # self.wheel_seperation = 0.271
        # self.wheel_base = 0.6089794635772705
        self.wheel_seperation = 0.5584
        self.wheel_base = 0.625 * 2
        self.wheel_radius = 0.1
        self.wheel_steering_y_offset = 0.00
        self.steering_track = self.wheel_seperation - 2 * self.wheel_steering_y_offset
        self.pi = 3.14159265359
        self.max_speed = 4.0

        # 조인트 이름 정의
        self.joint_names = [
            'l_front_wheel_rotate_joint', 'r_front_wheel_rotate_joint',
            'l_rear_wheel_rotate_joint', 'r_rear_wheel_rotate_joint',
            'l_front_wheel_joint', 'r_front_wheel_joint',
            'l_rear_wheel_joint', 'r_rear_wheel_joint',
            'roll_balance_joint', 'r_bogie_joint', 'l_bogie_joint',
            'l_roll_static_joint', 'r_roll_static_joint',
            'arm1_joint', 'arm2_joint', 'arm3_joint', 'arm4_joint', 'arm5_joint', 'arm6_joint'
        ]

        # 속도와 위치 배열 초기화
        self.steering_position = [0.0] * 4  # 조향 관절의 위치
        self.wheel_velocity = [0.0] * 4  # 바퀴 관절의 속도
        self.balance_position = [0.0] * 5  # 밸런스 관절의 위치
        self.current_steering = [0.0] * 4  # 현재 조향 각도
        self.arm_position = [0.0] * 6  # 현재 팔 각도
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 하나의 퍼블리셔로 통합된 JointState 메시지 발행
        self.publisher_joint_state = self.create_publisher(JointState, 'isaac_joint_commands', 10)

        self.sub = self.create_subscription(Twist, '/kimm_4ws/cmd_vel', self.cmd_callback, 10)
        self.subscription = self.create_subscription(Clock, 'clock', self.clock_callback, 10)
        self.subscription = self.create_subscription(
            JointState,
            'isaac_joint_states',
            self.joint_position,
            10
        )

        self.STOP = 0
        self.OPPOSITE_PHASE = 1
        self.IN_PHASE = 2
        self.PIVOT_TURN = 3
        self.steering_flag = False
        self.latest_clock_msg = None  # Clock 메시지 수신 확인용 플래그

    def joint_position(self, msg):
        self.current_steering = msg.position[:4]  # 조향 각도 업데이트

    def clock_callback(self, msg):
        # Clock 메시지를 수신하여 저장
        self.latest_clock_msg = msg

    def publish_joint_state(self):
        # 하나의 JointState 메시지로 position과 velocity를 모두 발행
        if self.latest_clock_msg is not None:
            msg = JointState()
            msg.header.stamp = self.latest_clock_msg.clock  # Clock 메시지로부터 타임스탬프 설정

            # 조인트 이름 설정
            msg.name = self.joint_names

            # position 필드는 조향 관절에 대해 값 설정, 나머지 바퀴는 0.0
            msg.position = self.steering_position + [float('nan')] * 4 + self.balance_position + self.arm_position  # 바퀴의 위치는 NaN으로 설정
            msg.position = [-pos for pos in msg.position]
            
            # velocity 필드는 바퀴에 대해 값 설정, 나머지 조향 관절은 NaN
            msg.velocity = [float('nan')] * 4 + self.wheel_velocity + [float('nan')] * 5 + [float('nan')] * 6 # 조향 관절은 NaN으로 설정

            # 오른쪽 바퀴 방향 수정
            # msg.velocity[5] = -msg.velocity[5]
            # msg.velocity[7] = -msg.velocity[7]

            self.publisher_joint_state.publish(msg)

    def cmd_callback(self, msg):
        vel_msg.linear.x = msg.linear.x * self.max_speed 
        vel_msg.linear.y = msg.linear.y * self.max_speed 
        vel_msg.angular.z = msg.angular.z * self.max_speed 

    def timer_callback(self):
        # 조향 각도와 바퀴 속도 설정
        if vel_msg.linear.x == 0.0 and vel_msg.linear.y == 0.0 and vel_msg.angular.z == 0.0:
            mode_selection = self.STOP
        elif vel_msg.linear.x != 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0:
            mode_selection = self.OPPOSITE_PHASE
        elif vel_msg.linear.x == 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0:
            mode_selection = self.PIVOT_TURN
        else:
            mode_selection = self.IN_PHASE

        # 반대 위상 모드
        if mode_selection == self.OPPOSITE_PHASE:
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            # 바퀴 속도 계산
            self.wheel_velocity[0] = sign * math.hypot(vel_msg.linear.x - vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) - vel_steerring_offset
            self.wheel_velocity[1] = sign * math.hypot(vel_msg.linear.x + vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) + vel_steerring_offset
            self.wheel_velocity[2] = sign * math.hypot(vel_msg.linear.x - vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) - vel_steerring_offset
            self.wheel_velocity[3] = sign * math.hypot(vel_msg.linear.x + vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) + vel_steerring_offset

            # 조향 각도 계산 (목표 각도를 설정)
            self.steering_position[0] = -math.atan(self.wheel_base / (2 * vel_msg.linear.x + vel_msg.angular.z * self.steering_track))
            self.steering_position[1] = -math.atan(self.wheel_base / (2 * vel_msg.linear.x - vel_msg.angular.z * self.steering_track))
            self.steering_position[2] = math.atan(self.wheel_base / (2 * vel_msg.linear.x + vel_msg.angular.z * self.steering_track))
            self.steering_position[3] = math.atan(self.wheel_base / (2 * vel_msg.linear.x - vel_msg.angular.z * self.steering_track))

        # 동위상 모드
        elif mode_selection == self.IN_PHASE:
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)

            if vel_msg.linear.x != 0.0:
                ang = -vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0.0

            # 목표 조향 각도 설정
            self.steering_position[0] = math.atan(ang)
            self.steering_position[1] = math.atan(ang)
            self.steering_position[2] = math.atan(ang)
            self.steering_position[3] = math.atan(ang)

            if vel_msg.linear.x == 0.0:
                if vel_msg.linear.y > 0.0:
                    self.steering_position = [-self.pi / 2.0] * 4
                else:
                    self.steering_position = [self.pi / 2.0] * 4
                self.wheel_velocity = [self.max_speed] * 4
            else:
                self.wheel_velocity = [sign * V] * 4

        # 회전 모드
        elif mode_selection == self.PIVOT_TURN:
            ang = (self.pi / 2) - math.atan(self.steering_track / self.wheel_base)
            self.steering_position[0] = ang
            self.steering_position[1] = -ang
            self.steering_position[2] = -ang
            self.steering_position[3] = ang

            self.wheel_velocity[0] = -vel_msg.angular.z
            self.wheel_velocity[1] = vel_msg.angular.z
            self.wheel_velocity[2] = self.wheel_velocity[0]
            self.wheel_velocity[3] = self.wheel_velocity[1]

        # 정지 모드
        elif mode_selection == self.STOP:
            self.wheel_velocity[:] = [0.0] * 4
            self.steering_position[:] = [0.0] * 4

        # 하나의 JointState로 position과 velocity 함께 발행
        self.publish_joint_state()

def main(args=None):
    rclpy.init(args=args)

    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
