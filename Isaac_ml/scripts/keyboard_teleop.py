#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import curses
import math
import time

def ackermann_steering(steering_angle, wheel_base, track_width):
    """
    단순 Ackermann 조향각 계산 예시:
      steering_angle: 중앙(평균) 조향 입력 (rad)
      wheel_base: 앞뒤 바퀴 축간 거리 (m)
      track_width: 좌우 바퀴 간 거리 (m)
    반환: (left_angle, right_angle)
      - 왼/오른쪽 앞바퀴 조향각(단위: rad)
    """
    if abs(steering_angle) < 1e-6:
        # 각도가 거의 0이면 직진
        return (0.0, 0.0)

    # Ackermann: R = L / tan(θ)
    R = wheel_base / math.tan(steering_angle)

    # 실제 왼/오른쪽 앞바퀴
    left = math.atan(wheel_base / (R - (track_width / 2.0)))
    right = math.atan(wheel_base / (R + (track_width / 2.0)))
    return (left, right)

class AckermannKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_keyboard_teleop')
        # 퍼블리시할 토픽 이름 (시뮬레이터와 맞춰야 함)
        self.pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        # 제어 변수
        self.velocity = 0.0       # 바퀴 회전 속도 (rad/s)
        self.steering_angle = 0.0 # 중앙(평균) 조향각 (rad)

        # Ackermann 차량 치수 (예시 값)
        self.wheel_base = 1.0     # 앞/뒤 바퀴 축간 거리 (m)
        self.track_width = 0.8    # 좌/우 바퀴 간 거리 (m)

        # 주기적으로 메시지를 퍼블리시
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # Ackermann 계산: 앞바퀴 2개 조향각
        left_steer, right_steer = ackermann_steering(
            self.steering_angle, 
            self.wheel_base, 
            self.track_width
        )

        # JointState 메시지 구성
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # 8개 조인트 순서:
        #   1. l_front_wheel_rotate_joint  -> 앞바퀴 왼쪽 조향(steering)
        #   2. l_rear_wheel_rotate_joint   -> (뒷바퀴 왼쪽 조향, 여기서는 0 rad 고정)
        #   3. r_front_wheel_rotate_joint  -> 앞바퀴 오른쪽 조향
        #   4. r_rear_wheel_rotate_joint   -> (뒷바퀴 오른쪽 조향, 여기서는 0 rad 고정)
        #   5. l_front_wheel_joint         -> 앞바퀴 왼쪽 회전(속도)
        #   6. l_rear_wheel_joint          -> 뒷바퀴 왼쪽 회전(속도)
        #   7. r_front_wheel_joint         -> 앞바퀴 오른쪽 회전(속도)
        #   8. r_rear_wheel_joint          -> 뒷바퀴 오른쪽 회전(속도)

        msg.name = [
            'l_front_wheel_rotate_joint',
            'l_rear_wheel_rotate_joint',
            'r_front_wheel_rotate_joint',
            'r_rear_wheel_rotate_joint',
            'l_front_wheel_joint',
            'l_rear_wheel_joint',
            'r_front_wheel_joint',
            'r_rear_wheel_joint'
        ]

        # 앞바퀴 조향(rotate) = left_steer, right_steer
        # 뒷바퀴 rotate는 0 rad(조향 없음)
        # position 배열은 name 순서대로
        position_list = [
            left_steer,  # l_front_wheel_rotate
            0.0,         # l_rear_wheel_rotate
            right_steer, # r_front_wheel_rotate
            0.0,         # r_rear_wheel_rotate
            0.0, 0.0, 0.0, 0.0  # 바퀴 회전 각도를 position으로 직접 제어하지 않음(회전은 velocity로)
        ]

        # 회전(구동) 속도 = velocity
        # 앞바퀴/뒷바퀴 모두 주행(4WD) 가정
        # rotate_joint(조향)는 velocity=0, wheel_joint(회전)는 self.velocity
        velocity_list = [
            0.0,  # l_front_wheel_rotate_joint
            0.0,  # l_rear_wheel_rotate_joint
            0.0,  # r_front_wheel_rotate_joint
            0.0,  # r_rear_wheel_rotate_joint
            self.velocity,  # l_front_wheel_joint
            self.velocity,  # l_rear_wheel_joint
            self.velocity,  # r_front_wheel_joint
            self.velocity   # r_rear_wheel_joint
        ]

        msg.position = position_list
        msg.velocity = velocity_list
        msg.effort   = []  # 필요시 사용

        # 퍼블리시
        self.pub.publish(msg)

def main(stdscr):
    # curses 초기화
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()

    rclpy.init()
    teleop_node = AckermannKeyboardTeleop()

    stdscr.addstr(0, 0, "Ackermann Keyboard Teleop\n")
    stdscr.addstr(2, 0, "Controls:\n")
    stdscr.addstr(3, 2, "↑ : Increase velocity\n")
    stdscr.addstr(4, 2, "↓ : Decrease velocity\n")
    stdscr.addstr(5, 2, "← : Steer left\n")
    stdscr.addstr(6, 2, "→ : Steer right\n")
    stdscr.addstr(7, 2, "[Space]: Zero velocity & steering\n")
    stdscr.addstr(8, 2, "[q]: Quit\n")

    try:
        while True:
            c = stdscr.getch()

            if c != curses.ERR:  # 키가 입력됨
                if c in (ord('q'), ord('Q')):
                    break
                elif c == curses.KEY_UP:
                    teleop_node.velocity += 0.2
                elif c == curses.KEY_DOWN:
                    teleop_node.velocity -= 0.2
                elif c == curses.KEY_LEFT:
                    teleop_node.steering_angle += 0.02
                elif c == curses.KEY_RIGHT:
                    teleop_node.steering_angle -= 0.02
                elif c == ord(' '):  # 스페이스: 긴급정지
                    teleop_node.velocity = 0.0
                    teleop_node.steering_angle = 0.0

                # 화면 갱신
                stdscr.clear()
                stdscr.addstr(0, 0, "Ackermann Keyboard Teleop\n")
                stdscr.addstr(2, 0, "Controls:\n")
                stdscr.addstr(3, 2, "↑ : Increase velocity\n")
                stdscr.addstr(4, 2, "↓ : Decrease velocity\n")
                stdscr.addstr(5, 2, "← : Steer left\n")
                stdscr.addstr(6, 2, "→ : Steer right\n")
                stdscr.addstr(7, 2, "[Space]: Zero velocity & steering\n")
                stdscr.addstr(8, 2, "[q]: Quit\n")

                stdscr.addstr(10, 0, f"Current velocity: {teleop_node.velocity:.2f} rad/s")
                stdscr.addstr(11, 0, f"Current steering angle: {teleop_node.steering_angle:.3f} rad")

            # ROS spin (non-blocking)
            rclpy.spin_once(teleop_node, timeout_sec=0.0)
            time.sleep(0.02)  # 50Hz 루프

    finally:
        rclpy.shutdown()
        stdscr.nodelay(False)
        curses.endwin()

if __name__ == '__main__':
    curses.wrapper(main)
