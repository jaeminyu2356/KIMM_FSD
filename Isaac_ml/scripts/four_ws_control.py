#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

vel_msg = Twist()  # robot velocity
mode_selection = 0 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.271
        self.wheel_base = 0.6089794635772705
        self.wheel_radius = 0.1
        self.wheel_steering_y_offset = 0.00
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset
        self.pi = 3.14159265359
        self.max_speed = 3

        # Define the joint names as provided
        self.joint_names = [
            'fl_steering_wheel_joint', 'fr_steering_wheel_joint',
            'rl_steering_wheel_joint', 'rr_steering_wheel_joint',
            'fl_wheel_joint', 'fr_wheel_joint',
            'rl_wheel_joint', 'rr_wheel_joint'
        ]

        self.velocity = [math.nan] * len(self.joint_names)  # All velocities are 0.0
        self.current_steering = [0 for x in range(4)]

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher_ = self.create_publisher(JointState, 'isaac_joint_commands', 10)
        
        self.sub = self.create_subscription(Twist, '/kimm_4ws/cmd_vel', self.cmd_callback, qos_profile=10)
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
        self.Kp = 0.5
        self.steering_flag = False
        # 초기화된 Clock 메시지 수신 확인용 플래그
        self.latest_clock_msg = None

    def joint_position(self, msg):
        self.current_steering = msg.position[:4]

    def clock_callback(self, msg):
        # Clock 메시지를 수신하면, 해당 메시지를 저장합니다.
        self.latest_clock_msg = msg

    def p_controller(self, target_velocity, current_velocity):

        # 오차 계산
        error = target_velocity - current_velocity
        
        # 제어 입력 계산 (P 제어)
        control_input = self.Kp * error
        
        return control_input

    def publish_joint_state(self, velocity):
        # Clock 메시지가 수신되었을 때 JointState 메시지를 발행합니다.
        if self.latest_clock_msg is not None:
            msg = JointState()

            # Set header using the latest clock message
            msg.header.stamp = self.latest_clock_msg.clock

            if self.steering_flag:
                velocity[4:] = [0] * (len(self.joint_names)-4)
            # Set the joint names and corresponding values
            msg.name = self.joint_names
            msg.position = [math.nan] * len(self.joint_names)  # Initialize all positions with NaN
            msg.velocity = [float(v) for v in velocity]

            
            ### temp
            msg.velocity[5] = -msg.velocity[5]
            msg.velocity[7] = -msg.velocity[7]
            # Publish the JointState message
            self.publisher_.publish(msg)

    def cmd_callback(self, msg):
        vel_msg.linear.x = msg.linear.x * self.max_speed 
        vel_msg.linear.y = msg.linear.y * self.max_speed 
        vel_msg.angular.z = msg.angular.z * self.max_speed 
        
    def timer_callback(self):
        if(vel_msg.linear.x == 0.0 and vel_msg.linear.y == 0.0 and vel_msg.angular.z == 0.0):
            mode_selection = self.STOP
        elif(vel_msg.linear.x != 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0):
            mode_selection = self.OPPOSITE_PHASE
        elif(vel_msg.linear.x == 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0):
            mode_selection = self.PIVOT_TURN
        else:
            mode_selection = self.IN_PHASE
        
        # opposite phase
        if(mode_selection == self.OPPOSITE_PHASE):
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            self.velocity[4] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.velocity[5] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
            self.velocity[6] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.velocity[7] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset

            self.velocity[0] = self.p_controller(-math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track)), self.current_steering[0])
            self.velocity[1] = self.p_controller(-math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track)), self.current_steering[1])
            self.velocity[2] = self.p_controller(math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track)), self.current_steering[2])
            self.velocity[3] = self.p_controller(math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track)), self.current_steering[3])

        # in-phase
        elif(mode_selection == self.IN_PHASE):
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
                
            if(vel_msg.linear.x != 0.0):
                ang = -vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.velocity[0] = self.p_controller(math.atan(ang),self.current_steering[0])
            self.velocity[1] = self.p_controller(math.atan(ang), self.current_steering[1])
            self.velocity[2] = self.p_controller(math.atan(ang),self.current_steering[2])
            self.velocity[3] = self.p_controller(math.atan(ang), self.current_steering[3])
            
            if(vel_msg.linear.x == 0.0):
                if(vel_msg.linear.y > 0.0):
                    self.velocity[0] = self.p_controller(-self.pi / 2.0,self.current_steering[0])
                    self.velocity[1] = self.p_controller(-self.pi / 2.0,self.current_steering[1])
                    self.velocity[2] = self.p_controller(-self.pi / 2.0,self.current_steering[2])
                    self.velocity[3] = self.p_controller(-self.pi / 2.0,self.current_steering[3])
                    
                    
                else:
                    self.velocity[0] = self.p_controller(self.pi / 2.0,self.current_steering[0])
                    self.velocity[1] = self.p_controller(self.pi / 2.0,self.current_steering[1])
                    self.velocity[2] = self.p_controller(self.pi / 2.0,self.current_steering[2])
                    self.velocity[3] = self.p_controller(self.pi / 2.0,self.current_steering[3])
                    
                self.velocity[4:] = [self.max_speed] * (len(self.velocity)-4)
            else:
                self.velocity[4:] = [sign*V] * (len(self.velocity)-4)
            
        # pivot turn
        elif(mode_selection == self.PIVOT_TURN):
            ang = (self.pi/2)-math.atan(self.steering_track/self.wheel_base)
            self.velocity[0] = self.p_controller(ang,self.current_steering[0])
            self.velocity[1] = self.p_controller(-ang,self.current_steering[1])
            self.velocity[2] = self.p_controller(-ang,self.current_steering[2])
            self.velocity[3] = self.p_controller(ang,self.current_steering[3])
            self.velocity[4] = -vel_msg.angular.z 
            self.velocity[5] = vel_msg.angular.z 
            self.velocity[6] = self.velocity[4]
            self.velocity[7] = self.velocity[5]

        elif(mode_selection == self.STOP):
            self.velocity[:] = [0] * len(self.velocity)

        self.publish_joint_state(self.velocity)

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