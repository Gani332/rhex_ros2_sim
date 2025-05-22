#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class RHexEffortStepper(Node):
    def __init__(self):
        super().__init__('rhex_effort_stepper')

        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)

        # Define tripod groups
        self.tripod_A = [0, 2, 4]  # FL, BL, CR
        self.tripod_B = [1, 3, 5]  # CL, FR, BR

        self.phase = 0      # 0 = A active, 1 = B active
        self.subphase = 0   # toggle every timer, switch phase every 2 toggles

        self.step_duration = 0.6  # seconds
        self.amplitude_linear = 2.0   # forward torque
        self.frequency = 0.2          # gait frequency

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.step_duration, self.timer_callback)

    def timer_callback(self):
        # Time tracking for future gait improvements (not needed now)
        # t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        # Straight walking: no turning
        base_effort = self.amplitude_linear
        efforts = [0.0] * 6

        active_tripod = self.tripod_A if self.phase == 0 else self.tripod_B
        for i in active_tripod:
            efforts[i] = base_effort

        # Subphase logic: toggle tripod every 2 calls
        self.subphase += 1
        if self.subphase % 2 == 0:
            self.phase = 1 - self.phase

        # Publish
        msg = Float64MultiArray()
        msg.data = efforts
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexEffortStepper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
