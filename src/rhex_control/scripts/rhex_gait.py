#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

# === PARAMETERS ===
FREQUENCY = 100.0          # Hz
GAIT_PERIOD = 2.0          # Full cycle (A + B phases)
SWING_TIME = GAIT_PERIOD / 2
ROTATION_DISTANCE = 2 * math.pi  # Full leg step rotation (adjust as needed)

# Joint groups
TRIPOD_A = ['front_left_leg_joint', 'centre_right_leg_joint', 'back_left_leg_joint']
TRIPOD_B = ['front_right_leg_joint', 'centre_left_leg_joint', 'back_right_leg_joint']
ALL_JOINTS = TRIPOD_A + TRIPOD_B

class RHexSequentialTripodController(Node):
    def __init__(self):
        super().__init__('rhex_sequential_tripod_controller')

        self.publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.timer = self.create_timer(1.0 / FREQUENCY, self.update)

        self.start_time = self.get_clock().now()
        self.joint_order = ALL_JOINTS
        self.stance_angles = {joint: 0.0 for joint in ALL_JOINTS}  # hold angle for passive legs

        self.get_logger().info("RHex sequential tripod gait started.")

    def update(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        phase_time = t % GAIT_PERIOD

        moving_tripod = TRIPOD_A if phase_time < SWING_TIME else TRIPOD_B
        passive_tripod = TRIPOD_B if moving_tripod == TRIPOD_A else TRIPOD_A

        positions = []

        for joint in self.joint_order:
            if joint in moving_tripod:
                # Compute how far through swing we are (0 to 1)
                local_time = phase_time % SWING_TIME
                progress = local_time / SWING_TIME
                angle = self.stance_angles[joint] + ROTATION_DISTANCE * progress
                self.stance_angles[joint] = angle  # update to new contact angle
            else:
                # Hold position
                angle = self.stance_angles[joint]

            positions.append(angle)

        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RHexSequentialTripodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
