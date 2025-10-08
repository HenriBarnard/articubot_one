import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import EncoderVals
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math
import time


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # Parameters
        self.declare_parameter('encoder_cpr', 360)
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_base', 0.297)    # meters (distance between wheels)

        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        self.encoder_sub = self.create_subscription(
            EncoderVals,
            'encoder_vals',
            self.encoder_callback,
            10
        )

        # Internal state
        self.last_enc_left = None
        self.last_enc_right = None
        self.last_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.get_logger().info("Encoder Odometry node started")

    def encoder_callback(self, msg: EncoderVals):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        left_enc = msg.mot_1_enc_val
        right_enc = msg.mot_2_enc_val

        if self.last_enc_left is None:
            # First reading
            self.last_enc_left = left_enc
            self.last_enc_right = right_enc
            self.last_time = current_time
            return

        # Compute encoder deltas
        delta_left = left_enc - self.last_enc_left
        delta_right = right_enc - self.last_enc_right

        # Save current values
        self.last_enc_left = left_enc
        self.last_enc_right = right_enc
        self.last_time = current_time

        # Convert encoder ticks to wheel angular displacement
        delta_left_rad = (2.0 * math.pi * delta_left) / self.encoder_cpr
        delta_right_rad = (2.0 * math.pi * delta_right) / self.encoder_cpr

        # Linear displacement per wheel
        d_left = delta_left_rad * self.wheel_radius
        d_right = delta_right_rad * self.wheel_radius

        # Robot linear and angular displacement
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # Integrate pose
        self.x += d_center * math.cos(self.th + d_theta / 2.0)
        self.y += d_center * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

        # --- Publish Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Velocity estimates
        vx = d_center / dt
        vth = d_theta / dt
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # --- Broadcast TF ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
