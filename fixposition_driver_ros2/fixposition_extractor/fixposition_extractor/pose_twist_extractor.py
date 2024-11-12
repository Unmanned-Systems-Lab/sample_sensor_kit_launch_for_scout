import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from fixposition_driver_ros2.msg import ODOMENU  # 引入消息类型
from geometry_msgs.msg import Pose, Twist

class PoseTwistExtractor(Node):

    def __init__(self):
        super().__init__('pose_twist_extractor')

        # 设置 QoS Profile，并使用与发布端一致的 ReliabilityPolicy
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # 或者改为 BEST_EFFORT

        self.pose_pub = self.create_publisher(Pose, 'pose', qos_profile)
        self.twist_pub = self.create_publisher(Twist, 'twist', qos_profile)

        self.subscription = self.create_subscription(
            ODOMENU,
            '/fixposition/fpa/odomenu',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        # 提取 Pose 和 Twist
        pose_msg = msg.pose.pose  # 从 PoseWithCovariance 中提取 Pose
        twist_msg = msg.velocity.twist  # 从 TwistWithCovariance 中提取 Twist

        # 发布 Pose 和 Twist
        self.pose_pub.publish(pose_msg)
        self.twist_pub.publish(twist_msg)

        self.get_logger().info("Published Pose and Twist")

def main(args=None):
    rclpy.init(args=args)
    node = PoseTwistExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
