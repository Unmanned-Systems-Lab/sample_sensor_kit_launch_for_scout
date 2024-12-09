import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from fixposition_driver_ros2.msg import ODOMENU  # 引入 ODOMENU 消息类型
from nav_msgs.msg import Odometry  # 引入 Odometry 消息类型
from geometry_msgs.msg import Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # 设置 QoS Profile，并使用与发布端一致的 ReliabilityPolicy
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE

        # 设置 QoS Profile，并使用与发布端一致的 ReliabilityPolicy
        qos_profile_sub = QoSProfile(depth=1)
        qos_profile_sub.reliability = ReliabilityPolicy.BEST_EFFORT

        # 创建 Pose, Twist 和 Odometry 发布者
        self.pose_pub = self.create_publisher(Pose, 'pose', qos_profile)
        self.twist_pub = self.create_publisher(Twist, 'twist', qos_profile)
        self.odometry_pub = self.create_publisher(Odometry, 'localization/kinematic_state', qos_profile)

        # 订阅 ODOMENU 消息
        self.subscription_odomenu = self.create_subscription(
            ODOMENU,
            '/fixposition/fpa/odomenu',
            self.listener_callback,
            qos_profile_sub
        )

        # 初始化 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        # 提取 Pose 和 Twist 信息
        pose_msg = msg.pose.pose  # 从 PoseWithCovariance 中提取 Pose
        twist_msg = msg.velocity.twist  # 从 TwistWithCovariance 中提取 Twist

        # 发布 Pose 和 Twist
        self.pose_pub.publish(pose_msg)
        self.twist_pub.publish(twist_msg)

        # 创建新的 Odometry 消息
        odometry_msg = Odometry()
        odometry_msg.header = msg.header
        odometry_msg.header.frame_id = "map"
        odometry_msg.child_frame_id = 'base_link'

        # 将 pose 和 twist 都设置为 0
        odometry_msg.pose.pose.position.x = 0.656501
        odometry_msg.pose.pose.position.y = 5.263
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.x = 0.0
        odometry_msg.pose.pose.orientation.y = 0.0
        odometry_msg.pose.pose.orientation.z = 0.7071
        odometry_msg.pose.pose.orientation.w = 0.7071  # 单位四元数表示无旋转

        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = 0.0

        # 发布组合后的 Odometry 消息
        self.odometry_pub.publish(odometry_msg)

        # 发布 TF (header.frame_id -> base_link)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'

        # 设置位置和方向为 0
        transform.transform.translation.x = 0.3064987063407898
        transform.transform.translation.y = 3.9130029678344727
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.6771090896887927
        transform.transform.rotation.w = 0.7358826541377467  # 单位四元数表示无旋转

        # 发布 transform
        self.tf_broadcaster.sendTransform(transform)

        # 输出日志信息
        #self.get_logger().info("Published zeroed Odometry and TF from map to base_link")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
