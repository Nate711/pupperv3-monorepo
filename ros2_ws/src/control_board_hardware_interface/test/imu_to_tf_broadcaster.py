import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from tf2_ros import TransformStamped, TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import tf2_py as tf2

class ImuToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_to_tf_broadcaster')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self.listener_callback,
            10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        try:
            # Get static transform from imu to base_link
            t_imu_to_base_link = self.tf_buffer.lookup_transform('imu', 'base_link', rclpy.time.Time())
            p_imu_to_base_link = Pose()
            p_imu_to_base_link.position.x = t_imu_to_base_link.transform.translation.x
            p_imu_to_base_link.position.y = t_imu_to_base_link.transform.translation.y
            p_imu_to_base_link.position.z = t_imu_to_base_link.transform.translation.z
            p_imu_to_base_link.orientation = t_imu_to_base_link.transform.rotation
        except (tf2.LookupException, tf2.ExtrapolationException):
            self.get_logger().info('Transform not found')
            return

        t_map_to_imu = TransformStamped()
        t_map_to_imu.header.stamp = self.get_clock().now().to_msg()
        t_map_to_imu.header.frame_id = 'map'
        t_map_to_imu.child_frame_id = 'imu'
        t_map_to_imu.transform.translation.x = 0.0
        t_map_to_imu.transform.translation.y = 0.0
        t_map_to_imu.transform.translation.z = 0.0
        t_map_to_imu.transform.rotation = msg.orientation

        # Transform imu to base_link
        p_map_to_base_link = do_transform_pose(p_imu_to_base_link, t_map_to_imu)

        t_map_to_base_link = TransformStamped()
        t_map_to_base_link.header.stamp = self.get_clock().now().to_msg()
        t_map_to_base_link.header.frame_id = 'map'
        t_map_to_base_link.child_frame_id = 'base_link'
        t_map_to_base_link.transform.translation.x = p_map_to_base_link.position.x
        t_map_to_base_link.transform.translation.y = p_map_to_base_link.position.y
        t_map_to_base_link.transform.translation.z = p_map_to_base_link.position.z
        t_map_to_base_link.transform.rotation = p_map_to_base_link.orientation

        t_map_to_imu.child_frame_id = 'imu_in_map'
        self.tf_broadcaster.sendTransform(t_map_to_imu)
        self.tf_broadcaster.sendTransform(t_map_to_base_link)

def main(args=None):
    rclpy.init(args=args)
    imu_to_tf_broadcaster = ImuToTFBroadcaster()
    rclpy.spin(imu_to_tf_broadcaster)
    imu_to_tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# class ImuToTFBroadcaster(Node):
#     def __init__(self):
#         super().__init__('imu_to_tf_broadcaster')
#         self.subscription = self.create_subscription(
#             Imu,
#             '/imu_sensor_broadcaster/imu',
#             self.listener_callback,
#             10)
#         self.subscription  # Prevent unused variable warning
#         self.tf_broadcaster = TransformBroadcaster(self)

#     def listener_callback(self, msg):
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'map'
#         t.child_frame_id = 'imu'
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0
#         t.transform.rotation = msg.orientation
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     imu_to_tf_broadcaster = ImuToTFBroadcaster()
#     rclpy.spin(imu_to_tf_broadcaster)
#     imu_to_tf_broadcaster.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
