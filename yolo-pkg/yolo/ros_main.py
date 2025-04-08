import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray

from yolo.importClass import PreProcessor
from yolo.importClass import YoloDetection

class RosMain(Node):
    def __init__(self):
        super().__init__('ros_main')

        self.callback_group = ReentrantCallbackGroup()
        self.camera = PreProcessor()
        self.yolo = YoloDetection()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_color = self.create_subscription(
            Image, '/camera/d405/color/image_raw', 
            self.camera.process_color, qos_profile,
            callback_group=self.callback_group
        )

        self.sub_depth = self.create_subscription(
            Image, '/camera/d405/aligned_depth_to_color/image_raw', 
            self.camera.process_depth, qos_profile,
            callback_group=self.callback_group
        )

        self.pose_publisher = self.create_publisher(PoseArray, '/yolo/objects', 10)

        self.create_timer(0.1, self.get_objects, callback_group=self.callback_group)

    def get_objects(self):
        if self.camera.color_image is None or self.camera.depth_image is None:
            self.get_logger().info("No images received yet")
            return

        pose_array = self.yolo.detect(self.camera.color_image, self.camera.depth_image)
        
        if pose_array.poses:
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "camera_link"
            self.pose_publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    ros_node = RosMain()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
