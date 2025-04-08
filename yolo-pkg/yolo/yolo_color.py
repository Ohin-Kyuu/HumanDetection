import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray

from ultralytics import YOLO
from yolo.importClass import PreProcessor
from yolo.importClass import convert_to_pose_array

class RosMain(Node):
    def __init__(self, model_path="yolov8n.pt"):
        super().__init__('ros_main')

        self.model = YOLO(model_path)
        self.callback_group = ReentrantCallbackGroup()

        self.camera = PreProcessor()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_color = self.create_subscription(
            Image, '/camera/image_raw', 
            self.camera.process_color, qos_profile,
            callback_group=self.callback_group
        )

        self.pose_publisher = self.create_publisher(PoseArray, '/yolo/objects', 10)
        self.results_publisher = self.create_publisher(Image, '/yolo/results', 10)

        self.create_timer(0.1, self.get_objects, callback_group=self.callback_group)

    def get_objects(self):
        if self.camera.color_image is None:
            self.get_logger().info("No images received yet")
            return

        pose_array = self.color_detect(self.camera.color_image)

        if pose_array.poses:
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "camera"
            self.pose_publisher.publish(pose_array)

    @convert_to_pose_array
    def color_detect(self, color_image):
        if color_image is None:
            return []

        results = self.model(color_image)
        detected_objects = []

        for object in results:
            results_img = object.plot()
            results_msg = self.camera.bridge.cv2_to_imgmsg(results_img, "bgr8")
            self.results_publisher.publish(results_msg)

            for box in object.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()

                if confidence >= 0.7 and object.names[0] == "pedestrian":
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    depth = 0.0
                    
                    detected_objects.append({"x": cx, "y": cy, "z": depth})
        return detected_objects
    
def main(args=None):
    rclpy.init(args=args)
    ros_node = RosMain()
    executor = MultiThreadedExecutor(num_threads=2)
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
