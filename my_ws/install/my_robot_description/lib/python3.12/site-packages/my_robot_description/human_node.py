import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import threading


class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            '/human_bbox',
            10
        )

        self.bridge = CvBridge()
        self.model = YOLO("/home/nga/yolov8n.pt")

        self.latest_frame = None
        self.lock = threading.Lock()

        # Timer riêng để hiển thị ảnh — tránh block callback
        self.timer = self.create_timer(0.033, self.display_frame)

    def image_callback(self, msg):
        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (640, 480))

        # YOLO tracking
        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=0.5,
            imgsz=320,
            verbose=False
        )

        for r in results:
            boxes = r.boxes
            if boxes.id is None:
                continue

            ids  = boxes.id.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()
            cls  = boxes.cls.cpu().numpy()

            for i in range(len(ids)):
                if int(cls[i]) != 0:  # chỉ lấy người (class 0)
                    continue

                x1, y1, x2, y2 = map(int, xyxy[i])
                track_id = int(ids[i])

                # Publish bbox
                msg_out = Int32MultiArray()
                msg_out.data = [track_id, x1, y1, x2, y2]
                self.publisher_.publish(msg_out)

                # Vẽ bbox + ID lên frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"ID {track_id}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2
                )

        # Lưu frame mới nhất để hiển thị
        with self.lock:
            self.latest_frame = frame

    def display_frame(self):
        with self.lock:
            frame = self.latest_frame

        if frame is not None:
            cv2.imshow("Human Tracking", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()