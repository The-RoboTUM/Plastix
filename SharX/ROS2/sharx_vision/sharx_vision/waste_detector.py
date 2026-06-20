import json
import os

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


class WasteDetector(Node):
    def __init__(self):
        super().__init__('waste_detector')

        default_model = os.path.expanduser(
            '~/sharx_yolo_project/'
            'runs/detect/runs/floating_waste_v1/weights/best.pt'
        )

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_size', 512)
        self.declare_parameter('device', '0')
        self.declare_parameter('show_image', True)

        self.model_path = (
            self.get_parameter('model_path')
            .get_parameter_value()
            .string_value
        )
        self.camera_index = (
            self.get_parameter('camera_index')
            .get_parameter_value()
            .integer_value
        )
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold')
            .get_parameter_value()
            .double_value
        )
        self.image_size = (
            self.get_parameter('image_size')
            .get_parameter_value()
            .integer_value
        )
        self.device = (
            self.get_parameter('device')
            .get_parameter_value()
            .string_value
        )
        self.show_image = (
            self.get_parameter('show_image')
            .get_parameter_value()
            .bool_value
        )

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f'Model not found: {self.model_path}'
            )

        self.publisher = self.create_publisher(
            String,
            '/sharx/waste_detection',
            10
        )

        self.model = YOLO(self.model_path)

        self.camera = cv2.VideoCapture(self.camera_index)

        if not self.camera.isOpened():
            raise RuntimeError(
                f'Could not open camera index {self.camera_index}'
            )

        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info('SharX waste detector started')
        self.get_logger().info(f'Model: {self.model_path}')
        self.get_logger().info(
            f'Camera index: {self.camera_index}'
        )
        self.get_logger().info(
            'Publishing: /sharx/waste_detection'
        )

    def process_frame(self):
        success, frame = self.camera.read()

        if not success:
            self.get_logger().warning('Failed to read webcam frame')
            return

        results = self.model.predict(
            source=frame,
            imgsz=self.image_size,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False
        )

        result = results[0]
        height, width = frame.shape[:2]

        message_data = {
            'detected': False,
            'class_name': 'floating_waste',
            'confidence': 0.0,
            'center_x': 0,
            'center_y': 0,
            'box_width': 0,
            'box_height': 0,
            'image_width': width,
            'image_height': height
        }

        if result.boxes is not None and len(result.boxes) > 0:
            confidences = result.boxes.conf
            best_index = int(confidences.argmax().item())

            confidence = float(
                result.boxes.conf[best_index].item()
            )

            class_id = int(
                result.boxes.cls[best_index].item()
            )

            x1, y1, x2, y2 = (
                result.boxes.xyxy[best_index]
                .cpu()
                .tolist()
            )

            center_x = int((x1 + x2) / 2.0)
            center_y = int((y1 + y2) / 2.0)
            box_width = int(x2 - x1)
            box_height = int(y2 - y1)

            class_name = result.names.get(
                class_id,
                'floating_waste'
            )

            message_data = {
                'detected': True,
                'class_name': class_name,
                'confidence': round(confidence, 3),
                'center_x': center_x,
                'center_y': center_y,
                'box_width': box_width,
                'box_height': box_height,
                'image_width': width,
                'image_height': height
            }

        ros_message = String()
        ros_message.data = json.dumps(message_data)
        self.publisher.publish(ros_message)

        if self.show_image:
            annotated_frame = result.plot()

            cv2.imshow(
                'SharX Floating Waste Detection',
                annotated_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = WasteDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'Waste detector error: {error}')
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
