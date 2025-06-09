import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

class RedCirclePublisher(Node):
    def __init__(self):
        super().__init__('red_circle_publisher')
        self.publisher_ = self.create_publisher(Point, 'red_circle_position', 10)
        self.timer = self.create_timer(0.1, self.detect_circle)

        self.cap = cv2.VideoCapture(0)  # 필요 시 장치 번호 조정

        if not self.cap.isOpened():
            self.get_logger().error(" 카메라를 열 수 없습니다.")
            rclpy.shutdown()

    def detect_circle(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 읽기 실패")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        red_only = cv2.bitwise_and(frame, frame, mask=mask)
        gray = cv2.cvtColor(red_only, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
            param1=100, param2=30, minRadius=10, maxRadius=100
        )

        if circles is not None:
            circles = np.uint16(np.around(circles[0]))
            largest = max(circles, key=lambda c: c[2])
            x, y, r = largest

            msg = Point()
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(r)
            self.publisher_.publish(msg)

            self.get_logger().info(f" 퍼블리시: x={x}, y={y}, r={r}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RedCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
