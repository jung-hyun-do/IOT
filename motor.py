import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 9600


class RedCircleFollower(Node):
    def __init__(self):
        super().__init__('red_circle_follower')

        # 시리얼 연결
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(2)
            self.get_logger().info(" 아두이노 연결됨")
        except Exception as e:
            self.get_logger().error(f" 시리얼 연결 실패: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            Point,
            'red_circle_position',
            self.listener_callback,
            10)

        self.last_received_time = self.get_clock().now()
        self.last_cmd = None

        # 1초 간격으로 "값 없음" 체크
        self.timer = self.create_timer(1.0, self.check_timeout)

    def listener_callback(self, msg: Point):
        x = msg.x
        r = msg.z

        self.last_received_time = self.get_clock().now()

        if r >= 70:
            cmd = 'S'
        elif x <= 230:
            cmd = 'L'

        elif x <= 410:
            cmd = 'B'
        elif x <= 640:
            cmd = 'R'

        else:
            cmd = 'S'

        self.send_command(cmd)

    def check_timeout(self):
        # 1초 이상 값 수신이 없으면 정지
        if (self.get_clock().now() - self.last_received_time).nanoseconds > 1e9:
            self.send_command('S')

    def send_command(self, cmd):
        if cmd != self.last_cmd:
            try:
                self.arduino.write(f"{cmd}\n".encode())
                self.get_logger().info(f" 전송: {cmd}")
                self.last_cmd = cmd
            except Exception as e:
                self.get_logger().error(f" 아두이노 전송 실패: {e}")

    def destroy_node(self):
        self.send_command('S')
        if hasattr(self, 'arduino'):
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RedCircleFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
