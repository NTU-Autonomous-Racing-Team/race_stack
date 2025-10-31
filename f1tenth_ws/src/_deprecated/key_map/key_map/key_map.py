import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class KeyMap(Node):

    def __init__(self):
        super().__init__('key_map')

        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)

        print("Mapping /cmd_vel to /teleop")


    def twist_callback(self, twist_msg):
        linear_velocity_x = twist_msg.linear.x
        angular_vel = twist_msg.angular.z

        drive_msg = AckermannDriveStamped()

        drive_msg.drive.speed = linear_velocity_x
        drive_msg.drive.steering_angle = max(min(angular_vel,0.3),-0.3)

        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)

    key_map = KeyMap()

    rclpy.spin(key_map)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_map.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
