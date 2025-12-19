import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose

class TurtleTFBroadcaster(Node):

    def __init__(self):
        super().__init__('turtle_tf_broadcaster')

        # 1. Declare a parameter 'turtlename'
        # This allows us to use this SAME code for 'turtle1' and 'predator'
        self.declare_parameter('turtlename', 'turtle1')
        self.turtlename = self.get_parameter('turtlename').get_parameter_value().string_value

        # 2. Create the Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. Subscribe to the turtle's pose
        # Topic will be: /turtle1/pose or /predator/pose
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.get_logger().info(f'Broadcaster started for: {self.turtlename}')

    def handle_turtle_pose(self, msg):
        # This function runs every time the turtle moves
        t = TransformStamped()

        # Header details
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'          # The parent frame
        t.child_frame_id = self.turtlename   # The child frame (the turtle)

        # Fill in the position (Translation)
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Fill in the orientation (Rotation)
        # We must convert the 2D "theta" angle to a 3D Quaternion
        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transform!
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w).
        """
        cx = math.cos(roll * 0.5)
        sx = math.sin(roll * 0.5)
        cy = math.cos(pitch * 0.5)
        sy = math.sin(pitch * 0.5)
        cz = math.cos(yaw * 0.5)
        sz = math.sin(yaw * 0.5)

        q = [0] * 4
        q[0] = sx * cy * cz - cx * sy * sz
        q[1] = cx * sy * cz + sx * cy * sz
        q[2] = cx * cy * sz - sx * sy * cz
        q[3] = cx * cy * cz + sx * sy * sz
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()