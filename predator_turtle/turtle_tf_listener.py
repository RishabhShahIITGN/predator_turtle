import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TurtleTFListener(Node):

    def __init__(self):
        super().__init__('turtle_tf_listener')

        # 1. Declare target and follower
        self.target_frame = 'turtle1'
        self.follower_frame = 'predator'

        # 2. Setup TF Listener (The "Ears")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Setup Publisher (The "Mouth" - to command the predator)
        self.publisher = self.create_publisher(Twist, f'/{self.follower_frame}/cmd_vel', 1)

        # 4. Create a timer loop (run 10 times per second)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Try to get the transform from Predator -> Turtle1
        from_frame_rel = self.target_frame
        to_frame_rel = self.follower_frame

        try:
            # "Where is [turtle1] relative to [predator]?"
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            # If we can't find the transform yet, just wait
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Calculate control command
        msg = Twist()
        
        # x and y here are the distance to turtle1 relative to the predator
        x = t.transform.translation.x
        y = t.transform.translation.y

        # MATH LOGIC:
        # Turn towards the target (atan2 gives the angle to the point (x,y))
        msg.angular.z = 1.0 * math.atan2(y, x)
        
        # Move forward based on distance (hypotenuse)
        distance = math.sqrt(x ** 2 + y ** 2)
        msg.linear.x = 0.5 * distance

        # Send command
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()