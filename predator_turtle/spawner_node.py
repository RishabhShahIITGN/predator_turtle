import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn # Import the service type to spawn turtles

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        # Create a Client that talks to the /spawn service
        self.client = self.create_client(Spawn, '/spawn')

        # Wait until the service is available (sim is running)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Turtlesim to start...')
        
        # Create the request data
        self.request = Spawn.Request()
        self.request.x = 2.0
        self.request.y = 2.0
        self.request.theta = 0.0
        self.request.name = 'predator' # The name of our second turtle

    def send_request(self):
        # Send the request asynchronously
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Predator Turtle Spawned!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    node.send_request() # Run the spawn logic
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
