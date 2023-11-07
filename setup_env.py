import rclpy
from rclpy.node import Node

import time
import random

from std_msgs.msg import String
import carla
from carla_msgs.msg import CarlaWorldInfo # pylint: disable=import-error

class SetupEnv(Node):

    def __init__(self):
        super().__init__('setup_env')
        self.publisher_ = self.create_publisher(String, 'topic', 10)


        self.get_logger().info('Connecting to Carla')
        



        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        self.world = client.get_world()
        #self.world = client.load_world('Town02')
        self.world = client.load_world('Town01_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)
        self.world.unload_map_layer(carla.MapLayer.Buildings)
        self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        self.world.unload_map_layer(carla.MapLayer.Foliage)


        self.get_logger().info('Connected to Carla!')


        self.actor_list = []
        blueprint_library = self.world.get_blueprint_library()

        #create ego vehicle
        bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))
        #init_pos = carla.Transform(carla.Location(x=21.4, y=-7.62, z=0.05), carla.Rotation(yaw=180))
        init_pos = carla.Transform(carla.Location(x=158.0, y=24.0, z=0.05), carla.Rotation(yaw=-90))
        self.vehicle = self.world.spawn_actor(bp, init_pos)

        


        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.actor_list.append(self.vehicle)

        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Starting timer')
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        location = self.vehicle.get_location()
        self.get_logger().info(str(location))
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SetupEnv()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()