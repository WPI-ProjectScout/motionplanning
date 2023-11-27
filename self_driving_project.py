import carla
import random

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(60)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()
    bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))

    # init_pos = carla.Transform(carla.Location(x=158.0, y=24.0, z=0.05), carla.Rotation(yaw=-90))
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, spawn_point)
    print(spawn_point)

    destination_point = random.choice(world.get_map().get_spawn_points())
    print(destination_point)

if __name__ == '__main__':
    main()
