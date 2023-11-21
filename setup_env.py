# import rclpy
# from rclpy.node import Node

import time
import random

# from std_msgs.msg import String
import carla
# from carla_msgs.msg import CarlaWorldInfo # pylint: disable=import-error


import scout.vehicle.displaymanager as dm
import scout.vehicle.sensormanager as sm
import pygame
from scout.navigation.global_route_planner import GlobalRoutePlanner

def main(args=None):
    vehicle_list=[]
    try:

        timer = sm.CustomTimer()

        client = carla.Client('localhost', 2000)
        client.set_timeout(60)
        world = client.get_world()

        world = client.load_world('Town02')
        world = client.load_world('Town01_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)
        original_settings = world.get_settings()
        world.unload_map_layer(carla.MapLayer.Buildings)
        world.unload_map_layer(carla.MapLayer.ParkedVehicles)
        world.unload_map_layer(carla.MapLayer.Foliage)

        # traffic_manager = client.get_trafficmanager(8000)
        # traffic_manager.set_synchronous_mode(True)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        print('Connected to Carla!')

        # Create ego vehicle out of the available blueprints of vehicles in this world
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))

        # Tell the world to spawn the vehicle and add to list
        #init_pos = carla.Transform(carla.Location(x=21.4, y=-7.62, z=0.05), carla.Rotation(yaw=180))
        init_pos = carla.Transform(carla.Location(x=158.0, y=24.0, z=0.05), carla.Rotation(yaw=-90))
        vehicle = world.spawn_actor(bp, init_pos)
        vehicle_list.append(vehicle)

        width, height=800,600
        # Load display
        display_manager = None
        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = dm.DisplayManager(grid_size=[2, 3], window_size=[width, height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position,
        sm.SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)),
                        vehicle, {}, display_pos=[0, 0])
        sm.SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                        vehicle, {}, display_pos=[0, 1])
        sm.SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)),
                        vehicle, {}, display_pos=[0, 2])
        sm.SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)),
                        vehicle, {}, display_pos=[1, 1])

        sm.SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                        vehicle, {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[1, 0])
        sm.SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                        vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])

        # global_route_plannner=GlobalRoutePlanner(world.get_map(),1000)

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        sync=True
        clock = pygame.time.Clock()
        ackermann_control = carla.VehicleAckermannControl()

        counter=0
        while True:
            counter+=1
            # Carla Tick
            if sync:
                world.tick()
            else:
                world.wait_for_tick()
            clock.tick_busy_loop(60)

            ackermann_control.speed=0.5
            if counter==100:
                ackermann_control.speed=ackermann_control.speed*-1
                counter=0
            vehicle.apply_ackermann_control(ackermann_control)
            # Render received data
            display_manager.render()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            if call_exit:
                break


    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)



if __name__ == '__main__':
    main()
