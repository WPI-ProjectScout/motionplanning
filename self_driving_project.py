import carla
import random
import time
import pygame
import live_plotter as lv
from scout.navigation.global_route_planner import GlobalRoutePlanner
import scout.vehicle.displaymanager as dm
import scout.vehicle.sensormanager as sm

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(60)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    world = client.get_world()
    map = world.get_map()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    blueprint_library = world.get_blueprint_library()
    bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))

    # init_pos = carla.Transform(carla.Location(x=158.0, y=24.0, z=0.05), carla.Rotation(yaw=-90))
    spawn_point = random.choice(world.get_map().get_spawn_points())
    # spawn_point = carla.Transform(carla.Location(x=26.382587, y=-57.401386, z=0.6), carla.Rotation(yaw=-0.023438))
    # This is the player
    # vehicle = world.try_spawn_actor(bp, spawn_point)
    vehicle = world.spawn_actor(bp, spawn_point)

    destination_point = random.choice(world.get_map().get_spawn_points())
    # destination_point = carla.Transform(carla.Location(x=-45.149696, y=55.715389, z=0.600000), carla.Rotation(yaw=-90.161217))

    sampling_resolution = 2.0
    global_route_plannner = GlobalRoutePlanner(map, sampling_resolution)

    start_waypoint = map.get_waypoint(spawn_point.location)
    end_waypoint = map.get_waypoint(destination_point.location)

    start_location = start_waypoint.transform.location
    print("Start location: ", start_location)
    end_location = end_waypoint.transform.location
    print("End location: ", end_location)
    # Returns list of (carla.Waypoint, RoadOption)
    # The carla.Waypoint class contains a carla.Transform object
    # The transform object contains a location and a rotation
    route_trace = global_route_plannner.trace_route(start_location, end_location)
    print(len(route_trace))
    x_points = []
    y_points = []
    for i in range(len(route_trace)):
        x_points.append(route_trace[i][0].transform.location.x)
        y_points.append(route_trace[i][0].transform.location.y)

    lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")

    FIGSIZE_X_INCHES   = 8      # x figure size of feedback in inches
    FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
    PLOT_LEFT          = 0.1    # in fractions of figure width and height
    PLOT_BOT           = 0.1    
    PLOT_WIDTH         = 0.8
    PLOT_HEIGHT        = 0.8

    ###
    # Add 2D position / trajectory plot
    ###
    trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
            title='Vehicle Trajectory',
            figsize=(FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES),
            edgecolor="black",
            rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])
    
    trajectory_fig.set_invert_x_axis() # Because UE4 uses left-handed 
                                           # coordinate system the X
                                           # axis in the graph is flipped
    trajectory_fig.set_axis_equal()    # X-Y spacing should be equal in size    

    # Add waypoint markers
    trajectory_fig.add_graph("waypoints", window_size=len(x_points),
                                x0=x_points, y0=y_points,
                                linestyle="-", marker="", color='g')
    
    lp_traj.refresh()

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

    
    ackermann_control = carla.VehicleAckermannControl()

    clock = pygame.time.Clock()

    call_exit = False

    time.sleep(5)

    while True:
        world.tick()
        clock.tick_busy_loop(60)

        ackermann_control.speed=0.5
        vehicle.apply_ackermann_control(ackermann_control)

        # Render received data
        display_manager.render()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                call_exit = True
                break

        if call_exit:
            break

    if display_manager:
        display_manager.destroy()

    time.sleep(10)
    

if __name__ == '__main__':
    main()
