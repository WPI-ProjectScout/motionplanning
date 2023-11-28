import carla
import random
import time
import pygame
import math
import numpy as np
import live_plotter as lv
import vehicle.controller2d
import planning.local_planner as local_planner
import planning.behavioural_planner as behavioural_planner
from agents.tools.misc import get_speed
from planning.global_route_planner import GlobalRoutePlanner
import scout.vehicle.displaymanager as dm
import scout.vehicle.sensormanager as sm

# Planning Constants
NUM_PATHS = 7
BP_LOOKAHEAD_BASE      = 8.0              # m
BP_LOOKAHEAD_TIME      = 2.0              # s
PATH_OFFSET            = 1.5              # m
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.5, 1.5, 1.5]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10
A_MAX                  = 1.5              # m/s^2
SLOW_SPEED             = 2.0              # m/s
STOP_LINE_BUFFER       = 3.5              # m
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 2                # Frequency divisor to make the 
                                          # local planner operate at a lower
                                          # frequency than the controller
                                          # (which operates at the simulation
                                          # frequency). Must be a natural
                                          # number.

def prepare_control_command(throttle, steer, brake, 
                         hand_brake=False, reverse=False, manual_gear_shift=False):
    """Send control command to CARLA client.
    
    Send control command to CARLA client.

    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
    """
    control = carla.VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    control.manual_gear_shift = manual_gear_shift
    return control

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
    # waypoints will be of the form
    # (rows = waypoints, columns = [x, y, v])
    # To initialize this, the v column will be set to 0
    # The velocity profile generator will update v later
    waypoints = []
    for i in range(len(route_trace)):
        x = route_trace[i][0].transform.location.x
        y = route_trace[i][0].transform.location.y
        waypoints.append([x, y, 0])
        x_points.append(x)
        y_points.append(y)

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
    
    # Add local path proposals
    for i in range(NUM_PATHS):
        trajectory_fig.add_graph("local_path " + str(i), window_size=200,
                                    x0=None, y0=None, color=[0.0, 0.0, 1.0])

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

    
    ################################
    # Local Planning
    ################################
    wp_goal_index   = 0
    local_waypoints = None
    path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)
    lp = local_planner.LocalPlanner(NUM_PATHS,
                                    PATH_OFFSET,
                                    CIRCLE_OFFSETS,
                                    CIRCLE_RADII,
                                    PATH_SELECT_WEIGHT,
                                    TIME_GAP,
                                    A_MAX,
                                    SLOW_SPEED,
                                    STOP_LINE_BUFFER)
    bp = behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE,
                                                [],#stopsign_fences,
                                                LEAD_VEHICLE_LOOKAHEAD)
    
    # controller = controller2d.Controller2D(waypoints)
    
    clock = pygame.time.Clock()

    time.sleep(3)
    call_exit = False
    LP_FREQUENCY_DIVISOR = 2
    initial_execution_time = time.time()
    current_timestamp = initial_execution_time
    WAIT_TIME_BEFORE_START = 1.00   # game seconds (time before controller start)
    frame = 0
    while True:
        world.tick()
        clock.tick()
        frame += 1

        vehicle_transform = vehicle.get_transform()
        prev_timestamp = current_timestamp
        current_timestamp = time.time() - initial_execution_time
        current_x = vehicle_transform.location.x
        current_y = vehicle_transform.location.y
        current_yaw = math.radians(vehicle_transform.rotation.yaw)
        current_speed = get_speed(vehicle)/ 3.6

        # Allow some initialization time for everything
        if current_timestamp <= WAIT_TIME_BEFORE_START:
            control = prepare_control_command(throttle=0.0, steer=0, brake=1.0)
            vehicle.apply_control(control)
            continue
        else:
            current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START

        ##############
        # Planning
        ##############
        if frame % LP_FREQUENCY_DIVISOR == 0:
            # Run the local planning tasks in this section

            # Compute open loop speed estimate.
            open_loop_speed = lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)

            # Calculate the goal state set in the local frame for the local planner.
            # Current speed should be open loop for the velocity profile generation.
            ego_state = [current_x, current_y, current_yaw, open_loop_speed]

            # Set lookahead based on current speed.
            bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)

            # Perform a state transition in the behavioural planner.
            bp.transition_state(waypoints, ego_state, current_speed)

            # Check to see if we need to follow the lead vehicle.
            bp.check_for_lead_vehicle(ego_state, [])

            # Compute the goal state set from the behavioural planner's computed goal state.
            goal_state_set = lp.get_goal_state_set(bp._goal_index, bp._goal_state, waypoints, ego_state)

            # Calculate planned paths in the local frame.
            paths, path_validity = lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

        ##############
        # Plots update
        ##############
        if frame % LP_FREQUENCY_DIVISOR == 0:
            path_counter = 0
            for i in range(NUM_PATHS):
                # If a path was invalid in the set, there is no path to plot.
                if path_validity[i]:
                    # Colour paths according to collision checking.
                    # if not collision_check_array[path_counter]:
                    #     colour = 'r'
                    # elif i == best_index:
                    #     colour = 'k'
                    # else:
                    colour = 'b'
                    trajectory_fig.update("local_path " + str(i), paths[path_counter][0], paths[path_counter][1], colour)
                    path_counter += 1
                else:
                    trajectory_fig.update("local_path " + str(i), [ego_state[0]], [ego_state[1]], 'r')
        
        lp_traj.refresh()

        control = prepare_control_command(throttle=0.0, steer=0, brake=1.0)
        vehicle.apply_control(control)

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
    

if __name__ == '__main__':
    main()
