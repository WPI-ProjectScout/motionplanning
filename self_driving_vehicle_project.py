import carla
import random
import time
import pygame
import math
import numpy as np
import live_plotter as lv
import vehicle.controller2d as controller2d
import planning.local_planner as local_planner
import planning.behavioural_planner as behavioural_planner
from agents.tools.misc import get_speed, get_trafficlight_trigger_location
from planning.global_route_planner import GlobalRoutePlanner
import vehicle.displaymanager as dm
import vehicle.sensormanager as sm

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
SLOW_SPEED             = 4.0              # m/s intermediate speed when decelerating
STOP_LINE_BUFFER       = 3.5              # m
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 2                # Frequency divisor to make the 
                                          # local planner operate at a lower
                                          # frequency than the controller
                                          # (which operates at the simulation
                                          # frequency). Must be a natural
                                          # number.

TRAFFIC_LIGHT_LOOKAHEAD = 20

TARGET_VELOCITY_WAYPOINTS = 8

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # selected path
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
                                       # simulation ends

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
    bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
    # print(blueprint_library.filter('vehicle.tesla.*'))

    ######################################
    # Global planning, spawn location and destrination
    ######################################

    # TODO add logi to retry find a rout between random start and end destination

    # init_pos = carla.Transform(carla.Location(x=158.0, y=24.0, z=0.05), carla.Rotation(yaw=-90))
    spawn_point = random.choice(map.get_spawn_points())
    # spawn_point = carla.Transform(carla.Location(x=26.382587, y=-57.401386, z=0.6), carla.Rotation(yaw=-0.023438))
    # This is the player
    # vehicle = world.try_spawn_actor(bp, spawn_point)
    vehicle = world.spawn_actor(bp, spawn_point)
    # vehicle.set_simulate_physics(False)

    destination_point = random.choice(map.get_spawn_points())
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

    ######################################
    # Converting carla global planner output to coursera waypoints format
    # [x, y, v]
    ######################################
    x_points = []
    y_points = []
    # waypoints will be of the form
    # (rows = waypoints, columns = [x, y, v])
    # To initialize this, the v column will be set to 10
    # The velocity profile generator will update v later
    waypoints = []

    min_x = float('Inf')
    min_y = float('Inf')
    max_x = 0
    max_y = 0
    for i in range(len(route_trace)):
        # print(route_trace[i])
        x = route_trace[i][0].transform.location.x
        y = route_trace[i][0].transform.location.y
        if i > 0:
            prev_x = route_trace[i-1][0].transform.location.x
            prev_y = route_trace[i-1][0].transform.location.y
            dist = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        else:
            dist = 1 # random number to allow appending to array
        # compare = (route_trace[i][0].transform.location == route_trace[i-1][0].transform.location)
        # if compare == False:
        if dist >= 0.25:
            waypoints.append([x, y, TARGET_VELOCITY_WAYPOINTS])
            x_points.append(x)
            y_points.append(y)

            if x > max_x:
                max_x = x
            if x < min_x:
                min_x = x

            if y > max_y:
                max_y = y
            if y < min_y:
                min_y = y

    vehicle_transform = vehicle.get_transform()
    start_x = vehicle_transform.location.x
    start_y = vehicle_transform.location.y
    start_yaw = math.radians(vehicle_transform.rotation.yaw)

    ######################################
    # Calling live plotter
    ######################################
    
    # The main objects to refresh plotting
    lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")
    lp_1d = lv.LivePlotter(tk_title="Controls Feedback")

    FIGSIZE_X_INCHES   = 8      # x figure size of feedback in inches
    FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
    PLOT_LEFT          = 0.1    # in fractions of figure width and height
    PLOT_BOT           = 0.1    
    PLOT_WIDTH         = 0.8
    PLOT_HEIGHT        = 0.8

    ######################################
    # Add 2D position / trajectory plot
    ######################################
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
    
    # Add lookahead path
    trajectory_fig.add_graph("selected_path", 
                                window_size=INTERP_MAX_POINTS_PLOT,
                                x0=[start_x]*INTERP_MAX_POINTS_PLOT, 
                                y0=[start_y]*INTERP_MAX_POINTS_PLOT,
                                color=[1, 0.5, 0.0],
                                linewidth=3)
    
    # Add local path proposals
    for i in range(NUM_PATHS):
        trajectory_fig.add_graph("local_path " + str(i), window_size=200,
                                    x0=None, y0=None, color=[0.0, 0.0, 1.0])
        
    ######################################
    # Add 1D control plots
    ######################################

    forward_speed_fig = lp_1d.plot_new_dynamic_figure(title="Forward Speed (m/s)")
    forward_speed_fig.add_graph("forward_speed", 
                                label="forward_speed", 
                                window_size=300)
    forward_speed_fig.add_graph("reference_signal", 
                                label="reference_Signal", 
                                window_size=300)
    
    yaw_fig = lp_1d.plot_new_dynamic_figure(title="Yaw (rad)")
    yaw_fig.add_graph("yaw", 
                            label="yaw", 
                            window_size=300)
    yaw_fig.add_graph("yaw_reference", 
                            label="yaw_reference", 
                            window_size=300)
    

    ################################
    # Display manager handling
    ################################

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
    # Handle traffic lights and creation of their graph
    ################################

    # Get a list of traffic lights located within the route area
    all_lights = world.get_actors().filter("*traffic_light*")
    lights_list = []
    lights_map = {}

    for light in all_lights:
        trigger_location = get_trafficlight_trigger_location(light)
        if (min_x < trigger_location.x < max_x) and (min_y < trigger_location.y < max_y):
            lights_list.append(light)

            if not light.id in lights_map:
                lights_map[light.id] = map.get_waypoint(trigger_location)

    # Add trafic lights
    for light_id in lights_map:
        trajectory_fig.add_graph("traffic_light"+str(light_id), window_size=1,
                                marker="H", markertext="Traffic light", marker_text_offset=1,
                                x0=None, y0=None, color="g")

    
    ################################
    # Bheavior and Local Planning objects, with their parameters
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
                                                lights_list,
                                                TRAFFIC_LIGHT_LOOKAHEAD,
                                                LEAD_VEHICLE_LOOKAHEAD,
                                                map)
    bp._lights_map = lights_map
    
    controller = controller2d.Controller2D(waypoints)
    
    ################################
    # Vars fro main loop execution and start of while loop
    ################################
    
    clock = pygame.time.Clock()

    time.sleep(3)
    call_exit = False
    reached_the_end = False
    LP_FREQUENCY_DIVISOR = 2
    initial_execution_time = time.time()
    current_timestamp = initial_execution_time
    WAIT_TIME_BEFORE_START = 1.00   # game seconds (time before controller start)
    frame = 0
    prev_bp_state = bp._state
    while True:
        world.tick()
        clock.tick()
        frame += 1

        
        ################################
        # Grabbing localization vars from the car
        ################################
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
            # TODO check if this is already in behavior planning
            bp.check_for_lead_vehicle(ego_state, [])

            # Compute the goal state set from the behavioural planner's computed goal state.
            goal_state_set = lp.get_goal_state_set(bp._goal_index, bp._goal_state, waypoints, ego_state)

            # Calculate planned paths in the local frame.
            paths, path_validity = lp.plan_paths(goal_state_set)

            # Transform those paths back to the global frame.
            paths = local_planner.transform_paths(paths, ego_state)

            # Perform collision checking.
            # TODO update this to actually check for collisions
            collision_check_array = lp._collision_checker.collision_check(paths, [])

            # Compute the best local path.
            best_index = lp._collision_checker.select_best_path_index(paths, collision_check_array, bp._goal_state)
            # If no path was feasible, continue to follow the previous best path.
            if best_index == None:
                best_path = lp._prev_best_path
            else:
                best_path = paths[best_index]
                lp._prev_best_path = best_path

            # Compute the velocity profile for the path, and compute the waypoints.
            # Use the lead vehicle to inform the velocity profile's dynamic obstacle handling.
            # In this scenario, the only dynamic obstacle is the lead vehicle at index 1.
            desired_speed = bp._goal_state[2]
            # lead_car_state = [lead_car_pos[1][0], lead_car_pos[1][1], lead_car_speed[1]]
            # TODO update to actually check for lead cars
            lead_car_state = [0, 0, 0]
            decelerate_to_stop = ((bp._state == behavioural_planner.DECELERATE_TO_STOP) or (bp._state == behavioural_planner.DECELERATE_TO_LIGHT))
            local_waypoints = lp._velocity_planner.compute_velocity_profile(best_path, desired_speed, ego_state, current_speed, decelerate_to_stop, lead_car_state, bp._follow_lead_vehicle)

            # Planning is finished, but need to intepolate waypoints
            if local_waypoints != None:
                # Update the controller waypoint path with the best local path.
                # Linear interpolation computation on the waypoints
                # is also used to ensure a fine resolution between points.
                wp_distance = []   # distance array
                local_waypoints_np = np.array(local_waypoints)
                for i in range(1, local_waypoints_np.shape[0]):
                    wp_distance.append(
                            np.sqrt((local_waypoints_np[i, 0] - local_waypoints_np[i-1, 0])**2 +
                                    (local_waypoints_np[i, 1] - local_waypoints_np[i-1, 1])**2))
                wp_distance.append(0)  # last distance is 0 because it is the distance
                                        # from the last waypoint to the last waypoint

                # Linearly interpolate between waypoints and store in a list
                wp_interp      = []    # interpolated values 
                                        # (rows = waypoints, columns = [x, y, v])
                for i in range(local_waypoints_np.shape[0] - 1):
                    # Add original waypoint to interpolated waypoints list (and append
                    # it to the hash table)
                    wp_interp.append(list(local_waypoints_np[i]))
            
                    # Interpolate to the next waypoint. First compute the number of
                    # points to interpolate based on the desired resolution and
                    # incrementally add interpolated points until the next waypoint
                    # is about to be reached.
                    num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                                    float(INTERP_DISTANCE_RES)) - 1)
                    wp_vector = local_waypoints_np[i+1] - local_waypoints_np[i]
                    wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                    for j in range(num_pts_to_interp):
                        next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                        wp_interp.append(list(local_waypoints_np[i] + next_wp_vector))
                # add last waypoint at the end
                wp_interp.append(list(local_waypoints_np[-1]))
                
                # Update the other controller values and controls
                controller.update_waypoints(wp_interp)
                pass

        ##############
        # Controls update
        ##############
        if local_waypoints != None and local_waypoints != []:
            controller.update_values(current_x, current_y, current_yaw,
                                     current_speed,
                                     current_timestamp, frame)
            controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
        else:
            cmd_throttle = 0.0
            cmd_steer = 0.0
            cmd_brake = 0.0
        control = prepare_control_command(throttle=cmd_throttle, steer=cmd_steer, brake=cmd_brake)
        # print(control.steer)
        vehicle.apply_control(control)

        ##############
        # Plots update
        ##############
        forward_speed_fig.roll("forward_speed", 
                                       current_timestamp, 
                                       current_speed)
        forward_speed_fig.roll("reference_signal", 
                                current_timestamp, 
                                controller._desired_speed)
        
        yaw_fig.roll("yaw", 
                    current_timestamp, 
                    current_yaw)
        yaw_fig.roll("yaw_reference", 
                    current_timestamp, 
                    controller._desired_yaw)

        if frame % LP_FREQUENCY_DIVISOR == 0:
            if not bp._state == prev_bp_state:
                print(bp)
            prev_bp_state = bp._state
            path_counter = 0
            for i in range(NUM_PATHS):
                # If a path was invalid in the set, there is no path to plot.
                if path_validity[i]:
                    # Colour paths according to collision checking.
                    if not collision_check_array[path_counter]:
                        colour = 'r'
                    elif i == best_index:
                        colour = 'k'
                    else:
                        colour = 'b'
                    trajectory_fig.update("local_path " + str(i), paths[path_counter][0], paths[path_counter][1], colour)
                    path_counter += 1
                else:
                    trajectory_fig.update("local_path " + str(i), [ego_state[0]], [ego_state[1]], 'r')

            # When plotting lookahead path, only plot a number of points
            # (INTERP_MAX_POINTS_PLOT amount of points). This is meant
            # to decrease load when live plotting
            wp_interp_np = np.array(wp_interp)
            path_indices = np.floor(np.linspace(0, 
                                                wp_interp_np.shape[0]-1,
                                                INTERP_MAX_POINTS_PLOT))
            trajectory_fig.update("selected_path", 
                    wp_interp_np[path_indices.astype(int), 0],
                    wp_interp_np[path_indices.astype(int), 1],
                    new_colour=[1, 0.5, 0.0])
            
            for light in lights_list:
                loc = lights_map[light.id].transform.location
                x = loc.x
                y = loc.y

                if light.state == carla.TrafficLightState.Green:
                    colour = 'g'
                elif light.state == carla.TrafficLightState.Yellow:
                    colour = 'y'
                elif light.state == carla.TrafficLightState.Red:
                    colour = 'r'

                trajectory_fig.update("traffic_light"+str(light.id), [x], [y], colour)

        if bp._previous_traffic_light:
            selection_colour = 'k'
            if bp._previous_traffic_light.state == carla.TrafficLightState.Green:
                colour = "g"
            elif bp._previous_traffic_light.state == carla.TrafficLightState.Yellow:
                colour = "y"
            elif bp._previous_traffic_light.state == carla.TrafficLightState.Red:
                colour = "r"
            if frame % LP_FREQUENCY_DIVISOR == 0:
                # trick to toogle colour
                colour = selection_colour
            loc = lights_map[bp._previous_traffic_light.id].transform.location
            trajectory_fig.update("traffic_light"+str(bp._previous_traffic_light.id), [loc.x], [loc.y], colour)
            
        
        lp_traj.refresh()
        lp_1d.refresh()

        # Render received data
        display_manager.render()

        # Find if reached the end of waypoint. If the car is within
        # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
        # the simulation will end.
        dist_to_last_waypoint = np.linalg.norm(np.array([
            waypoints[-1][0] - current_x,
            waypoints[-1][1] - current_y]))
        if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
            reached_the_end = True
        if reached_the_end:
            break

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                call_exit = True
                break

        if call_exit:
            break

    time.sleep(3)
    
    if display_manager:
        display_manager.destroy()
    

if __name__ == '__main__':
    main()
