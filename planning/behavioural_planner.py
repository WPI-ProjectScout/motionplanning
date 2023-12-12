#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: November 21, 2018

import numpy as np
import math
from agents.tools.misc import get_trafficlight_trigger_location
import carla

# State machine states
FOLLOW_LANE = 0
DECELERATE_TO_STOP = 1
STAY_STOPPED = 2
DECELERATE_TO_LIGHT = 3
STAY_STOPPED_UNTIL_GREEN = 4
# Stop speed threshold
STOP_THRESHOLD = 0.5
# Number of cycles before moving from stop sign.
STOP_COUNTS = 10

class BehaviouralPlanner:
    def __init__(self, lookahead, lights_list, traffic_light_lookahead, lead_vehicle_lookahead, map, default_speed):
        self._lookahead                     = lookahead
        self._stopsign_fences               = []
        self._previous_traffic_light        = None
        self._lights_list                   = lights_list
        self._lights_map                    = {}
        self._traffic_light_lookahead       = traffic_light_lookahead
        self._follow_lead_vehicle_lookahead = lead_vehicle_lookahead
        self._state                         = FOLLOW_LANE
        self._follow_lead_vehicle           = False
        self._goal_state                    = [0.0, 0.0, 0.0]
        self._goal_index                    = 0
        self._stop_count                    = 0
        self._map                           = map
        self._default_speed                 = default_speed

    def set_lookahead(self, lookahead):
        self._lookahead = lookahead

    # Handles state transitions and computes the goal state.
    def transition_state(self, waypoints, ego_state, closed_loop_speed):
        """Handles state transitions and computes the goal state.  
        
        args:
            waypoints: current waypoints to track (global frame). 
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                         [x1, y1, v1],
                         ...
                         [xn, yn, vn]]
                example:
                    waypoints[2][1]: 
                    returns the 3rd waypoint's y position

                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            closed_loop_speed: current (closed-loop) speed for vehicle (m/s)
        variables to set:
            self._goal_index: Goal index for the vehicle to reach
                i.e. waypoints[self._goal_index] gives the goal waypoint
            self._goal_state: Goal state for the vehicle to reach (global frame)
                format: [x_goal, y_goal, v_goal]
            self._state: The current state of the vehicle.
                available states: 
                    FOLLOW_LANE         : Follow the global waypoints (lane).
                    DECELERATE_TO_STOP  : Decelerate to stop.
                    STAY_STOPPED        : Stay stopped.
            self._stop_count: Counter used to count the number of cycles which
                the vehicle was in the STAY_STOPPED state so far.
        useful_constants:
            STOP_THRESHOLD  : Stop speed threshold (m). The vehicle should fully
                              stop when its speed falls within this threshold.
            STOP_COUNTS     : Number of cycles (simulation iterations) 
                              before moving from stop sign.
        """
        # In this state, continue tracking the lane by finding the
        # goal index in the waypoint list that is within the lookahead
        # distance. Then, check to see if the waypoint path intersects
        # with any stop lines. If it does, then ensure that the goal
        # state enforces the car to be stopped before the stop line.
        # You should use the get_closest_index(), get_goal_index(), and
        # check_for_stop_signs() helper functions.
        # Make sure that get_closest_index() and get_goal_index() functions are
        # complete, and examine the check_for_stop_signs() function to
        # understand it.
        if self._state == FOLLOW_LANE:
            # First, find the closest index to the ego vehicle.
            # ------------------------------------------------------------------
            closest_len, closest_index = get_closest_index(waypoints, ego_state)
            # ------------------------------------------------------------------

            # Next, find the goal index that lies within the traffic lookahead distance
            # along the waypoints.
            # ------------------------------------------------------------------

            # 1 store the previously set lookahead distance
            prev_lookahead = self._lookahead
            # 2 update the lookahead distance to use the traffic light lookahead distance
            # self._lookahead = self._traffic_light_lookahead
            self.set_lookahead(8 + 2 * ego_state[3])

            # 3 Find the goal index based on the traffic light look ahead distance
            goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
            # ------------------------------------------------------------------

            #### 4 search for stop signs and traffic lights considering the traffig light
            #### lookahead distance
            # Check the index set between closest_index and goal_index
            # for stop signs, and compute the goal state accordingly.
            # ------------------------------------------------------------------
            goal_index_stop_sign, stop_sign_found = self.check_for_stop_signs(waypoints, closest_index, goal_index)
            # ------------------------------------------------------------------

            # Check the index set between closest_index and goal_index
            # for traffic lights, and compute the goal state accordingly.
            # ------------------------------------------------------------------
            goal_index_traffic_light, traffic_light_found = self.check_for_traffic_lights(waypoints, closest_index, goal_index)
            # ------------------------------------------------------------------

            # If stop sign found, set the goal to zero speed, then transition to 
            # the deceleration state.
            # ------------------------------------------------------------------
            if stop_sign_found and not traffic_light_found:
                self._goal_index = goal_index_stop_sign
                self._goal_state = waypoints[goal_index_stop_sign]
                self._goal_state[2] = 0
                self._state = DECELERATE_TO_STOP
            # ------------------------------------------------------------------

            # If traffic light found, set the goal to zero speed, then transition to 
            # the deceleration state.
            # ------------------------------------------------------------------
            elif not stop_sign_found and traffic_light_found:
                self._goal_index = goal_index_traffic_light
                self._goal_state = waypoints[goal_index_traffic_light]
                self._goal_state[2] = 0
                self._state = DECELERATE_TO_LIGHT
            # ------------------------------------------------------------------

            # If both stop sign and traffic light found, set the goal to zero speed, then transition to 
            # the deceleration state. Choose the closest goal index
            # ------------------------------------------------------------------
            elif stop_sign_found and traffic_light_found:
                # If stop sign is closer, follow behavior for stop sign
                if goal_index_stop_sign <= goal_index_traffic_light:
                    self._goal_index = goal_index_stop_sign
                    self._goal_state = waypoints[goal_index_stop_sign]
                    self._state = DECELERATE_TO_STOP
                else:
                    self._goal_index = goal_index_traffic_light
                    self._goal_state = waypoints[goal_index_traffic_light]
                    self._state = DECELERATE_TO_LIGHT
                self._goal_state[2] = 0
            # ------------------------------------------------------------------

            # No traffic sign or lights were found
            # ------------------------------------------------------------------
            else:
                #### 5 If no stop signs or traffic lights were found wihin the long lookahead distance
                #### Then look again for the goal index considering a smaller lookahead

                # 6 Recover the originally set lookahead
                self._lookahead = prev_lookahead

                # 7 find the goal index that is wihtin the original (small) lookahead
                # distance
                goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
                self._goal_index = goal_index
                self._goal_state = waypoints[goal_index]
                # print("Goal state: ", self._goal_state, " and index: ", self._goal_index)
                self._goal_state[2] = self._default_speed

            pass

        # In this state, check if we have reached a complete stop. Use the
        # closed loop speed to do so, to ensure we are actually at a complete
        # stop, and compare to STOP_THRESHOLD.  If so, transition to the next
        # state.
        elif self._state == DECELERATE_TO_LIGHT:
            
            # Check if the traffic light has already changed to green
            # or stay on this state until decelerated to full stop
            # Then proceed to state to wait for green light
            # ------------------------------------------------------------------

            # Check if the previous light is still on red
            _, traffic_light_found = self.check_for_traffic_lights(waypoints, 0, 0)

            # If fully stopped and still onf read, proceed to wait only
            if closed_loop_speed <= STOP_THRESHOLD and traffic_light_found:
                self._state = STAY_STOPPED_UNTIL_GREEN
            elif not traffic_light_found:
                self._state = FOLLOW_LANE
            # ------------------------------------------------------------------

            pass

        # In this state, check if we have reached a complete stop. Use the
        # closed loop speed to do so, to ensure we are actually at a complete
        # stop, and compare to STOP_THRESHOLD.  If so, transition to the next
        # state.
        elif self._state == DECELERATE_TO_STOP:
            # ------------------------------------------------------------------
            if closed_loop_speed <= STOP_THRESHOLD:
                self._state = STAY_STOPPED
            # ------------------------------------------------------------------

        # In this state, check to see if we have stayed stopped for at
        # least STOP_COUNTS number of cycles. If so, we can now leave
        # the stop sign and transition to the next state.
        elif self._state == STAY_STOPPED:
            # We have stayed stopped for the required number of cycles.
            # Allow the ego vehicle to leave the stop sign. Once it has
            # passed the stop sign, return to lane following.
            # You should use the get_closest_index(), get_goal_index(), and 
            # check_for_stop_signs() helper functions.
            if self._stop_count == STOP_COUNTS:
                # --------------------------------------------------------------
                closest_len, closest_index = get_closest_index(waypoints, ego_state)
                goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
                # --------------------------------------------------------------

                # We've stopped for the required amount of time, so the new goal 
                # index for the stop line is not relevant. Use the goal index
                # that is the lookahead distance away.
                # --------------------------------------------------------------
                _, stop_sign_found = self.check_for_stop_signs(waypoints, closest_index, goal_index)
                self._goal_index = goal_index
                self._goal_state = waypoints[goal_index] 
                # --------------------------------------------------------------

                # If the stop sign is no longer along our path, we can now
                # transition back to our lane following state.
                # --------------------------------------------------------------
                if not stop_sign_found:
                    self._state = FOLLOW_LANE
                # --------------------------------------------------------------

                pass

            # Otherwise, continue counting.
            else:
                # --------------------------------------------------------------
                self._stop_count += 1
                # --------------------------------------------------------------

                pass
        # In this state, check to see if we have stayed stopped for at
        # least STOP_COUNTS number of cycles. If so, we can now leave
        # the stop sign and transition to the next state.
        elif self._state == STAY_STOPPED_UNTIL_GREEN:

            # Check if last traffic light is green now
            _, traffic_light_found = self.check_for_traffic_lights(waypoints, 0, 0)
            if not traffic_light_found:

                # --------------------------------------------------------------
                # Exit of stopped state if ready to move on after green light
                closest_len, closest_index = get_closest_index(waypoints, ego_state)
                goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
                self._goal_index = goal_index
                self._goal_state = waypoints[goal_index] 
                # print("Goal state: ", self._goal_state, " and index: ", self._goal_index)
                self._state = FOLLOW_LANE
                # --------------------------------------------------------------

            pass

        else:
            raise ValueError('Invalid state value.')

    # Gets the goal index in the list of waypoints, based on the lookahead and
    # the current ego state. In particular, find the earliest waypoint that has accumulated
    # arc length (including closest_len) that is greater than or equal to self._lookahead.
    def get_goal_index(self, waypoints, ego_state, closest_len, closest_index):
        """Gets the goal index for the vehicle. 
        
        Set to be the earliest waypoint that has accumulated arc length
        accumulated arc length (including closest_len) that is greater than or
        equal to self._lookahead.

        args:
            waypoints: current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                         [x1, y1, v1],
                         ...
                         [xn, yn, vn]]
                example:
                    waypoints[2][1]: 
                    returns the 3rd waypoint's y position

                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            closest_len: length (m) to the closest waypoint from the vehicle.
            closest_index: index of the waypoint which is closest to the vehicle.
                i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
        returns:
            wp_index: Goal index for the vehicle to reach
                i.e. waypoints[wp_index] gives the goal waypoint
        """
        # Find the farthest point along the path that is within the
        # lookahead distance of the ego vehicle.
        # Take the distance from the ego vehicle to the closest waypoint into
        # consideration.
        arc_length = closest_len
        wp_index = closest_index
        
        # In this case, reaching the closest waypoint is already far enough for
        # the planner.  No need to check additional waypoints.
        if arc_length > self._lookahead:
            return wp_index

        # We are already at the end of the path.
        if wp_index == len(waypoints) - 1:
            return wp_index

        # Otherwise, find our next waypoint.
        while wp_index < len(waypoints) - 1:
            wp_index += 1
            x_dist = waypoints[wp_index][0] - waypoints[wp_index-1][0]
            y_dist = waypoints[wp_index][1] - waypoints[wp_index-1][1]
            dist = np.linalg.norm(np.array([x_dist, y_dist]))
            arc_length += dist
            if arc_length >= self._lookahead:
                break
        # ------------------------------------------------------------------

        return wp_index

    # Checks the given segment of the waypoint list to see if it
    # intersects with a traffic light. If any index does, return the
    # new goal state accordingly.
    def check_for_traffic_lights(self, waypoints, closest_index, goal_index):
        if self._previous_traffic_light:
            if self._previous_traffic_light.state == carla.TrafficLightState.Green:
                self._previous_traffic_light = None
                return (goal_index, False)
            else:
                return (goal_index, True)

        # Start from the goal index backwards to guarantee we get close to the traffic light
        for i in range(closest_index, goal_index):

            # Get the corresponding waypoint object
            ego_vehicle_location = carla.Location(x=waypoints[i][0], y=waypoints[i][1])
            ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

            # Determine what the heading of the vehicle would be if following this route
            if i == len(waypoints)-1:
                delta_x = waypoints[i][0] - waypoints[i-1][0]
                delta_y = waypoints[i][1] - waypoints[i-1][1]
            else:
                delta_x = waypoints[i+1][0] - waypoints[i][0]
                delta_y = waypoints[i+1][1] - waypoints[i][1]
            heading = np.arctan2(delta_y, delta_x)
            ego_vehicle_waypoint.transform.rotation.yaw = heading
            
            for traffic_light in self._lights_list:
                # Get traffic light waypoint
                if traffic_light.id in self._lights_map:
                    trigger_wp = self._lights_map[traffic_light.id]
                else:
                    trigger_location = get_trafficlight_trigger_location(traffic_light)
                    trigger_wp = self._map.get_waypoint(trigger_location)
                    self._lights_map[traffic_light.id] = trigger_wp

                # Ignore if trafic light is green
                if traffic_light.state == carla.TrafficLightState.Green:
                    continue

                # Ignore if light is beyond look ahead distance
                if trigger_wp.transform.location.distance(ego_vehicle_location) > self._traffic_light_lookahead:
                    continue

                # print("Close to a traffic light, checking angles now. ID: ", traffic_light.id)

                ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
                wp_dir = trigger_wp.transform.get_forward_vector()
                dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

                # Ignore traffic light if its not facing our direction
                if dot_ve_wp < 0:
                    continue

                # print("Dot product passed. ID: ", traffic_light.id)

                target_vector = np.array([
                    trigger_wp.transform.location.x - ego_vehicle_location.x,
                    trigger_wp.transform.location.y - ego_vehicle_location.y
                ])
                norm_target = np.linalg.norm(target_vector)

                is_within_range = False
                if norm_target > self._traffic_light_lookahead:
                    # print("norm bigger than look ahead. ID: ", traffic_light.id)
                    is_within_range = False
                # If the vector is too short, we can simply stop here
                elif norm_target < 0.001:
                    is_within_range = True
                else:
                    min_angle = 0
                    max_angle = 90

                    forward_vector = np.array([ve_dir.x, ve_dir.y])
                    angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

                    is_within_range = min_angle < angle < max_angle
                    # if not is_within_range:
                        # print("angle did not match. ID: ", traffic_light.id)

                if is_within_range:
                    # print("Detected ID: ", traffic_light.id)
                    self._previous_traffic_light = traffic_light
                    break
                    # print("light location: ", trigger_wp.transform.location, " goal waypoint: ", waypoints[i])
                    
                    # return i, True
            
            
            # If we have assinged a trafiic light it means we found which one is afecting us
            if self._previous_traffic_light:
                break
        
        if self._previous_traffic_light:
            # Lets find the closest index to the traffic light location
            traffic_light_wp = self._lights_map[self._previous_traffic_light.id]
            traffic_light_state = [traffic_light_wp.transform.location.x, traffic_light_wp.transform.location.y, 0, 0]
            closest_dist, closest_index = get_closest_index(waypoints, traffic_light_state)

            print("light location: ", traffic_light_wp.transform.location, " goal waypoint: ", waypoints[closest_index], "with index: ", closest_index, " and dist: ", closest_dist)
            return closest_index, True

        # By default return the original goal index and a False flag
        return goal_index, False

    # Checks the given segment of the waypoint list to see if it
    # intersects with a stop line. If any index does, return the
    # new goal state accordingly.
    def check_for_stop_signs(self, waypoints, closest_index, goal_index):
        """Checks for a stop sign that is intervening the goal path.

        Checks for a stop sign that is intervening the goal path. Returns a new
        goal index (the current goal index is obstructed by a stop line), and a
        boolean flag indicating if a stop sign obstruction was found.
        
        args:
            waypoints: current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                         [x1, y1, v1],
                         ...
                         [xn, yn, vn]]
                example:
                    waypoints[2][1]: 
                    returns the 3rd waypoint's y position

                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
                closest_index: index of the waypoint which is closest to the vehicle.
                    i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
                goal_index (current): Current goal index for the vehicle to reach
                    i.e. waypoints[goal_index] gives the goal waypoint
        variables to set:
            [goal_index (updated), stop_sign_found]: 
                goal_index (updated): Updated goal index for the vehicle to reach
                    i.e. waypoints[goal_index] gives the goal waypoint
                stop_sign_found: Boolean flag for whether a stop sign was found or not
        """
        for i in range(closest_index, goal_index):
            # Check to see if path segment crosses any of the stop lines.
            intersect_flag = False
            for stopsign_fence in self._stopsign_fences:
                wp_1   = np.array(waypoints[i][0:2])
                wp_2   = np.array(waypoints[i+1][0:2])
                s_1    = np.array(stopsign_fence[0:2])
                s_2    = np.array(stopsign_fence[2:4])

                v1     = np.subtract(wp_2, wp_1)
                v2     = np.subtract(s_1, wp_2)
                sign_1 = np.sign(np.cross(v1, v2))
                v2     = np.subtract(s_2, wp_2)
                sign_2 = np.sign(np.cross(v1, v2))

                v1     = np.subtract(s_2, s_1)
                v2     = np.subtract(wp_1, s_2)
                sign_3 = np.sign(np.cross(v1, v2))
                v2     = np.subtract(wp_2, s_2)
                sign_4 = np.sign(np.cross(v1, v2))

                # Check if the line segments intersect.
                if (sign_1 != sign_2) and (sign_3 != sign_4):
                    intersect_flag = True

                # Check if the collinearity cases hold.
                if (sign_1 == 0) and pointOnSegment(wp_1, s_1, wp_2):
                    intersect_flag = True
                if (sign_2 == 0) and pointOnSegment(wp_1, s_2, wp_2):
                    intersect_flag = True
                if (sign_3 == 0) and pointOnSegment(s_1, wp_1, s_2):
                    intersect_flag = True
                if (sign_3 == 0) and pointOnSegment(s_1, wp_2, s_2):
                    intersect_flag = True

                # If there is an intersection with a stop line, update
                # the goal state to stop before the goal line.
                if intersect_flag:
                    goal_index = i
                    return goal_index, True

        return goal_index, False
                
    # Checks to see if we need to modify our velocity profile to accomodate the
    # lead vehicle.
    def check_for_lead_vehicle(self, ego_state, lead_car_position):
        """Checks for lead vehicle within the proximity of the ego car, such
        that the ego car should begin to follow the lead vehicle.

        args:
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            lead_car_position: The [x, y] position of the lead vehicle.
                Lengths are in meters, and it is in the global frame.
        sets:
            self._follow_lead_vehicle: Boolean flag on whether the ego vehicle
                should follow (true) the lead car or not (false).
        """
        # TODO update later
        self._follow_lead_vehicle = False
        return

        # Check lead car position delta vector relative to heading, as well as
        # distance, to determine if car should be followed.
        # Check to see if lead vehicle is within range, and is ahead of us.
        if not self._follow_lead_vehicle:
            # Compute the angle between the normalized vector between the lead vehicle
            # and ego vehicle position with the ego vehicle's heading vector.
            lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                     lead_car_position[1] - ego_state[1]]
            lead_car_distance = np.linalg.norm(lead_car_delta_vector)
            # In this case, the car is too far away.   
            if lead_car_distance > self._follow_lead_vehicle_lookahead:
                return

            lead_car_delta_vector = np.divide(lead_car_delta_vector, 
                                              lead_car_distance)
            ego_heading_vector = [math.cos(ego_state[2]), 
                                  math.sin(ego_state[2])]
            # Check to see if the relative angle between the lead vehicle and the ego
            # vehicle lies within +/- 45 degrees of the ego vehicle's heading.
            if np.dot(lead_car_delta_vector, 
                      ego_heading_vector) < (1 / math.sqrt(2)):
                return

            self._follow_lead_vehicle = True

        else:
            lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                     lead_car_position[1] - ego_state[1]]
            lead_car_distance = np.linalg.norm(lead_car_delta_vector)

            # Add a 15m buffer to prevent oscillations for the distance check.
            if lead_car_distance < self._follow_lead_vehicle_lookahead + 15:
                return
            # Check to see if the lead vehicle is still within the ego vehicle's
            # frame of view.
            lead_car_delta_vector = np.divide(lead_car_delta_vector, lead_car_distance)
            ego_heading_vector = [math.cos(ego_state[2]), math.sin(ego_state[2])]
            if np.dot(lead_car_delta_vector, ego_heading_vector) > (1 / math.sqrt(2)):
                return

            self._follow_lead_vehicle = False

    def __str__(self):
        if self._state == FOLLOW_LANE:
            return "Behavior state: FOLLOW_LANE"
        if self._state == DECELERATE_TO_STOP:
            return "Behavior state: DECELERATE_TO_STOP"
        if self._state == STAY_STOPPED:
            return "Behavior state: STAY_STOPPED"
        if self._state == DECELERATE_TO_LIGHT:
            return "Behavior state: DECELERATE_TO_LIGHT"
        if self._state == STAY_STOPPED_UNTIL_GREEN:
            return "Behavior state: STAY_STOPPED_UNTIL_GREEN"


# Compute the waypoint index that is closest to the ego vehicle, and return
# it as well as the distance from the ego vehicle to that waypoint.
def get_closest_index(waypoints, ego_state):
    """Gets closest index a given list of waypoints to the vehicle position.

    args:
        waypoints: current waypoints to track. (global frame)
            length and speed in m and m/s.
            (includes speed to track at each x,y location.)
            format: [[x0, y0, v0],
                     [x1, y1, v1],
                     ...
                     [xn, yn, vn]]
            example:
                waypoints[2][1]: 
                returns the 3rd waypoint's y position

                waypoints[5]:
                returns [x5, y5, v5] (6th waypoint)
        ego_state: ego state vector for the vehicle. (global frame)
            format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                ego_x and ego_y     : position (m)
                ego_yaw             : top-down orientation [-pi to pi]
                ego_open_loop_speed : open loop speed (m/s)

    returns:
        [closest_len, closest_index]:
            closest_len: length (m) to the closest waypoint from the vehicle.
            closest_index: index of the waypoint which is closest to the vehicle.
                i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
    """
    closest_len = float('Inf')
    closest_index = 0

    for i in range(len(waypoints)):
        x_dist = waypoints[i][0] - ego_state[0]
        y_dist = waypoints[i][1] - ego_state[1]
        dist = np.linalg.norm(np.array([x_dist, y_dist]))
        if dist < closest_len:
            closest_len = dist
            closest_index = i

    return closest_len, closest_index

# Checks if p2 lies on segment p1-p3, if p1, p2, p3 are collinear.        
def pointOnSegment(p1, p2, p3):
    if (p2[0] <= max(p1[0], p3[0]) and (p2[0] >= min(p1[0], p3[0])) and \
       (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
        return True
    else:
        return False
