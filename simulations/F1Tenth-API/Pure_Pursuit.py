#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from environs import Env
import lgsvl
import time
import yaml
import numpy as np
from numba import njit
from argparse import Namespace
import math
import matplotlib.pyplot as plt

#######################################################################################
#####################    PLANNER HELPERS     ##########################################
#######################################################################################

njit(fastmath=False, cache=True)
def nearest_point_on_trajectory(point, trajectory):
    '''
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    '''
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1,:] + (t*diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

@njit(fastmath=False, cache=True)
def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
    ''' starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

    Assumes that the first segment passes within a single radius of the point

    http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
    '''
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    trajectory = np.ascontiguousarray(trajectory)
    for i in range(start_i, trajectory.shape[0]-1):
        start = trajectory[i,:]
        end = trajectory[i+1,:]+1e-6
        V = np.ascontiguousarray(end - start)

        a = np.dot(V,V)
        b = 2.0*np.dot(V, start - point)
        c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
        discriminant = b*b-4*a*c

        if discriminant < 0:
            continue
        #   print "NO INTERSECTION"
        # else:
        # if discriminant >= 0.0:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0*a)
        t2 = (-b + discriminant) / (2.0*a)
        if i == start_i:
            if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif t1 >= 0.0 and t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif t2 >= 0.0 and t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break
    # wrap around to the beginning of the trajectory if no intersection is found1
    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0],:]
            end = trajectory[(i+1) % trajectory.shape[0],:]+1e-6
            V = end - start

            a = np.dot(V,V)
            b = 2.0*np.dot(V, start - point)
            c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
            discriminant = b*b-4*a*c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0*a)
            t2 = (-b + discriminant) / (2.0*a)
            if t1 >= 0.0 and t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif t2 >= 0.0 and t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t

#@njit(fastmath=False, cache=True)
def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
    waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2]-position)
    speed = lookahead_point[2]
    if np.abs(waypoint_y) < 1e-6:
        return speed, 0.
    radius = 1/(2.0*waypoint_y/lookahead_distance**2)
    steering_angle = np.arctan(wheelbase/radius)
    return speed, steering_angle

@njit(fastmath=False, cache=True)
def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi
    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle

#######################################################################################
#####################    Pure Pursuit Planner    ######################################
#######################################################################################

class PurePursuitPlanner:
    """
    This is the PurePursuit ALgorithm that is traccking the desired path. In this case we are following the curvature
    optimal raceline.
    """
    def __init__(self, conf, wb):
        self.wheelbase = wb
        self.conf = conf
        self.load_waypoints(conf)
        self.max_reacquire = 20.

    def load_waypoints(self, conf):
        # Loading the x and y waypoints in the "..._raceline.vsv" that include the path to follow
        self.waypoints = np.loadtxt(conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip)

    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        # Find the current waypoint on the map and calculate the lookahead point for the controller
        wpts = np.vstack((self.waypoints[:, self.conf.wpt_xind], self.waypoints[:, self.conf.wpt_yind])).T
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)

        ###########################################
        #                    DEBUG
        ##########################################

        debugplot = 0
        if debugplot == 1:
            plt.cla()
            # plt.axis([-40, 2, -10, 10])
            plt.axis([position[0] - 10, position[0] + 8.5, position[1] - 3.5, position[1] + 3.5])
            plt.plot(self.waypoints[:, [1]], self.waypoints[:, [2]], linestyle='solid', linewidth=2, color='#005293')
            plt.plot(position[0], position[1], marker='o', color='green')
            plt.plot(nearest_point[0], nearest_point[1], marker='o', color='red')
            plt.pause(0.001)
            plt.axis('equal')

        ###########################################
        #                    DEBUG
        ###########################################

        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty((3, ))
            # x, y
            current_waypoint[0:2] = wpts[i2, :]
            # speed
            current_waypoint[2] = waypoints[i, self.conf.wpt_vind]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return np.append(wpts[i, :], waypoints[i, self.conf.wpt_vind])
        else:
            return None

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain):
        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(self.waypoints, lookahead_distance, position, pose_theta)

        if lookahead_point is None:
            return 4.0, 0.0

        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        speed = vgain * speed

        return speed, steering_angle


#######################################################################################
#####################    SVL Simulator Environment    #################################
#######################################################################################

# Create the environment variable
env = Env()

# set simhost and adehost in C:\Windows\System32\drivers\etc\hosts
SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", '192.168.1.41')
# SIMULATOR_BRIDGE = env.str("LGSVL__SIMULATOR_BRIDGE", 'adehost1')
SIMULATOR_BRIDGE = None
SIMULATOR_BRIDGE_PORT = env.str("LGSVL__SIMULATOR_BRIDGE_PORT", '9091')

if SIMULATOR_HOST is None:
    print("Set env LGSVL__SIMULATOR_BRIDGE")
    exit()
if SIMULATOR_BRIDGE is None:
    print("Set env LGSVL__SIMULATOR_HOST")

# Create the sim instance and connect to the SVL Simulator
sim = lgsvl.Simulator(SIMULATOR_HOST, env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
map_uuid = "781b04c8-43b4-431e-af55-1ae2b2efc873" #Red Bull
# map_uuid = "62765742-57bf-4ccd-85e5-db8295d34ead" #IMS
# map_uuid = "a3be7bf6-b5a6-4e48-833d-a1f1dd6d7a1e" #LVMS

# Load the Racetrack and create the scene/environment
if sim.current_scene == map_uuid:
    sim.reset()
else:
    sim.load(map_uuid)

spawns = sim.get_spawn()

# Load the EGO vehicle and spawn it on the track
state = lgsvl.AgentState()
state.transform = spawns[0]

ego_sensors_id = "3c0417f5-98d4-41ad-ad6d-10d2b0130f77" # Vehicle F1Tenth Config WITH sensors and bridge, run and enable ADE lgsvl_bridge host 

ego = sim.add_agent(name = ego_sensors_id, agent_type = lgsvl.AgentType.EGO, state = None)     # Vehicle F1Tenth Config WITHOUT sensors

if SIMULATOR_BRIDGE is not None:
    ego.connect_bridge(SIMULATOR_BRIDGE, int(SIMULATOR_BRIDGE_PORT)) #connect to LGSVL bridge
    print("Waiting for connection to ROS2 bridge ...")
    while not ego.bridge_connected:
        time.sleep(1)

# Load the Sensors of the vehicle
sensors = ego.get_sensors()

# Set a new daytime for the simulator, Time of day can be set from 0 ... 24
print("Current Day time in Simulation:", sim.time_of_day)
sim.set_time_of_day(11.8)


# The simulator can be run for a set amount of time.
# time_limit is optional and if omitted or set to 0, then the simulator will run indefinitely
# Create Steps for running the simulation step by step
step_time = 0.1                     # Simulation Timestep in s
duration = 100                      # Duration of Simulation in s
step_rate = int(1.0 / step_time)
steps = duration * step_rate        # Calculate total of time steps
print("Stepping forward for {} steps of {}s per step" .format(steps, step_time))


# Initial Ego Position - needs to be special because of our coordinate system
s = ego.state
s.position.x = -0.044086    # equals original x in our raceline data
s.position.z = 0.8491629    # equals original (-)y in our raceline data
s.rotation.y = 270-195      # 270 =- original value from raceline heading, if statement for pi and -pi
ego.state = s

# Create Planner object for Correct Planner class
with open('config_Spielberg_map.yaml') as file:
    conf_dict = yaml.load(file, Loader=yaml.FullLoader)
conf = Namespace(**conf_dict)
planner = PurePursuitPlanner(conf, 0.17145 + 0.15875)

# Create Controller object for the SVL Simulator
c = lgsvl.VehicleControl()

# Set Parameter for the Planner and the environment
lookahead_distance= 1.7
vgain = 1.05
lap_counter = 0

# Providing Time Measurements: Real Time and Simulation Time
t0 = time.time()
current_sim_time = sim.current_time

for i in range(steps):

    sim.run(time_limit=step_time)

    # Get the current Information from the vehicle State
    state = ego.state           # Create a state variable for the vehicle
    pos = state.position        # Get vehicle position: X,Y,Y
    rot= state.rotation         # Get vehicle rotation/heading: X,Y,Y
    speed = state.speed         # Get vehicle speed: X,Y,Y

    #Make correct transformation for the usage with the F1TENTH coordination
    pp_vehicle_x = -pos.x
    pp_vehicle_y = -pos.z
    pp_heading = np.deg2rad(270-rot.y)

    # Call the Pure Pursuit Planner
    pp_speed, pp_steer = planner.plan(pp_vehicle_x,pp_vehicle_y,pp_heading, lookahead_distance, vgain)

    # Match the steering angle from the pp calculations to the SVL Simulator steering
    # 30 Degree: max steering angle of F1TENTH car = 0.523599 rad
    pp_steer = -pp_steer/0.523599

    # Match Steering angle from the pp calculations to the max Steering of SVL Simulator: -1/1
    if pp_steer > 1:
        pp_steer = 1
    elif pp_steer < -1:
        pp_steer = -1

    # Match the steering angle from the pp calculations to the SVL Simulator steering
    # Throttle Position 1 = 7.07888 m/s
    pp_speed = pp_speed / 7.078882266

    # Create Control Commands for the SVL simulator and send steering and speed to SVL
    c.throttle = pp_speed
    c.steering = pp_steer

    # a True in apply_control means the control will be continuously applied ("sticky"). False means the control will be applied for 1 frame
    ego.apply_control(c, True)



t1 = time.time()
print("Real time elapsed =", t1 - t0)
print("Simulation time elapsed =", sim.current_time - current_sim_time)
