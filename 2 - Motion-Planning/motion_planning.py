import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, closest_point
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from prunning import prune_path

from graph import create_graph, a_star_graph
from sampler import Sampler

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class Planners(Enum):
    GRID = 1
    SAMPLE = 2
    SAMPLE_MARGIN = 3
    MEDIAL_AXIS = 4
    

class MotionPlanning(Drone):

    def __init__(self, connection, planner, goal):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.planner = planner
        self.goal = goal

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            # Checks if the takeoff has reached the desired height with a small error margin
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # check how far we are from the target position
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                # if there is still a waypoint to go, go
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    # all the waypoint are gone. Land
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        # check if the drone has reached the floor, with a small error margin and diasrm if so
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        # call the appropriate function accorrding to its state
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        # arm the drone to start the flight
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        # take off until the height of the target_position
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        # take the next position from the queue and move to the new location
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        # land the drone
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        # disarm the drone
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        # set the drone to manual state
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values 
        # read the first line and extract only the latitude and longitude vals
        with open('colliders.csv', 'r') as f:
            temp = f.readline()
            lat, lon = temp.replace('lat0 ', '').replace('lon0', '').split(', ')
            lat, lon = float(lat), float(lon)
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon, lat, 0)

        # TODO: retrieve current global position
        curr_global_pos = self.global_position
 
        # TODO: convert to current local position using global_to_local()
        curr_local_pos = global_to_local(curr_global_pos, self.global_home)
        print("Current Local position {}".format(curr_local_pos))
        
        # start_local = int(curr_local_pos[0]), int(curr_local_pos[1]) #int(start_local[0]), int(start_local[1])
        # print(start_local, end_local)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # # Define starting point on the grid (this is just grid center)
        # grid_start = start_local
        # # TODO: convert start position to current position rather than map center
        grid_start = (int(curr_local_pos[0]-north_offset), int(curr_local_pos[1]-east_offset))
        
        # # Set goal as some arbitrary position on the grid
        # grid_goal = end_local
        # TODO: adapt to set goal as latitude / longitude position and convert
        end_geo = self.goal
        end_local = global_to_local(end_geo, self.global_home)
        grid_goal = end_local[0]-north_offset, end_local[1]-east_offset
        grid_goal = int(grid_goal[0]), int(grid_goal[1])

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Grid Start and Goal: ', grid_start, grid_goal)

        # TODO (if you're feeling ambitious): Try a different approach altogether!
        if self.planner == 1:
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)

            # TODO: prune path to minimize number of waypoints
            path = prune_path(path)
        else:
            if self.planner == 2:
                sampler = Sampler(data)
                polygons = sampler._polygons
                # Example: sampling 100 points and removing
                # ones conflicting with obstacles.
                nodes = sampler.sample(300)
                print(nodes[0])

            elif self.planner == 3:
                pass

            elif self.planner == 4:    
                pass

            #create the graph and calculate the a_star
            t0 = time.time()
            print('building graph ... ', )
            g = create_graph(nodes, 10, polygons)
            print('graph took {0} seconds to build'.format(time.time()-t0))
            start = closest_point(g, (grid_start[0], grid_start[1], TARGET_ALTITUDE))
            goal = closest_point(g, (grid_goal[0], grid_goal[1], TARGET_ALTITUDE))
            print(start, goal)
            

            # print(start, start_ne)
            print('finding path ... ', )
            path, cost = a_star_graph(g, heuristic, start, goal)
            print('done. path size and cost: {}'.format((len(path), cost)))
            # print(len(path), path)
            # path_pairs = zip(path[:-1], path[1:])

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--planner', type=int, default=1, help="Choose the planner")
    parser.add_argument('--goal', type=float, nargs='+', default=( -122.401843, 37.791277, 53), help="Choose the planner")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, args.planner, tuple(args.goal))
    time.sleep(1)

    drone.start()
