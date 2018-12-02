import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import (
    a_star, 
    create_grid, 
    get_gridrelative_position, 
    get_latlon_fromfile,
    heuristic, 
    prune_path
)
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_longitude, goal_latitude):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        #goal coordinates
        self.goal_longitude = goal_longitude
        self.goal_latitude = goal_latitude

        print("Longitude/Latitude (from args): {}, {}".format(self.goal_longitude, self.goal_latitude))

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
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
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        
        if len(self.waypoints) != 0:
            self.target_position = self.waypoints.pop(0)
            print('target position', self.target_position)
            self.cmd_position(self.target_position[0], 
                              self.target_position[1], 
                              self.target_position[2], 
                              self.target_position[3])
        else:
            print("No waypoints found.")

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
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
        SAFETY_DISTANCE = 8 # extra margin of safety around obstacles

        self.target_position[2] = TARGET_ALTITUDE

        # debug flag
        debug = True
        test_run = False

        # Requirement #1: 
        # * Read lat0, lon0 from colliders into floating point values
        # * set home position to (lon0, lat0, 0)
        lat0, lon0 = get_latlon_fromfile('colliders.csv')
        self.set_home_position(lon0, lat0, 0)
        
        # Requirement #2: 
        # * retrieve current global position
        # * convert to current local position using global_to_local() 
        curr_global_pos = (self._longitude, self._latitude, self._altitude)
        curr_global_home = self.global_home
        curr_local_pos = global_to_local(curr_global_pos, self.global_home)
        
        if debug:
            print()
            print("curr_global_pos: ", curr_global_pos)
            print("curr_global_home: ", curr_global_home)
            print("curr_local_pos: ", curr_local_pos)
        
        #print('global home {0}\nposition {1}\nlocal position {2}'.format(self.global_home, self.global_position,
                                                                         #self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        # grid_start_test = (-north_offset, -east_offset)
        # print("grid_start (orig): ", grid_start_test)

        # save offsets as tuple
        offsets = (north_offset, east_offset)
        print("offsets: ", offsets)
        
        # Requirement #3: convert start position to current position rather than map center
        grid_start = get_gridrelative_position(curr_local_pos[0:2], offsets)
        print("grid_start: ", grid_start)

        # Set goal as some arbitrary position on the grid
        # grid_goal_test = (-north_offset + 10, -east_offset + 10)
        # print("grid_goal (orig): ", grid_goal_test)

        # Requirement #4: set goal as latitude/longitude
        # specified as command line args
        custom_goal_pos = (self.goal_longitude, self.goal_latitude, 0)
        goal_local = global_to_local(custom_goal_pos, self.global_home)[0:2]
        grid_goal = get_gridrelative_position(goal_local, offsets)
        print("grid_goal: ", grid_goal)
        
        """
        print("is test run?: ", test_run)
        if test_run:
            # grid_start = grid_start_test
            grid_goal = grid_goal_test
            print("test run (start, end): ", (grid_start, grid_goal))
        """

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('START: Path planning....')
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Path planning complete!')
        
        # Requirement 6: prune path to minimize number of waypoints
        path = prune_path(path)

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        
        if debug:
            print("waypoints:")
            for i, waypoint in enumerate(waypoints):
                print("{}: {}".format(i, waypoint))
        
        # Set self.waypoints
        self.waypoints = waypoints
        
        # Send waypoints to sim (this is just for visualization of waypoints)
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
    # custom args
    parser.add_argument('--goal_longitude', type=float, default=-122.398414, help='Goal Longitude')
    parser.add_argument('--goal_latitude', type=float, default=37.7939265, help='Goal Latitude')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, args.goal_longitude, args.goal_latitude)
    time.sleep(1)

    drone.start()
