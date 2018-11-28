import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils_diag import a_star, heuristic, create_grid
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

    def __init__(self, connection):
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

        # save orig values
        orig_global_pos = self.global_position
        orig_global_home = self.global_home
        orig_local_pos = global_to_local(self.global_position, self.global_home)
        
        # 3.1: 
        # * Read lat0, lon0 from colliders into floating point values
        # * set home position to (lon0, lat0, 0)
        lat0, lon0 = self.get_latlon_fromfile('colliders.csv')
        self.set_home_position(lon0, lat0, 0)
        
        # 3.2: 
        # * retrieve current global position
        # * convert to current local position using global_to_local()
        
        # orig: working
        # curr_global_pos0 = self.global_position
        # curr_global_home0 = self.global_home
        # curr_local_pos0 = global_to_local(curr_global_pos0, curr_global_home0)
        
        # new 11/24/18 4:35 PM: 
        curr_global_pos = (self._longitude, self._latitude, self._altitude)
        curr_global_home = self.global_home
        curr_local_pos = global_to_local(curr_global_pos, self.global_home)
        
        if debug:
            print()
            print("curr_global_pos0: ", curr_global_pos0)
            print("curr_global_pos: ", curr_global_pos)

            print()
            print("curr_global_home0: ", curr_global_home0)
            print("curr_global_home: ", curr_global_home)

            print()
            print("curr_local_pos0: ", curr_local_pos0)
            print("curr_local_pos: ", curr_local_pos)
        
        
        if debug:
            print()
            print("orig_global_home: ", orig_global_home)
            print("new_global_home: ", curr_global_home)
            print("are equal: ", orig_global_home==curr_global_home)
            
            print()
            print("orig_global_pos: ", orig_global_pos)
            print("curr_global_pos: ", curr_global_pos)
            print("are equal: ", orig_global_pos==curr_global_pos)            
            
            print()
            print("orig_local_pos: ", orig_local_pos)
            print("curr_local_pos: ", curr_local_pos)
            print("offset (n, e): ", curr_local_pos[0]-orig_local_pos[0], curr_local_pos[1]-orig_local_pos[1])
        
        #print('global home {0}\nposition {1}\nlocal position {2}'.format(self.global_home, self.global_position,
                                                                         #self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid (this is just grid center)
        grid_start_test = (-north_offset, -east_offset)
        print("grid_start (orig): ", grid_start_test)

        # save offsets as tuple
        offsets = (north_offset, east_offset)
        print("offsets: ", offsets)

        # use old for now
        # curr_local_pos = orig_local_pos
        
        # 3.3: convert start position to current position rather than map center
        grid_start = self.get_gridrelative_position(curr_local_pos[0:2], offsets)
        print("grid_start: ", grid_start)

        # Set goal as some arbitrary position on the grid
        grid_goal_test = (-north_offset + 10, -east_offset + 10)
        print("grid_goal (orig): ", grid_goal_test)

        # 3.4: adapt to set goal as latitude / longitude position and convert
        # set goal position as some desired position on the map
        custom_goal_pos = (-122.398414, 37.7939265, 0)
        goal_local = global_to_local(custom_goal_pos, self.global_home)[0:2]
        grid_goal = self.get_gridrelative_position(goal_local, offsets)
        print("grid_goal: ", grid_goal)
        
        print("is test run?: ", test_run)
        if test_run:
            # grid_start = grid_start_test
            grid_goal = grid_goal_test
            print("test run (start, end): ", (grid_start, grid_goal))

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('START: Path planning....')
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Path planning complete!')
        
        # TODO: prune path to minimize number of waypoints
        orig_path = path[:]
        path = self.prune_path(path)
        print("path length (pre-prune): ", len(orig_path))
        print("path length (post-prune): ", len(path))

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        
        if debug:
            print("waypoints:")
            for i, waypoint in enumerate(waypoints):
                print("{}: {}".format(i, waypoint))
        
        # Set self.waypoints
        self.waypoints = waypoints
        
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def prune_path(self, path):
        """
        Perform collinearily check on each set of 3 points and
        determine those points that are collinear.
        """
        pruned_path = [p for p in path]
        
        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            
            if self.collinearity_test(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
                
        return pruned_path

    def point(self, p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_test(self, p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def get_gridrelative_position(self, local_position, offsets):
        local_pos_north, local_pos_east = int(local_position[0]), int(local_position[1])
        north_offset, east_offset = offsets
        return (local_pos_north - north_offset, local_pos_east - east_offset)

    def get_latlon_fromfile(self, input_file):
        """Get latitude/longitude data from first line of input file."""
        with open(input_file) as f:
            parts = f.readline().replace(',', '').split()
            lat = float(parts[1])
            lon = float(parts[3])

            return lat, lon

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
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
