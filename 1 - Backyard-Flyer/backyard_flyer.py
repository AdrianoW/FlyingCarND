import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # set the current waypoints to 0 and define how precise it will be to change the waypoint (bigger=less precise)
        self.current_waypoint = -1
        self.delta = 0.7


    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # print("local_position_callback {}".format(self.local_position))
        if self.flight_state == States.WAYPOINT:
            
            diffX = abs(self.local_position[0] - self.all_waypoints[self.current_waypoint][0])
            diffY = abs(self.local_position[1] - self.all_waypoints[self.current_waypoint][1])
            diffZ = abs(-self.local_position[2] - self.all_waypoints[self.current_waypoint][2])
            # print( "{}".format(diffZ))



            # coordinate conversion 
            if ( (diffX < self.delta)  and (diffY < self.delta)  and (diffZ < self.delta) ):
                # print(self.current_waypoint)
                self.waypoint_transition()

            # # check if altitude is within 95% of target
            # if altitude > 0.95 * self.target_position[2]:
            #     self.landing_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # print("velocity_callback {}".format(self.local_velocity))
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.5) and
               abs(self.local_position[2]) < 0.09):
                self.disarming_transition()
        

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
                self.waypoint_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        side_size = 10
        waypoints = [
            [0 ,0, 3,0],
            [0 ,side_size, 3,0],
            [side_size ,side_size, 3,0],
            [side_size ,0, 3,0],
            [1 ,0, 3,0]
        ]
        return waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 2.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        
        # fix a bug that was making it to loose its marbles
        time.sleep(3)

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        if self.current_waypoint+1 < len(self.all_waypoints):
            new_pos = self.all_waypoints[self.current_waypoint+1]
            print("Moving to {}".format(new_pos))
            self.cmd_position(new_pos[0], new_pos[1], new_pos[2], new_pos[3]) 
            self.current_waypoint += 1

            self.flight_state = States.WAYPOINT
            
        else:
            self.landing_transition()

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
