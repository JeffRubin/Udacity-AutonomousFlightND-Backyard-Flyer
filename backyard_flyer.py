import argparse
import time
import visdom
from enum import Enum

import numpy as np
import copy

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

    def __init__(self, connection, alt, side, plot):
        super().__init__(connection)
        self.alt = alt
        self.side = side
        self.plot = plot
        self.pos_plot_update_count = 0
        self.vel_plot_update_count = 0
        self.plot_update_rate = 15
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box()
        self.cur_waypoint = -1
        self.in_mission = True

        # initial state
        self.flight_state = States.MANUAL

        # plotting
        if plot:
            # default opens up to http://localhost:8097
            self.v = visdom.Visdom()
            assert self.v.check_connection()

            # Plot XY Position (NE)
            ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
            self.ne_plot = self.v.scatter(ne, opts=dict(
                title="Local position (north, east)",
                xlabel='North (m)',
                ylabel='East (m)'
            ))

            # Plot Altitude (D)
            d = np.array([self.local_position[2]])
            self.t_d = 0
            self.d_plot = self.v.line(d, X=np.array([self.t_d]), opts=dict(
                title="Altitude",
                xlabel='Timestep',
                ylabel='Down (m)'
            ))

            # Plot Horizontal Speed
            h_speed = np.array([np.sqrt(self.local_velocity[0]**2 + self.local_velocity[1]**2)])
            self.t_h_speed = 0
            self.h_speed_plot = self.v.line(h_speed, X=np.array([self.t_h_speed]), opts=dict(
                title="Hoizontal Speed",
                xlabel='Timestep',
                ylabel='Hoizontal Speed (m/s)'
            ))

            # Plot Veritcal Speed
            v_speed = np.array([abs(self.local_velocity[2])])
            self.t_v_speed = 0
            self.v_speed_plot = self.v.line(v_speed, X=np.array([self.t_v_speed]), opts=dict(
                title="Vertical Speed",
                xlabel='Timestep',
                ylabel='Verical Speed (m/s)'
            ))

        # Register callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def dist_to_target(self):
        # calculate 3D Euclidean distance to the target
        # ignore the last element of the target position since that is heading and reverse the altitutde sign
        cur_target_position = copy.deepcopy(self.target_position[:-1])
        cur_target_position[2] = -cur_target_position[2]
        dist = np.sqrt(sum((self.local_position - cur_target_position)**2))
        return dist

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                # set the initial waypoint
                self.cur_waypoint = 0
                if self.cur_waypoint < len(self.all_waypoints):
                    self.waypoint_transition()
                else:
                    self.landing_transition()
        elif self.flight_state == States.WAYPOINT:
            dist_to_target = self.dist_to_target()
            if dist_to_target < 0.25:
                self.cur_waypoint += 1
                if self.cur_waypoint < len(self.all_waypoints):
                    self.waypoint_transition()
                else:
                    self.landing_transition()
        elif self.flight_state == States.LANDING:
            if ((abs(self.global_position[2] - self.global_home[2]) < 0.1) and
                    abs(self.local_position[2]) < 0.1):
                self.disarming_transition()

        # plotting
        if self.plot:
            self.pos_plot_update_count += 1
            # reduce the number of datapoint in the plots (reduces processing burden)
            if self.pos_plot_update_count % self.plot_update_rate == 0:
                self.pos_plot_update_count = 0
                self.update_ne_plot()
                self.update_d_plot()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # plotting
        if self.plot:
            self.vel_plot_update_count += 1
            # reduce the number of datapoint in the plots (reduces processing burden)
            if self.vel_plot_update_count % self.plot_update_rate == 0:
                self.vel_plot_update_count = 0
                self.update_h_speed_plot()
                self.update_v_speed_plot()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """
        1. Return waypoints to fly a box (as needed for cmd_position where altitude is reversed sign than NED,
        i.e. + is up rather than - is up as in NED)
        """
        waypoints = list()
        waypoints.append(np.array([self.side, 0, self.alt, 0]))
        waypoints.append(np.array([self.side, self.side, self.alt, np.pi/2]))
        waypoints.append(np.array([0, self.side, self.alt, np.pi]))
        waypoints.append(np.array([0, 0, self.alt, -np.pi/2]))
        # reorient before landing
        waypoints.append(np.array([0, 0, self.alt/2, 0]))
        return waypoints

    def arming_transition(self):
        """
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
        """
        1. Set target_position altitude (+ is up)
        2. Command a takeoff to target_position altitude
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = self.alt
        self.takeoff(self.alt)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        cur_waypoint_pos = self.all_waypoints[self.cur_waypoint]
        self.target_position = cur_waypoint_pos
        self.cmd_position(*cur_waypoint_pos)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
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

    #plotting
    
    def update_ne_plot(self):
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        d = np.array([self.local_position[2]])
        # update timestep
        self.t_d += 1
        self.v.line(d, X=np.array([self.t_d]), win=self.d_plot, update='append')

    def update_h_speed_plot(self):
        h_speed = np.array([np.sqrt(self.local_velocity[0]**2 + self.local_velocity[1]**2)])
        # update timestep
        self.t_h_speed += 1
        self.v.line(h_speed, X=np.array([self.t_h_speed]), win=self.h_speed_plot, update='append')

    def update_v_speed_plot(self):
        v_speed = np.array([self.local_velocity[2]])
        # update timestep
        self.t_v_speed += 1
        self.v.line(v_speed, X=np.array([self.t_v_speed]), win=self.v_speed_plot, update='append')

    def start(self):
        """
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
    parser.add_argument('--alt', type=float, default=10.0, help="altitude (meters) at which to fly the square; up is +")
    parser.add_argument('--side', type=float, default=20.0, help="size (meters) of each side of the square to fly")
    parser.add_argument('--plot', type=bool, default=False, help="turn on plotting (True/False) at \
        http://localhost:8097 (only set True if your computer has a lot of resources otherwise it will slow everything \
        down a lot...like it does on my computer); make sure to start the following first if plotting is set to True:\
        python -m visdom.server")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn, args.alt, args.side, args.plot)
    time.sleep(2)
    drone.start()
