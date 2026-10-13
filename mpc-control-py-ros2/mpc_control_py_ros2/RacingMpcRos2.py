import warnings

import ament_index_python
import rclpy.node
from rclpy.time import Time

import numpy as np

from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Pose2D
from lar_msgs.msg import CarStateStamped
from lar_msgs.msg import CarControlStamped
from mpc_msgs_racing_ros2.msg import CarTrajectoryWithTrackingStamped, CarStateWithTracking, CarControlWithTracking
from mpc_msgs_racing_ros2.msg import RacingMpcDiagnosticsStamped
from mpc_control_py_ros2.utils_racing.PathUtils import PathLocator


class RacingMpcRos2(rclpy.node.Node):

    def __init__(self, node_name, optimizer):
        super().__init__(node_name)

        # "Public"
        self.is_planner = False

        self.T_opt = np.empty((1, 0))

        self.pos_opt = np.empty((2, 0))
        self.vel_opt = np.empty((3, 0))
        self.angle_opt = np.empty((1, 0))
        self.path_opt = np.empty((1, 0))
        self.pathvel_opt = np.empty((1, 0))

        self.steering_opt = np.empty((1, 0))
        self.throttle_opt = np.empty((1, 0))

        self.tracklocator = PathLocator()

        # "Protected"
        self.deb_node_start_time = self.get_clock().now()

        # Specific properties #
        self.planner = None
        self.optimizer = optimizer
        self.computation_duration_us = 0

        # Messages, callbacks, publish -------------------------------
        class Subscribers:
            state = None
            track = None
        self.sub = Subscribers()

        self.state_tc = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.capture_time = self.get_clock().now()

        self.track_isAvail = False
        self.track_pos = np.array([0, 0])
        self.track_angle = 0.0

        class Messages:
            control = CarControlStamped()
            traj = CarTrajectoryWithTrackingStamped()
            diag = RacingMpcDiagnosticsStamped()
        self.msg = Messages()

        class Publisher:
            carControl = None
            carTraj = None
            diagnostics = None
        self.pub = Publisher()

        # Constructor work, like in RacingMpcRos2()
        self.init_topics()

        self.declare_parameter('node_rate', 1000.0)
        self.p_rate = self.get_parameter('node_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1 / self.p_rate, self.timer_callback)

        self.pub_time = self.get_clock().now()

        default_track_yaml = (ament_index_python.get_package_share_directory('lar_utils') +
                              "/config/track/sampled/la_track_sampled.yaml")
        self.declare_parameter('track_yaml', default_track_yaml)
        track_yaml_filepath = self.get_parameter('track_yaml').get_parameter_value().string_value
        if track_yaml_filepath:
            self.optimizer.track.load_from_yaml(track_yaml_filepath)
            if self.planner: self.planner.track.load_from_yaml(track_yaml_filepath)

        self.declare_parameter('params_yaml', '')
        param_yaml_filepath = self.get_parameter('params_yaml').get_parameter_value().string_value
        if param_yaml_filepath:
            self.optimizer.load_params_from_yaml(param_yaml_filepath)
            if self.planner: self.planner.load_params_from_yaml(param_yaml_filepath)

        self.declare_parameter('delay_comp_mode', 0)
        self.optimizer.delay_comp_mode = self.get_parameter('delay_comp_mode').get_parameter_value().integer_value
    def init_topics(self):
        self.declare_parameter('state_source', 'sim')
        if self.get_parameter('state_source').get_parameter_value().string_value == 'sim':
            print('(State: ROS2 interface to SIMULATOR)')
            self.sub.state = self.create_subscription(CarStateStamped, '/sim/car/state', self.state_callback, 1)
            self.sub.track = self.create_subscription(Pose2D, '/sim/track/pose2d', self.track_callback, 1)
        else:
            print('(State: ROS2 interface to LAR_SYSTEM)')
            self.sub.state = self.create_subscription(CarStateStamped, '/car/state', self.state_callback, 1)
            self.sub.track = self.create_subscription(Pose2D, '/mocap/track/pose2d', self.track_callback, 1)

        self.pub.carControl = self.create_publisher(CarControlStamped, '/mpc/control', 1)
        self.pub.carTraj = self.create_publisher(CarTrajectoryWithTrackingStamped, '/mpc/trajectory', 1)
        self.pub.diagnostics = self.create_publisher(RacingMpcDiagnosticsStamped, '/mpc/diagnostics', 1)

    def run(self):
        if self.optimizer.init():
            print('Optimizer initialization OK')
        else:
            print('Optimizer initialization failure')

        if not self.optimizer.check_validity:
            print('Validity check OFF')

        rclpy.spin(self)

    def run_once(self):

        state_max_uses = 10

        if self.optimizer.state.used_counter < state_max_uses:
            self.optimizer.state.used_counter += 1

            self.capture_time = self.state_tc

            # Run the planner, if it exists #
            if self.planner:
                warnings.warn("Planner not implemented yet")

            if self.optimizer.configParameters.needs_update:
                self.optimizer.update_config_params()
            if self.optimizer.tuningParameters.needs_update:
                self.optimizer.update_tuning_params()

            # Run the optimizer #
            self.optimizer.set_initial_state(self.optimizer.compose_state())

            np.set_printoptions(precision=3, suppress=True)
            print(f"\nFrom ({self.node_sec(self.state_tc):.3f}s) x0: {self.optimizer.x_comp.T}")
            np.set_printoptions()

            t0 = self.get_clock().now()
            self.optimizer.compute_control()
            self.computation_duration_us = (self.get_clock().now() - t0).nanoseconds / 1000
            if self.computation_duration_us > 1e6:
                self.computation_duration_us = 1

            # Evaluate trajectory and publish #
            self.extract_solution()

            # Assess whether trajectory is inside track boundaries
            # Find closest centerline path parameters (not necessarily optimal planned ones)
            # print("\n\npath_opt: ", self.path_opt)
            t1 = self.get_clock().now()
            if False:
                pos_opt_pathparam = np.zeros(self.pos_opt.shape[1],)
                for i in range(pos_opt_pathparam.size):
                    pos_opt_pathparam[i] = self.tracklocator.find_closest_path_param(self.optimizer.track, self.pos_opt[:, i], self.path_opt[i], range_=0.1)
                traj_valid = self.optimizer.assess_trajectory(self.pos_opt, self.optimizer.track, pos_opt_pathparam)
            else:
                traj_valid = True

            gs_duration_us = (self.get_clock().now() - t1).nanoseconds / 1000

            valid_ = "OK" if traj_valid else "Invalid"
            action_ = "(reset)" if (self.optimizer.check_validity and not traj_valid) else "(publish)"
            print(f" ({self.computation_duration_us / 1000.0:.1f}ms)"
                  f", gs: ({gs_duration_us / 1000.0:.1f}ms)"
                  f" {valid_} {action_}")

            self.pub_time = self.get_clock().now()
            if self.optimizer.check_validity and not traj_valid: self.optimizer.reset()
            else: self.publish_control()

            self.publish_trajectory()
            self.publish_diagnostics()

    def extract_solution(self):
        self.T_opt = self.optimizer.get_time_grid()

        X_opt = self.optimizer.get_state_trajectory()
        U_opt = self.optimizer.get_control_trajectory()

        # if not self.is_planner:
        #     print("")

        pos_from_config = self.is_planner
        path_from_config = self.is_planner or (not self.is_planner and self.optimizer.state_idx.path_param == -1)

        if pos_from_config:
            pos_opt = self.optimizer.configParameters.raceline_refs.T

            angle_opt = np.empty(X_opt.shape[1])
            #todo
        else:
            self.pos_opt = np.zeros((2, X_opt.shape[1]))  # resize
            self.pos_opt[0, :] = X_opt[self.optimizer.state_idx.pos_x, :]
            self.pos_opt[1, :] = X_opt[self.optimizer.state_idx.pos_y, :]
            self.angle_opt = X_opt[self.optimizer.state_idx.turn_angle]

        if path_from_config:
            #todo
            print("Not implemented")
        else:
            self.path_opt = X_opt[self.optimizer.state_idx.path_param, :]
            self.pathvel_opt = X_opt[self.optimizer.state_idx.path_param_vel, :]

        if (self.optimizer.state_idx.vel_x > 0 and
            self.optimizer.state_idx.vel_y > 0 and
            self.optimizer.state_idx.turn_rate > 0):
            #todo
            pass

        self.steering_opt = U_opt[0, :]
        self.throttle_opt = U_opt[1, :]

    # "Protected"
    def node_sec(self, time):
        return (time - self.deb_node_start_time).nanoseconds / 1_000_000_000
    def node_msec(self, time):
        return (time - self.deb_node_start_time).nanoseconds / 1_000_000

    def timer_callback(self):
        self.run_once()
        return

    # Messages, callbacks, publish ------------------------------- #
    def track_callback(self, msg):

        self.track_pos = np.array([msg.x, msg.y])
        self.track_angle = msg.theta
        self.track_isAvail = True
        return
    def state_callback(self, msg):

        if Time.from_msg(msg.header.stamp) > self.state_tc:

            # print(f"sim pos: [{msg.pos_x:.4f}, {msg.pos_y:.4f}, {msg.turn_angle:.4f}, w_({msg.vel_x:.4f}, {msg.vel_y:.4f}, {msg.turn_rate:.4f})]")

            pos_global = np.array([msg.pos_x, msg.pos_y])
            vel_global = np.array([msg.vel_x, msg.vel_y])
            turn_angle_global = msg.turn_angle
            turn_rate = msg.turn_rate

            if not self.track_isAvail: return

            T_track_global = np.array([[np.cos(self.track_angle), np.sin(self.track_angle)],
                                       [-np.sin(self.track_angle), np.cos(self.track_angle)]])
            pos = T_track_global @ (pos_global - self.track_pos)
            vel = T_track_global @ vel_global
            turn_angle = turn_angle_global - self.track_angle

            # Normalize turn_angle
            while rclpy.ok() and turn_angle > np.pi: turn_angle -= 2.0 * np.pi
            while rclpy.ok() and turn_angle < -np.pi: turn_angle += 2.0 * np.pi

            self.optimizer.state.pos = pos
            self.optimizer.state.vel = vel
            self.optimizer.state.turn_angle = turn_angle
            self.optimizer.state.turn_rate = turn_rate

            self.state_tc = Time.from_msg(msg.header.stamp)
            self.optimizer.state.used_counter = 0
        return
    def publish_control(self):

        if self.is_planner:
            return

        u0 = self.optimizer.get_control()

        self.msg.control.header.stamp = self.pub_time.to_msg()
        self.msg.control.steering = u0[0]
        self.msg.control.throttle = u0[1]

        self.pub.carControl.publish(self.msg.control)
        return
    def publish_trajectory(self):

        self.msg.traj.header.stamp = self.pub_time.to_msg()
        self.msg.traj.capture_time = self.capture_time.to_msg()
        self.msg.traj.time = [float(t) for t in self.T_opt]

        # Transform position and angle back to global #
        T_global_local = np.array([[np.cos(-self.track_angle), np.sin(-self.track_angle)],
                                   [-np.sin(-self.track_angle), np.cos(-self.track_angle)]])

        pos_opt_global = (T_global_local @ self.pos_opt) + self.track_pos.reshape(2, 1)
        angle_opt_global = self.angle_opt + self.track_angle

        self.msg.traj.state = []
        for i in range(self.pos_opt.shape[1]):
            s = CarStateWithTracking()
            s.pos_x = pos_opt_global[0, i]
            s.pos_y = pos_opt_global[1, i]
            s.turn_angle = angle_opt_global[i]
            s.path_param = self.path_opt[i]
            s.path_param_vel = self.pathvel_opt[i]

            if self.vel_opt.shape[1] > 0:
                s.vel_x = self.vel_opt[0, i]
                s.vel_y = self.vel_opt[1, i]
                s.turn_rate = self.vel_opt[2, i]

            self.msg.traj.state.append(s)

        self.msg.traj.control = []
        if self.is_planner:
            track_length = self.optimizer.track.get_info().length
            for i in range(self.throttle_opt.size):
                c = CarControlWithTracking()
                c.throttle = self.throttle_opt[i] * track_length / 8.6
                self.msg.traj.control.append(c)
        else:
            for i in range(self.throttle_opt.size):
                c = CarControlWithTracking()
                c.steering = self.steering_opt[i]
                c.throttle = self.throttle_opt[i]
                self.msg.traj.control.append(c)

        self.pub.carTraj.publish(self.msg.traj)

        return

    def publish_diagnostics(self):

        self.msg.diag.header.stamp = self.pub_time.to_msg()
        self.msg.diag.computation_time_ms = self.computation_duration_us / 1000.0
        self.msg.diag.cost = self.optimizer.get_cost()
        self.msg.diag.iter = self.optimizer.get_no_iterations()
        self.msg.diag.iter_low = self.optimizer.get_no_iterations_low()
        self.pub.diagnostics.publish(self.msg.diag)
        return
