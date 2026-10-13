import ament_index_python
import rclpy
import yaml
from mpc_control_py_ros2.RacingMpcRos2 import RacingMpcRos2
from mpc_control_py_ros2.mpc_control_racing.spline_following_tire.optimizer.SplineFollowingTire_MAMH_Optimizer import SplineFollowingTire_MAMH_Optimizer

import json
import socket
import struct

def send_message(conn, msg: dict):
    data = json.dumps(msg).encode()
    conn.sendall(struct.pack("I", len(data)))
    conn.sendall(data)

def recv_message(conn) -> dict | None:
    raw_len = _recv_exact(conn, 4)
    if not raw_len:
        return None
    (length,) = struct.unpack("I", raw_len)
    return json.loads(_recv_exact(conn, length).decode())

def _recv_exact(conn, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf

def run_socket_interface(optimizer):
    """ ==== Define and launch socket for communication with C++ node ==== """
    SOCKET_PATH = "/tmp/cpp_python_poc.sock"

    # --- handlers ---
    def handle_call(request: dict) -> dict:
        op = request["operation"]

        if op == "init":
            optimizer.init()
            results = 0

        elif op == "compose_state":
            values = request["values"]
            optimizer.state.pos = values[0:2]
            optimizer.state.turn_angle = values[2]
            optimizer.state.vel = values[3:5]
            optimizer.state.turn_rate = values[5]
            x_comp = optimizer.compose_state()
            optimizer.set_initial_state(x_comp)
            results = x_comp.tolist()

        elif op == "solve":
            optimizer.compute_control()
            solve_succeeded = (optimizer.status in [0, 2])
            QP_solver_failed = (optimizer.status == 4)
            results = [optimizer.comp_time_us, solve_succeeded, QP_solver_failed]

        else:
            return {"status": "error", "message": f"unknown operation: {op}"}

        return {"status": "ok", "operation": op, "results": results}

    def handle_set(request: dict) -> dict:
        op = request["operation"]
        values = request["values"]

        if op == "max_iter":
            optimizer.solver_max_iter = values
            results = 0
        elif op == "set_tol":
            optimizer.solver_tol = values
            results = 0
        else:
            return {"status": "error", "message": f"unknown operation: {op}"}

        return {"status": "ok", "operation": op, "results": results}

    def handle_get(request: dict) -> dict:
        op = request["operation"]

        if op == "control_traj":
            Uopt = optimizer.get_control_trajectory()[:, 0:2]
            results = Uopt.tolist()
        elif op == "cost":
            results = optimizer.get_cost()
        elif op == "iter":
            results = optimizer.get_no_iterations()
        elif op == "iter_low":
            results = optimizer.get_no_iterations_low()
        else:
            return {"status": "error", "message": f"unknown operation: {op}"}

        return {"status": "ok", "operation": op, "results": results}

    HANDLERS = {
        "call" : handle_call,
        "set" : handle_set,
        "get" : handle_get,
    }

    def dispatch(request: dict) -> dict:
        msg_type = request.get("type", "")
        handler = HANDLERS.get(msg_type)
        if handler is None:
            return {"status": "error", "message": f"unknown type: {msg_type}"}
        return handler(request)

    ''' ==== Launch socket ==== '''
    conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    while True:
        try:
            conn.connect(SOCKET_PATH)
            print("[Socket] Connected to C++ client.")
            break
        except Exception as e:
            # Try again later
            import time
            time.sleep(0.5)

    while True:
        request = recv_message(conn)
        if request is None or request.get("type") == "shutdown":
            print("[Socket] Shutting down.")
            break

        # print(f"[Socket] Received: {request}")
        response = dispatch(request)
        # print(f"[Socket] Sending:  {response}")
        send_message(conn, response)

    conn.close()
    print("[Socket] Disconnected.")


def main(args=None):

    rclpy.init(args=args)

    node_name = "spline_following_tire_mpc_MAMH"

    optimizer = SplineFollowingTire_MAMH_Optimizer()
    node = RacingMpcRos2(node_name, optimizer)

    # Node-specific default config paths #
    if not node.get_parameter('params_yaml').get_parameter_value().string_value:
        node.set_parameters([
            rclpy.Parameter('params_yaml', rclpy.Parameter.Type.STRING,
                            ament_index_python.get_package_share_directory('mpc_control_racing_ros2') + "/config/spline_following_tire_params.yaml")
        ])

    params_yaml = node.get_parameter('params_yaml').get_parameter_value().string_value
    track_default_yaml = ament_index_python.get_package_share_directory("lar_utils") + "/config/track/sampled/la_track_sampled.yaml"

    # Spline for optimization
    with open(params_yaml, "r") as f:
        yamlNode = yaml.safe_load(f)
        raceline_default_yaml = ament_index_python.get_package_share_directory('mpc_control_racing_ros2') + "/config/" + yamlNode["raceline_name"]
        node.declare_parameter('raceline_yaml', raceline_default_yaml)
        spline_yaml = node.get_parameter('raceline_yaml').get_parameter_value().string_value

    rclpy.shutdown() # Only need ros for obtaining config file paths

    # optimizer.set_acados_json('acados_ocp.json')

    while True:
        optimizer = SplineFollowingTire_MAMH_Optimizer()
        optimizer.check_validity = False
        optimizer.load_params_from_yaml(params_yaml)
        optimizer.track.load_from_yaml(track_default_yaml)
        optimizer.load_spline_from_yaml(spline_yaml)

        # optimizer.tuningParameters.throttle_max = 0.45

        # Do not call init() before! Afterwards, it will not be possible to set different max_iter!
        run_socket_interface(optimizer)

if __name__ == '__main__':
    main()
