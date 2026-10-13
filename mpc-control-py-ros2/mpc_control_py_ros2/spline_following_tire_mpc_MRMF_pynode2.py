import ament_index_python
import rclpy
import yaml
from mpc_control_py_ros2.RacingMpcRos2 import RacingMpcRos2
from mpc_control_py_ros2.mpc_control_racing.spline_following_tire.optimizer.SplineFollowingTire_MRMF_Optimizer import SplineFollowingTire_MRMF_Optimizer

def main(args=None):
    rclpy.init(args=args)

    node_name = "spline_following_tire_mpc_MRMF"

    optimizer = SplineFollowingTire_MRMF_Optimizer()
    node = RacingMpcRos2(node_name, optimizer)

    optimizer.check_validity = False

    # Node-specific default config paths #
    if not node.get_parameter('params_yaml').get_parameter_value().string_value:
        node.set_parameters([
            rclpy.Parameter('params_yaml', rclpy.Parameter.Type.STRING,
                            ament_index_python.get_package_share_directory('mpc_control_racing_ros2') + "/config/spline_following_tire_params.yaml")
        ])

    params_yaml = node.get_parameter('params_yaml').get_parameter_value().string_value
    optimizer.load_params_from_yaml(params_yaml)

    # Node-specific spline coefficients (hardcoded) #
    # Spline/sampled for positioning
    track_default_yaml = ament_index_python.get_package_share_directory("lar_utils") + "/config/track/sampled/la_track_sampled.yaml"
    optimizer.track.load_from_yaml(track_default_yaml)

    # Spline for optimization
    with open(params_yaml, "r") as f:
        yamlNode = yaml.safe_load(f)
        raceline_default_yaml = ament_index_python.get_package_share_directory('mpc_control_racing_ros2') + "/config/" + yamlNode["raceline_name"]
        node.declare_parameter('raceline_yaml', raceline_default_yaml)
        optimizer.load_spline_from_yaml(node.get_parameter('raceline_yaml').get_parameter_value().string_value)

    # optimizer.set_acados_json('acados_ocp.json')

    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
