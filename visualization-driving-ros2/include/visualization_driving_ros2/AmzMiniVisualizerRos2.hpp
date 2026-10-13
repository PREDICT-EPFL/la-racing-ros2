#ifndef SRC_AMZMINIVISUALIZERROS2_HPP
#define SRC_AMZMINIVISUALIZERROS2_HPP

#include <memory>

#include "visualization_common_ros2/VisualizerRos2Base.hpp"
#include "visualization_common_ros2/VisualizationRos2Utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "mpc_msgs_racing_ros2/msg/car_trajectory_with_tracking_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_velocity_profile_stamped.hpp"
#include "mpc_msgs_racing_ros2/msg/racing_mpc_learning_data_stamped.hpp"

#include "lar_msgs/msg/car_state_stamped.hpp"
#include "lar_msgs/msg/car_control_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "utils_racing/Path.hpp"
#include "EigenVelocityProfileSplineLA.hpp"

//#include "mpc_control_racing/spline_following/laopt_ocp/EigenVelocityProfileSplineLA.hpp"

namespace visualization_ros2 {
namespace driving {

struct VehicleProperties
{
    std::string mesh_path;
    Eigen::Quaterniond q_b_mesh{1, 0, 0, 0};
    Eigen::Vector3d mesh_body_offset{0, 0, 0};
    double mesh_scale{1};

    double force_scale{1};
    double trace_scale{1};
    double velocity_scale{1};
};

struct MpcState
{
    double pos_x;
    double pos_y;
    double vel_x;
    double vel_y;
    double turn_angle;
    double turn_rate;
    double steering;
    double throttle;
};

class AmzMiniVisualizer : public VisualizerRos2Base
{
public:
    explicit AmzMiniVisualizer(const std::string &name);
    void load_track_from_yaml(const std::string &yaml_filepath);
    void load_raceline_from_yaml(const std::string &yaml_filepath);

protected:
    using MarkerM = visualization_msgs::msg::Marker;
    using MarkerArrayM = visualization_msgs::msg::MarkerArray;
    using TransformStampedM = geometry_msgs::msg::TransformStamped;
    using CarStateM = lar_msgs::msg::CarStateStamped;
    using CarControlM = lar_msgs::msg::CarControlStamped;
    using Pose2dM = geometry_msgs::msg::Pose2D;
    using Vector3StampedM = geometry_msgs::msg::Vector3Stamped;
    using CarTrajectoryWithTrackingStampedM = mpc_msgs_racing_ros2::msg::CarTrajectoryWithTrackingStamped;
    using RacingMpcVelocityProfileStampedM = mpc_msgs_racing_ros2::msg::RacingMpcVelocityProfileStamped;
    using RacingMpcLearningDataStampedM = mpc_msgs_racing_ros2::msg::RacingMpcLearningDataStamped;

    void init_impl() override;
    void update_in_impl() override;
    void update_out_impl() override;

    VehicleProperties vehicle;
    VehicleProperties vehicle_prediction;
    CarTrajectoryWithTrackingStampedM mpc_traj;
    bool mpc_traj_is_avail{false};

    CarTrajectoryWithTrackingStampedM closed_loop_traj;
    bool closed_loop_traj_is_avail{false};

    /* Trace */
    utils::TraceVisualizer m_traceVis;

    /* Constant quaternions */
    const Eigen::Quaterniond q_ned_enu{0, sqrt(2) / 2.0, sqrt(2) / 2.0, 0};
    const Eigen::Quaterniond q_flu_b{0, 1, 0, 0};

    /** Data **/
    Eigen::Vector3d pos{0.2, 0.1, 0};
    Eigen::Vector3d vel{0, 0, 0};
    Eigen::Quaterniond q_enu_b{1, 0, 0, 0}; // Quaternion transforms a vector from body to enu frame

    utils_racing::PathSpline m_track;
    Eigen::Vector2d track_pos{0, 0};
    double track_angle{0};
    Eigen::Matrix2d T_enu_track = Eigen::Matrix2d::Identity();
    bool track_isAvail = true;
    static constexpr unsigned p_track_N_sample_points{100};
    Eigen::Matrix<double, 6, p_track_N_sample_points> m_track_points_center_edges_XY;

    utils_racing::PathSpline m_path;
    Eigen::Matrix<double, 2, p_track_N_sample_points> m_raceline_points_XY;

    mpc_control_racing::spline_following::EigenVelocityProfileLA m_velocity_profile{true};
    RacingMpcVelocityProfileStampedM velocity_profile;
    static constexpr unsigned p_vel_profile_N_sample_points{13 * 4 * 10 + 1};
    bool velocity_profile_is_avail{false};

    RacingMpcLearningDataStampedM learning_data;
    bool learning_data_is_avail{false};

    RosTime m_track_pub_time = rclcpp::Node::now();

    Eigen::Vector2d pos_ref{0, 0};
    double rate_ref{0};
    bool control_ref_isAvail{false};
    Eigen::Vector2d control{0, 0};

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_static_broadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    /* Callback functions =========================================================================================== */
    struct Subscribers
    {
        RosSub<CarStateM> state;
        RosSub<CarControlM> control;
        RosSub<Pose2dM> track;
        RosSub<Vector3StampedM> refPoint;
        RosSub<CarTrajectoryWithTrackingStampedM> mpcTraj;
        RosSub<CarTrajectoryWithTrackingStampedM> closedLoopTraj;
        RosSub<RacingMpcVelocityProfileStampedM> velocityProfile;
        RosSub<RacingMpcLearningDataStampedM> learningData;
    } m_sub;
    void stateCallback(const CarStateM::SharedPtr msg);
    void controlCallback(const CarControlM::SharedPtr msg);
    void trackCallback(const Pose2dM::SharedPtr msg);
    void controlRefCallback(const Vector3StampedM::SharedPtr msg);
    void mpcTrajCallback(const CarTrajectoryWithTrackingStampedM &msg);
    void closedLoopTrajCallback(const CarTrajectoryWithTrackingStampedM &msg);
    void velProfileCallback(const RacingMpcVelocityProfileStampedM &msg);
    void learningDataCallback(const RacingMpcLearningDataStampedM &msg);

    /* Publishing functions ========================================================================================= */
    struct Publishers
    {
        RosPub<MarkerArrayM> env;
        RosPub<MarkerM> trace;
        RosPub<MarkerArrayM> car;
        RosPub<MarkerArrayM> mpcTraj;
        RosPub<MarkerArrayM> closedLoopTraj;
        RosPub<MarkerArrayM> velocityProfile;
        RosPub<MarkerArrayM> learningData;
    } m_pub;
    RosTime m_pub_time;

    struct AmzMarkers
    {
        TransformStampedM tf_b_ned;
        MarkerM track_centerline;
        MarkerM track_edges;
        MarkerM raceline;
        MarkerM body;
        MarkerM velocity;
        MarkerM control;
        MarkerM refPoint;
//        MarkerM velocity_profile;
    } m_amz_msg;

    // Once
    void broadcastEnuTransfrom();

    // Continuously
    void broadcastBodyTransform();

    void setupTrackMarker();
    void prepareTrackMarker();

    void setupRacelineMarker();
    void prepareRacelineMarker();

//    void setupVelocityProfileMarker();

    void setupBodyMarker();
    void prepareBodyMarker();

    void setupVelocityMarker();
    void prepareVelocityMarker();

    void setupControlMarker();
    void prepareControlMarker();

    void setupRefPointMarker();
    void prepareRefPointMarker();

    void publishEnvMarkers();
    void publishCarMarkers();
    void publishMpcTrajMarkers();
    void publishClosedLoopTrajMarkers();
    void publishVelocityProfileMarkers();
    void publishLearningDataMarkers();

    /* Helper functions ============================================================================================= */
    void setMeshMarker(MarkerM &marker, const Eigen::Vector3d &position, const Eigen::Quaterniond &quat)
    {
        marker.type = MarkerM::MESH_RESOURCE;
        marker.mesh_resource = vehicle.mesh_path;
        marker.scale.x = marker.scale.y = marker.scale.z = vehicle.mesh_scale;
        Eigen::Quaterniond q_enu_mesh = quat * vehicle.q_b_mesh;
        q_enu_mesh.normalize();
        marker.pose.position.x = position(0) + vehicle.mesh_body_offset(0);
        marker.pose.position.y = position(1) + vehicle.mesh_body_offset(1);
        marker.pose.position.z = position(2) + vehicle.mesh_body_offset(2);

        /* Mesh needs to be rotated*/
        marker.pose.orientation.w = q_enu_mesh.w();
        marker.pose.orientation.x = q_enu_mesh.x();
        marker.pose.orientation.y = q_enu_mesh.y();
        marker.pose.orientation.z = q_enu_mesh.z();
    }
    void setPredMeshMarker(MarkerM &marker, const Eigen::Vector3d &position, const Eigen::Quaterniond &quat)
    {
        marker.type = MarkerM::MESH_RESOURCE;
        marker.mesh_resource = vehicle_prediction.mesh_path;
        marker.scale.x = marker.scale.y = marker.scale.z = vehicle_prediction.mesh_scale;
        Eigen::Quaterniond q_enu_mesh = quat * vehicle_prediction.q_b_mesh;
        q_enu_mesh.normalize();
        marker.pose.position.x = position(0) + vehicle_prediction.mesh_body_offset(0);
        marker.pose.position.y = position(1) + vehicle_prediction.mesh_body_offset(1);
        marker.pose.position.z = position(2) + vehicle_prediction.mesh_body_offset(2);

        /* Mesh needs to be rotated */
        marker.pose.orientation.w = q_enu_mesh.w();
        marker.pose.orientation.x = q_enu_mesh.x();
        marker.pose.orientation.y = q_enu_mesh.y();
        marker.pose.orientation.z = q_enu_mesh.z();
    }
};

} // namespace driving
} // namespace visualization_ros2

#endif //SRC_AMZMINIVISUALIZERROS2_HPP
