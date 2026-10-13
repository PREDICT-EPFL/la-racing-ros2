#include "visualization_driving_ros2/AmzMiniVisualizerRos2.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace visualization_ros2::driving;

AmzMiniVisualizer::AmzMiniVisualizer(const std::string &name) : VisualizerRos2Base(name, 50)
{
    /* Set up vehicle visual properties */
    vehicle.mesh_path = "package://visualization_driving_ros2/mesh_files/tuning-urus-small.stl";
    vehicle.mesh_scale = 1.0;
    vehicle.q_b_mesh = Eigen::Quaterniond(Eigen::AngleAxis<double>(90 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    vehicle.force_scale = 0.05;
    vehicle.trace_scale = 0.03;
    vehicle.velocity_scale = 1.0;

    vehicle_prediction.mesh_path = "package://visualization_driving_ros2/mesh_files/muscle_low_poly.stl";
    vehicle_prediction.mesh_scale = 0.5 * 0.015;
    vehicle_prediction.q_b_mesh = Eigen::Quaterniond(
            Eigen::AngleAxis<double>(90 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
    vehicle_prediction.force_scale = 0.03;
    vehicle_prediction.trace_scale = 0.02;
    vehicle_prediction.velocity_scale = 1.0;
    vehicle_prediction.mesh_body_offset(2) = -0.02;

    declare_parameter<std::string>("source", "real");

    const std::string default_track_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                           "/config/track/spline/la_track_spline.yaml";
    declare_parameter<std::string>("track_yaml", default_track_yaml);
    load_track_from_yaml(get_parameter("track_yaml").as_string());

    const std::string default_raceline_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                              "/config/track/spline/la_raceline_spline.yaml";
    declare_parameter<std::string>("raceline_yaml", default_raceline_yaml);
    load_raceline_from_yaml(get_parameter("raceline_yaml").as_string());

    std::string vis_source = get_parameter("source").as_string();
    if (vis_source == "sim")
    {
        std::cout << "Running driving visualization (ROS2 interface to SIMULATOR)\n";

        m_sub.state = create_subscription<CarStateM>(
                "/sim/car/state", 1, std::bind(&AmzMiniVisualizer::stateCallback, this, _1));

        m_sub.control = create_subscription<CarControlM>(
                "/sim/car/control", 1, std::bind(&AmzMiniVisualizer::controlCallback, this, _1));

        m_sub.track = create_subscription<Pose2dM>(
                "/sim/track/pose2d", 1, std::bind(&AmzMiniVisualizer::trackCallback, this, _1));
        track_isAvail = true; // If not published, can be used as {0, 0, 0}
    }
    else
    {
        std::cout << "Running driving visualization (ROS2 interface to REAL system)\n";

        m_sub.state = create_subscription<CarStateM>(
                "/car/state", 1, std::bind(&AmzMiniVisualizer::stateCallback, this, _1));

        m_sub.control = create_subscription<CarControlM>(
                "/car/set/control", rclcpp::SensorDataQoS(),
                std::bind(&AmzMiniVisualizer::controlCallback, this, _1));

        m_sub.track = create_subscription<Pose2dM>(
                "/mocap/track/pose2d", 1, std::bind(&AmzMiniVisualizer::trackCallback, this, _1));

    }

    m_sub.refPoint = create_subscription<Vector3StampedM>(
            "/diag/control_ref", 1, std::bind(&AmzMiniVisualizer::controlRefCallback, this, _1));

    m_sub.mpcTraj = create_subscription<CarTrajectoryWithTrackingStampedM>(
            "/mpc/trajectory", 1, std::bind(&AmzMiniVisualizer::mpcTrajCallback, this, _1));

    m_sub.closedLoopTraj = create_subscription<CarTrajectoryWithTrackingStampedM>(
            "/data_agent/car_traj", 1, std::bind(&AmzMiniVisualizer::closedLoopTrajCallback, this, _1));

    m_sub.velocityProfile = create_subscription<RacingMpcVelocityProfileStampedM>(
            "/learning_agent/vel_profile", 10, std::bind(&AmzMiniVisualizer::velProfileCallback, this, _1));

    m_sub.learningData = create_subscription<RacingMpcLearningDataStampedM>(
            "/data_agent/mismatch_buffer", 10, std::bind(&AmzMiniVisualizer::learningDataCallback, this, _1));

    /* Publishers */
    m_tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_pub.trace = create_publisher<MarkerM>("/vis/trace", 10);
    m_pub.env = create_publisher<MarkerArrayM>("/vis/env", 10);
    m_pub.car = create_publisher<MarkerArrayM>("/vis/car", 10);
    m_pub.mpcTraj = create_publisher<MarkerArrayM>("/vis/mpc_traj", 10);
    m_pub.closedLoopTraj = create_publisher<MarkerArrayM>("/vis/past_traj", 10);
    m_pub.velocityProfile = create_publisher<MarkerArrayM>("/vis/vel_profile", 10);
    m_pub.learningData = create_publisher<MarkerArrayM>("/vis/learning_data", 10);

    declare_parameter<int>("numTracePoints", 50);
    unsigned trace_n_points = get_parameter("numTracePoints").get_value<unsigned>();
    m_traceVis = utils::TraceVisualizer(
            m_pub.trace, "trace", "enu", trace_n_points,
            1e9 / p_nodeRate.period().count(), 1.0 / 0.2,
            vehicle.trace_scale);
}

void AmzMiniVisualizer::load_track_from_yaml(const std::string &yaml_filepath)
{
    m_track.load_from_yaml(yaml_filepath, {"track", "highRes"});

    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        const double eval_arg = i / static_cast<double>(p_track_N_sample_points);
        const std::array<Eigen::Vector2d, 3> center_with_edges = m_track.eval_centerline_with_edge_pos(eval_arg);
        m_track_points_center_edges_XY.col(i) << center_with_edges[0], center_with_edges[1], center_with_edges[2];
    }
}
void AmzMiniVisualizer::load_raceline_from_yaml(const std::string &yaml_filepath)
{
    m_path.load_from_yaml(yaml_filepath, {"track", "highRes"});

    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        const double eval_arg = i / static_cast<double>(p_track_N_sample_points);
        const Eigen::Vector2d pos = m_path.eval(eval_arg);
        m_raceline_points_XY.col(i) = pos;
    }
}

/** Protected members ============================================================================ **/
void AmzMiniVisualizer::init_impl()
{
    std::cout << "AMZ Mini car\n";

    m_pub_time = rclcpp::Node::now();
    m_traceVis.clear(m_pub_time);

    broadcastEnuTransfrom();
    broadcastBodyTransform();

    setupTrackMarker();
    setupRacelineMarker();
    setupBodyMarker();
    setupVelocityMarker();
    setupControlMarker();
    setupRefPointMarker();
}
void AmzMiniVisualizer::update_in_impl()
{
    control_ref_isAvail = false;
    mpc_traj_is_avail = false;
    closed_loop_traj_is_avail = false;
    velocity_profile_is_avail = false;

    VisualizerRos2Base::update_in_impl();
}
void AmzMiniVisualizer::update_out_impl()
{
    VisualizerRos2Base::update_out_impl();

    m_pub_time = rclcpp::Node::now();
    m_traceVis.publish(m_pub_time, pos);

    broadcastBodyTransform();
    publishEnvMarkers();
    publishCarMarkers();
    if (mpc_traj_is_avail) { publishMpcTrajMarkers(); }
    if (closed_loop_traj_is_avail) { publishClosedLoopTrajMarkers(); }
    publishVelocityProfileMarkers(); // Check is done inside function
    if (learning_data_is_avail) { publishLearningDataMarkers(); }
}

/** Private members ============================================================================== **/
/* Callback functions ============================================================================= */
void AmzMiniVisualizer::stateCallback(const CarStateM::SharedPtr msg)
{
    pos = {msg->pos_x, msg->pos_y, 0.0245};
    vel = {msg->vel_x, msg->vel_y, 0};
    q_enu_b = Eigen::Quaterniond(Eigen::AngleAxis<double>(msg->turn_angle, Eigen::Vector3d::UnitZ()));
}
void AmzMiniVisualizer::controlCallback(const CarControlM::SharedPtr msg)
{
    control = {msg->steering, msg->throttle};
}
void AmzMiniVisualizer::trackCallback(const Pose2dM::SharedPtr msg)
{
    track_pos = {msg->x, msg->y};
    track_angle = msg->theta;
    T_enu_track << std::cos(-track_angle), std::sin(-track_angle),
            -std::sin(-track_angle), std::cos(-track_angle);
    track_isAvail = true;
}
void AmzMiniVisualizer::controlRefCallback(const Vector3StampedM::SharedPtr msg)
{
    pos_ref = {msg->vector.x, msg->vector.y};
    rate_ref = msg->vector.z;
    control_ref_isAvail = true;
}

void AmzMiniVisualizer::mpcTrajCallback(const CarTrajectoryWithTrackingStampedM &msg)
{
    mpc_traj = msg;
    mpc_traj_is_avail = true;
}
void AmzMiniVisualizer::closedLoopTrajCallback(const CarTrajectoryWithTrackingStampedM &msg)
{
    closed_loop_traj = msg;
    closed_loop_traj_is_avail = true;
}
void AmzMiniVisualizer::velProfileCallback(const RacingMpcVelocityProfileStampedM &msg)
{
    velocity_profile = msg;
    velocity_profile_is_avail = true;
}
void AmzMiniVisualizer::learningDataCallback(const RacingMpcLearningDataStampedM &msg)
{
    learning_data = msg;
    learning_data_is_avail = true;
}

/* Publishing functions =========================================================================== */
/* Publish once */
void AmzMiniVisualizer::broadcastEnuTransfrom()
{
    TransformStampedM tf_ned_world{};     // All entries = 0

    /* Reference */
    tf_ned_world.header.stamp = rclcpp::Node::now();
    tf_ned_world.header.frame_id = "world";
    tf_ned_world.child_frame_id = "enu";

    /* Rotation */
    //const Quaternion q_ned_enu(Eigen::AngleAxisd(M_PI / 2.0, Vector3(0, 0, 1)) * Eigen::AngleAxisd(M_PI, Vector3(1, 0, 0)));
    tf_ned_world.transform.rotation.w = 1;
    tf_ned_world.transform.rotation.x = 0;
    tf_ned_world.transform.rotation.y = 0;
    tf_ned_world.transform.rotation.z = 0;

    m_tf_static_broadcaster->sendTransform(tf_ned_world);
}

/* Publish continuously */
void AmzMiniVisualizer::broadcastBodyTransform()
{
    /* Reference */
    m_amz_msg.tf_b_ned.header.stamp = m_pub_time;
    m_amz_msg.tf_b_ned.header.frame_id = "enu";
    m_amz_msg.tf_b_ned.child_frame_id = "body";

    /* Position */
    m_amz_msg.tf_b_ned.transform.translation.x = pos(0);
    m_amz_msg.tf_b_ned.transform.translation.y = pos(1);
    m_amz_msg.tf_b_ned.transform.translation.z = pos(2);

    /* Rotation */
    m_amz_msg.tf_b_ned.transform.rotation.w = q_enu_b.w();
    m_amz_msg.tf_b_ned.transform.rotation.x = q_enu_b.x();
    m_amz_msg.tf_b_ned.transform.rotation.y = q_enu_b.y();
    m_amz_msg.tf_b_ned.transform.rotation.z = q_enu_b.z();

    m_tf_broadcaster->sendTransform(m_amz_msg.tf_b_ned);
}

void AmzMiniVisualizer::setupTrackMarker()
{
    /* Centerline ------------------------------------------------------ */
    m_amz_msg.track_centerline = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.track_centerline.ns = "track_centerline";
    m_amz_msg.track_centerline.id = 0;

    /* Frame and time */
    m_amz_msg.track_centerline.header.frame_id = "enu";
    m_amz_msg.track_centerline.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_amz_msg.track_centerline.frame_locked = false;

    m_amz_msg.track_centerline.points.resize(p_track_N_sample_points);

    /* Shape and scale */
    m_amz_msg.track_centerline.type = MarkerM::LINE_LIST;
    m_amz_msg.track_centerline.scale.x = 0.4 * vehicle.force_scale;

    /* Color */
    // White
    double grey_scale = 0.8;
    m_amz_msg.track_centerline.color.r = grey_scale * 1.0;
    m_amz_msg.track_centerline.color.g = grey_scale * 1.0;
    m_amz_msg.track_centerline.color.b = grey_scale * 1.0;
    m_amz_msg.track_centerline.color.a = 1.0;

    /* Edges ------------------------------------------------------ */
    m_amz_msg.track_edges = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.track_edges.ns = "track_edges";
    m_amz_msg.track_edges.id = 0;

    /* Frame and time */
    m_amz_msg.track_edges.header.frame_id = "enu";
    m_amz_msg.track_edges.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_amz_msg.track_edges.frame_locked = false;

    m_amz_msg.track_edges.points.resize(2 * p_track_N_sample_points + 2);

    /* Shape and scale */
    m_amz_msg.track_edges.type = MarkerM::LINE_STRIP;
    m_amz_msg.track_edges.scale.x = 0.4 * vehicle.force_scale;

    /* Color */
    // White
    m_amz_msg.track_edges.color.r = grey_scale * 1.0;
    m_amz_msg.track_edges.color.g = grey_scale * 1.0;
    m_amz_msg.track_edges.color.b = grey_scale * 1.0;
    m_amz_msg.track_edges.color.a = 1.0;

}
void AmzMiniVisualizer::prepareTrackMarker()
{
    /* Prepare center line */
    Eigen::Matrix<double, 2, p_track_N_sample_points>
            track_centerline_pointsXY = m_track_points_center_edges_XY.topRows(2);

    track_centerline_pointsXY = T_enu_track * track_centerline_pointsXY;
    track_centerline_pointsXY.row(0).array() += track_pos(0);
    track_centerline_pointsXY.row(1).array() += track_pos(1);

    m_amz_msg.track_centerline.header.stamp = m_track_pub_time;

    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        m_amz_msg.track_centerline.points[i].x = track_centerline_pointsXY(0, i);
        m_amz_msg.track_centerline.points[i].y = track_centerline_pointsXY(1, i);
    }

    /* Prepare edges ------------------------------- */
    Eigen::Matrix<double, 2, p_track_N_sample_points>
            track_leftedge_pointsXY = m_track_points_center_edges_XY.middleRows<2>(2);
    Eigen::Matrix<double, 2, p_track_N_sample_points>
            track_rightedege_pointsXY = m_track_points_center_edges_XY.bottomRows(2);

    track_leftedge_pointsXY = T_enu_track * track_leftedge_pointsXY;
    track_leftedge_pointsXY.row(0).array() += track_pos(0);
    track_leftedge_pointsXY.row(1).array() += track_pos(1);

    track_rightedege_pointsXY = T_enu_track * track_rightedege_pointsXY;
    track_rightedege_pointsXY.row(0).array() += track_pos(0);
    track_rightedege_pointsXY.row(1).array() += track_pos(1);

    m_amz_msg.track_edges.header.stamp = m_track_pub_time;

    unsigned point_counter{0};
    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        m_amz_msg.track_edges.points[point_counter].x = track_leftedge_pointsXY(0, i);
        m_amz_msg.track_edges.points[point_counter].y = track_leftedge_pointsXY(1, i);
        point_counter++;
    }
    // Close
    m_amz_msg.track_edges.points[point_counter].x = track_leftedge_pointsXY(0, 0);
    m_amz_msg.track_edges.points[point_counter].y = track_leftedge_pointsXY(1, 0);
    point_counter++;

    // Cross to other edge
    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        m_amz_msg.track_edges.points[point_counter].x = track_rightedege_pointsXY(0, i);
        m_amz_msg.track_edges.points[point_counter].y = track_rightedege_pointsXY(1, i);
        point_counter++;
    }
    // Close
    m_amz_msg.track_edges.points[point_counter].x = track_rightedege_pointsXY(0, 0);
    m_amz_msg.track_edges.points[point_counter].y = track_rightedege_pointsXY(1, 0);
    point_counter++;
}

void AmzMiniVisualizer::setupRacelineMarker()
{
    m_amz_msg.raceline = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.raceline.ns = "track_raceline";
    m_amz_msg.raceline.id = 0;

    /* Frame and time */
    m_amz_msg.raceline.header.frame_id = "enu";
    m_amz_msg.raceline.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_amz_msg.raceline.frame_locked = false;

    m_amz_msg.raceline.points.resize(p_track_N_sample_points);

    /* Shape and scale */
    m_amz_msg.raceline.type = MarkerM::LINE_STRIP;
    m_amz_msg.raceline.scale.x = 0.2 * vehicle.force_scale;

    /* Color */
    // White
    double grey_scale = 1.0;
    m_amz_msg.raceline.color.r = grey_scale * magenta.r;
    m_amz_msg.raceline.color.g = grey_scale * magenta.g;
    m_amz_msg.raceline.color.b = grey_scale * magenta.b;
    m_amz_msg.raceline.color.a = 1.0;
}
void AmzMiniVisualizer::prepareRacelineMarker()
{
    /* Prepare center line */
    Eigen::Matrix<double, 2, p_track_N_sample_points> track_raceline_pointsXY = m_raceline_points_XY;

    track_raceline_pointsXY = T_enu_track * track_raceline_pointsXY;
    track_raceline_pointsXY.row(0).array() += track_pos(0);
    track_raceline_pointsXY.row(1).array() += track_pos(1);

    m_amz_msg.raceline.header.stamp = m_track_pub_time;

    for (unsigned i = 0; i < p_track_N_sample_points; i++)
    {
        m_amz_msg.raceline.points[i].x = track_raceline_pointsXY(0, i);
        m_amz_msg.raceline.points[i].y = track_raceline_pointsXY(1, i);
    }
}

void AmzMiniVisualizer::setupBodyMarker()
{
    /* Namespace and identifier */
    m_amz_msg.body.ns = "mesh";
    m_amz_msg.body.id = 0;

    /* Frame and time */
    m_amz_msg.body.header.stamp = RosTime(0);
    m_amz_msg.body.header.frame_id = "body";
    m_amz_msg.body.frame_locked = true;

    /* Color */
    // White
    m_amz_msg.body.color.r = 1.0;
    m_amz_msg.body.color.g = 1.0;
    m_amz_msg.body.color.b = 1.0;
    m_amz_msg.body.color.a = 1.0;

    /* Mesh */
    setMeshMarker(m_amz_msg.body, Eigen::Vector3d{0, 0, 0}, Eigen::Quaterniond{1, 0, 0, 0});
}
void AmzMiniVisualizer::prepareBodyMarker()
{
    m_amz_msg.body.header.stamp = m_pub_time;
}

void AmzMiniVisualizer::setupVelocityMarker()
{
    m_amz_msg.velocity = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.velocity.ns = "velocity";
    m_amz_msg.velocity.id = 0;

    /* Frame and time */
    m_amz_msg.velocity.header.frame_id = "body";
    m_amz_msg.velocity.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_amz_msg.velocity.frame_locked = true;

    m_amz_msg.velocity.points.resize(2);
    // points[0] is 0,0,0
    // points[1] will be updated

    /* Shape and scale */
    m_amz_msg.velocity.type = MarkerM::ARROW;
    m_amz_msg.velocity.scale.x = vehicle.velocity_scale * 0.008; // Shaft diameter
    m_amz_msg.velocity.scale.y = vehicle.velocity_scale * 0.04; // Head diameter
    m_amz_msg.velocity.scale.z = vehicle.velocity_scale * 0.04; // Head Length

    /* Color */
    m_amz_msg.velocity.color.r = 1.0;
    m_amz_msg.velocity.color.g = 1.0;
    m_amz_msg.velocity.color.b = 1.0;
    m_amz_msg.velocity.color.a = 1.0;
}
void AmzMiniVisualizer::prepareVelocityMarker()
{
    m_amz_msg.velocity.header.stamp = m_pub_time;

    const Eigen::Vector3d b_vel = q_enu_b.inverse() * vel;
    m_amz_msg.velocity.points[1].x = vehicle.velocity_scale * 0.15 * b_vel(0);
    m_amz_msg.velocity.points[1].y = vehicle.velocity_scale * 0.15 * b_vel(1);
}

void AmzMiniVisualizer::setupControlMarker()
{
    m_amz_msg.control = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.control.ns = "control";
    m_amz_msg.control.id = 0;

    /* Frame and time */
    m_amz_msg.control.header.frame_id = "body";
    m_amz_msg.control.lifetime = rclcpp::Duration::from_nanoseconds(0); // Unlimited
    m_amz_msg.control.frame_locked = true;

    /* Position and orientation */
    m_amz_msg.control.pose.orientation.w = 1;

    m_amz_msg.control.points.resize(4);
    m_amz_msg.control.points[0].x = m_amz_msg.control.points[1].x = 0.09;
    m_amz_msg.control.points[0].z = m_amz_msg.control.points[1].z = -0.01;

    m_amz_msg.control.points[2].x = m_amz_msg.control.points[3].x = -0.02;
    m_amz_msg.control.points[2].z = m_amz_msg.control.points[3].z = 0.03;

    /* Shape and scale */
    m_amz_msg.control.type = MarkerM::LINE_LIST;
    m_amz_msg.control.scale.x = 0.2 * vehicle.force_scale;

    /* Color */
    // White
    m_amz_msg.control.color.r = 1.0;
    m_amz_msg.control.color.g = 1.0;
    m_amz_msg.control.color.b = 1.0;
    m_amz_msg.control.color.a = 1.0;
}
void AmzMiniVisualizer::prepareControlMarker()
{
    m_amz_msg.control.header.stamp = m_pub_time;

    /* Steering */
    m_amz_msg.control.points[1].y = control(0) * 2.0 * vehicle.force_scale;

    /* Throttle */
    m_amz_msg.control.points[3].x = m_amz_msg.control.points[2].x + control(1) * 5.0 * vehicle.force_scale;
}

void AmzMiniVisualizer::setupRefPointMarker()
{
    m_amz_msg.refPoint = MarkerM();

    /* Namespace and identifier */
    m_amz_msg.refPoint.ns = "pos_ref";
    m_amz_msg.refPoint.id = 0;

    /* Frame and time */
    m_amz_msg.refPoint.header.frame_id = "enu";
    m_amz_msg.refPoint.lifetime = rclcpp::Duration::from_nanoseconds(1e3); // Unlimited
    m_amz_msg.refPoint.frame_locked = false;

    /* Shape and scale */
    m_amz_msg.refPoint.type = MarkerM::SPHERE;
    m_amz_msg.refPoint.scale.x =
    m_amz_msg.refPoint.scale.y =
    m_amz_msg.refPoint.scale.z = 2.0 * vehicle.trace_scale;

    /* Color */
    m_amz_msg.refPoint.color.r = magenta.r;
    m_amz_msg.refPoint.color.g = magenta.g;
    m_amz_msg.refPoint.color.b = magenta.b;
    m_amz_msg.refPoint.color.a = 1.0;
}
void AmzMiniVisualizer::prepareRefPointMarker()
{
    m_amz_msg.refPoint.header.stamp = m_pub_time;

    Eigen::Vector2d w_ref_point = T_enu_track * pos_ref;
    w_ref_point += track_pos;

    m_amz_msg.refPoint.pose.position.x = w_ref_point(0);
    m_amz_msg.refPoint.pose.position.y = w_ref_point(1);

    if (!control_ref_isAvail || mpc_traj_is_avail)
    {
        m_amz_msg.refPoint.type = MarkerM::DELETE;
        m_amz_msg.refPoint.pose.position.x = -5;
        m_amz_msg.refPoint.pose.position.y = -5;
    }
    else
    {
        m_amz_msg.refPoint.type = MarkerM::SPHERE;
    }
}

void AmzMiniVisualizer::publishEnvMarkers()
{
    MarkerArrayM markerArray;

    if (track_isAvail && (rclcpp::Node::now() - m_track_pub_time).seconds() > 2)
    {
        m_track_pub_time = rclcpp::Node::now();
        prepareTrackMarker();
        markerArray.markers.push_back(m_amz_msg.track_edges);
        markerArray.markers.push_back(m_amz_msg.track_centerline);

        prepareRacelineMarker();
        markerArray.markers.push_back(m_amz_msg.raceline);
    }

    m_pub.env->publish(markerArray);
}
void AmzMiniVisualizer::publishCarMarkers()
{
    MarkerArrayM markerArray;

    prepareBodyMarker();
    markerArray.markers.push_back(m_amz_msg.body);

    prepareVelocityMarker();
    markerArray.markers.push_back(m_amz_msg.velocity);

    prepareControlMarker();
    markerArray.markers.push_back(m_amz_msg.control);

    prepareRefPointMarker();
    markerArray.markers.push_back(m_amz_msg.refPoint);

    m_pub.car->publish(markerArray);
}
void AmzMiniVisualizer::publishMpcTrajMarkers()
{
    MarkerArrayM markerArray{};

    std::size_t n_state = mpc_traj.state.size();
    std::size_t n_control = mpc_traj.control.size();

    MarkerM steeringDeflM{};
    steeringDeflM.ns = "steering_defl";
    steeringDeflM.id = 0;
    steeringDeflM.header.frame_id = "enu";
    steeringDeflM.header.stamp = m_pub_time;
    steeringDeflM.type = MarkerM::LINE_LIST;
    steeringDeflM.scale.x = 0.1 * vehicle.force_scale;
    steeringDeflM.points.resize(2 * n_control);
    steeringDeflM.color.r = red.r;
    steeringDeflM.color.g = red.g;
    steeringDeflM.color.b = red.b;
    steeringDeflM.color.a = 1.0;

    MarkerM throttleDeflM{};
    throttleDeflM.ns = "throttle_defl";
    throttleDeflM.id = 0;
    throttleDeflM.header.frame_id = "enu";
    throttleDeflM.header.stamp = m_pub_time;
    throttleDeflM.type = MarkerM::LINE_LIST;
    throttleDeflM.scale.x = 0.1 * vehicle.force_scale;
    throttleDeflM.points.resize(2 * n_control);
    throttleDeflM.color.r = red.r;
    throttleDeflM.color.g = red.g;
    throttleDeflM.color.b = red.b;
    throttleDeflM.color.a = 1.0;

    MarkerM steeringM{};
    steeringM.ns = "steering";
    steeringM.id = 0;
    steeringM.header.frame_id = "enu";
    steeringM.header.stamp = m_pub_time;
    steeringM.type = MarkerM::LINE_STRIP;
    steeringM.scale.x = 0.15 * vehicle.force_scale;
    steeringM.points.resize(n_control);
    steeringM.color.r = 1.0;
    steeringM.color.g = 1.0;
    steeringM.color.b = 1.0;
    steeringM.color.a = 1.0;

    MarkerM throttleM{};
    throttleM.ns = "throttle";
    throttleM.id = 0;
    throttleM.header.frame_id = "enu";
    throttleM.header.stamp = m_pub_time;
    throttleM.type = MarkerM::LINE_STRIP;
    throttleM.scale.x = 0.15 * vehicle.force_scale;
    throttleM.points.resize(n_control);
    throttleM.color.r = 1.0;
    throttleM.color.g = 1.0;
    throttleM.color.b = 1.0;
    throttleM.color.a = 1.0;

    MarkerM posM{};
    posM.ns = "pos";
    posM.id = 0;
    posM.header.frame_id = "enu";
    posM.header.stamp = m_pub_time;
    posM.type = MarkerM::LINE_STRIP;
    posM.scale.x = 0.1 * vehicle.force_scale;
    posM.points.resize(n_state);
    posM.color.r = 0.5;
    posM.color.g = 0.5;
    posM.color.b = 0.5;
    posM.color.a = 1.0;

    MarkerM path_paramM{};
    path_paramM.ns = "path_param";
    path_paramM.id = 0;
    path_paramM.header.frame_id = "enu";
    path_paramM.header.stamp = m_pub_time;
    path_paramM.type = MarkerM::SPHERE_LIST;
    path_paramM.scale.x =
    path_paramM.scale.y =
    path_paramM.scale.z = 1.5 * vehicle_prediction.trace_scale;
    path_paramM.points.resize(n_state);
    path_paramM.color.r = blue.r;
    path_paramM.color.g = blue.g;
    path_paramM.color.b = blue.b;
    path_paramM.color.a = 1.0;
    Eigen::MatrixXd path_params_pos(2, n_state);

    MarkerM path_errM{};
    path_errM.ns = "pos_err";
    path_errM.id = 0;
    path_errM.header.frame_id = "enu";
    path_errM.header.stamp = m_pub_time;
    path_errM.type = MarkerM::LINE_LIST;
    path_errM.scale.x =
    path_errM.scale.y =
    path_errM.scale.z = 0.1 * vehicle_prediction.force_scale;
    path_errM.points.resize(2 * n_state);
    path_errM.color.r = white.r;
    path_errM.color.g = white.g;
    path_errM.color.b = white.b;
    path_errM.color.a = 1.0;

    if (mpc_traj.state[0].path_param != 0 || mpc_traj.state[1].path_param != 0)
    {
        for (std::size_t i = 0; i < n_state; i++)
        {
            path_params_pos.col(i) = T_enu_track * m_path.eval_pos(mpc_traj.state[i].path_param) + track_pos;
            path_paramM.points[i].x = path_params_pos(0, i);
            path_paramM.points[i].y = path_params_pos(1, i);
        }
    }

    if (mpc_traj.state[0].pos_x != 0)
    {
        for (std::size_t i = 0; i < n_state; i++)
        {
            Eigen::Vector3d tPos(mpc_traj.state[i].pos_x, mpc_traj.state[i].pos_y, 0.0245);
            Eigen::Vector3d tVel(mpc_traj.state[i].vel_x, mpc_traj.state[i].vel_y, 0.0);
            Eigen::Quaterniond tq_enu_b(
                    Eigen::AngleAxis<double>(mpc_traj.state[i].turn_angle, Eigen::Vector3d::UnitZ()));

            MarkerM meshM{};
            meshM.ns = "pose_points";
            meshM.id = i;
            meshM.header.frame_id = "enu";
            meshM.header.stamp = m_pub_time;
            meshM.color.r = blue.r;
            meshM.color.g = blue.g;
            meshM.color.b = blue.b;
            meshM.color.a = 1.0;
            setPredMeshMarker(meshM, tPos, tq_enu_b);
            markerArray.markers.push_back(meshM);

            posM.points[i].x = tPos(0);
            posM.points[i].y = tPos(1);
            posM.points[i].z = tPos(2);

//        MarkerM velocityM;
//        velocityM.ns = "velocity";
//        velocityM.id = i;
//        velocityM.header.frame_id = "enu";
//        velocityM.header.stamp = m_pub_time;
//        velocityM.points.resize(2);
//        velocityM.points[0].x = tPos(0);
//        velocityM.points[0].y = tPos(1);
//        velocityM.points[0].z = tPos(2);
//        velocityM.points[1].x = tPos(0) + vehicle.velocity_scale * 0.15 * tVel(0);
//        velocityM.points[1].y = tPos(1) + vehicle.velocity_scale * 0.15 * tVel(1);
//        velocityM.points[1].z = tPos(2);
//        velocityM.type = MarkerM::ARROW;
//        velocityM.scale.x = vehicle.velocity_scale * 0.008; // Shaft diameter
//        velocityM.scale.y = vehicle.velocity_scale * 0.04; // Head diameter
//        velocityM.scale.z = vehicle.velocity_scale * 0.04; // Head Length
//        velocityM.color.r = yellow.r;
//        velocityM.color.g = yellow.g;
//        velocityM.color.b = yellow.b;
//        velocityM.color.a = 1.0;
//        markerArray.markers.push_back(velocityM);

            if (i < n_control)
            {
                Eigen::Vector3d tSteeringBody(0.0, mpc_traj.control[i].steering * 2.0 * vehicle.force_scale, 0.0);
                Eigen::Vector3d tSteeringWorld = tq_enu_b * tSteeringBody;
                steeringDeflM.points[2 * i].x = tPos(0);
                steeringDeflM.points[2 * i].y = tPos(1);
                steeringDeflM.points[2 * i].z = tPos(2);
                steeringDeflM.points[2 * i + 1].x = tPos(0) + tSteeringWorld(0);
                steeringDeflM.points[2 * i + 1].y = tPos(1) + tSteeringWorld(1);
                steeringDeflM.points[2 * i + 1].z = tPos(2) + tSteeringWorld(2);
                steeringM.points[i].x = steeringDeflM.points[2 * i + 1].x;
                steeringM.points[i].y = steeringDeflM.points[2 * i + 1].y;
                steeringM.points[i].z = steeringDeflM.points[2 * i + 1].z;

                Eigen::Vector3d tThrottleBody(0.0, 0.0, mpc_traj.control[i].throttle * 3 * 10.0 * vehicle.force_scale);
                Eigen::Vector3d tThrottleWorld = tq_enu_b * tThrottleBody;
                throttleDeflM.points[2 * i].x = tPos(0);
                throttleDeflM.points[2 * i].y = tPos(1);
                throttleDeflM.points[2 * i].z = tPos(2);
                throttleDeflM.points[2 * i + 1].x = tPos(0) + tThrottleWorld(0);
                throttleDeflM.points[2 * i + 1].y = tPos(1) + tThrottleWorld(1);
                throttleDeflM.points[2 * i + 1].z = tPos(2) + tThrottleWorld(2);
                throttleM.points[i].x = throttleDeflM.points[2 * i + 1].x;
                throttleM.points[i].y = throttleDeflM.points[2 * i + 1].y;
                throttleM.points[i].z = throttleDeflM.points[2 * i + 1].z;
            }

            path_errM.points[2 * i].x = mpc_traj.state[i].pos_x;
            path_errM.points[2 * i].y = mpc_traj.state[i].pos_y;
            path_errM.points[2 * i + 1].x = path_params_pos(0, i);
            path_errM.points[2 * i + 1].y = path_params_pos(1, i);
        }
    }

    markerArray.markers.push_back(steeringDeflM);
    markerArray.markers.push_back(throttleDeflM);
    markerArray.markers.push_back(posM);
    markerArray.markers.push_back(path_paramM);
    markerArray.markers.push_back(path_errM);
    markerArray.markers.push_back(steeringM);
    markerArray.markers.push_back(throttleM);

    m_pub.mpcTraj->publish(markerArray);
}
void AmzMiniVisualizer::publishClosedLoopTrajMarkers()
{
    MarkerArrayM markerArray{};

    std::size_t n_state = closed_loop_traj.state.size();
    std::size_t n_control = closed_loop_traj.control.size();

    MarkerM steeringDeflM{};
    steeringDeflM.ns = "steering_defl";
    steeringDeflM.id = 0;
    steeringDeflM.header.frame_id = "enu";
    steeringDeflM.header.stamp = m_pub_time;
    steeringDeflM.type = MarkerM::LINE_LIST;
    steeringDeflM.scale.x = 0.5 * 0.1 * vehicle.force_scale;
    steeringDeflM.points.resize(2 * n_control);
    steeringDeflM.color.r = 1.0; //red.r;
    steeringDeflM.color.g = 1.0; //red.g;
    steeringDeflM.color.b = 1.0; //red.b;
    steeringDeflM.color.a = 1.0;

//    MarkerM throttleDeflM{};
//    throttleDeflM.ns = "throttle_defl";
//    throttleDeflM.id = 0;
//    throttleDeflM.header.frame_id = "enu";
//    throttleDeflM.header.stamp = m_pub_time;
//    throttleDeflM.type = MarkerM::LINE_LIST;
//    throttleDeflM.scale.x = 0.1 * vehicle.force_scale;
//    throttleDeflM.points.resize(2 * n_control);
//    throttleDeflM.color.r = red.r;
//    throttleDeflM.color.g = red.g;
//    throttleDeflM.color.b = red.b;
//    throttleDeflM.color.a = 1.0;
    MarkerM velocityDeflM{}; // Draw past velocity profile instead of throttle
    velocityDeflM.ns = "velocity_defl";
    velocityDeflM.id = 0;
    velocityDeflM.header.frame_id = "enu";
    velocityDeflM.header.stamp = m_pub_time;
    velocityDeflM.type = MarkerM::LINE_LIST;
    velocityDeflM.scale.x = 0.5 * 0.1 * vehicle.force_scale;
    velocityDeflM.points.resize(2 * n_state);
    velocityDeflM.color.r = 1.0; //red.r;
    velocityDeflM.color.g = 1.0; //red.g;
    velocityDeflM.color.b = 1.0; //red.b;
    velocityDeflM.color.a = 1.0;

    MarkerM steeringM{};
    steeringM.ns = "steering";
    steeringM.id = 0;
    steeringM.header.frame_id = "enu";
    steeringM.header.stamp = m_pub_time;
    steeringM.type = MarkerM::LINE_STRIP;
    steeringM.scale.x = 0.5 * 0.15 * vehicle.force_scale;
    steeringM.points.resize(n_control);
    steeringM.color.r = 1.0;
    steeringM.color.g = 1.0;
    steeringM.color.b = 1.0;
    steeringM.color.a = 1.0;

//    MarkerM throttleM{};
//    throttleM.ns = "throttle";
//    throttleM.id = 0;
//    throttleM.header.frame_id = "enu";
//    throttleM.header.stamp = m_pub_time;
//    throttleM.type = MarkerM::LINE_STRIP;
//    throttleM.scale.x = 0.15 * vehicle.force_scale;
//    throttleM.points.resize(n_control);
//    throttleM.color.r = 1.0;
//    throttleM.color.g = 1.0;
//    throttleM.color.b = 1.0;
//    throttleM.color.a = 1.0;
    MarkerM velocityM{}; // Draw past velocity profile instead of throttle
    velocityM.ns = "velocity";
    velocityM.id = 0;
    velocityM.header.frame_id = "enu";
    velocityM.header.stamp = m_pub_time;
    velocityM.type = MarkerM::LINE_STRIP;
    velocityM.scale.x = 0.5 * 0.15 * vehicle.force_scale;
    velocityM.points.resize(n_control);
    velocityM.color.r = 1.0;
    velocityM.color.g = 1.0;
    velocityM.color.b = 1.0;
    velocityM.color.a = 1.0;

    MarkerM posM{};
    posM.ns = "pos";
    posM.id = 0;
    posM.header.frame_id = "enu";
    posM.header.stamp = m_pub_time;
    posM.type = MarkerM::LINE_STRIP;
    posM.scale.x = 0.1 * vehicle.force_scale;
    posM.points.resize(n_state);
    posM.color.r = 0.5;
    posM.color.g = 0.5;
    posM.color.b = 0.5;
    posM.color.a = 1.0;

//    MarkerM path_paramM{};
//    path_paramM.ns = "path_param";
//    path_paramM.id = 0;
//    path_paramM.header.frame_id = "enu";
//    path_paramM.header.stamp = m_pub_time;
//    path_paramM.type = MarkerM::SPHERE_LIST;
//    path_paramM.scale.x =
//    path_paramM.scale.y =
//    path_paramM.scale.z = 1.5 * vehicle_prediction.trace_scale;
//    path_paramM.points.resize(n_state);
//    path_paramM.color.r = blue.r;
//    path_paramM.color.g = blue.g;
//    path_paramM.color.b = blue.b;
//    path_paramM.color.a = 1.0;
//    Eigen::MatrixXd path_params_pos(2, n_state);

//    MarkerM path_errM{};
//    path_errM.ns = "pos_err";
//    path_errM.id = 0;
//    path_errM.header.frame_id = "enu";
//    path_errM.header.stamp = m_pub_time;
//    path_errM.type = MarkerM::LINE_LIST;
//    path_errM.scale.x =
//    path_errM.scale.y =
//    path_errM.scale.z = 0.1 * vehicle_prediction.force_scale;
//    path_errM.points.resize(2 * n_state);
//    path_errM.color.r = white.r;
//    path_errM.color.g = white.g;
//    path_errM.color.b = white.b;
//    path_errM.color.a = 1.0;

//    if (mpc_traj.state[0].path_param != 0 || mpc_traj.state[1].path_param != 0)
//    {
//        for (std::size_t i = 0; i < n_state; i++)
//        {
//            path_params_pos.col(i) = T_enu_track * m_track.eval_pos(mpc_traj.state[i].path_param) + track_pos;
//            path_paramM.points[i].x = path_params_pos(0, i);
//            path_paramM.points[i].y = path_params_pos(1, i);
//        }
//    }

    if (closed_loop_traj.state[0].pos_x != 0)
    {
        for (std::size_t i = 0; i < n_state; i++)
        {
            Eigen::Vector3d tPos(closed_loop_traj.state[i].pos_x, closed_loop_traj.state[i].pos_y, 0.0245);
            Eigen::Vector3d tVel(closed_loop_traj.state[i].vel_x, closed_loop_traj.state[i].vel_y, 0.0);
            Eigen::Quaterniond tq_enu_b(
                    Eigen::AngleAxis<double>(closed_loop_traj.state[i].turn_angle, Eigen::Vector3d::UnitZ()));

            MarkerM meshM{};
            meshM.ns = "pose_points";
            meshM.id = i;
            meshM.header.frame_id = "enu";
            meshM.header.stamp = m_pub_time;
            meshM.color.r = 1.0; //blue.r;
            meshM.color.g = 1.0; //blue.g;
            meshM.color.b = 1.0; //blue.b;
            meshM.color.a = 1.0;
            setPredMeshMarker(meshM, tPos, tq_enu_b);
            markerArray.markers.push_back(meshM);

            posM.points[i].x = tPos(0);
            posM.points[i].y = tPos(1);
            posM.points[i].z = tPos(2);

//        MarkerM velocityM;
//        velocityM.ns = "velocity";
//        velocityM.id = i;
//        velocityM.header.frame_id = "enu";
//        velocityM.header.stamp = m_pub_time;
//        velocityM.points.resize(2);
//        velocityM.points[0].x = tPos(0);
//        velocityM.points[0].y = tPos(1);
//        velocityM.points[0].z = tPos(2);
//        velocityM.points[1].x = tPos(0) + vehicle.velocity_scale * 0.15 * tVel(0);
//        velocityM.points[1].y = tPos(1) + vehicle.velocity_scale * 0.15 * tVel(1);
//        velocityM.points[1].z = tPos(2);
//        velocityM.type = MarkerM::ARROW;
//        velocityM.scale.x = vehicle.velocity_scale * 0.008; // Shaft diameter
//        velocityM.scale.y = vehicle.velocity_scale * 0.04; // Head diameter
//        velocityM.scale.z = vehicle.velocity_scale * 0.04; // Head Length
//        velocityM.color.r = yellow.r;
//        velocityM.color.g = yellow.g;
//        velocityM.color.b = yellow.b;
//        velocityM.color.a = 1.0;
//        markerArray.markers.push_back(velocityM);

//            Eigen::Vector3d tVelocity(0.0, 0.0, tVel.norm() * 0.15 * vehicle.force_scale);
            Eigen::Vector3d tVelWorld = tq_enu_b * tVel;
            Eigen::Vector3d tVelWorldVis(0.0, 0.0, tVelWorld.norm()* 1.0 * vehicle.force_scale);
            velocityDeflM.points[2 * i].x = tPos(0);
            velocityDeflM.points[2 * i].y = tPos(1);
            velocityDeflM.points[2 * i].z = tPos(2);
            velocityDeflM.points[2 * i + 1].x = tPos(0) + tVelWorldVis(0);
            velocityDeflM.points[2 * i + 1].y = tPos(1) + tVelWorldVis(1);
            velocityDeflM.points[2 * i + 1].z = tPos(2) + tVelWorldVis(2);
            velocityM.points[i].x = velocityDeflM.points[2 * i + 1].x;
            velocityM.points[i].y = velocityDeflM.points[2 * i + 1].y;
            velocityM.points[i].z = velocityDeflM.points[2 * i + 1].z;

            if (i < n_control)
            {
                Eigen::Vector3d tSteeringBody(0.0, closed_loop_traj.control[i].steering * 2.0 * vehicle.force_scale, 0.0);
                Eigen::Vector3d tSteeringWorld = tq_enu_b * tSteeringBody;
                steeringDeflM.points[2 * i].x = tPos(0);
                steeringDeflM.points[2 * i].y = tPos(1);
                steeringDeflM.points[2 * i].z = tPos(2);
                steeringDeflM.points[2 * i + 1].x = tPos(0) + tSteeringWorld(0);
                steeringDeflM.points[2 * i + 1].y = tPos(1) + tSteeringWorld(1);
                steeringDeflM.points[2 * i + 1].z = tPos(2) + tSteeringWorld(2);
                steeringM.points[i].x = steeringDeflM.points[2 * i + 1].x;
                steeringM.points[i].y = steeringDeflM.points[2 * i + 1].y;
                steeringM.points[i].z = steeringDeflM.points[2 * i + 1].z;

//                Eigen::Vector3d tThrottleBody(0.0, 0.0, closed_loop_traj.control[i].throttle * 10.0 * vehicle.force_scale);
//                Eigen::Vector3d tThrottleWorld = tq_enu_b * tThrottleBody;
//                throttleDeflM.points[2 * i].x = tPos(0);
//                throttleDeflM.points[2 * i].y = tPos(1);
//                throttleDeflM.points[2 * i].z = tPos(2);
//                throttleDeflM.points[2 * i + 1].x = tPos(0) + tThrottleWorld(0);
//                throttleDeflM.points[2 * i + 1].y = tPos(1) + tThrottleWorld(1);
//                throttleDeflM.points[2 * i + 1].z = tPos(2) + tThrottleWorld(2);
//                throttleM.points[i].x = throttleDeflM.points[2 * i + 1].x;
//                throttleM.points[i].y = throttleDeflM.points[2 * i + 1].y;
//                throttleM.points[i].z = throttleDeflM.points[2 * i + 1].z;
            }

//            path_errM.points[2 * i].x = closed_loop_traj.state[i].pos_x;
//            path_errM.points[2 * i].y = closed_loop_traj.state[i].pos_y;
//            path_errM.points[2 * i + 1].x = path_params_pos(0, i);
//            path_errM.points[2 * i + 1].y = path_params_pos(1, i);
        }
    }

    markerArray.markers.push_back(steeringDeflM);
    markerArray.markers.push_back(velocityDeflM);
    markerArray.markers.push_back(posM);
//    markerArray.markers.push_back(path_paramM);
//    markerArray.markers.push_back(path_errM);
    markerArray.markers.push_back(steeringM);
    markerArray.markers.push_back(velocityM);
    
    m_pub.closedLoopTraj->publish(markerArray);
}
void AmzMiniVisualizer::publishVelocityProfileMarkers()
{
    if (velocity_profile_is_avail)
    {
        /* Copy profile from message */
        std::vector<double> coll = velocity_profile.s;
        std::vector<double> vel = velocity_profile.vel;
        m_velocity_profile.collocation_points = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(coll.data(), coll.size());
        m_velocity_profile.max_velocity = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(vel.data(), vel.size());

        m_velocity_profile.D_poly = 1;
        m_velocity_profile.N_seg = std::round(coll.size() - 1) / m_velocity_profile.D_poly;
        m_velocity_profile.init();

        velocity_profile_is_avail = false;
    }
    else if (velocity_profile.s.empty())
    {
        /* Not avail is due topic not alive. Use default profile */
    }
    else
    {
        /* Topic is alive but no message available at this moment */
        return;
    }

    MarkerArrayM markerArray{};

    MarkerM velProfileDeflM{};
    velProfileDeflM.ns = "vel_profile_defl";
    velProfileDeflM.id = 0;
    velProfileDeflM.header.frame_id = "enu";
    velProfileDeflM.header.stamp = m_pub_time;
    velProfileDeflM.type = MarkerM::LINE_LIST;
    velProfileDeflM.scale.x = 0.1 * vehicle.force_scale;
    velProfileDeflM.points.resize(2 * (m_velocity_profile.N_seg * m_velocity_profile.D_poly + 1));

    double grey_scale = 1.0;
    velProfileDeflM.color.r = grey_scale * indigo.r;
    velProfileDeflM.color.g = grey_scale * indigo.g;
    velProfileDeflM.color.b = grey_scale * indigo.b;
    velProfileDeflM.color.a = 1.0;

    const unsigned N_points = p_vel_profile_N_sample_points; //m_velocity_profile.N_seg * m_velocity_profile.D_poly + 1;
    MarkerM velProfileM{};
    velProfileM.ns = "velocity";
    velProfileM.id = 0;
    velProfileM.header.frame_id = "enu";
    velProfileM.header.stamp = m_pub_time;
    velProfileM.type = MarkerM::LINE_STRIP;
    velProfileM.scale.x = 0.15 * vehicle.force_scale;
    velProfileM.points.resize(N_points);
    velProfileM.color.r = grey_scale * indigo.r;
    velProfileM.color.g = grey_scale * indigo.g;
    velProfileM.color.b = grey_scale * indigo.b;
    velProfileM.color.a = 1.0;

    MarkerM velGradProfileM{};
    velGradProfileM.ns = "velocity_grad";
    velGradProfileM.id = 0;
    velGradProfileM.header.frame_id = "enu";
    velGradProfileM.header.stamp = m_pub_time;
    velGradProfileM.type = MarkerM::LINE_STRIP;
    velGradProfileM.scale.x = 0.075 * vehicle.force_scale;
    velGradProfileM.points.resize(N_points);
    velGradProfileM.color.r = grey_scale * yellow.r;
    velGradProfileM.color.g = grey_scale * yellow.g;
    velGradProfileM.color.b = grey_scale * yellow.b;
    velGradProfileM.color.a = 0.5;

//    Eigen::Matrix<double, 2, p_vel_profile_N_sample_points> track_raceline_pointsXY = m_raceline_points_XY;
//
//    track_raceline_pointsXY = T_enu_track * track_raceline_pointsXY;
//    track_raceline_pointsXY.row(0).array() += track_pos(0);
//    track_raceline_pointsXY.row(1).array() += track_pos(1);

    // Collocation points
    for (unsigned iPoint = 0; iPoint < (velProfileDeflM.points.size() / 2); iPoint++)
    {
        const double eval_arg = m_velocity_profile.collocation_points(iPoint);
        Eigen::Vector2d pos = T_enu_track * m_path.eval(eval_arg) + track_pos;

        velProfileDeflM.points[2*iPoint].x = pos(0);
        velProfileDeflM.points[2*iPoint].y = pos(1);
        velProfileDeflM.points[2*iPoint].z = 0;

        velProfileDeflM.points[2*iPoint+1].x = pos(0);
        velProfileDeflM.points[2*iPoint+1].y = pos(1);
        velProfileDeflM.points[2*iPoint+1].z = 5 * vehicle.force_scale * m_velocity_profile.max_velocity(iPoint);
    }
    markerArray.markers.push_back(velProfileDeflM);

    // Interpolating polynomial
    for (unsigned i = 0; i < N_points; i++)
    {
        const double eval_arg = (i == (N_points - 1)) ?
                                 1 - 0.1 / static_cast<double>(N_points - 1) :
                                 i / static_cast<double>(N_points - 1);
        const Eigen::Vector2d pos = T_enu_track * m_path.eval(eval_arg) + track_pos;

        velProfileM.points[i].x = pos(0);
        velProfileM.points[i].y = pos(1);
        velProfileM.points[i].z = 5 * vehicle.force_scale * m_velocity_profile.eval(eval_arg);

        velGradProfileM.points[i].x = velProfileM.points[i].x;
        velGradProfileM.points[i].y = velProfileM.points[i].y;
        velGradProfileM.points[i].z = velProfileM.points[i].z + 0.1 * 5 * vehicle.force_scale * m_velocity_profile.eval_grad(eval_arg);

//        std::cout << "eval_arg " << eval_arg << ": " << m_velocity_profile.eval(eval_arg) << ", ";
    }
//    std::cout << "\n";
    markerArray.markers.push_back(velProfileM);
    markerArray.markers.push_back(velGradProfileM);

    m_pub.velocityProfile->publish(markerArray);
}
void AmzMiniVisualizer::publishLearningDataMarkers()
{
    learning_data_is_avail = false;

    MarkerArrayM markerArray{};

    const unsigned batch_size = learning_data.path_params.size();
    Eigen::VectorXd path_params(batch_size);
    Eigen::VectorXd mismatch_measures(batch_size);

    for(unsigned i = 0; i < batch_size; i++)
    {
        path_params[i] = learning_data.path_params[i];
        mismatch_measures[i] = learning_data.mismatch_measures[i];
    }

    MarkerM mismatchDeflM{};
    mismatchDeflM.ns = "mismatch_defl";
    mismatchDeflM.id = 0;
    mismatchDeflM.header.frame_id = "enu";
    mismatchDeflM.header.stamp = m_pub_time;
    mismatchDeflM.type = MarkerM::LINE_LIST;
    mismatchDeflM.scale.x = 0.1 * vehicle.force_scale;
    mismatchDeflM.points.resize(2*batch_size);
    double grey_scale = 1.0;
    mismatchDeflM.color.r = grey_scale * yellow.r;
    mismatchDeflM.color.g = grey_scale * yellow.g;
    mismatchDeflM.color.b = grey_scale * yellow.b;
    mismatchDeflM.color.a = 1.0;

    for (unsigned i = 0; i < batch_size; i++)
    {
        const double eval_arg = path_params[i];
        const double local_angle = m_path.eval_tangent_angle(eval_arg);
        const Eigen::Quaterniond q_enu_l(Eigen::AngleAxis<double>(local_angle + track_angle, Eigen::Vector3d::UnitZ()));

        const Eigen::Vector2d pos = T_enu_track * m_path.eval(eval_arg) + track_pos;
        const Eigen::Vector3d l_mismatch(0.0, mismatch_measures[i] * 2.0, 0);
        const Eigen::Vector3d enu_mismatch = q_enu_l * l_mismatch; //not correct

        mismatchDeflM.points[2 * i].x = pos(0);
        mismatchDeflM.points[2 * i].y = pos(1);
        mismatchDeflM.points[2 * i].z = 0;
        mismatchDeflM.points[2 * i + 1].x = mismatchDeflM.points[2 * i].x + enu_mismatch[0];
        mismatchDeflM.points[2 * i + 1].y = mismatchDeflM.points[2 * i].y + enu_mismatch[1];
        mismatchDeflM.points[2 * i + 1].z = 0;
    }

    markerArray.markers.push_back(mismatchDeflM);
    m_pub.learningData->publish(markerArray);
}
/* Helper functions =============================================================================== */
