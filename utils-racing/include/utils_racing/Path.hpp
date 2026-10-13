#ifndef UTILS_RACING_PATH_HPP
#define UTILS_RACING_PATH_HPP

#include <iostream>
#include <iomanip>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "utils_racing/Spline1D.hpp"

namespace utils_racing {

struct PathInfo
{
    PathInfo(double length_, double width_, double x_init_, double y_init_, double yaw_init_) :
            length(length_), width(width_), x_init(x_init_), y_init(y_init_), yaw_init(yaw_init_) {}
    PathInfo() = default;
    friend std::ostream &operator<<(std::ostream &os, const PathInfo &info)
    {
        os << "length: " << info.length << ", width: " << info.width << ", "
           << "xy_init: (" << info.x_init << ", " << info.y_init << "), "
           << "yaw_init: " << info.yaw_init * 180.0 / M_PI << " deg";
        return os;
    }
    double length{0};
    double width{0};
    double x_init{0};
    double y_init{0};
    double yaw_init{0};
};

class PathBase
{
public:
    bool is_loaded() { return m_isLoaded; }
    const PathInfo &get_info() const { return m_info; }
    virtual unsigned get_size() const = 0;

    // Separate properties
    virtual Eigen::Vector2d eval_pos(double arg, int incr = 0) const = 0;
    virtual Eigen::Vector2d eval_grad(double arg, int incr = 0) const = 0;
    virtual double eval_arclength(double arg, int incr = 0) const = 0;
    virtual double eval_tangent_angle(double arg, int incr = 0) const = 0;
    virtual double eval_curvature(double arg, int incr = 0) const = 0;

    // All in one vector
    virtual Eigen::VectorXd eval(double arg, int incr = 0) const = 0;

protected:
    bool m_isLoaded{false};
    PathInfo m_info{};
};

class PathSpline : public PathBase
{
public:
    PathSpline() = default;
    explicit PathSpline(const std::string &yaml_filepath) { load_from_yaml(yaml_filepath); }

    void load_from_yaml(const std::string &yaml_filepath,
                        const std::vector<std::string> &keys = {"track", "lowRes"})
    {
        YAML::Node yamlNode;
        try
        {
            yamlNode = YAML::LoadFile(yaml_filepath);
            std::cout << "Loading spline track from " << yaml_filepath << "\n";
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Spline track YAML file " << yaml_filepath << " could not be opened:\n" << ex.what() << "\n";
        }

        // Navigate to sub-node through yaml data structure
        for (const std::string &key: keys)
        {
            yamlNode = yamlNode[key];
        }

        /* Load info */
        const auto length = yamlNode["trackLength"].as<double>();
        const auto width = yamlNode["trackWidth"].as<double>();
        const auto x_init = yamlNode["x_init"].as<double>();
        const auto y_init = yamlNode["y_init"].as<double>();
        const auto yaw_init = yamlNode["yaw_init"].as<double>();
        m_info = PathInfo(length, width, x_init, y_init, yaw_init);

        /* Load data */
        {
            std::vector<std::string> property_path = keys;
            property_path.emplace_back("");

            const std::string breaks_name = "breaks";
            const std::string coeffs_name = "coefs";

            property_path.back() = "xCoords";
            m_x.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            property_path.back() = "yCoords";
            m_y.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            property_path.back() = "xRate";
            m_x_grad.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            property_path.back() = "yRate";
            m_y_grad.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            property_path.back() = "tangentAngle";
            m_tanAngle.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            property_path.back() = "arcLength";
            m_arcLength.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);

            try
            {
                property_path.back() = "curvature";
                m_curv.load_from_yaml(yaml_filepath, property_path, breaks_name, coeffs_name);
                m_curvature_isLoaded = true;
            }
            catch (const std::exception &ex)
            {
                std::cerr << "Property [curvature] could not be loaded.\n" << ex.what() << "\n";
            }
        }
        m_isLoaded = true;
    }

    unsigned get_size() const { return m_x.get_size(); }
    void print_preview() const
    {
        std::cout << "eval() samples:\n" << std::fixed << std::setprecision(4)
                  << "0.0: " << eval(0).transpose() << "\n"
                  << "0.1: " << eval(0.1).transpose() << "\n"
                  << "0.4: " << eval(0.4).transpose() << "\n"
                  << "0.5: " << eval(0.5).transpose() << "\n"
                  << "0.8: " << eval(0.8).transpose() << "\n" << std::defaultfloat;
        std::cout << "eval_pos() samples:\n" << std::fixed << std::setprecision(4)
                  << "0.0: " << eval_pos(0).transpose() << "\n"
                  << "0.1: " << eval_pos(0.1).transpose() << "\n"
                  << "0.4: " << eval_pos(0.4).transpose() << "\n"
                  << "0.5: " << eval_pos(0.5).transpose() << "\n"
                  << "0.8: " << eval_pos(0.8).transpose() << "\n" << std::defaultfloat;
    }

    Eigen::Vector2d eval_pos(double v, int incr = 0) const override { return {m_x.eval(v), m_y.eval(v)}; }
    Eigen::Vector2d eval_grad(double v, int incr = 0) const override { return {m_x_grad.eval(v), m_y_grad.eval(v)}; }
    double eval_arclength(double v, int incr = 0) const override { return m_arcLength.eval(v); }
    double eval_tangent_angle(double v, int incr = 0) const override { return m_tanAngle.eval(v); }
    double eval_curvature(double v, int incr = 0) const override
    {
        if (m_curvature_isLoaded) return m_curv.eval(v);
        else std::cerr << "Curvature not loaded. Returning 0.\n";
        return 0;
    }

    // All in one vector
    Eigen::VectorXd eval(double v, int incr = 0) const
    {
        Eigen::VectorXd vec;
        if (m_curvature_isLoaded)
        {
            vec.resize(7);
            vec << eval_pos(v, incr), eval_grad(v), eval_tangent_angle(v), eval_arclength(v), eval_curvature(v);
        }
        else
        {
            vec.resize(6);
            vec << eval_pos(v, incr), eval_grad(v), eval_tangent_angle(v), eval_arclength(v);
        }
        return vec;
    }

    std::array<Eigen::Vector2d, 3> eval_centerline_with_edge_pos(double arg, int incr = 0) const
    {
        const Eigen::Vector2d centerline_point(m_x.eval(arg), m_y.eval(arg));
        const Eigen::Vector2d gradient(m_x_grad.eval(arg), m_y_grad.eval(arg));
        const Eigen::Vector2d cross_gradient = {-gradient(1), gradient(0)};
        const Eigen::Vector2d edge_point1 = m_info.width / 2.0 * cross_gradient;
        const Eigen::Vector2d edge_point2 = -m_info.width / 2.0 * cross_gradient;

        const std::array<Eigen::Vector2d, 3> edge_points = {centerline_point,
                                                            centerline_point + edge_point1,
                                                            centerline_point + edge_point2};
        return edge_points;
    }

private:
    Spline1D m_x{};
    Spline1D m_y{};
    Spline1D m_x_grad{};
    Spline1D m_y_grad{};
    Spline1D m_tanAngle{};
    Spline1D m_arcLength{};

    Spline1D m_curv{};
    bool m_curvature_isLoaded{false};
};

class PathSampled : public PathBase
{
public:
    PathSampled() = default;
    explicit PathSampled(const std::string &yaml_filepath) { load_from_yaml(yaml_filepath); }

    void load_from_yaml(const std::string &yaml_filepath)
    {
        YAML::Node yamlNode;
        try
        {
            yamlNode = YAML::LoadFile(yaml_filepath);
            std::cout << "Loading sampled track from " << yaml_filepath << "\n";
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Sampled track YAML file " << yaml_filepath << " could not be opened:\n" << ex.what() << "\n";
        }

        // Navigate to sub-node through yaml data structure
        yamlNode = yamlNode["track"];

        std::vector<double> xCoords, yCoords, xRate, yRate, tangentAngle, arcLength, curvature;
        try
        {
            const auto trackLength = yamlNode["trackLength"].as<double>();
            const auto trackWidth = yamlNode["trackWidth"].as<double>();
            const auto x_init = yamlNode["x_init"].as<double>();
            const auto y_init = yamlNode["y_init"].as<double>();
            const auto yaw_init = yamlNode["yaw_init"].as<double>();
            m_info = PathInfo(trackLength, trackWidth, x_init, y_init, yaw_init);

            xCoords = yamlNode["xCoords"].as<std::vector<double>>();
            yCoords = yamlNode["yCoords"].as<std::vector<double>>();
            xRate = yamlNode["xRate"].as<std::vector<double>>();
            yRate = yamlNode["yRate"].as<std::vector<double>>();
            tangentAngle = yamlNode["tangentAngle"].as<std::vector<double>>();
            arcLength = yamlNode["arcLength"].as<std::vector<double>>();
            curvature = yamlNode["curvature"].as<std::vector<double>>();
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Sampled track YAML file " << yaml_filepath << " is not in the expected format.\n"
                      << ex.what() << "\n";
        }

        track_data.resize(xCoords.size(), 7);
        track_data.col(0) = vecToEigen(xCoords);
        track_data.col(1) = vecToEigen(yCoords);
        track_data.col(2) = vecToEigen(xRate);
        track_data.col(3) = vecToEigen(yRate);
        track_data.col(4) = vecToEigen(tangentAngle);
        track_data.col(5) = vecToEigen(arcLength);
        track_data.col(6) = vecToEigen(curvature);

        track_data.col(0).array() -= m_info.x_init;
        track_data.col(1).array() -= m_info.y_init;

        // Use half of track only
        //track_info.length *=1; // track length in yaml is for 1x track
//        Eigen::MatrixXd temp(xCoords.size() / 2, 7);
//        temp = track_data.topRows(xCoords.size() / 2);
//        track_data = Eigen::MatrixXd(xCoords.size() / 2, 7);
//        track_data = temp;
        track_data = track_data.topRows(xCoords.size() / 2);

        m_isLoaded = true;
    }

    unsigned get_size() const { return track_data.rows(); }
    void print_preview() const
    {
        std::cout << "eval() samples:\n" << std::fixed << std::setprecision(4)
                  << "0.0: " << eval(0).transpose() << "\n"
                  << "0.1: " << eval(0.1).transpose() << "\n"
                  << "0.4: " << eval(0.4).transpose() << "\n"
                  << "0.5: " << eval(0.5).transpose() << "\n"
                  << "0.8: " << eval(0.8).transpose() << "\n" << std::defaultfloat;
        std::cout << "eval_pos() samples:\n" << std::fixed << std::setprecision(4)
                  << "0.0: " << eval_pos(0).transpose() << "\n"
                  << "0.1: " << eval_pos(0.1).transpose() << "\n"
                  << "0.4: " << eval_pos(0.4).transpose() << "\n"
                  << "0.5: " << eval_pos(0.5).transpose() << "\n"
                  << "0.8: " << eval_pos(0.8).transpose() << "\n" << std::defaultfloat;
    }

    Eigen::Vector2d eval_pos(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        const unsigned idx = bounded_index(arg, incr);
        return track_data.block<1, 2>(idx, 0);
    }
    Eigen::Vector2d eval_grad(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        const unsigned idx = bounded_index(arg, incr);
        return track_data.block<1, 2>(idx, 2);
    }
    double eval_arclength(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        return track_data(bounded_index(arg, incr), 5);
    }
    double eval_tangent_angle(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        return track_data(bounded_index(arg, incr), 4);
    }
    double eval_curvature(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        return track_data(bounded_index(arg, incr), 6);
    }

    // All in one vector
    Eigen::VectorXd eval(double arg, int incr = 0) const override
    {
        /// Convention: (0 <= arg): first points, (arg < 1): last point
        return track_data.row(bounded_index(arg, incr));
    }

    // Get boundaries
    std::array<Eigen::Vector2d, 3> eval_centerline_with_edge_pos(double arg, int incr = 0) const
    {
        const unsigned bounded_idx = bounded_index(arg, incr);
        const Eigen::Vector2d centerpoint = track_data.block<1, 2>(bounded_idx, 0);
        const Eigen::Vector2d gradient = track_data.block<1, 2>(bounded_idx, 2);
        const Eigen::Vector2d cross_gradient = {-gradient(1), gradient(0)};

        const Eigen::Vector2d edgepoint1 = m_info.width / 2.0 * cross_gradient;
        const Eigen::Vector2d edgepoint2 = -m_info.width / 2.0 * cross_gradient;

        const std::array<Eigen::Vector2d, 3> edge_points = {centerpoint,
                                                            centerpoint + edgepoint1,
                                                            centerpoint + edgepoint2};
        return edge_points;
    }

private:
    Eigen::MatrixXd track_data;

    inline unsigned bounded_index(double arg, int increment) const
    {
        return normalize_idx((int) std::floor(normalize_arg(arg) * (double) track_data.rows()) + increment);
    }

    static inline double normalize_arg(double arg)
    {
        while (arg >= 1) { arg -= 1; }
        while (arg < 0) { arg += 1; }
        return arg;
    }
    inline int normalize_idx(int index) const
    {
        while (index > (int) get_size()) { index -= (int) get_size(); }
        while (index < 0) { index += (int) get_size(); }
        return index;
    }

    static inline Eigen::VectorXd vecToEigen(std::vector<double> &vec)
    {
        return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), static_cast<Eigen::Index>(vec.size()));
    }
};

} // namespace utils_racing

#endif //UTILS_RACING_PATH_HPP
