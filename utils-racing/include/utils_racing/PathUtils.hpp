#ifndef SRC_PATHUTILS_HPP
#define SRC_PATHUTILS_HPP

#include <iostream>
#include <iomanip>
#include <chrono>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "utils_racing/Path.hpp"

namespace utils_racing {

class PathAnalyzer
{
public:
    struct PathPoint
    {
        Eigen::Vector2d end_pos{};
        double end_path_param{};
//        double total_dist{}; //TODO Wait to have sparse path representation (only node points)
        double dist_remaining{};
    };

    void update(const std::shared_ptr<PathBase> &path, const double closest_path_param, const int path_fwd)
    {
        /* Determine whether on straight leg and final point of this leg */
        constexpr double pparam_search_step{0.001};

        /* Find end of curve */
        double pparam_curve_end{closest_path_param};
        while (std::abs(path->eval_curvature(pparam_curve_end + path_fwd * pparam_search_step)) > 1e-6)
        {
            pparam_curve_end += path_fwd * pparam_search_step;
        }
        m_curve.end_path_param = pparam_curve_end;
        m_curve.end_pos = path->eval_pos(pparam_curve_end);
//        m_curve.total_dist = 0; (m_curve.end_pos - m_straight.end_pos).norm();  //TODO use path arc
        m_curve.dist_remaining = (m_curve.end_pos - path->eval_pos(closest_path_param)).norm(); //TODO use path arc
//            std::cout << "New curve: ~~ " << m_curve.total_dist << " ~~ " << pparam_curve_end << "\n";

        /* Find end of straight */
        double pparam_straight_end{pparam_curve_end};
        while (std::abs(path->eval_curvature(pparam_straight_end + path_fwd * pparam_search_step)) < 1e-6)
        {
            pparam_straight_end += path_fwd * pparam_search_step;
        }
        m_straight.end_path_param = pparam_straight_end;
        m_straight.end_pos = path->eval_pos(pparam_straight_end);
//            m_straight.total_dist = 0;  //TODO use path arc

        m_straight.dist_remaining = on_straight() ?
                                    (m_straight.end_pos - path->eval_pos(closest_path_param)).norm() :
                                    (m_straight.end_pos - m_curve.end_pos).norm();
    }

    bool in_curve() const { return (m_curve.dist_remaining > 0); }
    bool on_straight() const { return !in_curve(); }

    const PathPoint &curve() const { return m_curve; }
    const PathPoint &straight() const { return m_straight; }

protected:
    PathPoint m_curve;
    PathPoint m_straight;
};

class PathLocator
{
public:
    double find_closest_path_param(const std::shared_ptr<PathBase> &path, const Eigen::Vector2d &pos,
                                   const double guess = 0.5, const double range = 0.5, const double tol_m = 5e-3) const
    {
        return grid_search(path, pos, guess, range, tol_m);
    }

    template<unsigned MAX_ITERATIONS = 10, unsigned N_GRID_POINTS = 17>
    double grid_search(const std::shared_ptr<PathBase> &path, const Eigen::Vector2d &pos,
                       const double guess = 0.5, const double range = 0.5, const double tol_m = 5e-3) const
    {
        using namespace std::chrono;
        const steady_clock::time_point start = steady_clock::now();

        Eigen::Matrix<double, N_GRID_POINTS, 1> param_grid;
        Eigen::Matrix<double, N_GRID_POINTS, 1> value_grid;

        double minimizing_param = guess;

        /* Start with area around the guess */
        double path_param_upper = guess + range;
        double path_param_lower = guess - range;

        unsigned iterations_taken = MAX_ITERATIONS;

        for (unsigned k = 0; k < MAX_ITERATIONS; k++) // Iterations
        {
            double minimum = std::numeric_limits<double>::max();

            /* Populate grid */
            param_grid = Eigen::Matrix<double, N_GRID_POINTS, 1>::LinSpaced(path_param_lower, path_param_upper);

            if (m_VERBOSE >= PrintLevel2)
            {
                std::cout << "Grid search iteration " << k + 1 << ": "
                          << "param = [" << path_param_lower << ", " << path_param_upper << "]\n";
            }

            unsigned grid_id_min = 0;

            for (unsigned l = 0; l < N_GRID_POINTS; l++)
            {
                value_grid(l) = distance_to_point(path, param_grid(l), pos);
                if (m_VERBOSE >= PrintLevel2)
                {
                    std::cout << "test " << param_grid(l) << ": " << value_grid(l);
                }
                if (value_grid(l) <= minimum)
                {
                    grid_id_min = l;
                    minimum = value_grid(l);
                    if (m_VERBOSE >= PrintLevel2) { std::cout << " -- new min!\n"; }
                }
                else if (m_VERBOSE >= PrintLevel2) { std::cout << "\n"; }
            }
            minimizing_param = param_grid(grid_id_min);
            if (m_VERBOSE >= PrintLevel2)
            {
                std::cout << "==> At " << minimizing_param << ": " << minimum << "\n";
            }

            /* Zoom in: neighbors of the minimum are the new bounds */
            const unsigned grid_upper_neighbor = std::min(grid_id_min + 1, N_GRID_POINTS - 1);
            const unsigned grid_lower_neighbor = (grid_id_min > 0 ? grid_id_min - 1 : 0);

            path_param_upper = param_grid(grid_upper_neighbor);
            path_param_lower = param_grid(grid_lower_neighbor);

            /* Converged when the distances at the neighbors of the minimum agree */
            if (std::abs(value_grid(grid_upper_neighbor) - value_grid(grid_lower_neighbor)) < tol_m)
            {
                iterations_taken = k;
                break;
            }
        }

        if (m_VERBOSE >= PrintLevel1)
        {
            const steady_clock::time_point stop = steady_clock::now();
            std::cout << "Grid search finished after " << iterations_taken << " iterations (" << duration_cast<microseconds>(stop - start).count() << " us)\n";
        }

        // Find local min with binary search
//            return binary_search(path, pos, 0.5 * (path_param_lower + path_param_upper),
//                                 0.5 * (path_param_upper - path_param_lower));
//        std::cout << "Grid search: RES " << minimizing_param << " | " << grid_id_min << ", index grid: " << index_grid.transpose() << "\n";
//        std::cout << "dist: " << distance_to_point(path, minimizing_param, pos) << "\n";
        return wrap01(minimizing_param);
    }

    static double distance_to_point(const std::shared_ptr<PathBase>& path,
                                    const double path_param, const Eigen::Vector2d& pos)
    {
        return (path->eval_pos(path_param) - pos).norm();
    }

    static double wrap01(double arg)
    {
        while (arg > 1.0) { arg -= 1; }
        while (arg < 0) { arg += 1; }
        return arg;
    }

private:
    /* Debug helpers */
    enum PrintLevel
    {
        PrintLevel0, // Nothing
        PrintLevel1, // One-time settings before run
        PrintLevel2  // Continuous prints during run
    };
    static constexpr int m_VERBOSE = PrintLevel0;
};

} // namespace utils_racing

#endif //SRC_PATHUTILS_HPP
