#ifndef SRC_EIGENVELOCITYPROFILESPLINELA_HPP
#define SRC_EIGENVELOCITYPROFILESPLINELA_HPP

#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include <yaml-cpp/yaml.h>

#include "laopt/laopt.hpp"

namespace mpc_control_racing {
namespace spline_following {

class EigenVelocityProfileLA
{
public:
    int N_seg = 13 * 4;
    int D_poly = 1;

    Eigen::ArrayXd collocation_points = (Eigen::ArrayXd(N_seg * D_poly + 1) <<
        0.0, 0.016333887556857923, 0.04542562581225118, 0.07010861849902277,
        0.07692307692307693, 0.09325696447993485, 0.12234870273532811, 0.1470316954220997,
        0.15384615384615385, 0.17018004140301177, 0.19927177965840503, 0.22395477234517663,
        0.23076923076923078, 0.2471031183260887, 0.27619485658148196, 0.30087784926825356,
        0.3076923076923077, 0.32402619524916565, 0.3531179335045589, 0.3778009261913305,
        0.38461538461538464, 0.4009492721722426, 0.4300410104276358, 0.4547240031144074,
        0.46153846153846156, 0.4778723490953195, 0.5069640873507127, 0.5316470800374843,
        0.5384615384615384, 0.5547954260183964, 0.5838871642737896, 0.6085701569605612,
        0.6153846153846154, 0.6317185029414734, 0.6608102411968666, 0.6854932338836381,
        0.6923076923076923, 0.7086415798645502, 0.7377333181199435, 0.762416310806715,
        0.7692307692307693, 0.7855646567876272, 0.8146563950430205, 0.839339387729792,
        0.8461538461538461, 0.8624877337107041, 0.8915794719660973, 0.9162624646528689,
        0.9230769230769231, 0.9394108106337811, 0.9685025488891743, 0.9931855415759459,
        1.0).finished();

    Eigen::ArrayXd max_velocity = (Eigen::ArrayXd(N_seg * D_poly + 1) <<
        3.302500000000005, 3.377525000000005, 3.477625000000005, 3.452550000000005,
        3.4521500000000045, 3.1272750000000036, 3.2518500000000037, 2.876625000000003,
        2.6264750000000023, 2.651450000000002, 2.4017000000000013, 2.3014000000000014,
        2.3014000000000014, 2.2011000000000003, 2.2014000000000005, 2.576825000000002,
        2.5519500000000024, 3.0272750000000044, 3.2021000000000037, 2.7761250000000017,
        2.501200000000002, 2.176125000000001, 2.1509500000000004, 2.15185,
        2.15185, 2.7520500000000023, 3.0266750000000022, 2.826275000000002,
        2.4763250000000014, 2.3018000000000014, 2.7265750000000026, 2.676725000000002,
        2.676725000000002, 2.6761250000000016, 1.976125, 2.201400000000001,
        2.351450000000001, 2.5269750000000024, 3.5026000000000046, 3.977025000000005,
        3.227275000000004, 3.1023000000000045, 3.2766250000000032, 2.5015000000000023,
        2.5015000000000023, 2.476325000000002, 2.401900000000001, 2.8018000000000023,
        2.9516500000000025, 2.652450000000004, 3.1523500000000038, 3.1524500000000044,
        3.302500000000005).finished();

    void init()
    {
        // Duplicate velocity profile
        // max_velocity.insert(max_velocity.end(), max_velocity.begin(), max_velocity.end());
        // std::size_t n_points = collocation_points.size();
        // for (std::size_t i = 1; i < n_points; i++)
        // {
        //     collocation_points.push_back(collocation_points[n_points - 1] + collocation_points[i]);
        // }

        const unsigned n_points = N_seg * D_poly + 1;

        if (collocation_points.size() == n_points) // If profile has not yet been duplicated
        {
            // Duplicate velocity profile (append second round)
            const Eigen::ArrayXd collocation_points_2nd = collocation_points + collocation_points(N_seg * D_poly);
            collocation_points.conservativeResize(N_seg * D_poly + n_points);
            collocation_points.bottomRows(n_points) = collocation_points_2nd;

            // For velocity, second round values are identical with first (do not count up)
            const Eigen::ArrayXd max_velocity_2nd = max_velocity;
            max_velocity.conservativeResize(N_seg * D_poly + n_points);
            max_velocity.bottomRows(n_points) = max_velocity_2nd;
            
            // std::cout << "EigenVelocityProfileLA::init()\n"
            //              "collocation_points after duplication:\n"
            //           <<  collocation_points.topRows(2 * N_seg * D_poly).reshaped(D_poly, 2 * N_seg).transpose()
            //           << "\n" << collocation_points.bottomRows(1) << "\n";
            // std::cout << "velocity_points after duplication:\n"
            //           <<  max_velocity.topRows(2 * N_seg * D_poly).reshaped(D_poly, 2 * N_seg).transpose()
            //           << "\n" << max_velocity.bottomRows(1) << "\n";
        }
        else
        {
            std::cout << "EigenVelocityProfileLA::init(): Profile is already duplicated\n";
        }

//        test();
    }
public:
    explicit EigenVelocityProfileLA(const bool initialize = false)
    {
        if (initialize) { init(); }
    }
    void load_from_yaml(const std::string &yaml_filepath)
    {
        YAML::Node yamlFile;
        try
        {
            yamlFile = YAML::LoadFile(yaml_filepath);
            std::cout << "Loaded velocity profile from " << yaml_filepath << "\n";
        }
        catch (const std::exception &ex)
        {
            std::cerr << "YAML file " << yaml_filepath << " could not be opened: \n" << ex.what() << "\n";
        }
        N_seg = yamlFile["velocity_profile"]["N_seg"].as<double>();
        D_poly = yamlFile["velocity_profile"]["D_poly"].as<double>();

        auto temp1 = yamlFile["velocity_profile"]["max_velocity"].as<std::vector<double>>();
        max_velocity = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(temp1.data(), temp1.size());

        auto temp2 = yamlFile["velocity_profile"]["collocation_points"].as<std::vector<double>>();
        collocation_points = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(temp2.data(), temp2.size());

        init();
    }

    void test() const
    {
        std::cout << "test()\n";
        std::cout << "      Velocity profile has " << N_seg << " segments and the order of polynomial is " << D_poly << "\n";
        std::cout << "N_seg = " << N_seg << " D_poly = " << D_poly << " len(collocation_points): " << collocation_points.size() << "\n";
        std::cout << std::setprecision(5);
        std::cout << "      Testing Lagrangian Polynomial\n";
        std::cout << L(0, 0, 0.01) << "\n";
        std::cout << L(1, 0, 0.01) << "\n";
        std::cout << "      Testing derivative Lagrangian Polynomial\n";
        std::cout << dL(0, 0, 0.01) << "\n";
        std::cout << dL(1, 0, 0.01) << "\n";

        std::cout << "      Testing Approximation (segment 0 and last)\n";
        std::cout << dapr(0, 0.01) << "\n";
        std::cout << dapr(0, 0.02) << "\n";
        std::cout << dapr(N_seg - 1, 0.99) << "\n";

        std::cout << "      Testing derivative Approximation (segment 0 and last)\n";
        std::cout << dapr(0, 0.01) << "\n";
        std::cout << dapr(0, 0.02) << "\n";
        std::cout << dapr(N_seg - 1, 0.99) << "\n";

        std::cout << "      Testing global eval function\n";
        std::cout << eval(0.0) << "\n";
        std::cout << eval(0.01) << "\n";
        std::cout << eval(0.0768) << "\n";
        std::cout << eval(0.077) << "\n";
        std::cout << eval(0.94) << "\n";
        std::cout << eval(0.96) << "\n";
        std::cout << eval(0.98) << "\n";
        std::cout << eval(1.0) << "\n";

        std::cout << "      Testing global eval_grad function\n";
        std::cout << eval_grad(0.0) << "\n";
        std::cout << eval_grad(0.01) << "\n";
        std::cout << eval_grad(0.0768) << "\n";
        std::cout << eval_grad(0.077) << "\n";
        std::cout << eval_grad(0.94) << "\n";
        std::cout << eval_grad(0.96) << "\n";
        std::cout << eval_grad(0.98) << "\n";
        std::cout << eval_grad(1.0) << "\n";
    }

    template<typename T>
    inline T eval(const T &s) const
    {
        // std::cout << "eval(" << s << ")\n";

        // Find the segment s belongs to
        std::size_t seg_idx = 0;

        // Sample at most from last segment && find segment s lies in: s \in [left_edge, right_edge)
        while (seg_idx < static_cast<std::size_t>(2 * N_seg - 1) // Spline is duplicated
               && s >= collocation_points[(seg_idx + 1) * D_poly]) { seg_idx++; }
        // std::cout << "seg_idx: " << seg_idx << ", seg_borders: " << collocation_points[seg_idx * D_poly] << ", " << collocation_points[(seg_idx + 1) * D_poly] << "\n";

        T v = apr(seg_idx, s);
        return v;
    }

    template<typename T>
    inline T eval_grad(const T &s) const
    {
        // std::cout << "eval_grad(" << s << ")\n";

        // Find the segment s belongs to
        int seg_idx = 0;

        while (seg_idx < (2 * N_seg - 1) // Spline is duplicated
               && s >= collocation_points[(seg_idx + 1) * D_poly]) { seg_idx++; }
        //        std::cout << "          Path param: " << s <<" belongs to segment: " << seg_idx << "\n";

        T v = dapr(seg_idx, s);
        return v;
    }

    // Evaluate using Lagrange Polynomials
    template<typename T>
    T apr(int kSeg, const T &s_eval) const
    {
        // std::cout << "apr(seg_idx: " << kSeg << ", s_eval: " << s_eval << ")\n";
        const int kSeg_start = kSeg * D_poly;

        T v = T(0);
        for (int i = 0; i < D_poly + 1; ++i)
        {
            v += T(max_velocity[kSeg_start + i] * L(i, kSeg, s_eval));
        }
        return v;
    }
    template<typename T>
    T dapr(int kSeg, T s_eval) const
    {
        const int kSeg_start = kSeg * D_poly;

        T dv = T(0);
        for (int i = 0; i < D_poly + 1; ++i)
        {
            dv += max_velocity[kSeg_start + i] * dL(i, kSeg, s_eval);
        }
        return dv;
    }

    // Lagrange polynomials
    template<typename T>
    T L(int j, int kSeg, T s_eval) const
    {
        const int kSeg_start = kSeg * D_poly;

        T L = T(1);
        for (int l = 0; l < D_poly + 1; ++l)
        {
            if (l == j) continue;
            L *= (s_eval - collocation_points[kSeg_start + l]) /
                 (collocation_points[kSeg_start + j] - collocation_points[kSeg_start + l]);
        }
////        std::cout << "          j: " << j << " segment: " << seg << " s_eval: " << s_eval << " L: " << L << "\n";
        return L;
    }
    template<typename T>
    T dL(int j, int kSeg, T s_eval) const
    {
        const int kSeg_start = kSeg * D_poly;

        T dL = T(0);
        for (int i = 0; i < D_poly + 1; ++i)
        {
            if (i == j) continue;
            T L = T(1);
            for (int l = 0; l < D_poly + 1; ++l)
            {
                if (l == j || l == i) continue;
                L *= (s_eval - collocation_points[kSeg_start + l]) /
                     (collocation_points[kSeg_start + j] - collocation_points[kSeg_start + l]);
            }
            dL += L / (collocation_points[kSeg_start + j] - collocation_points[kSeg_start + i]);
        }
//        std::cout << "          j: " << j << " segment: " << seg << " s_eval: " << s_eval << " dL: " << dL << "\n";

        return dL;
    }
};
}
};
#endif //SRC_EIGENVELOCITYPROFILESPLINELA_HPP
