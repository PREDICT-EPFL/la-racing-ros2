#ifndef SRC_EIGEN_TRACK_SPLINE_LA_HPP
#define SRC_EIGEN_TRACK_SPLINE_LA_HPP

#include <vector>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>
#include <yaml-cpp/yaml.h>
#include "laopt/laopt.hpp"

namespace mpc_control_racing {
namespace spline_following {

class EigenSpline
{
public:
    struct SplineInfo
    {
        unsigned pieces{10};
        unsigned order{4};
        const unsigned dim{2};
        double spline_length{10.710722313718323};
    } spline_info{};

    EigenSpline()
    {
        set_default_data();
        init();
    }

    void load_from_yaml(const std::string &yaml_filepath, const std::string &coeff_key = "pos")
    {
        // Try to open file
        YAML::Node yamlFile;
        try
        {
            yamlFile = YAML::LoadFile(yaml_filepath);
            std::cout << "Loaded spline from " << yaml_filepath << "\n";
        }
        catch (const std::exception &ex)
        {
            std::cerr << "YAML file " << yaml_filepath << " could not be opened:\n" << ex.what() << "\n";
        }

        // Try to read file
        try
        {
            std::cout << "Loading spline data: " << coeff_key << "... ";
            std::vector<double> breaks = yamlFile["spline_data"]["breaks"].as<std::vector<double>>();
            break_data = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(breaks.data(), breaks.size());

            // Load via container
            std::vector<std::vector<double>> coefs;
            for (const auto &yamlRow : yamlFile["spline_data"][coeff_key])
            {
                coefs.push_back(yamlRow.as<std::vector<double>>());
            }

            coef_data.resize(coefs.size(), coefs[0].size());
            for (unsigned i = 0; i < coefs.size(); ++i)
            {
                coef_data.row(i) = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(coefs[i].data(), coefs[i].size());
            }

            // New spline file
            spline_info.spline_length = yamlFile["spline_info"]["spline_length"].as<double>();
            spline_info.pieces = static_cast<unsigned>(yamlFile["spline_info"]["pieces"].as<double>());
            spline_info.order = static_cast<unsigned>(yamlFile["spline_info"]["order"].as<double>());
        }
        catch (const std::exception &ex)
        {
            std::cout << "Loading legacy spline file... " << std::flush;
            // Legacy spline file
            std::vector<double> breaks = yamlFile["track"]["breaks_data"].as<std::vector<double>>();
            break_data = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(breaks.data(), breaks.size());

            std::vector<double> coefs = yamlFile["track"]["coeff_data"].as<std::vector<double>>();
            coef_data = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(coefs.data(), coefs.size());

            spline_info.pieces = break_data.rows() - 1;

            spline_info.order = 4;
            coef_data = coef_data.reshaped(spline_info.order, spline_info.dim * spline_info.pieces).transpose();
        }
        std::cout << "OK\n";

        init();
    }

    template<typename T>
    inline Eigen::Vector2<T> eval(const T& t)
    {
        T t_scaled = break_data.tail(1)(0) * t;
        int id_low = 0;
        while ((id_low < breaks_.size() - 1) && (breaks_[id_low + 1] <= t_scaled)) { id_low++; }
        T tau = t_scaled - breaks_[id_low];

        const Eigen::ArrayX<T> coefs_x = coefs_.row(2 * id_low).template cast<T>();
        const Eigen::ArrayX<T> coefs_y = coefs_.row(2 * id_low + 1).template cast<T>();

        return Eigen::Vector2<T>(poly_eval_horner(coefs_x, tau), poly_eval_horner(coefs_y, tau));
//        // Evaluate polynomial gradient using Horner's method
//        const unsigned D = spline_info.order - 1; // Polynomial degree
//        T x_pos = static_cast<T>(coefs_(2 * id_low, 0));
//        T y_pos = static_cast<T>(coefs_(2 * id_low + 1, 0));
//        for (int i = 1; i <= D; i++)
//        {
//            x_pos *= tau;
//            x_pos += coefs_(2 * id_low, i);
//            y_pos *= tau;
//            y_pos += coefs_(2 * id_low + 1, i);
//        }
//        return Eigen::Vector2<T>(x_pos, y_pos);
    }
    template<typename T>
    inline Eigen::Vector2<T> eval_grad(const T& t)
    {
        T t_scaled = break_data.tail(1)(0) * t;
        int id_low = 0;
        while ((id_low < breaks_.size() - 1) && (breaks_[id_low + 1] <= t_scaled)) { id_low++; }
        T tau = t_scaled - breaks_[id_low];

        const Eigen::ArrayX<T> coefs_x = coefs_.row(2 * id_low).template cast<T>();
        const Eigen::ArrayX<T> coefs_y = coefs_.row(2 * id_low + 1).template cast<T>();

        return Eigen::Vector2<T>(poly_deval_nth(coefs_x, tau, 1), poly_deval_nth(coefs_y, tau, 1)) / static_cast<T>(spline_info.spline_length);
//        // Evaluate polynomial gradient using Horner's method (hardcoded for first derivative)
//        const unsigned D = spline_info.order - 1; // Polynomial degree
//        T x_grad = static_cast<T>(D * coefs_(2 * id_low, 0));
//        T y_grad = static_cast<T>(D * coefs_(2 * id_low + 1, 0));
//        for (int i = 1; i <= D - 1; i++)
//        {
//            x_grad *= tau;
//            x_grad += (D - i) * coefs_(2 * id_low, i);
//            y_grad *= tau;
//            y_grad += (D - i) * coefs_(2 * id_low + 1, i);
//        }
    }
    template<typename T>
    inline T eval_tangent(const T& t)
    {
        Eigen::Vector<T, 2> grad = eval_grad(t);
        return atan2(grad(1), grad(0));
    }
    template<typename T>
    inline Eigen::Vector2<T> eval_hes_diag(const T& t)
    {
        T t_scaled = break_data.tail(1)(0) * t;
        std::size_t id_low = 0;
        while ((id_low < breaks_.size() - 1) && (breaks_[id_low + 1] <= t_scaled)) { id_low++; }
        T tau = t_scaled - breaks_[id_low];

        const Eigen::ArrayX<T> coefs_x = coefs_.row(2 * id_low).template cast<T>();
        const Eigen::ArrayX<T> coefs_y = coefs_.row(2 * id_low + 1).template cast<T>();

        return Eigen::Vector2<T>(poly_deval_nth(coefs_x, tau, 2), poly_deval_nth(coefs_y, tau, 2) / static_cast<T>(spline_info.spline_length * spline_info.spline_length));
//        // Evaluate polynomial gradient using Horner's method (hardcoded for 2nd derivative)
//        const unsigned D = spline_info.order - 1; // Polynomial degree
//        T x_hess = static_cast<T>(D * coefs_(2 * id_low, 0));
//        T y_grad = static_cast<T>(D * coefs_(2 * id_low + 1, 0));
//        for (int i = 1; i <= D - 2; i++)
//        {
//            x_hess *= tau;
//            x_hess += (D - i) * (D - i - 1) * coefs_(2 * id_low, i);
//            y_grad *= tau;
//            y_grad += (D - i) * (D - i - 1) * coefs_(2 * id_low + 1, i);
//        }
    }
//    template<typename T>
//    inline T eval_curv(const T& v)
//    {
//        T t_scaled = break_end * t;
//        std::size_t id_low = 0;
//        while ((id_low < break_data.size() - 1) && (break_data[id_low + 1] <= t_scaled)) { id_low++; }
//        T tau = t_scaled - break_data[id_low];
//
//        // evaluate curvature using Horner's method
//        T curv = T(curv_data[(curv_deg + 1) * lowest_idx]);
//        for (int i = 1; i <= curv_deg; i++)
//        {
//            curv *= v_poly;
//            curv += curv_data[(curv_deg + 1) * lowest_idx + i];
//        }
//
//        return curv;
//    }

    inline double get_path_length() const
    {
        return spline_info.spline_length;
    }
    inline Eigen::ArrayXd& breaks() { return breaks_; }
    inline Eigen::ArrayXXd& coefs() { return coefs_; }
protected:
    double break_end = 0;
    Eigen::ArrayXd break_data;
    Eigen::ArrayXXd coef_data;

    void set_default_data()
    {
        spline_info.pieces = 14;
        spline_info.order = 4;

        std::vector<double> breaks = {
                           /* Original points */
//                           0,
//                           0.5,
//                           1,
//                           1.75,
//                           1.75 + M_PI / 4,
//                           1.75 + M_PI / 2,
//                           2.25 + M_PI / 2,
//                           2.75 + M_PI / 2,
//                           2.75 + 3 * M_PI / 4,
//                           3.25 + 3 * M_PI / 4,
//                           3.25 + M_PI,
//                           3.75 + M_PI,
//                           3.75 + 5 * M_PI / 4,
//                           4.25 + 5 * M_PI / 4,
//                           4.75 + 5 * M_PI / 4,
//                           5.25 + 5 * M_PI / 4,
//                           5.25 + 3 * M_PI / 2,
//                           6 + 3 * M_PI / 2

                           /* Resampled points */
                           0,
                           0.765170641456049,
                           1.53034128291210,
                           2.29551192436815,
                           3.06068256582420,
                           3.82585320728025,
                           4.59102384873630,
                           5.35619449019235,
                           6.12136513164839,
                           6.88653577310444,
                           7.65170641456049,
                           8.41687705601654,
                           9.18204769747259,
                           9.94721833892864,
                           10.7123889803847
                   };
        break_data = Eigen::Map<Eigen::ArrayXd, Eigen::Unaligned>(breaks.data(), breaks.size());

        std::vector<double> coefs = {
            /* Orig spline */
//            -0.00503998603813471,     0.00251999301906736,                        1,          0,
//            0.0607610821504232,       -0.0303805410752116,                        0,          0,
//            0.0151199581144041,       -0.00503998603813471,       0.998740003490466,          0.5,
//            -0.18228324645127,        0.0607610821504232,         0.0151902705376058,         0,
//            -0.0324799100235342,      0.0176399511334711,         1.00503998603813,           1,
//            0.391571418302727,        -0.212663787526481,         -0.0607610821504232,         0,
//            -0.480712987631269,       -0.0554398464194809,        0.976690064573627,          1.75,
//            -0.274520908913039,       0.668371903654656,          0.281020004945707,          0,
//            0.48064314406223,         -1.18809313924018,          2.15415123864804e-05,       2.25,
//            -0.329390532080862,       0.0215472506311594,         0.822881241614507,          0.5,
//            0.0183137453967395,       -0.0556044114520233,        -0.976776230623173,         1.75,
//            0.520427161151567,        -0.754560906179291,         0.247173662801754,          1,
//            0.130848918824398,        -0.028133793356915,         -1.01864533302764,          1.25,
//            0.416107818959331,        0.0260798355480589,         -0.117066872513862,         1,
//            0.291750123188161,        0.168139584879683,          -0.948642437266259,         0.75,
//            -0.154287983410118,       0.650241563987056,          0.221093827253695,          1,
//            -1.13259538038057,        0.855559617648566,          -0.144630963729139,         0.25,
//            -0.40131066504744,        0.286709067573247,          0.956973132475236,          1.5,
//            0.26627313092645,         -0.843333452922294,         -0.138517881366004,         0.25,
//            -0.0947996903627679,      -0.315256929997914,         0.942699201262904,          2,
//            0.313768375150007,        -0.215942168947128,         -0.970471009313938,         -0.25,
//            -0.0110056373772507,      -0.538623438102608,         0.272063128395617,          2.5,
//            0.185487554572231,        0.254710393777885,          -0.95108689689856,          -0.75,
//            0.12027932627157,         -0.555131894168484,         -0.274814537739929,         2.5,
//            -0.552571900612397,       0.691755147860229,          -0.207734598777015,         -1.25,
//            0.240383502645178,        -0.271730408323362,         -0.924230671499612,         2,
//            -0.00416108837892315,     -0.137102703058369,         0.0695916236239153,         -1.25,
//            -0.114995879932454,       0.0888448456444073,         -1.01567345283909,          1.5,
//            0.569216254128092,        -0.143344335626754,         -0.070631895718646,         -1.25,
//            0.219600017084644,        -0.083648974254275,         -1.01307551714402,          1,
//            -0.217760962797411,       0.710480045565384,          0.212935959250669,          -1.25,
//            0.165992115031202,        0.245751051372691,          -0.932024478584815,         0.5,
//            -0.131595243228762,       0.197392864843143,          0.925977675683821,          -0.75,
//            -0.424573838816389,       0.636860758224583,          -0.238822784334219,         0

            /* Periodic spline fit to resampled points */
            0.00522015132294312,    -0.00673753476715322,   0.984346331848934,      -0.00246129470095528,
            -0.0197687659904417,    -0.0818434306068664,    0.0848776239050533,     -0.00925513732634816,
            0.0164368953173074,     0.00524538484166887,    0.983204582533302,      0.749125497427565,
            0.221225809843044,      -0.127222868667969,     -0.0750937704179146,    -0.00108380682048688,
            -0.405814424856174,     0.0429764740421389,     1.02010253322763,       1.51187955471725,
            0.0118051512615771,     0.380603615804740,      0.118785738401342,      -0.0339221267740268,
            0.170005073446387,      -0.888575377295810,     0.373075078010487,      2.13579063631256,
            -0.469267127337286,     0.407702481294660,      0.721974420382605,      0.285096033337023,
            0.221861247325611,      -0.498326703996547,     -0.688141677168717,     1.97716991744362,
            0.285451688742776,      -0.669505805222065,     0.521650203077747,      0.866003640422350,
            0.219882393024045,      0.0109584347945847,     -1.06106156833931,      1.25825387023079,
            0.289084908902802,      -0.0142480498819975,    -0.00153817283027566,   1.00105014923253,
            -0.222344674726119,     0.515701089939884,      -0.658077161969297,     0.551283325388401,
            -0.286332264778738,     0.649349805659265,      0.484423045027679,      1.12104025836893,
            -0.242179702412380,     0.00530623778632099,    -0.259417650809735,     0.250068127646740,
            -0.225827586260867,     -0.00792932247166700,   0.975219167591381,      1.74361590914406,
            0.342183902561027,      -0.550620156941227,     -0.676675852124406,     -0.0538196416382319,
            -0.0363216300107808,    -0.526319239584763,     0.566427852665689,      2.38401244489134,
            0.156371166646934,      0.234867071714440,      -0.918280842889113,     -0.740675304078852,
            0.193020761735331,      -0.609695974386998,     -0.302817637312917,     2.49300183037417,
            -0.354047828683640,     0.593818949379835,      -0.284194628562746,     -1.23575165777978,
            0.0711834128266431,     -0.166614514172922,     -0.896827631813369,     1.99079882019298,
            0.361497077227009,      -0.218902063160113,     0.00268076575870438,    -1.26414858517151,
            0.0867386609438621,     -0.00321214121214236,   -1.02677400265069,      1.23891210195212,
            -0.203679788308727,     0.610918788238720,      0.302640454748602,      -1.22831193188793,
            0.164294709545497,      0.195897489288219,      -0.879336831264142,     0.490232756693328,
            -0.0653914878842232,    0.143369405423256,      0.879799635735663,      -0.730303765689508,
            -0.285287729423421,     0.573037954160510,      -0.290970004762186,     0.00568878983071339
        };
        coef_data = Eigen::Map<Eigen::ArrayXXd, Eigen::Unaligned>(coefs.data(),
                                                                  spline_info.order,
                                                                  spline_info.dim * spline_info.pieces).transpose();

//        std::cout << "Default.\n"
//                  << "breaks: [" << break_data.rows() << ", " << break_data.cols() << "]\n"
//                  << "coefs: [" << coef_data.rows() << ", " << coef_data.cols() << "]\n";
    }
    void init()
    {
        std::cout << "Init.\n"
                  << "Spline Info: dim: " << spline_info.dim << ", order: " << spline_info.order
                  << ", pieces: " << spline_info.pieces << ", spline_length: " << spline_info.spline_length << "\n"
                  << "breaks: [" << break_data.rows() << ", " << break_data.cols() << "]\n"
                  << "coefs: [" << coef_data.rows() << ", " << coef_data.cols() << "]\n";

        // Duplicate path to cover positive transition across periodicity
        breaks_.resize(2 * break_data.rows() - 1);
        breaks_ << break_data, break_data.tail(1)(0) + break_data.tail(break_data.rows() - 1);

        coefs_ = coef_data.replicate<2, 1>();

//        std::cout << "Duplicated data:\n"
//                << "breaks: [" << breaks_.rows() << ", " << breaks_.cols() << "]\n"
//                << "coefs: [" << coefs_.rows() << ", " << coefs_.cols() << "]\n";
//        std::cout << "breaks: " << break_data.transpose() << "\n"
//                << "coefs: " << coef_data << "\n";
    }

private:
    Eigen::ArrayXd breaks_;
    Eigen::ArrayXXd coefs_;

    template<typename T>
    inline T poly_eval_horner(const Eigen::ArrayX<T>& coefs, const T& tau)
    {
        const unsigned D = coefs.size() - 1; // Polynomial degree

        T y = coefs(0);
        for (unsigned k = 1; k <= D; k++)
        {
            y = tau * y + coefs(k);
        }
        return y;
    }
    template<typename T>
    inline T poly_deval_nth(const Eigen::ArrayX<T>& coefs, const T& tau, const unsigned n)
    {
        const unsigned D = coefs.size() - 1; // Polynomial degree
        if (n == 0)
        {
            return poly_eval_horner(coefs, tau);
        }
        if (n > D)
        {
            return static_cast<T>(0);
        }

        Eigen::ArrayX<T> deriv_coefs = Eigen::ArrayX<T>::Zero(D);
        for (unsigned k = 0; k < D; k++)
        {
            deriv_coefs(k) = coefs(k) * (D - k);
        }
        return poly_deval_nth(deriv_coefs, tau, n - 1);
    }
};

inline bool shift_warm_path_param(double state_path_param, Eigen::MatrixXd &Gamma_warm)
{
    const bool print_debug{false};
    if (print_debug) std::cout << "gamma: " << state_path_param << ", warm: " << Gamma_warm << "\n";

    bool modify{false};
    for (unsigned i = 0; i < 10; i++)
    {
        double gamma_err = state_path_param - Gamma_warm(0);
        int warm_correction{0};
        if (gamma_err < -0.5) { warm_correction = -1; }
        else if (gamma_err > 0.5) { warm_correction = +1; }

        if (warm_correction != 0)
        {
            Gamma_warm.array() += warm_correction;
            modify = true;
            std::cout << "Warm-start gamma += " << warm_correction;
            if (print_debug) std::cout << " => " << Gamma_warm << "\n";
            std::cout << "\n";
        }
        else { break; }

    }
    return modify;
}

} // spline_following
} // mpc_control_racing

#endif //SRC_EIGEN_TRACK_SPLINE_LA_HPP
