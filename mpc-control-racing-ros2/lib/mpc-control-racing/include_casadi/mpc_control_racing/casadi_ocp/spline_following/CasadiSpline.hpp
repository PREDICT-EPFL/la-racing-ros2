#ifndef SRC_TRACK_SPLINE_LA_HPP
#define SRC_TRACK_SPLINE_LA_HPP

#include "casadi/casadi.hpp"
#include "utils_racing/Path.hpp"
#include "mpc_control_common/casadi_ocp/eigen_casadi_conversion.hpp"

namespace mpc_control_racing {
namespace spline_following {
namespace casadi_ocp {

class CasadiSpline
{
public:
    enum SplineType
    {
        Pos, Generic
    };

    struct SplineInfo
    {
        unsigned pieces{10};
        unsigned order{4};
        const unsigned dim{2};
        double spline_length{10.710722313718323};
    } spline_info{};
    casadi::Function spline_func;

    CasadiSpline()
    {
        set_default_data();
        init(SplineType::Pos);
    }

    void load_from_yaml(const std::string &yaml_filepath, const std::string &coeff_key = "pos", const SplineType &spline_type = Generic)
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
            break_data = yamlFile["spline_data"]["breaks"].as<std::vector<double>>();

            std::vector<std::vector<double>> coefs;
            for (const auto& yamlRow : yamlFile["spline_data"][coeff_key])
            {
                coefs.push_back(yamlRow.as<std::vector<double>>());
            }
            coef_data = casadi::DM(coefs);

            // New spline file
            spline_info.spline_length = yamlFile["spline_info"]["spline_length"].as<double>();
            spline_info.pieces = static_cast<unsigned>(yamlFile["spline_info"]["pieces"].as<double>());
            spline_info.order = static_cast<unsigned>(yamlFile["spline_info"]["order"].as<double>());
        }
        catch (const std::exception &ex)
        {
            std::cout << "Loading legacy spline file... " << std::flush;
            // Legacy spline file
            break_data = yamlFile["track"]["breaks_data"].as<std::vector<double>>();
            coef_data = yamlFile["track"]["coeff_data"].as<std::vector<double>>();

            spline_info.pieces = break_data.size1() - 1;
            spline_info.order = 4;
            coef_data = casadi::DM::reshape(coef_data, spline_info.order, spline_info.dim * spline_info.pieces).T();
        }
        std::cout << "OK\n";

        init(spline_type);
    }

    /** Experimental! **/
    double get_closest_path_param(const casadi::DM &pos, double init_guess = 0)
    {
        // Minimizes for orthogonality between path gradient and pos err vector
        /// Can lead to wrong result if too far away
        casadi::MX theta_sym = casadi::MX::sym("theta");
        casadi::MX residual_sym = spline_func(theta_sym)[0] - casadi::MX(pos);

        casadi::MX spline_grad = casadi::MX::jacobian(spline_func(theta_sym)[0], theta_sym);
        casadi::Function grad_fun = casadi::Function("gradient", {theta_sym}, {spline_grad});
        casadi::Function res_fun = casadi::Function("residual", {theta_sym}, {residual_sym});

        const double tol = 1e-1;
        const unsigned max_iter = 10;

        casadi::DM theta_i = init_guess;
        casadi::DM orthogonality = 1;
        unsigned counter = 0;

        while ((casadi::norm_2(orthogonality.nonzeros()) >= tol) && (counter <= max_iter))
        {
            /** try Gauss-Newton */
            const casadi::DM th_grad = grad_fun(theta_i)[0];
            const casadi::DM th_res  = res_fun(theta_i)[0];
            const casadi::DM th_step = -casadi::DM::dot(th_grad, th_res) / casadi::DM::dot(th_grad, th_grad);
            theta_i += th_step;
            if (theta_i.nonzeros()[0] > 1.0) theta_i -= 1;
            if (theta_i.nonzeros()[0] < 0.0) theta_i += 1;

            orthogonality = casadi::DM::dot(th_res, th_grad);

            counter++;
        }
        return theta_i.nonzeros()[0];
    }
    /** Experimental! **/
    double get_closest_path_param2(const casadi::DM &pos, double init_guess = 0)
    {
        // Minimizes distance - how to do gradient step?
        casadi::MX theta_sym = casadi::MX::sym("theta");
        casadi::MX poserr_sym = spline_func(theta_sym)[0] - casadi::MX(pos);

        casadi::MX distsq = casadi::MX::dot(poserr_sym, poserr_sym);
        casadi::MX distsq_grad = casadi::MX::jacobian(distsq, theta_sym);
        casadi::Function res_fun = casadi::Function("residual", {theta_sym}, {distsq});
        casadi::Function grad_fun = casadi::Function("gradient", {theta_sym}, {distsq_grad});

        const double tol = 0.01 * 0.01;
        const unsigned max_iter = 50;

        casadi::DM theta_i = init_guess;
        casadi::DM distsq_i = 99999;
        unsigned counter = 0;

        while ((casadi::DM::sqrt(distsq_i).nonzeros()[0] >= tol) && (counter <= max_iter))
        {
            /** try Gauss-Newton */
            const casadi::DM th_grad = grad_fun(theta_i)[0];
            const casadi::DM th_res  = res_fun(theta_i)[0];
            const casadi::DM th_step = -th_grad * th_res / (th_grad * th_grad);
            theta_i += th_step;
            if (theta_i.nonzeros()[0] > 1.0) theta_i -= 1;
            if (theta_i.nonzeros()[0] < 0.0) theta_i += 1;

            distsq_i = th_res;

            //std::cout << "grad: " << th_grad << " res: " << th_res << " step: " << th_step << "\n";
            counter++;
        }
        return theta_i.nonzeros()[0];
    }

    double get_path_length() const { return spline_info.spline_length; }

protected:
    casadi::DM break_data;
    casadi::DM coef_data;
    casadi::DM curv_coef_data;

    void set_default_data()
    {
        spline_info.pieces = 14;
        spline_info.order = 4;

        break_data = {
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

        coef_data = {
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
        coef_data = casadi::DM::reshape(coef_data, spline_info.order, spline_info.dim * spline_info.pieces).T();

        curv_coef_data = {
                        -2.61315466876859,
                        0.804705610028430,
                        0.194082797485241,
                        0.0484826934689287,
                        -19.4293276814858,
                        12.4037963184328,
                        -0.961077594062770,
                        0.0200561611225831,
                        -13.6833193911051,
                        15.8683552398343,
                        -3.12927703674428,
                        0.211800483513683,
                        -11.5451724067866,
                        15.1265214976909,
                        -2.41734564948267,
                        1.01814216023981,
                        12.6353029793682,
                        -12.4999700036129,
                        -0.0215895682301614,
                        2.85703711778528,
                        37.6917578859569,
                        -31.1972531925878,
                        3.72577382022391,
                        1.25093483981652,
                        -21.1148378971914,
                        13.7809962768272,
                        0.797339042103787,
                        0.0259781875261359,
                        -11.7346268981647,
                        11.0738374993344,
                        -1.25779310396259,
                        1.23054204063590,
                        3.09868103984299,
                        9.75300782121560,
                        -5.57857332634711,
                        1.38846259898934,
                        14.6518397730648,
                        -20.4291693119378,
                        6.49844527475073,
                        1.42476302110006,
                        6.68874995329724,
                        -5.93545055099493,
                        1.52235534289327,
                        1.02531313720721,
                        -2.83720086761796,
                        1.98889897991017,
                        0.603467256871272,
                        1.13872191506727,
                        22.9457831993861,
                        -13.8486828715845,
                        -1.52276193796037,
                        1.46498944970360,
                        -1.74948290120920,
                        -2.79436507369986,
                        1.83789258999474,
                        0.109660662750553,
                        -28.5628640795518,
                        24.1293831401118,
                        -2.26858465961202,
                        0.111330326671810,
                        -1.23837865066691,
                        0.0688823187582868,
                        0.438650420835895,
                        1.43902577194977,
                        2.82574831029689,
                        -1.88635690423754,
                        -1.74483211978445,
                        1.22606997353438
        };

//        std::cout << "Default.\n"
//                  << "breaks: " << break_data.size() << "\n"
//                  << "coefs: " << coef_data.size() << "\n"
//                  << "curv_coeff: " << curv_coef_data.size() << "\n";
    }
    void init(const SplineType &type = SplineType::Generic)
    {
        std::cout << "Init.\n"
                  << "Spline Info: dim: " << spline_info.dim << ", order: " << spline_info.order
                  << ", pieces: " << spline_info.pieces << ", spline_length: " << spline_info.spline_length << "\n"
                  << "breaks: " << break_data.size() << "\n"
                  << "coefs: " << coef_data.size() << "\n"
                  << "curv_coeff: " << curv_coef_data.size() << "\n";

        using namespace casadi;
        Slice all;

        MX breaks(break_data);
        MX coeff(coef_data);

        // Duplicate path to cover positive transition across periodicity
        breaks = MX::vertcat({breaks, breaks(-1) + breaks(Slice(1, -1))});
        coeff = MX::vertcat({coeff, coeff});
        //        curv_coeff = MX::vertcat({curv_coeff, curv_coeff});

//        std::cout << "init.\n"
//           << "breaks: " << breaks.size() << "\n"
//           << "coefs: " << coeff.size() << "\n";
//        std::cout << "init.\n"
//           << "breaks: " << breaks << "\n"
//           << "coefs: " << coeff << "\n";

        MX t = MX::sym("t");
        MX t_scaled = break_data(-1) * t;
        MX id_low = MX::low(breaks, t_scaled);
        MX tau = t_scaled - breaks(id_low);

        if (type == SplineType::Pos)
        {
            /** Source spline contains position data.
             * Derive gradient, hessian, tangent angle, and curvature expressions from that.
             * Gradient, hessian, tangent, and curvature will be exactly consistent with position polynomials (but of lower order).
             */
            MX coefs_x = coeff(2 * id_low, all);
            MX coefs_y = coeff(2 * id_low + 1, all);
            MX pos = MX::vertcat({poly_eval_horner(coefs_x, tau), poly_eval_horner(coefs_y, tau)});

            //        MX grad = MX::jacobian(pos, t) / spline_info.spline_length;
            //        MX hessn = MX::jacobian(grad, t) / spline_info.spline_length;
            MX grad = MX::vertcat({poly_deval_nth(coefs_x, tau, 1), poly_deval_nth(coefs_y, tau, 1)}) / spline_info.spline_length;
            MX hessn = MX::vertcat({poly_deval_nth(coefs_x, tau, 2), poly_deval_nth(coefs_y, tau, 2)}) / (spline_info.spline_length * spline_info.spline_length);

            MX tngt = MX::atan2(grad(1), grad(0));
            //        MX curv = (grad(0) * hessn(1) - grad(1) * hessn(0)) / pow(sq(grad(0)) + sq(grad(1)), 1.5);
            MX curv = (grad(0) * hessn(1) - grad(1) * hessn(0)) / pow(pow(grad(0), 2) + pow(grad(1), 2), 1.5);

            spline_func = casadi::Function("spline_func", {t}, {pos, grad, hessn, tngt, curv});
        }
        else
        {
            /** Source spline contains position data + additional separate data.
             * Gradient, curvature, etc. will be inconsistent with position polynomials but of higher individual accuracy.
             */
            MXVector splines;
            for (unsigned int i = 0; i < spline_info.dim; i++)
            {
                MX coefs_i = coeff(spline_info.dim * id_low + i, all);
                MX eval_i = poly_eval_horner(coefs_i, tau);
                splines.push_back(eval_i);
            }
            spline_func = casadi::Function("spline_func", {t}, splines);
        }
    }
//    static casadi::MX poly_eval(const casadi::MX& coefs, const casadi::MX& tau)
//    {
//        using casadi::MX;
//
////        MX basis = MX::horzcat({pow(tau, 3), sq(tau), tau, 1});
//        unsigned D = coefs.size2() - 1;
//        MX basis = MX(1, D + 1);
//        for (int i = 0; i <= D; i++) { basis(i) = pow(tau, D - i); }
//
//        return dot(basis, coefs);
//    }

    static casadi::MX poly_eval_horner(const casadi::MX& coefs, const casadi::MX& tau)
    {
        const unsigned D = coefs.size2() - 1;

        casadi::MX y = coefs(0);
        for (unsigned k = 1; k <= D; k++)
        {
            y = tau * y + coefs(k);
        }
        return y;
    }
    static casadi::MX poly_deval_nth(const casadi::MX coefs, const casadi::MX& tau, const unsigned n)
    {
        const unsigned D = coefs.size2() - 1;

        if (n == 0)
        {
            return poly_eval_horner(coefs, tau);
        }
        if (n > D)
        {
            return 0;
        }

        casadi::MX deriv_coefs = casadi::MX::zeros(1, D);
        for (unsigned k = 0; k < D; k++)
        {
            deriv_coefs(k) = coefs(k) * (D - k);
        }
        return poly_deval_nth(deriv_coefs, tau, n - 1);
    }
};

class CasadiSpline_Path : public CasadiSpline, public utils_racing::PathBase
{
public:
    Eigen::Vector2d eval_pos(double arg, int incr = 0) const override
    {
        return eigen_casadi_conversion::toEigen<double, 2, 1>(spline_func(casadi::DM(arg))[0]);
    }
    Eigen::Vector2d eval_grad(double arg, int incr = 0) const override
    {
        return eigen_casadi_conversion::toEigen<double, 2, 1>(spline_func(casadi::DM(arg))[1]);
    }
    double eval_arclength(double arg, int incr = 0) const override
    {
        return arg * get_path_length();
    }
    double eval_tangent_angle(double arg, int incr = 0) const override
    {
        return spline_func(casadi::DM(arg))[3].nonzeros()[0];
    }
    double eval_curvature(double arg, int incr = 0) const override
    {
        return spline_func(casadi::DM(arg))[4].nonzeros()[0];
    }

     unsigned get_size() const override
    {
        return break_data.size1();
    }

    // All in one vector
    Eigen::VectorXd eval(double arg, int incr = 0) const
    {
        Eigen::Vector<double, 5> pos_curv_grad;
        pos_curv_grad.segment<2>(0) = eigen_casadi_conversion::toEigen<double, 2, 1>(spline_func(casadi::DM(arg))[0]);
        pos_curv_grad(2) = spline_func(casadi::DM(arg))[4].nonzeros()[0];
        pos_curv_grad.segment<2>(4) = eigen_casadi_conversion::toEigen<double, 2, 1>(spline_func(casadi::DM(arg))[1]);
       return pos_curv_grad;
    }
};

static bool shift_warm_path_param(double state_path_param, casadi::DM &Gamma_warm)
{
    const bool print_debug{false};
    if (print_debug) std::cout << "gamma: " << state_path_param << ", warm: " << Gamma_warm.nonzeros() << "\n";

    bool modify{false};
    for (unsigned i = 0; i < 10; i++)
    {
        double gamma_err = state_path_param - Gamma_warm(0).nonzeros()[0];
        int warm_correction{0};
        if (gamma_err < -0.5) { warm_correction = -1; }
        else if (gamma_err > 0.5) { warm_correction = +1; }

        if (warm_correction != 0)
        {
            Gamma_warm += warm_correction;
            modify = true;
            std::cout << "Warm-start gamma += " << warm_correction;
            if (print_debug) std::cout << " => " << Gamma_warm << "\n";
            std::cout << "\n";
        }
        else { break; }

    }
    return modify;
}

}
}
}

#endif //SRC_TRACK_SPLINE_LA_HPP
