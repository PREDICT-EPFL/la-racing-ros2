#include "mpc_control_racing_benchmark/benchmark.hpp"

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "mpc_control_racing_benchmark/json.hpp"

namespace MAMHpy_link {

using json = nlohmann::json;

static const char* SOCKET_PATH = "/tmp/cpp_python_poc.sock";

bool send_message(int fd, const json& msg) {
    std::string data = msg.dump();
    uint32_t len = static_cast<uint32_t>(data.size());
    if (send(fd, &len, sizeof(len), 0) != sizeof(len)) return false;
    if (send(fd, data.data(), len, 0) != static_cast<ssize_t>(len)) return false;
    return true;
}

bool recv_message(int fd, json& msg) {
    uint32_t len = 0;
    if (recv(fd, &len, sizeof(len), MSG_WAITALL) != sizeof(len)) return false;
    std::string buf(len, '\0');
    if (recv(fd, buf.data(), len, MSG_WAITALL) != static_cast<ssize_t>(len)) return false;
    msg = json::parse(buf);
    return true;
}

json send_receive(int fd, const json& request)
{
//    std::cout << "[Socket] Sending: " << request.dump() << std::endl;

    if (!send_message(fd, request))
    {
        std::cerr << "[Socket] Failed to send." << std::endl;
        return json();
    }
    json response;
    if (!recv_message(fd, response))
    {
        std::cerr << "[Socket] Failed to receive." << std::endl;
        return json();
    }

//    std::cout << "[Socket] Received: " << response.dump() << std::endl;
    return response;
}

class SplineFollowingTire_MAMH_Optimizer_link : public mpc_control_racing::RacingOptimizerInterface
{
public:
    SplineFollowingTire_MAMH_Optimizer_link()
    {
        connect();
    }
    ~SplineFollowingTire_MAMH_Optimizer_link() override
    {
        disconnect();
    }

    void set_max_iter(unsigned max_iter)
    {
        json request = {
            {"type", "set"},
            {"operation", "max_iter"},
            {"values", max_iter}
        };
        json res = send_receive(client_fd, request);
    }
    void set_tol(unsigned tol)
    {
        json request = {
            {"type", "set"},
            {"operation", "tol"},
            {"values", tol}
        };
        json res = send_receive(client_fd, request);
    }

    bool init() override
    {
        json request = {
            {"type", "call"},
            {"operation", "init"},
        };
        json res = send_receive(client_fd, request);
        return true;
    }

    void update_config_params() override {}
    void update_tuning_params() override {}

    Eigen::VectorXd compose_state() override
    {
        Eigen::VectorXd xcomp(6);
        xcomp.setZero();

        Eigen::VectorXd states(6);
        states << state.pos, state.turn_angle, state.vel, state.turn_rate;

        json request = {
            {"type", "call"},
            {"operation", "compose_state"},
            {"values", states}
        };
        json res = send_receive(client_fd, request);
        std::vector<double> stdVec = res["results"].get<std::vector<double>>();
        xcomp = Eigen::Map<Eigen::VectorXd>(stdVec.data(), stdVec.size());
        return xcomp;
    }
    void set_initial_state(const Eigen::VectorXd &x0) override
    {
        // Do nothing. Has already been set during compose_state() request
    }

    bool compute_control() override
    {
        json request = {
            {"type", "call"},
            {"operation", "solve"},
            {"values", {}}
        };
        json res = send_receive(client_fd, request);
        const double comp_time_us = res["results"][0];
        const bool solve_succeeded = res["results"][1].get<bool>();
        const bool QP_solver_failed = res["results"][2].get<bool>();
        return solve_succeeded;
    }

    Eigen::VectorXd get_control() const override
    {
        std::cerr << "Not implemented!\n";
        return {};
    }

    Eigen::VectorXd get_time_grid() const override
    {
        std::cerr << "Not implemented!\n";
        return {};
    }
    Eigen::MatrixXd get_control_trajectory() const override
    {
        Eigen::MatrixXd control_trajectory(3, 2);

        json request = {
            {"type", "get"},
            {"operation", "control_traj"}
        };
        json res = send_receive(client_fd, request);
        auto stdVec = res["results"].get<std::vector<std::vector<double>>>();
        for (unsigned i = 0; i < stdVec.size(); ++i)
        {
            control_trajectory.row(i) = Eigen::Map<Eigen::VectorXd>(stdVec[i].data(), stdVec[i].size());
        }
        return control_trajectory;
    }
    Eigen::MatrixXd get_state_trajectory() const override
    {
        std::cerr << "Not implemented!\n";
        return {};
    }

    double get_cost() const override
    {
        double cost = -1;

        json request = {
            {"type", "get"},
            {"operation", "cost"},
        };
        json res = send_receive(client_fd, request);
        cost = res["results"];
        return cost;
    }
    int get_no_iterations() const override
    {
        int it = -1;

        json request = {
            {"type", "get"},
            {"operation", "iter"},
        };
        json res = send_receive(client_fd, request);
        it = res["results"];
        return it;
    }
    int get_no_iterations_low() const override
    {
        int it_low = -1;

        json request = {
            {"type", "get"},
            {"operation", "iter_low"},
        };
        json res = send_receive(client_fd, request);
        it_low = res["results"];
        return it_low;
    }

    void test_connection() const
    {
        // Example loop: send different types of requests
        for (int i = 1; i <= 3; ++i) {
            json request = {
                {"type", "compute"},
                {"operation", "square"},
                {"values", {i, i + 10, i + 100}}
            };
            json res = send_receive(client_fd, request);
        }
    }
protected:
    int server_fd;
    int client_fd;

    int connect()
    {
        unlink(SOCKET_PATH);

        server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd < 0) { perror("socket"); return 1; }

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("bind"); return 1;
        }
        if (listen(server_fd, 1) < 0) { perror("listen"); return 1; }

        std::cout << "[Socket] Waiting for Python to connect ..." << std::endl;
        client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd < 0) { perror("accept"); return 1; }
        std::cout << "[Socket] Connected to Python worker." << std::endl;
        return EXIT_SUCCESS;
    }
    void disconnect() const
    {
        close(client_fd);
        close(server_fd);
        unlink(SOCKET_PATH);
        std::cout << "[Socket] Disconnected." << std::endl;
    }
};

}

int main(int argc, char** argv)
{
    const std::string optimizer_name = "spft_MAMH";

    /* Setup */
    const std::map<std::string, std::vector<double>> hyperparams{
//        {"max_iter", {1, 2, 4, 6, 8, 10, 12, 14, 17, 20, 23}},
//        {"max_iter", {1, 2, 7, 12, 18, 25}},
//          {"max_iter", {16, 13, 7, 2}},
//        {"max_iter", {24, 20, 16, 13}},
        {"max_iter", {24, 20, 16, 13, 10, 7, 4, 2, 1}}, // Same spacing for all controllers
//        {"max_iter", {16, 13}},
        {"tol", {1e-6}}, //, 1e-3}},
    };

    for (const int max_iter : hyperparams.at("max_iter")) // Intentional cast to int!
    {
        for (const auto &tol : hyperparams.at("tol"))
        {
            std::cout << "\nBenchmark optimizer for max_iter: " << max_iter << ", tol: " << tol << "\n\n";

            // Construct optimizer
            using Optimizer = MAMHpy_link::SplineFollowingTire_MAMH_Optimizer_link;
            const std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>();

            auto filepaths = benchmark::get_filepaths();
            optimizer->track->load_from_yaml(filepaths["track_filepath"]);

            optimizer->set_max_iter(max_iter);
//            optimizer->set_tol(tol);

//            optimizer->init(); // Will be called in benchmark

            benchmark::run(optimizer, optimizer_name, {{"maxit", max_iter}, {"tol", tol}});
        }
    }
    return EXIT_SUCCESS;
}
