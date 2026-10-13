#include "utils_racing/Path.hpp"
#include "utils_racing/PathUtils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char **argv)
{

    const std::string default_track_yaml = ament_index_cpp::get_package_share_directory("lar_utils") +
                                       "/config/track/sampled/la_track_sampled.yaml";

    std::shared_ptr<utils_racing::PathSampled> track = std::make_shared<utils_racing::PathSampled>();
    track->load_from_yaml(default_track_yaml);

    std::cout << "Track pos at pp = 0.05: " << track->eval_pos(0.05).transpose() << "\n";

    utils_racing::PathLocator track_locator{};

    Eigen::Vector2d pos_local(0, 0);
    double path_param = 0.05;

    std::cout << "=============\n"
                 "pp guess: " << path_param << "\n";
    path_param = track_locator.find_closest_path_param(track, pos_local, path_param);

    std::cout << "=============\n"
             "pp found: " << path_param << "\n";

    return EXIT_SUCCESS;
}
