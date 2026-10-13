#include <iostream>

#include "utils_racing/Path.hpp"

int main()
{
    const std::string filepath = "track_test.yaml";

    utils_racing::PathSampled track(filepath);
    track.load_from_yaml(filepath); // Redundant, for testing only

    std::cout << track.get_info() << "\n";

    track.eval(0);
    track.eval(0.499);
    track.eval(0.5);
    track.eval(0.99);
    track.eval(1);
    track.eval(1.4);
    track.eval(2.8);

    return 0;
}
