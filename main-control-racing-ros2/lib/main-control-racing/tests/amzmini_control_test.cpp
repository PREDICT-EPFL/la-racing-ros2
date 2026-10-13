#include "main_control_racing/amzmini/AmzminiControl.hpp"

int main()
{
    using namespace control::racing::amzmini;
    AmzminiControl amzminiControl;
    amzminiControl.load_track_from_yaml("/home/johannes/tracks/la_track_sampled.yaml");
    amzminiControl.test();

    return EXIT_SUCCESS;
}
