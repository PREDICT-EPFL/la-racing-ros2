#include "../include/mpc_control_racing_benchmark/export_CSV.hpp"

int main()
{
    std::vector<CarStateMsg> states = {
        {
            1770465600501502910LL, 1770465600LL, 501452594LL, //"state",
            0.7695379904311405, -3.0213868663477212,
            -5.381275416076064e-05, -5.553014292222202e-06,
            0.16434762146872153, 0.002165693817977717
        },
        {
            1770465600511538440LL, 1770465600LL, 511466141LL, //"state",
            0.7695117109320043, -3.021385131516067,
            -0.0009516466215979324, 2.8875258397043377e-05,
            0.16394325679714894, -0.011546447791819696
        },
        {
            1770465600521519785LL, 1770465600LL, 521460371LL, //"state",
            0.769539942134433, -3.021381042857566,
            0.00012119544393252797, 0.00017451880750667548,
            0.16435992234599803, 0.0032636553708765338
        },
    };

    const std::string outputFile = "car_state.csv";
    std::ofstream file(outputFile);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open file " << outputFile << std::endl;
        return 1;
    }

    file << MsgBase::get_header_str(CarStateMsg::get_fieldnames());
    for (const auto &state : states)
    {
        file << state;
    }

    std::cout << "CSV written to " << outputFile << std::endl;
    return 0;
}
