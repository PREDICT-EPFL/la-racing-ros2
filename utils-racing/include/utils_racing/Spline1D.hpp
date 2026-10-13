#ifndef SRC_SPLINE_HPP
#define SRC_SPLINE_HPP

#include <iostream>
#include <vector>
#include <string>

#include <yaml-cpp/yaml.h>

namespace utils_racing {

class Spline1D
{
public:
    explicit Spline1D(int degree_ = 3) : degree(degree_) {}
    void load_from_yaml(const std::string &yaml_filepath,
                        const std::vector<std::string> &node_path = {"track"},
                        const std::string &breaks_fieldname = "breaks",
                        const std::string &coeffs_fieldname = "coeffs")
    {
        YAML::Node yamlNode;
        try
        {
            yamlNode = YAML::LoadFile(yaml_filepath);
        }
        catch (const std::exception &ex)
        {
            std::cerr << "YAML file " << yaml_filepath << " could not be opened:\n" << ex.what() << "\n";
        }

        // Navigate to sub-node through yaml data structure
        for (const std::string &key: node_path)
        {
            yamlNode = yamlNode[key];
        }

        // Read out breaks and coeffs from sub-node
        breaks_data = yamlNode[breaks_fieldname].as<std::vector<double>>();
        coeff_data = yamlNode[coeffs_fieldname].as<std::vector<double>>();

        // Call init routine
        init();
    }
    unsigned get_size() const { return breaks_data.size(); }
    double get_max_break() const { return max_break; }

    template<typename T>
    inline T eval(const T &v) const
    {
        T v_scaled = max_break * v;

        std::size_t lowest_idx = 0;
        while (lowest_idx < (breaks_data.size() - 2) // max idx shall be at last break point that carries coeffs aft. ++
               && breaks_data[lowest_idx + 1] <= v_scaled) // Find highest idx that does not exceed v_scaled
        {
            lowest_idx++;
        }
        // If v exceeds 2.0 (2x path length), extrapolate from last break/coeff point
        T v_poly = v_scaled - breaks_data[lowest_idx];

        // Evaluate polynomial using Horner's method
        std::size_t coeff_idx_start = (degree + 1) * lowest_idx;
        T x = T(coeff_data[coeff_idx_start]);
        for (int k = 1; k <= degree; k++)
        {
            x *= v_poly;
            x += coeff_data[coeff_idx_start + k];
        }
        return x;
    }

protected:
    std::vector<double> breaks_data;
    std::vector<double> coeff_data;

    const int degree{3};
    double max_break = 0;

    void init()
    {
        /* Expecting data in format:
         * c0     c0       c0       /
         * c1     c1       c1       /
         * c2     c2       c2       /
         * c3     c3       c3       /
         * | - - - |  - - - | - - - |
         * b0      b1       b2      bN
         *
         * For each break point, there are (degree + 1) coefficients stored. The last break doesn't carry coeffs.
         * */
        max_break = breaks_data[breaks_data.size() - 1];

        // Duplicate track (keep original max_break)
        std::size_t n_breaks_orig = breaks_data.size();
        for (std::size_t k = 1; k < n_breaks_orig; k++)
        {
            breaks_data.push_back(max_break + breaks_data[k]);
        }
        coeff_data.insert(coeff_data.end(), coeff_data.begin(), coeff_data.end());
    }
};

} // namesapce utils_racing

#endif //SRC_SPLINE_HPP
