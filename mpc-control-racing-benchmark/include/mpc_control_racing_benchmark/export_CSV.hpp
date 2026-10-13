#ifndef ROS2_MASTER_EXPORT_CSV_HPP
#define ROS2_MASTER_EXPORT_CSV_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>

template <typename T>
class TopicBase
{
public:
    explicit TopicBase(std::string topic_name) : m_topic_name(topic_name)
    {
        create_file();
    }
    int create_file()
    {
        const std::string filepath = m_topic_name + ".csv";

        // Create directory if necessary
        std::filesystem::path dir = std::filesystem::path(filepath).parent_path();
        if (!std::filesystem::exists(dir)) { std::filesystem::create_directories(dir); }

        m_publish_file = std::ofstream(filepath);
        if (!m_publish_file.is_open())
        {
            std::cerr << "Error: Could not open file " << filepath << std::endl;
            return 1;
        }
        m_publish_file << T::get_header_str(T::get_fieldnames());
        return 0;
    }
    int publish(const T &msg)
    {
        if (!m_publish_file.is_open())
        {
            std::cerr << "Error: Need to create_file() first!\n";
            return 1;
        }
        m_publish_file << msg;
        return 0;
    }

private:
    std::string m_topic_name;
    std::ofstream m_publish_file;
};

struct MsgBase
{
    static std::string get_header_str(const std::vector<std::string> &fieldnames)
    {
        std::string out;
        for (size_t i = 0; i < fieldnames.size(); ++i)
        {
            if (i > 0) out += ",";
            out += fieldnames[i];
        }
        return out + "\n";
    }

    static void to_header_stamp(const double time_sec,
                                uint64_t &time_ns, unsigned &header_stamp_sec, unsigned &header_stamp_nanosec)
    {
        time_ns = static_cast<uint64_t>(time_sec * 1e9);
        header_stamp_sec = std::floor(time_sec);
        header_stamp_nanosec = static_cast<unsigned>((time_sec - header_stamp_sec) * 1e9);
    }
};

/* ================================== */
struct Pose2DMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns", "header.stamp.sec", "header.stamp.nanosec", //"header.frame_id",
            "x", "y", "theta"
        };
    }
    uint64_t log_time_ns;
    uint32_t header_stamp_sec;
    uint32_t header_stamp_nanosec;
    //    std::string header_frame_id;
    double x;
    double y;
    double theta;

    Pose2DMsg(uint64_t log_time_ns, uint32_t header_stamp_sec, uint32_t header_stamp_nanosec,
//                const std::string &header_frame_id,
              double x, double y, double theta)
        : log_time_ns(log_time_ns),
          header_stamp_sec(header_stamp_sec),
          header_stamp_nanosec(header_stamp_nanosec),
//          header_frame_id(header_frame_id),
          x(x), y(y), theta(theta)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const Pose2DMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.header_stamp_sec << ","
                << s.header_stamp_nanosec << ","
//                << s.header_frame_id << ","
                << s.x << ","
                << s.y << ","
                << s.theta << "\n";
        return os;
    }
};

struct CarStateMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns", "header.stamp.sec", "header.stamp.nanosec", //"header.frame_id",
            "pos_x", "pos_y", "vel_x", "vel_y", "turn_angle", "turn_rate"
        };
    }
    uint64_t log_time_ns;
    uint32_t header_stamp_sec;
    uint32_t header_stamp_nanosec;
//    std::string header_frame_id;
    double pos_x;
    double pos_y;
    double vel_x;
    double vel_y;
    double turn_angle;
    double turn_rate;

    CarStateMsg(uint64_t log_time_ns, uint32_t header_stamp_sec, uint32_t header_stamp_nanosec,
//                const std::string &header_frame_id,
                double pos_x, double pos_y,
                double vel_x, double vel_y,
                double turn_angle, double turn_rate)
        : log_time_ns(log_time_ns), header_stamp_sec(header_stamp_sec), header_stamp_nanosec(header_stamp_nanosec),
//          header_frame_id(header_frame_id)
          pos_x(pos_x), pos_y(pos_y),
          vel_x(vel_x), vel_y(vel_y),
          turn_angle(turn_angle), turn_rate(turn_rate)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const CarStateMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.header_stamp_sec << ","
                << s.header_stamp_nanosec << ","
//                << s.header_frame_id << ","
                << s.pos_x << ","
                << s.pos_y << ","
                << s.vel_x << ","
                << s.vel_y << ","
                << s.turn_angle << ","
                << s.turn_rate << "\n";
        return os;
    }
};

struct CarControlMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns", "header.stamp.sec", "header.stamp.nanosec", //"header.frame_id",
            "steering", "throttle", "id"
        };
    }
    uint64_t log_time_ns;
    uint32_t header_stamp_sec;
    uint32_t header_stamp_nanosec;
    //    std::string header_frame_id;
    double steering;
    double throttle;
    unsigned id;

    CarControlMsg(uint64_t log_time_ns, uint32_t header_stamp_sec, uint32_t header_stamp_nanosec,
//                const std::string &header_frame_id,
                  double steering, double throttle, unsigned id)
        : log_time_ns(log_time_ns),
          header_stamp_sec(header_stamp_sec),
          header_stamp_nanosec(header_stamp_nanosec),
//          header_frame_id(header_frame_id),
          steering(steering), throttle(throttle), id(id)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const CarControlMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.header_stamp_sec << ","
                << s.header_stamp_nanosec << ","
//                << s.header_frame_id << ","
                << s.steering << ","
                << s.throttle << ","
                << s.id << "\n";
        return os;
    }
};

struct RacingMpcDiagnosticsStampedMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns", "header.stamp.sec", "header.stamp.nanosec", //"header.frame_id",
            "computation_time_ms", "cost", "iter", "iter_low"
        };
    }
    uint64_t log_time_ns;
    uint32_t header_stamp_sec;
    uint32_t header_stamp_nanosec;
    //    std::string header_frame_id;
    double computation_time_ms;
    double cost;
    int32_t iter;
    int32_t iter_low;

    RacingMpcDiagnosticsStampedMsg(uint64_t log_time_ns, uint32_t header_stamp_sec, uint32_t header_stamp_nanosec,
//                const std::string &header_frame_id,
                                   double computation_time_ms, double cost, int32_t iter, int32_t iter_low)
        : log_time_ns(log_time_ns),
          header_stamp_sec(header_stamp_sec),
          header_stamp_nanosec(header_stamp_nanosec),
//          header_frame_id(header_frame_id),
          computation_time_ms(computation_time_ms), cost(cost), iter(iter), iter_low(iter_low)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const RacingMpcDiagnosticsStampedMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.header_stamp_sec << ","
                << s.header_stamp_nanosec << ","
//                << s.header_frame_id << ","
                << s.computation_time_ms << ","
                << s.cost << ","
                << s.iter << ","
                << s.iter_low << "\n";
        return os;
    }
};

struct Vector3StampedMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns", "header.stamp.sec", "header.stamp.nanosec", //"header.frame_id",
            "vector.x", "vector.y", "vector.z"
        };
    }
    uint64_t log_time_ns;
    uint32_t header_stamp_sec;
    uint32_t header_stamp_nanosec;
    //    std::string header_frame_id;
    double vector_x;
    double vector_y;
    double vector_z;

    Vector3StampedMsg(uint64_t log_time_ns, uint32_t header_stamp_sec, uint32_t header_stamp_nanosec,
//                const std::string &header_frame_id,
                      double vector_x, double vector_y, double vector_z)
        : log_time_ns(log_time_ns), header_stamp_sec(header_stamp_sec), header_stamp_nanosec(header_stamp_nanosec),
//          header_frame_id(header_frame_id)
          vector_x(vector_x), vector_y(vector_y), vector_z(vector_z)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const Vector3StampedMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.header_stamp_sec << ","
                << s.header_stamp_nanosec << ","
//                << s.header_frame_id << ","
                << s.vector_x << ","
                << s.vector_y << ","
                << s.vector_z << "\n";
        return os;
    }
};

struct FloatMsg : MsgBase
{
    static std::vector<std::string> get_fieldnames()
    {
        return {
            "log_time_ns",
            "data"
        };
    }
    uint64_t log_time_ns;
    //    std::string header_frame_id;
    double data;

    FloatMsg(uint64_t log_time_ns, double data)
        : log_time_ns(log_time_ns), data(data)
    {
    }
    friend std::ostream& operator<<(std::ostream &os, const FloatMsg &s)
    {
        os << std::setprecision(16)
                << s.log_time_ns << ","
                << s.data << "\n";
        return os;
    }
};

#endif //ROS2_MASTER_EXPORT_CSV_HPP
