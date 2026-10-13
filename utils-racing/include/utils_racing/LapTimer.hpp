#ifndef SRC_LAPTIMER_HPP
#define SRC_LAPTIMER_HPP

#include <chrono>
#include <iostream>
#include <Eigen/Core>

namespace utils_racing {

class LapTimer
{
public:
    LapTimer(double track_width = 0.83) : m_track_width(track_width)
    {
        init();
    }

    using TimePoint = std::chrono::steady_clock::time_point;
    using SteadyClock = std::chrono::steady_clock;

    bool terminate_lap(const TimePoint &state_tc, const Eigen::Vector2d &pos, double timeout = 999)
    {
        using namespace std::chrono;
        double pos_x = pos(0);
        double pos_y = pos(1);

        lap_completed = false;

        const TimePoint t_now = state_tc;

        // x=0 line crossed?
        if (pos_x * x_prev < 0.0)
        {
            // On final straight or elsewhere?
            if (std::abs(pos_y) < 0.5 * m_track_width)
            {
                // Completion condition fulfilled?
                if (m_completion_armed)
                {
                    if (m_lap_count >= 0)
                    {
                        // Count this lap time
                        const double lap_time = duration_cast<nanoseconds>(t_now - m_crossing_time_last).count() / 1e9;
                        if (lap_time < 30) // Anything over 30s is not considered a lap
                        {
                            update_avg(lap_time);
                            update_fastest(lap_time);
                            update_history(lap_time);
                            lap_completed = true;
                        }
                    }
                    else
                    {
                        // Start the lap time counting
                        std::cout << "Lap count start\n";
                        m_lap_count = 0;
                    }

                    // Store this crossing time, reset completion condition
                    m_crossing_time_last = t_now;
                    m_completion_armed = false;
                }
            }
            else
            {
                // Elsewhere on the track. Allow next crossing to be completion
                m_completion_armed = true;
            }
        }
        x_prev = pos_x;

        return lap_completed;
    }
    void declare_lap_failed(const TimePoint &t_stamp)
    {
        std::cout << "Lap failed\n";
        update_history(999);

        x_prev = 0;

        m_lap_count = -99;
        m_crossing_time_last = t_stamp;

        lap_completed = false;
    }

    double get_fastest() { return m_fastest_lap_time; }
    double get_latest() { return m_last_lap_times(m_index_latest); }
    double get_avg() { return m_avg_lap_time; }
    double get_avg_N(unsigned n)
    {
        n = std::min(static_cast<unsigned>(m_last_lap_times.RowsAtCompileTime), n); // Limit by buffer length
        const int N = std::min(m_lap_count, static_cast<int>(n)); // Limit by measurements so far

        int index = m_index_latest;
        double avg = 0;
        for (int i = 0; i < N; i++)
        {
            // Go backwards through measurements, wrap around when buffer end is reached
            avg += m_last_lap_times(index);
            index--;
            if (index < 0) { index = m_last_lap_times.RowsAtCompileTime - 1; }
        }
        avg /= N; // Average over measurements
        return avg;
    }
    bool get_completed() { return lap_completed; }

    void reset()
    {
        std::cout << "Reset lap timer\n";
        init();
    }

    void print()
    {
        std::cout << "Lap " << m_lap_count << ": " << get_latest() << "s,"
                  << " best: " << get_fastest() << "s,"
                  << " avg: " << get_avg() << "s\n";
    }

private:
    const double m_track_width{1.0};
    bool m_completion_armed{false};
    double x_prev{0};

    int m_lap_count{-99};
    double m_lap_sum{0};
    double m_fastest_lap_time{999};
    double m_avg_lap_time{0};
    TimePoint m_crossing_time_last = SteadyClock::now();

    bool lap_completed{false};
    void init()
    {
        x_prev = 0;

        m_lap_count = -99;
        m_lap_sum = 0;
        m_fastest_lap_time = 999;
        m_avg_lap_time = 0;
        m_crossing_time_last = SteadyClock::now();

        lap_completed = false;

        m_last_lap_times.setZero();
        m_index_latest = 0;
    }

    void update_avg(double lap_time)
    {
        m_lap_sum += lap_time;
        m_avg_lap_time = (m_lap_sum / ++m_lap_count);
    }
    void update_fastest(double lap_time)
    {
        if (lap_time < m_fastest_lap_time) { m_fastest_lap_time = lap_time; }
    }

    void update_history(double lap_time)
    {
        m_index_latest++;
        if (m_index_latest >= m_last_lap_times.RowsAtCompileTime) { m_index_latest = 0; }
        m_last_lap_times(m_index_latest) = lap_time;
    }

    static const unsigned p_N_times{10};
    Eigen::Matrix<double, p_N_times, 1> m_last_lap_times = Eigen::Matrix<double, p_N_times, 1>::Zero();
    unsigned m_index_latest{p_N_times - 1};
};

} // Namespace utils_racing

#endif //SRC_LAPTIMER_HPP
