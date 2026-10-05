#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

inline double exploration_steady_seconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Preparation is wall-bounded; active exploration uses simulator time and starts
// exactly once, at the first accepted exploration navigation (not an approach).
class ExplorationBudget {
public:
    ExplorationBudget(double wall_start, double active_limit)
        : preparation_deadline_(wall_start + 10.0), active_limit_(active_limit) {}

    void accepted(int64_t sim_ns, double wall_now) {
        if (sim_ns <= 0 || !std::isfinite(wall_now) || wall_now >= preparation_deadline_)
            return;
        int64_t unset = 0;
        first_accept_ns_.compare_exchange_strong(unset, sim_ns);
    }

    int64_t first_accept_ns() const { return first_accept_ns_.load(); }
    bool clock_rolled_back() const { return clock_rollback_.load(); }

    double elapsed(int64_t now_ns) const {
        const auto start = first_accept_ns();
        return start == 0 ? 0.0 : static_cast<double>(now_ns - start) / 1e9;
    }

    double remaining(double wall_now, int64_t sim_now) const {
        if (!std::isfinite(wall_now) || !std::isfinite(preparation_deadline_)
                || !std::isfinite(active_limit_) || active_limit_ < 0)
            return 0.0;
        if (first_accept_ns() == 0) return preparation_deadline_ - wall_now;
        auto previous = latest_sim_ns_.load();
        while (sim_now >= previous && !latest_sim_ns_.compare_exchange_weak(previous, sim_now)) {}
        if (sim_now < previous) clock_rollback_ = true;
        if (clock_rolled_back()) return 0.0;
        const double used = elapsed(sim_now);
        if (used < 0) return 0.0;  // clock rollback must not extend the action
        return active_limit_ > 0 ? active_limit_ - used
                                : std::numeric_limits<double>::infinity();
    }

private:
    double preparation_deadline_;
    double active_limit_;
    std::atomic<int64_t> first_accept_ns_{0};
    mutable std::atomic<int64_t> latest_sim_ns_{0};
    mutable std::atomic<bool> clock_rollback_{false};
};
