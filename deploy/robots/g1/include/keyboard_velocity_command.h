#pragma once

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace g1
{
inline const std::unordered_map<std::string, std::vector<float>>& keyboard_velocity_targets()
{
    // Nonzero targets must exceed the trained gait-phase command gate (0.1).
    static const std::unordered_map<std::string, std::vector<float>> targets{
        {"w", {0.4f, 0.0f, 0.0f}},
        {"s", {-0.3f, 0.0f, 0.0f}},
        {"a", {0.0f, 0.2f, 0.0f}},
        {"d", {0.0f, -0.2f, 0.0f}},
        {"q", {0.0f, 0.0f, 0.3f}},
        {"e", {0.0f, 0.0f, -0.3f}},
        {"x", {0.0f, 0.0f, 0.0f}},
        {" ", {0.0f, 0.0f, 0.0f}},
    };
    return targets;
}

inline void smooth_keyboard_velocity(std::vector<float>& current,
                                     const std::vector<float>& target)
{
    for (size_t i = 0; i < current.size(); ++i) {
        current[i] += (target[i] - current[i]) * 0.15f;
        // Snap to zero only when stopping, never while starting a small command.
        if (target[i] == 0.0f && std::abs(current[i]) < 0.01f) current[i] = 0.0f;
    }
}
}
