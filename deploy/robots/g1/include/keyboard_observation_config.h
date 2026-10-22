#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace g1
{
// ObservationManager concatenates terms in YAML iteration order. Preserve that
// order when changing the command source; remove + append corrupts policy input.
inline YAML::Node keyboard_velocity_observations(const YAML::Node& observations)
{
    YAML::Node result(YAML::NodeType::Map);
    for (const auto& entry : observations) {
        const auto name = entry.first.as<std::string>();
        auto replacement = name;
        if (name == "velocity_commands") replacement = "keyboard_velocity_commands";
        if (name == "gait_phase") replacement = "keyboard_gait_phase";
        result[replacement] = YAML::Clone(entry.second);
    }
    return result;
}
}
