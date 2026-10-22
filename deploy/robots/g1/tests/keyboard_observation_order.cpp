#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include "keyboard_observation_config.h"
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/algorithms/algorithms.h"

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv)
{
    require(argc == 3, "Usage: test DEPLOY_YAML POLICY_ONNX");
    const auto original = YAML::LoadFile(argv[1])["observations"];
    const auto fixed = g1::keyboard_velocity_observations(original);

    // Feed distinguishable values through the actual ObservationManager,
    // independent of DDS, graphics, and robot hardware.
    size_t offset = 0;
    for (const auto& entry : original) {
        const auto name = entry.first.as<std::string>();
        const auto size = entry.second["scale"].size();
        std::vector<float> values(size);
        for (size_t i = 0; i < size; ++i) values[i] = 0.01f * (offset + i + 1);
        offset += size;
        isaaclab::observations_map()[name] =
            [values](isaaclab::ManagerBasedRLEnv*, YAML::Node) { return values; };
    }
    isaaclab::observations_map()["keyboard_velocity_commands"] =
        isaaclab::observations_map().at("velocity_commands");
    isaaclab::observations_map()["keyboard_gait_phase"] =
        isaaclab::observations_map().at("gait_phase");

    isaaclab::ObservationManager baseline(original, nullptr);
    isaaclab::ObservationManager keyboard(fixed, nullptr);
    const auto expected = baseline.compute().at("obs");
    const auto actual = keyboard.compute().at("obs");
    require(expected.size() == 98, "Unexpected G1 policy input size");
    require(actual == expected, "Keyboard changed policy observation values/order");
    require(original["velocity_commands"].IsDefined(), "Original config was mutated");
    require(!original["keyboard_velocity_commands"].IsDefined(), "Original config was mutated");

    // Reproduce the previous bug: same shape, wrong semantic positions.
    auto broken = YAML::Clone(original);
    auto command = YAML::Clone(broken["velocity_commands"]);
    broken.remove("velocity_commands");
    broken["keyboard_velocity_commands"] = command;
    isaaclab::ObservationManager old_keyboard(broken, nullptr);
    const auto corrupted = old_keyboard.compute().at("obs");
    require(corrupted.size() == expected.size(), "Bug reproduction changed input size");
    require(corrupted != expected, "Failed to reproduce the old ordering bug");
    for (size_t i = 0; i < 3; ++i)
        require(actual[6 + i] == corrupted[95 + i], "Unexpected command offsets");

    isaaclab::OrtRunner policy(argv[2]);
    const auto reference_action = policy.act({{"obs", expected}});
    const auto fixed_action = policy.act({{"obs", actual}});
    const auto broken_action = policy.act({{"obs", corrupted}});
    require(reference_action.size() == 29, "Unexpected G1 policy output size");
    require(reference_action == fixed_action, "Fixed keyboard changed policy output");
    float max_delta = 0;
    for (size_t i = 0; i < reference_action.size(); ++i) {
        require(std::isfinite(fixed_action[i]), "Nonfinite policy action");
        max_delta = std::max(max_delta, std::abs(reference_action[i] - broken_action[i]));
    }
    std::cout << "PASS: 98 observations identical to original config; commands at indices 6..8.\n"
              << "PASS: actual ONNX inference returns 29 finite, identical actions.\n"
              << "Old remove/append bug: commands moved to 95..97, max raw action delta = "
              << max_delta << "\n";
}
