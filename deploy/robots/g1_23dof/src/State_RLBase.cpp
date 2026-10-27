#include <algorithm>
#include <cmath>

#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "keyboard_observation_config.h"
#include "keyboard_velocity_command.h"

namespace isaaclab
{
namespace
{
// The command term precedes gait phase in deploy.yaml. Share its smoothed
// command so the gait term does not read an idle gamepad or smooth a second time.
std::vector<float> keyboard_current_cmd{0.0f, 0.0f, 0.0f};
}
// Loopback mode selects these command and gait terms in their original slots.
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    const std::string key = FSMState::keyboard->key();
    const auto& key_commands = g1::keyboard_velocity_targets();
    auto& current_cmd = keyboard_current_cmd;
    static std::vector<float> target_cmd{0.0f, 0.0f, 0.0f};
    static std::string last_key;

    // Env construction/reset happens before the first policy step. Never carry
    // a latched walking command over a Passive -> FixStand -> Velocity restart.
    if (env->episode_length == 0) {
        current_cmd.assign(3, 0.0f);
        target_cmd.assign(3, 0.0f);
        last_key = key;
        return current_cmd;
    }

    if (key != last_key) {
        if (const auto it = key_commands.find(key); it != key_commands.end()) {
            target_cmd = it->second;
            spdlog::info("Keyboard velocity key: {} -> [{:.2f}, {:.2f}, {:.2f}]",
                         key == " " ? "space" : key,
                         target_cmd[0], target_cmd[1], target_cmd[2]);
        }
        last_key = key;
    }

    g1::smooth_keyboard_velocity(current_cmd, target_cmd);
    return current_cmd;
}

REGISTER_OBSERVATION(keyboard_gait_phase)
{
    return mdp::gait_phase_from_command(env, params, keyboard_current_cmd);
}

}

namespace
{
// The shared isaaclab::mdp::bad_orientation is disabled upstream (always false)
// for every robot, so G1-23DoF keeps its own active tilt check here.
bool torso_tilt_exceeds(isaaclab::ManagerBasedRLEnv* env, float limit_angle)
{
    const auto& gravity = env->robot->data.projected_gravity_b;
    const float cos_tilt = std::clamp(-gravity[2], -1.0f, 1.0f);
    return std::fabs(std::acos(cos_tilt)) > limit_angle;
}
}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    auto deploy_config = YAML::LoadFile(policy_dir / "params" / "deploy.yaml");
    if (FSMState::keyboard_fsm_shortcuts_enabled &&
        deploy_config["observations"]["velocity_commands"]) {
        deploy_config["observations"] =
            g1::keyboard_velocity_observations(deploy_config["observations"]);
        spdlog::info("Using laptop keyboard velocity commands (original observation order preserved).");
    }

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_config,
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    spdlog::info("ONNX policy loaded successfully: {}",
                 (policy_dir / "exported" / "policy.onnx").string());

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{
                const bool bad = torso_tilt_exceeds(env.get(), 0.85f);
                if (bad) {
                    spdlog::warn("Safety trigger: bad_orientation (> 0.85 rad) detected! Switching to Passive.");
                }
                return bad;
            },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}
