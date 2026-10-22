#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    // Keyboard mode switching is enabled by the G1 entry point only for loopback
    // simulation. This prevents these shortcuts from becoming active on a robot NIC.
    inline static bool keyboard_fsm_shortcuts_enabled = false;

    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );

                // Laptop-keyboard shortcuts for local simulation. The existing
                // joystick transition remains available when a gamepad is used.
                std::string keyboard_key;
                if (state_string == "Passive" && target_fsm == "FixStand") {
                    keyboard_key = "u";
                } else if (state_string == "FixStand" && target_fsm == "Velocity") {
                    keyboard_key = "v";
                } else if ((state_string == "FixStand" || state_string == "Velocity") &&
                           target_fsm == "Passive") {
                    keyboard_key = "p";
                }
                if (!keyboard_key.empty()) {
                    registered_checks.emplace_back(
                        std::make_pair(
                            [keyboard_key]()->bool {
                                return FSMState::keyboard_fsm_shortcuts_enabled &&
                                       FSMState::keyboard &&
                                       FSMState::keyboard->on_pressed &&
                                       FSMState::keyboard->key() == keyboard_key;
                            },
                            fsm_id
                        )
                    );
                }
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
