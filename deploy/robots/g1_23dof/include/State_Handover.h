#pragma once

// Hand the robot back to Unitree's built-in controller from the remote, the
// same as Ctrl-C g1_ctrl + `motion_mode.py select ai` on the PC.
//
// Entered from Passive only (config.yaml). On enter, rt/lowcmd stops (post_run
// does not publish), and after a short gap a worker thread calls
// MotionSwitcherClient::SelectMode("ai"). On success g1_ctrl exits, so it can
// no longer react to the remote while the built-in controller owns it: the
// robot is then in zero torque (purple LED), weaker than Passive damping, so
// do this hung on the gantry only. On failure the Passive damping command is
// published again and the state stays here (LT + B goes back to Passive).

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include "FSM/FSMState.h"

class State_Handover : public FSMState
{
public:
    State_Handover(int state, std::string state_string = "Handover")
    : FSMState(state, state_string)
    {
        auto mode = param::config["FSM"][state_string]["mode"];
        if (mode.IsDefined()) mode_ = mode.as<std::string>();
    }

    ~State_Handover()
    {
        if (worker_.joinable()) worker_.join();
    }

    void enter()
    {
        if (worker_.joinable()) worker_.join();
        publishing_ = false;
        spdlog::warn("Handover: rt/lowcmd stopped, selecting motion mode '{}' ...", mode_);
        worker_ = std::thread(&State_Handover::select_mode, this);
    }

    void run()
    {
        // Passive damping command, published again only if the handover fails.
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        for (int i(0); i < kd.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.q() = lowstate->msg_.motor_state()[i].q();
            motor.kp() = 0;
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }
    }

    void post_run()
    {
        if (publishing_) lowcmd->unlockAndPublish();
    }

    void exit()
    {
        if (worker_.joinable()) worker_.join();
        publishing_ = true;
    }

private:
    // Seconds without rt/lowcmd before asking for the mode, so the built-in
    // controller never sees two writers.
    static constexpr double kQuietS = 0.5;

    void select_mode()
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(kQuietS));
        unitree::robot::b2::MotionSwitcherClient msc;
        msc.SetTimeout(5.0f);
        msc.Init();
        int32_t code = msc.SelectMode(mode_);
        if (code == 0)
        {
            std::string form, name;
            int32_t check = msc.CheckMode(form, name);
            // The built-in controller owns the robot now: never publish again.
            spdlog::warn("Handover: SelectMode('{}') ok, CheckMode code {} form '{}' name '{}'. "
                         "g1_ctrl exits; use the Unitree remote sequence from here.",
                         mode_, check, form, name);
            spdlog::default_logger()->flush();
            std::cout.flush();
            std::_Exit(0);
        }
        spdlog::error("Handover: SelectMode('{}') failed, code {} (3104 = no reply). "
                      "Passive damping published again; LT + B returns to Passive.",
                      mode_, code);
        publishing_ = true;
    }

    std::string mode_ = "ai";
    std::atomic_bool publishing_{true};
    std::thread worker_;
};

REGISTER_FSM(State_Handover)
