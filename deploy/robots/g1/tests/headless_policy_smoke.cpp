// Numerical MuJoCo/ONNX test; deliberately excludes DDS, FSM timing and terminal
// input. Start from the policy's nominal upright pose, then stand/walk/stop.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <mujoco/mujoco.h>
#include "keyboard_observation_config.h"
#include "keyboard_velocity_command.h"
#include "LinearInterpolator.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

std::vector<float> command{0, 0, 0};
float last_gait_amplitude = 0;

namespace isaaclab
{
REGISTER_OBSERVATION(keyboard_velocity_commands) { return command; }
REGISTER_OBSERVATION(keyboard_gait_phase)
{
    auto phase = mdp::gait_phase_from_command(env, params, command);
    last_gait_amplitude = std::hypot(phase[0], phase[1]);
    return phase;
}
}

class SimArticulation : public isaaclab::Articulation
{
public:
    SimArticulation(mjModel* model, mjData* sim) : m(model), d(sim)
    {
        const int quat_id = mj_name2id(m, mjOBJ_SENSOR, "imu_quat");
        const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
        if (quat_id < 0 || gyro_id < 0) throw std::runtime_error("Missing IMU sensors");
        quat = m->sensor_adr[quat_id];
        gyro = m->sensor_adr[gyro_id];
        data.joystick = &joystick;
    }
    void update() override
    {
        for (int i = 0; i < 3; ++i) data.root_ang_vel_b[i] = d->sensordata[gyro + i];
        data.root_quat_w = Eigen::Quaternionf(d->sensordata[quat], d->sensordata[quat + 1],
                                              d->sensordata[quat + 2], d->sensordata[quat + 3]);
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        for (size_t i = 0; i < data.joint_ids_map.size(); ++i) {
            const int id = data.joint_ids_map[i];
            data.joint_pos[i] = d->sensordata[id];
            data.joint_vel[i] = d->sensordata[id + m->nu];
        }
    }
private:
    mjModel* m;
    mjData* d;
    int quat, gyro;
    unitree::common::UnitreeJoystick joystick;
};

int main(int argc, char** argv)
{
    if (argc != 5 && argc != 6) {
        std::cerr << "Usage: headless_policy_smoke SCENE_XML DEPLOY_YAML POLICY_ONNX fixed|broken|reset|w|s|a|d|q|e [FSM_CONFIG]\n";
        return 1;
    }
    const std::string scenario = argv[4];
    const std::string key = scenario.size() == 1 ? scenario : "w";
    const auto& key_targets = g1::keyboard_velocity_targets();
    if (key_targets.find(key) == key_targets.end() || key == "x" || key == " ")
        throw std::runtime_error("Unknown motion key");
    const auto& motion_target = key_targets.at(key);
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
        mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!m) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    auto cfg = YAML::LoadFile(argv[2]);
    const std::vector<std::string> range_names{"lin_vel_x", "lin_vel_y", "ang_vel_z"};
    for (size_t i = 0; i < 3; ++i) {
        const auto range = cfg["commands"]["base_velocity"]["ranges"][range_names[i]];
        if (motion_target[i] < range[0].as<float>() || motion_target[i] > range[1].as<float>())
            throw std::runtime_error("Keyboard target exceeds policy command range");
    }
    if (m->nu != 29) throw std::runtime_error("Expected 29 actuators");
    if (std::string(argv[4]) != "reset") {
        const int key = mj_name2id(m.get(), mjOBJ_KEY, "stand_ready");
        if (key < 0) throw std::runtime_error("Missing stand_ready keyframe");
        mj_resetDataKeyframe(m.get(), d.get(), key);
    }
    mj_forward(m.get(), d.get());
    if (argc == 6) {
        const auto stand = YAML::LoadFile(argv[5])["FSM"]["FixStand"];
        const auto kp = stand["kp"].as<std::vector<float>>();
        const auto kd = stand["kd"].as<std::vector<float>>();
        const auto ts = stand["ts"].as<std::vector<float>>();
        auto qs = stand["qs"].as<std::vector<std::vector<float>>>();
        qs[0].assign(d->sensordata, d->sensordata + m->nu);
        while (d->time < ts.back()) {
            const auto q = linear_interpolate(d->time, ts, qs);
            for (int i = 0; i < m->nu; ++i)
                d->ctrl[i] = kp[i] * (q[i] - d->sensordata[i])
                           - kd[i] * d->sensordata[i + m->nu];
            mj_step(m.get(), d.get());
        }
        std::cout << "FixStand from "
                  << (std::string(argv[4]) == "reset" ? "ordinary Reset" : "stand_ready")
                  << " complete, pelvis z=" << d->qpos[2] << "\n";
    }
    if (std::string(argv[4]) == "broken") {
        auto velocity = YAML::Clone(cfg["observations"]["velocity_commands"]);
        cfg["observations"].remove("velocity_commands");
        cfg["observations"]["keyboard_velocity_commands"] = velocity;
    } else if (scenario == "fixed" || scenario == "reset" || scenario == key) {
        cfg["observations"] = g1::keyboard_velocity_observations(cfg["observations"]);
    } else {
        throw std::runtime_error("Unknown scenario");
    }
    auto robot = std::make_shared<SimArticulation>(m.get(), d.get());
    isaaclab::ManagerBasedRLEnv env(cfg, robot);
    env.alg = std::make_unique<isaaclab::OrtRunner>(argv[3]);
    env.reset();
    // Match starting Velocity while the viewer is paused: the policy can run
    // on the same upright state before physics is resumed.
    for (int i = 0; i < 100; ++i) env.step();
    const double start_time = d->time;
    double next_policy = start_time, min_height = d->qpos[2], max_tilt = 0;
    auto yaw_of = [](const Eigen::Quaternionf& q) {
        return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                         1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    };
    double previous_yaw = yaw_of(robot->data.root_quat_w);
    double yaw = previous_yaw;
    double start_x = 0, start_y = 0, start_yaw = 0, dx = 0, dy = 0, dyaw = 0;
    double max_gait_amplitude = 0;
    bool walking = false, stopped = false;
    std::vector<float> targets = env.action_manager->processed_actions();
    while (d->time - start_time < 20) {
        if (d->time >= next_policy) {
            const double elapsed = d->time - start_time;
            const bool moving = elapsed >= 5 && elapsed < 15;
            g1::smooth_keyboard_velocity(command, moving ? motion_target : key_targets.at("x"));
            if (!walking && elapsed >= 5) {
                walking = true;
                start_x = d->qpos[0];
                start_y = d->qpos[1];
                start_yaw = yaw;
                std::cout << "Key " << key << ": stand phase complete at t=" << d->time
                          << ", pelvis z=" << d->qpos[2] << "\n";
            }
            if (!stopped && elapsed >= 15) {
                stopped = true;
                const double world_dx = d->qpos[0] - start_x;
                const double world_dy = d->qpos[1] - start_y;
                dx = std::cos(start_yaw) * world_dx + std::sin(start_yaw) * world_dy;
                dy = -std::sin(start_yaw) * world_dx + std::cos(start_yaw) * world_dy;
                dyaw = yaw - start_yaw;
                std::cout << "Motion phase complete: dx=" << dx << ", dy=" << dy
                          << " m, yaw change=" << dyaw << " rad\n";
            }
            env.step();
            if (moving) max_gait_amplitude = std::max(max_gait_amplitude, double(last_gait_amplitude));
            targets = env.action_manager->processed_actions();
            next_policy += env.step_dt;
        }
        for (int i = 0; i < m->nu; ++i)
            d->ctrl[i] = robot->data.joint_stiffness[i] * (targets[i] - d->sensordata[i])
                       - robot->data.joint_damping[i] * d->sensordata[i + m->nu];
        mj_step(m.get(), d.get());
        robot->update();
        const double raw_yaw = yaw_of(robot->data.root_quat_w);
        yaw += std::remainder(raw_yaw - previous_yaw, 2 * M_PI);
        previous_yaw = raw_yaw;
        const double tilt = std::acos(std::clamp(-double(robot->data.projected_gravity_b[2]), -1.0, 1.0));
        min_height = std::min(min_height, d->qpos[2]);
        max_tilt = std::max(max_tilt, tilt);
        if (!std::isfinite(d->qpos[2]) || !std::isfinite(tilt) ||
            d->qpos[2] < 0.45 || tilt > 1.0) {
            std::cout << "FAIL: t=" << d->time << ", pelvis z=" << d->qpos[2]
                      << ", tilt=" << tilt << " rad\n";
            return 2;
        }
    }
    std::cout << "Survived 20 simulated seconds: min pelvis z=" << min_height
              << ", max tilt=" << max_tilt << " rad, final vx=" << d->qvel[0]
              << ", vy=" << d->qvel[1] << ", yaw_rate=" << robot->data.root_ang_vel_b[2] << "\n";
    const std::vector<double> displacement{dx, dy, dyaw};
    bool moved = false;
    for (size_t i = 0; i < 3; ++i) {
        if (motion_target[i] != 0) {
            moved = displacement[i] * std::copysign(1.0, motion_target[i])
                    > std::abs(motion_target[i]) * 10 * 0.25;
            break;
        }
    }
    if (!moved || max_gait_amplitude == 0 || std::hypot(d->qvel[0], d->qvel[1]) > 0.2 ||
        std::abs(robot->data.root_ang_vel_b[2]) > 0.2) {
        std::cout << "FAIL: insufficient motion in requested direction, inactive gait or failed to stop\n";
        return 3;
    }
    std::cout << "PASS: key " << key << ", stand 5s, move 10s, stop 5s (no DDS/FSM/GUI)\n";
}
