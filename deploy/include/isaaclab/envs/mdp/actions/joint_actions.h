// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/manager/action_manager.h"

namespace isaaclab
{

class JointAction : public ActionTerm
{
public:
    JointAction(YAML::Node cfg, ManagerBasedRLEnv* env)
    :ActionTerm(cfg, env)
    {
        if(cfg["joint_ids"].IsNull()) {
            _action_dim = env->robot->data.joint_ids_map.size();
        } else {
            _joint_ids = cfg["joint_ids"].as<std::vector<int>>();
            _action_dim = _joint_ids.size();
        }
        _raw_actions.resize(_action_dim, 0.0f);
        _processed_actions.resize(_action_dim, 0.0f);
        if(!cfg["scale"].IsNull()) {
            _scale = cfg["scale"].as<std::vector<float>>();
        }
        if(!cfg["offset"].IsNull()) {
            _offset = cfg["offset"].as<std::vector<float>>();
            _processed_actions = _offset;
        }
        if(!cfg["clip"].IsNull()) {
            _clip = cfg["clip"].as<std::vector<std::vector<float> >>();
        }
        if(cfg["arm_ema_alpha"] && !cfg["arm_ema_alpha"].IsNull()) {
            _arm_ema_alpha = cfg["arm_ema_alpha"].as<float>();
        }
    }

    virtual void process_actions(std::vector<float> actions)
    {
        // TODO: modify action by joint_ids
        _raw_actions = actions;
        for(int i(0); i<_action_dim; ++i)
        {
            float target = _raw_actions[i];
            if(!_scale.empty()) {
                target *= _scale[i];
            }
            if(!_offset.empty()) {
                target += _offset[i];
            }
            if(!_clip.empty()) {
                target = std::clamp(target, _clip[i][0], _clip[i][1]);
            }

            // Apply low-pass EMA filter on arm joints (SDK motor_id >= 15) to protect
            // shoulder plastic mounting tabs from high-frequency RL jitter.
            const bool is_arm_joint =
                (i < static_cast<int>(env->robot->data.joint_ids_map.size())) &&
                (static_cast<int>(env->robot->data.joint_ids_map[i]) >= 15);
            if (is_arm_joint) {
                _processed_actions[i] =
                    _arm_ema_alpha * target + (1.0f - _arm_ema_alpha) * _processed_actions[i];
            } else {
                _processed_actions[i] = target;
            }
        }
    }


    int action_dim() 
    {
        return _action_dim;
    }

    std::vector<float> raw_actions() 
    {
        return _raw_actions;
    }
    
    std::vector<float> processed_actions() 
    {
        return _processed_actions;
    }

    void reset()
    {
        _raw_actions.assign(_action_dim, 0.0f);
        if (!_offset.empty()) {
            _processed_actions = _offset;
        } else {
            _processed_actions.assign(_action_dim, 0.0f);
        }
    }

protected:
    int _action_dim;
    std::vector<int> _joint_ids;

    std::vector<float> _raw_actions;
    std::vector<float> _processed_actions;

    std::vector<float> _scale;
    std::vector<float> _offset;
    std::vector<std::vector<float> > _clip;
    float _arm_ema_alpha = 1.0f;
};


class JointPositionAction : public JointAction
{
public:
    JointPositionAction(YAML::Node cfg, ManagerBasedRLEnv* env)
    :JointAction(cfg, env)
    {
    }
};

class JointVelocityAction : public JointAction
{
public:
    JointVelocityAction(YAML::Node cfg, ManagerBasedRLEnv* env)
    :JointAction(cfg, env)
    {
    }
};

REGISTER_ACTION(JointPositionAction);
REGISTER_ACTION(JointVelocityAction);

};