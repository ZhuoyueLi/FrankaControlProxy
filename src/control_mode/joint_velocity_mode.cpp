#include "joint_velocity_mode.hpp"
#include <franka/exception.h>
#include <franka/robot_state.h>
#include "utils/logger.hpp"
#include "protocol/codec.hpp"

JointVelocityMode::JointVelocityMode():
    desired_velocities_(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
{};
JointVelocityMode::~JointVelocityMode() = default;


void JointVelocityMode::controlLoop() {
    LOG_INFO("[JointVelocityMode] Started.");
    is_running_ = true;
    // Initialize desired velocities to zero
    desired_velocities_.write(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        LOG_ERROR("[JointVelocityMode] Robot or model not set.");
        return;
    }
    robot_->automaticErrorRecovery();

    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> joint_velocity_callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::JointVelocities {
            // if (!is_running_) {
            //     throw franka::ControlException("JointVelocityMode stopped.");
            // }
            updateRobotState(state);
            auto desired = desired_velocities_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired_velocities);
            }
            return desired;
        };
    bool is_robot_operational = true;
    while (is_running_ && is_robot_operational) {
        try {
            robot_->control(joint_velocity_callback);
    } catch (const std::exception &ex) {
        LOG_ERROR("[JointVelocityMode] Robot is unable to be controlled: {}", ex.what());
        is_robot_operational = false;
    }
    for (int i = 0; i < 3; i++) {
        LOG_WARN("[JointVelocityMode] Waiting {} seconds before recovery attempt...", 3);
        usleep(1000 * 3);
        try {
            robot_->automaticErrorRecovery();
            LOG_INFO("[JointVelocityMode] Robot operation recovered.");
            is_robot_operational = true;
            break;
            } catch (const std::exception &ex) {
                LOG_ERROR("[JointVelocityModes] Recovery failed: {}", ex.what());
            }
        }
    }
}


protocol::ModeID JointVelocityMode::getModeID() const {
    return protocol::ModeID::JOINT_VELOCITY;
}

void JointVelocityMode::writeCommand(const protocol::ByteView& data) {
    franka::JointVelocities velocities = protocol::decode<franka::JointVelocities>(data);
    desired_velocities_.write(velocities);
}

void JointVelocityMode::writeZeroCommand() {
    desired_velocities_.write(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
}
