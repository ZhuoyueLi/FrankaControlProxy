#include "control_mode/joint_position_mode.hpp"
#include <franka/exception.h>
#include <unistd.h>
#include "utils/logger.hpp"
#include "protocol/codec.hpp"

JointPositionMode::JointPositionMode():
    desired_positions_(franka::JointPositions{{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}})
{};
JointPositionMode::~JointPositionMode() = default;

void JointPositionMode::controlLoop() {
    LOG_INFO("[JointPositionMode] Started.");
    is_running_ = true;

    desired_positions_.write(franka::JointPositions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

    if (!robot_ || !model_) {
        LOG_ERROR("[JointPositionMode] Robot or model not set.");
        return;
    }

    robot_->automaticErrorRecovery();

    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> callback =
        [this](const franka::RobotState& state, franka::Duration) -> franka::JointPositions {
            // if (!is_running_) {
            //     throw franka::ControlException("JointPositionMode stopped.");
            // }
            updateRobotState(state);
            auto desired = desired_positions_.read();
            if (!is_running_) {
                return franka::MotionFinished(desired);
            }
            return desired;
        };
    bool is_robot_operational = true;
    while (is_running_ && is_robot_operational) {
        try {
            robot_->control(callback);
        } catch (const std::exception &ex) {
            LOG_ERROR("[JointPositionMode] Robot is unable to be controlled: {}", ex.what());
            is_robot_operational = false;
        }
        if (!is_robot_operational) {
            for (int i = 0; i < 3; i++) {
                LOG_WARN("[JointPositionMode] Waiting {} seconds before recovery attempt...", 3);
                usleep(1000 * 3);
                try {
                    robot_->automaticErrorRecovery();
                    LOG_INFO("[JointPositionMode] Robot operation recovered.");
                    is_robot_operational = true;
                    break;
                } catch (const franka::Exception& recovery_error) {
                    LOG_ERROR("[JointPositionMode] Recovery failed: {}", recovery_error.what());
                }
            }
        }
    }
}


protocol::ModeID JointPositionMode::getModeID() const {
    return protocol::ModeID::JOINT_POSITION;
}

void JointPositionMode::writeCommand(const protocol::ByteView& data) {
    franka::JointPositions positions = protocol::decode<franka::JointPositions>(data);
    desired_positions_.write(positions);
}

void JointPositionMode::writeZeroCommand() {
    franka::JointPositions current_positions = current_state_->read().q;
    desired_positions_.write(current_positions);
}
