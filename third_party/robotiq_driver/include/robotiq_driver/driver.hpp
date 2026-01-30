#pragma once

#include <cstdint>

namespace robotiq_driver
{
class Driver
{
public:
  enum class ActivationStatus
  {
    RESET,
    ACTIVE
  };

  enum class ActionStatus
  {
    STOPPED,
    MOVING
  };

  enum class GripperStatus
  {
    RESET,
    IN_PROGRESS,
    COMPLETED,
  };

  enum class ObjectDetectionStatus
  {
    MOVING,
    OBJECT_DETECTED_OPENING,
    OBJECT_DETECTED_CLOSING,
    AT_REQUESTED_POSITION
  };

  virtual void set_slave_address(uint8_t slave_address) = 0;
  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  virtual void set_gripper_position(uint8_t pos) = 0;
  virtual uint8_t get_gripper_position() = 0;
  virtual bool gripper_is_moving() = 0;
  virtual void set_speed(uint8_t speed) = 0;
  virtual void set_force(uint8_t force) = 0;
};
}  // namespace robotiq_driver