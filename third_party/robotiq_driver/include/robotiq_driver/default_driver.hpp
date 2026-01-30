#pragma once

#include <memory>
#include <string>

#include "robotiq_driver/driver.hpp"
#include "robotiq_driver/serial.hpp"

namespace robotiq_driver
{
class DefaultDriver : public Driver
{
public:
  explicit DefaultDriver(std::unique_ptr<Serial> serial);

  bool connect() override;
  void disconnect() override;

  void set_slave_address(uint8_t slave_address) override;
  void activate() override;
  void deactivate() override;

  void set_gripper_position(uint8_t pos) override;
  uint8_t get_gripper_position() override;
  bool gripper_is_moving() override;

  void set_speed(uint8_t speed) override;
  void set_force(uint8_t force) override;

private:
  std::vector<uint8_t> send(const std::vector<uint8_t>& request, size_t response_size) const;

  std::vector<uint8_t> create_read_command(uint16_t first_register, uint8_t num_registers);
  std::vector<uint8_t> create_write_command(uint16_t first_register, const std::vector<uint16_t>& data);

  void update_status();

  std::unique_ptr<Serial> serial_ = nullptr;
  uint8_t slave_address_;

  ActivationStatus activation_status_;
  ActionStatus action_status_;
  GripperStatus gripper_status_;
  ObjectDetectionStatus object_detection_status_;

  uint8_t gripper_position_;
  uint8_t commanded_gripper_speed_;
  uint8_t commanded_gripper_force_;
};
}  // namespace robotiq_driver