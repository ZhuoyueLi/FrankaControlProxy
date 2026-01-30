#pragma once

#include "robotiq_driver/driver.hpp"

namespace robotiq_driver
{
class FakeDriver : public Driver
{
public:
  void set_slave_address(uint8_t slave_address) override;
  bool connect() override;
  void disconnect() override;
  void activate() override;
  void deactivate() override;
  void set_gripper_position(uint8_t position) override;
  uint8_t get_gripper_position() override;
  bool gripper_is_moving() override;
  void set_speed(uint8_t speed) override;
  void set_force(uint8_t force) override;

private:
  uint8_t slave_address_ = 0x00;
  bool connected_ = false;
  bool activated_ = false;
  uint8_t position_ = 0;
  bool gripper_is_moving_ = false;
  uint8_t speed_ = 0;
  uint8_t force_ = 0;
};

}  // namespace robotiq_driver