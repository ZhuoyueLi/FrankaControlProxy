#include "robotiq_driver/fake/fake_driver.hpp"

#include <zerolancom/zerolancom.hpp>

namespace robotiq_driver
{
void FakeDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  zlc::info("slave_address set to: {}", slave_address);
}

bool FakeDriver::connect()
{
  connected_ = true;
  zlc::info("Gripper connected.");
  return true;
}

void FakeDriver::disconnect()
{
  zlc::info("Gripper disconnected.");
  connected_ = false;
}

void FakeDriver::activate()
{
  zlc::info("Gripper activated.");
  activated_ = true;
}

void FakeDriver::deactivate()
{
  zlc::info("Gripper deactivated.");
  activated_ = false;
}

void FakeDriver::set_gripper_position(uint8_t position)
{
  position_ = position;
}

uint8_t FakeDriver::get_gripper_position()
{
  return position_;
}

bool FakeDriver::gripper_is_moving()
{
  return gripper_is_moving_;
}

void FakeDriver::set_speed(uint8_t speed)
{
  zlc::info("Set gripper speed.");
  speed_ = speed;
}

void FakeDriver::set_force(uint8_t force)
{
  zlc::info("Set gripper force.");
  force_ = force;
}

}  // namespace robotiq_driver