#include "robotiq_driver/default_driver.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "robotiq_driver/data_utils.hpp"
#include "robotiq_driver/crc_utils.hpp"
#include "robotiq_driver/driver_exception.hpp"

#include <zerolancom/zerolancom.hpp>

namespace robotiq_driver
{
constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint16_t kFirstOutputRegister = 0x07D0;
constexpr uint16_t kNumOutputRegisters = 0x0006;
constexpr int kReadResponseSize = 2 * kNumOutputRegisters + 5;
constexpr uint8_t kWriteFunctionCode = 0x10;
constexpr uint16_t kActionRequestRegister = 0x03E8;
constexpr int kWriteResponseSize = 8;
constexpr size_t kResponseHeaderSize = 3;
constexpr size_t kGripperStatusIndex = 0;
constexpr size_t kPositionIndex = 4;
constexpr auto kMaxRetries = 5;

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial)
  : serial_{ std::move(serial) }, commanded_gripper_speed_(0x80), commanded_gripper_force_(0x80)
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries)
  {
    try
    {
      serial_->write(request);
      response = serial_->read(response_size);
      break;
    }
    catch (const serial::IOException& e)
    {
      zlc::warn("Resending the command because the previous attempt ({} of {}) failed: {}", retry_count + 1, kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries)
  {
    zlc::error("Reached maximum retries. Operation failed.");
    return {};
  }

  return response;
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
}

void DefaultDriver::activate()
{
  zlc::info("Activate...");

  const auto request = create_write_command(kActionRequestRegister, { 0x0100, 0x0000, 0x0000 });
  auto response = send(request, kWriteResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to activate the gripper." };
  }

  update_status();
  if (gripper_status_ == GripperStatus::COMPLETED)
  {
    return;
  }
  while (gripper_status_ == GripperStatus::IN_PROGRESS)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    update_status();
  }
}

void DefaultDriver::deactivate()
{
  zlc::info("Deactivate...");

  const auto request = create_write_command(kActionRequestRegister, { 0x0000, 0x0000, 0x0000 });
  auto response = send(request, kWriteResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to deactivate the gripper." };
  }
}

void DefaultDriver::set_gripper_position(uint8_t pos)
{
  uint8_t action_register = 0x09;
  uint8_t gripper_options_1 = 0x00;
  uint8_t gripper_options_2 = 0x00;

  const auto request =
      create_write_command(kActionRequestRegister,
                           { uint16_t(action_register << 8 | gripper_options_1), uint16_t(gripper_options_2 << 8 | pos),
                             uint16_t(commanded_gripper_speed_ << 8 | commanded_gripper_force_) });

  auto response = send(request, kWriteResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to set gripper position." };
  }
}

uint8_t DefaultDriver::get_gripper_position()
{
  update_status();
  return gripper_position_;
}

bool DefaultDriver::gripper_is_moving()
{
  update_status();
  return object_detection_status_ == ObjectDetectionStatus::MOVING;
}

void DefaultDriver::set_speed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
}

void DefaultDriver::set_force(uint8_t force)
{
  commanded_gripper_force_ = force;
}

std::vector<uint8_t> DefaultDriver::create_read_command(uint16_t first_register, uint8_t num_registers)
{
  std::vector<uint8_t> request = { slave_address_,
                                   kReadFunctionCode,
                                   data_utils::get_msb(first_register),
                                   data_utils::get_lsb(first_register),
                                   data_utils::get_msb(num_registers),
                                   data_utils::get_lsb(num_registers) };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));
  return request;
}

std::vector<uint8_t> DefaultDriver::create_write_command(uint16_t first_register, const std::vector<uint16_t>& data)
{
  uint16_t num_registers = data.size();
  uint8_t num_bytes = 2 * num_registers;

  std::vector<uint8_t> request = { slave_address_,
                                   kWriteFunctionCode,
                                   data_utils::get_msb(first_register),
                                   data_utils::get_lsb(first_register),
                                   data_utils::get_msb(num_registers),
                                   data_utils::get_lsb(num_registers),
                                   num_bytes };
  for (auto d : data)
  {
    request.push_back(data_utils::get_msb(d));
    request.push_back(data_utils::get_lsb(d));
  }

  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  return request;
}

void DefaultDriver::update_status()
{
  const auto request = create_read_command(kFirstOutputRegister, kNumOutputRegisters);
  auto response = send(request, kReadResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to read the gripper status." };
  }

  uint8_t gripper_status_byte = response[kResponseHeaderSize + kGripperStatusIndex];

  activation_status_ = ((gripper_status_byte & 0x01) == 0x00) ? ActivationStatus::RESET : ActivationStatus::ACTIVE;

  action_status_ = ((gripper_status_byte & 0x08) == 0x00) ? ActionStatus::STOPPED : ActionStatus::MOVING;

  switch ((gripper_status_byte & 0x30) >> 4)
  {
    case 0x00:
      gripper_status_ = GripperStatus::RESET;
      break;
    case 0x01:
      gripper_status_ = GripperStatus::IN_PROGRESS;
      break;
    case 0x03:
      gripper_status_ = GripperStatus::COMPLETED;
      break;
  }

  switch ((gripper_status_byte & 0xC0) >> 6)
  {
    case 0x00:
      object_detection_status_ = ObjectDetectionStatus::MOVING;
      break;
    case 0x01:
      object_detection_status_ = ObjectDetectionStatus::OBJECT_DETECTED_OPENING;
      break;
    case 0x02:
      object_detection_status_ = ObjectDetectionStatus::OBJECT_DETECTED_CLOSING;
      break;
    case 0x03:
      object_detection_status_ = ObjectDetectionStatus::AT_REQUESTED_POSITION;
      break;
  }

  gripper_position_ = response[kResponseHeaderSize + kPositionIndex];
}

}  // namespace robotiq_driver
