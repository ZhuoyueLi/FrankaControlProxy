#pragma once

#include <serial/serial.h>

#include <memory>
#include <string>
#include <vector>

#include "robotiq_driver/serial.hpp"

namespace serial
{
class Serial;
}

namespace robotiq_driver
{
class DefaultSerial : public Serial
{
public:
  DefaultSerial();

  void open() override;
  [[nodiscard]] bool is_open() const override;
  void close() override;

  [[nodiscard]] std::vector<uint8_t> read(size_t size = 1) override;
  void write(const std::vector<uint8_t>& data) override;

  void set_port(const std::string& port) override;
  [[nodiscard]] std::string get_port() const override;

  void set_timeout(std::chrono::milliseconds timeout_ms) override;
  [[nodiscard]] std::chrono::milliseconds get_timeout() const override;

  void set_baudrate(uint32_t baudrate) override;
  [[nodiscard]] uint32_t get_baudrate() const override;

private:
  std::unique_ptr<serial::Serial> serial_ = nullptr;
};
}  // namespace robotiq_driver