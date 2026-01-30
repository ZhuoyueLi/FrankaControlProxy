#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace robotiq_driver
{
class Serial
{
public:
  virtual ~Serial() = default;
  virtual void open() = 0;
  [[nodiscard]] virtual bool is_open() const = 0;
  virtual void close() = 0;
  [[nodiscard]] virtual std::vector<uint8_t> read(size_t size = 1) = 0;
  virtual void write(const std::vector<uint8_t>& data) = 0;
  virtual void set_port(const std::string& port) = 0;
  [[nodiscard]] virtual std::string get_port() const = 0;
  virtual void set_timeout(std::chrono::milliseconds timeout_ms) = 0;
  [[nodiscard]] virtual std::chrono::milliseconds get_timeout() const = 0;
  virtual void set_baudrate(uint32_t baudrate) = 0;
  [[nodiscard]] virtual uint32_t get_baudrate() const = 0;
};
}  // namespace robotiq_driver