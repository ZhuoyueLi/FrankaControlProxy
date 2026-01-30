#pragma once

#include <exception>
#include <string>
#include <sstream>

namespace robotiq_driver
{
class DriverException : public std::exception
{
  std::string what_;

public:
  explicit DriverException(const std::string& description)
  {
    std::stringstream ss;
    ss << "DriverException: " << description << ".";
    what_ = ss.str();
  }

  DriverException(const DriverException& other) : what_(other.what_)
  {
  }

  ~DriverException() override = default;

  DriverException& operator=(const DriverException&) = delete;

  [[nodiscard]] const char* what() const throw() override
  {
    return what_.c_str();
  }
};
}  // namespace robotiq_driver