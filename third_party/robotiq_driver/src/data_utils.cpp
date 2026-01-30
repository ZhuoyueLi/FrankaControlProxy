#include "robotiq_driver/data_utils.hpp"

namespace robotiq_driver
{
namespace data_utils
{
uint8_t get_lsb(uint16_t value)
{
  return static_cast<uint8_t>(value & 0xFF);
}

uint8_t get_msb(uint16_t value)
{
  return static_cast<uint8_t>((value >> 8) & 0xFF);
}

}  // namespace data_utils
}  // namespace robotiq_driver