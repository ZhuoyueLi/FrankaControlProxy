#include "robotiq_driver/crc_utils.hpp"

namespace robotiq_driver
{
namespace crc_utils
{
uint16_t compute_crc(const std::vector<uint8_t>& bytes)
{
  uint16_t crc = 0xFFFF;
  for (auto b : bytes)
  {
    crc ^= static_cast<uint16_t>(b);
    for (int i = 0; i < 8; ++i)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

}  // namespace crc_utils
}  // namespace robotiq_driver