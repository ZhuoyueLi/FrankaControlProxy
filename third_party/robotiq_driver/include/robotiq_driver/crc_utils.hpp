#pragma once

#include <cstdint>
#include <vector>

namespace robotiq_driver
{
namespace crc_utils
{
uint16_t compute_crc(const std::vector<uint8_t>& bytes);

}  // namespace crc_utils
}  // namespace robotiq_driver