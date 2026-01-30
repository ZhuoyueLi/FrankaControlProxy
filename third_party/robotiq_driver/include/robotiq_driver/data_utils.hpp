#pragma once

#include <cstdint>

namespace robotiq_driver
{
namespace data_utils
{
uint8_t get_lsb(uint16_t value);
uint8_t get_msb(uint16_t value);

}  // namespace data_utils
}  // namespace robotiq_driver
