// code fragments for LIS3DE accelerometer
// check schematic for pin connections!

#include "lis35de.h"

uint8_t lis35_reboot[] = {LIS35_WRITE | LIS35_ADDR_NO_INC | LIS35_REG_CR2, LIS35_CR2_BOOT};
uint8_t lis35_setup[] = {LIS35_WRITE | LIS35_ADDR_NO_INC | LIS35_REG_CR1, LIS35_CR1_XEN | LIS35_CR1_YEN | LIS35_CR1_ZEN | LIS35_CR1_ACTIVE};

struct lis35_data_ lis_data;
uint8_t lis35_cmd[] = {LIS35_READ | LIS35_ADDR_INC | LIS35_REG_OUTX, 0, 0, 0, 0, 0};

