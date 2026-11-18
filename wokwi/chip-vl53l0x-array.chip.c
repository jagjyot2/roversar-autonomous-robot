#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  pin_t xshut[5];
  uint32_t center_dist_attr;
  uint8_t reg_addr;
} vl53_array_t;

static bool on_i2c_connect(void *user_data, uint32_t address, bool read) {
  (void)user_data;
  (void)read;
  // Default 0x29 or reassigned 0x30..0x34 per roversar_config.h
  return address == 0x29 || (address >= 0x30 && address <= 0x34);
}

static uint8_t on_i2c_read(void *user_data) {
  vl53_array_t *chip = (vl53_array_t *)user_data;
  uint32_t mm = attr_read(chip->center_dist_attr);
  if (chip->reg_addr == 0x1E) {
    return (uint8_t)((mm >> 8) & 0xFF);
  }
  if (chip->reg_addr == 0x1F) {
    return (uint8_t)(mm & 0xFF);
  }
  return 0xEE;  // VL53L0X model ID stub
}

static bool on_i2c_write(void *user_data, uint8_t data) {
  vl53_array_t *chip = (vl53_array_t *)user_data;
  chip->reg_addr = data;
  return true;
}

void chip_init(void) {
  vl53_array_t *chip = (vl53_array_t *)malloc(sizeof(vl53_array_t));
  chip->xshut[0] = pin_init("XSHUT0", INPUT);
  chip->xshut[1] = pin_init("XSHUT1", INPUT);
  chip->xshut[2] = pin_init("XSHUT2", INPUT);
  chip->xshut[3] = pin_init("XSHUT3", INPUT);
  chip->xshut[4] = pin_init("XSHUT4", INPUT);
  chip->center_dist_attr = attr_init("centerDistanceMm", 1200);
  chip->reg_addr = 0;

  const i2c_config_t i2c_cfg = {
    .user_data = chip,
    .address = 0x32,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_i2c_connect,
    .read = on_i2c_read,
    .write = on_i2c_write,
  };
  i2c_init(&i2c_cfg);
}
