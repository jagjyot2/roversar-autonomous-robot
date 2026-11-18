#include "wokwi-api.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  pin_t pwma;
  pin_t ain1;
  pin_t ain2;
  pin_t pwmb;
  pin_t bin1;
  pin_t bin2;
  pin_t stby;
  pin_t enc_la;
  pin_t enc_lb;
  pin_t enc_ra;
  pin_t enc_rb;
  uint8_t phase_l;
  uint8_t phase_r;
} chip_state_t;

static const uint8_t QUAD_SEQ[4][2] = {
  {0, 0},
  {1, 0},
  {1, 1},
  {0, 1}
};

static void on_timer(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (!pin_read(chip->stby)) {
    return;
  }

  int dir_l = pin_read(chip->ain1) && !pin_read(chip->ain2) ? 1 :
              (!pin_read(chip->ain1) && pin_read(chip->ain2) ? -1 : 0);
  int dir_r = pin_read(chip->bin1) && !pin_read(chip->bin2) ? 1 :
              (!pin_read(chip->bin1) && pin_read(chip->bin2) ? -1 : 0);

  if (dir_l != 0 && pin_read(chip->pwma)) {
    chip->phase_l = (uint8_t)((chip->phase_l + (dir_l > 0 ? 1 : 3)) & 0x03);
    pin_write(chip->enc_la, QUAD_SEQ[chip->phase_l][0]);
    pin_write(chip->enc_lb, QUAD_SEQ[chip->phase_l][1]);
  }
  if (dir_r != 0 && pin_read(chip->pwmb)) {
    chip->phase_r = (uint8_t)((chip->phase_r + (dir_r > 0 ? 1 : 3)) & 0x03);
    pin_write(chip->enc_ra, QUAD_SEQ[chip->phase_r][0]);
    pin_write(chip->enc_rb, QUAD_SEQ[chip->phase_r][1]);
  }
}

void chip_init(void) {
  chip_state_t *chip = (chip_state_t *)malloc(sizeof(chip_state_t));
  chip->pwma = pin_init("PWMA", INPUT);
  chip->ain1 = pin_init("AIN1", INPUT);
  chip->ain2 = pin_init("AIN2", INPUT);
  chip->pwmb = pin_init("PWMB", INPUT);
  chip->bin1 = pin_init("BIN1", INPUT);
  chip->bin2 = pin_init("BIN2", INPUT);
  chip->stby = pin_init("STBY", INPUT);

  chip->enc_la = pin_init("ENC_LA", OUTPUT_LOW);
  chip->enc_lb = pin_init("ENC_LB", OUTPUT_LOW);
  chip->enc_ra = pin_init("ENC_RA", OUTPUT_LOW);
  chip->enc_rb = pin_init("ENC_RB", OUTPUT_LOW);
  chip->phase_l = 0;
  chip->phase_r = 0;

  const timer_config_t timer_cfg = {
    .callback = on_timer,
    .user_data = chip,
  };
  timer_t timer = timer_init(&timer_cfg);
  timer_start(timer, 1000, true);  // 1 kHz quadrature tick stub
}
