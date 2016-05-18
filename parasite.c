#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "twi_master_config.h"
#include "twi_master.h"

#include "parasite.h"

bool parasite_turnon() {
  uint8_t address = PARASITE_ADDRESS;  // address of ESP8266 
  uint8_t twiBuf[6];
  bool success = true;

  twi_master_init();
  
  twiBuf[0] = PARASITE_INIT_AI_REGISTER;
  // Set PSC0, PWM0, PSC1, PWM1 (can remove if not needed)
  twiBuf[1] = 0x2b;  // PSC0 = 1Hz
  twiBuf[2] = 0x80;  // PWM0 = 128/256 = 0.5
  twiBuf[3] = 0x0a;  // PSC1 = 4Hz
  twiBuf[4] = 0xc0;  // PWM1 = (256 - 192)/256 = 0.25
  // Set LEDs
  twiBuf[5] = 0x17; // 00 01 01 11

  success &= twi_master_transfer(address << 1, twiBuf, 6, true);

  twi_master_deinit();
  
  return success;
}

bool parasite_turnoff() {
  uint8_t address = PARASITE_ADDRESS;  // address of ESP8266 
  uint8_t twiBuf[2];
  bool success = true;

  twi_master_init();

  twiBuf[0] = PARASITE_LED_REGISTER;
  // Set LEDs
  twiBuf[1] = 0x55; // 01 01 01 01

  success &= twi_master_transfer(address << 1, twiBuf, 2, true);

  twi_master_deinit();

  return success;
}

bool parasite_reset() {
  uint8_t address = PARASITE_ADDRESS;  // address of ESP8266 
  uint8_t twiBuf[2];
  bool success = true;

  twi_master_init();

  twiBuf[0] = PARASITE_LED_REGISTER;
  // Set LEDs
  twiBuf[1] = 0x04; // 00 00 01 00

  success &= twi_master_transfer(address << 1, twiBuf, 2, true);

  twi_master_deinit();

  return success;
}
