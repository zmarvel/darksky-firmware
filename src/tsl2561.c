
#include "tsl2561.h"

void tsl2561_powerOn(void) {
  uint8_t buf = 0x3;
  tsl2561_cmd(0x40 | TSL2561_REG_CONTROL, &buf, 1, NULL, 0);
}

void tsl2561_powerOff(void) {
  uint8_t buf = 0;
  tsl2561_cmd(TSL2561_REG_CONTROL, &buf, 1, NULL, 0);
}

uint8_t tsl2561_getPower(void) {
  uint8_t buf = 0;
  tsl2561_cmd(TSL2561_REG_CONTROL, NULL, 0, &buf, 1);
  return buf & TSL2561_POWER_MASK;
}

/* Set the gain and integration time.
 */
void tsl2561_setGainInteg(tsl2561_gain gain, tsl2561_integ integ) {
  uint8_t reg = (gain << TSL2561_TIMING_GAIN_POS)
    | (integ << TSL2561_TIMING_INTEG_POS);

  // write new value
  tsl2561_cmd(TSL2561_REG_TIMING, &reg, 1, NULL, 0);
}

static uint8_t tsl2561_getTimingReg(void) {
  uint8_t reg;
  tsl2561_cmd(TSL2561_REG_TIMING, NULL, 0, &reg, 1);
  return reg;
}

void tsl2561_setGain(tsl2561_gain gain) {
  uint8_t reg = tsl2561_getTimingReg();
  reg |= TSL2561_TIMING_GAIN_POS << gain;
}

tsl2561_gain tsl2561_getGain(void) {
  uint8_t reg = tsl2561_getTimingReg();
  return (reg >> TSL2561_TIMING_GAIN_POS) & 1;
}

void tsl2561_setInteg(tsl2561_integ integ) {
  uint8_t reg = tsl2561_getTimingReg();
  reg |= TSL2561_TIMING_INTEG_POS << integ;
}

tsl2561_integ  tsl2561_getInteg(void) {
  uint8_t reg = tsl2561_getTimingReg();
  return (reg >> TSL2561_TIMING_INTEG_POS) & 3;
}

void tsl2561_startInteg(void) {
  uint8_t reg = tsl2561_getTimingReg();
  reg &= ~TSL2561_TIMING_RESERVED_MASK;
  reg |= 1 << TSL2561_TIMING_MANUAL_POS;
  tsl2561_cmd(TSL2561_REG_TIMING, &reg, 1, NULL, 0);
}

void tsl2561_stopInteg(void) {
  uint8_t reg = tsl2561_getTimingReg();
  reg &= ~TSL2561_TIMING_RESERVED_MASK;
  reg &= ~(1 << TSL2561_TIMING_MANUAL_POS);
  tsl2561_cmd(TSL2561_REG_TIMING, &reg, 1, NULL, 0);
}

uint8_t tsl2561_readID(void) {
  uint8_t reg = 0;
  tsl2561_cmd(TSL2561_REG_ID, NULL, 0, &reg, 1);
  return reg;
}

uint16_t tsl2561_getChannel0(void) {
  uint8_t reglow, reghigh;
  tsl2561_cmd(TSL2561_REG_DATA0LOW, NULL, 0, &reglow, 1);
  tsl2561_cmd(TSL2561_REG_DATA0HIGH, NULL, 0, &reghigh, 1);
  return (reghigh << 8) | reglow;
}

uint16_t tsl2561_getChannel1(void) {
  uint8_t reglow, reghigh;
  tsl2561_cmd(TSL2561_REG_DATA1LOW, NULL, 0, &reglow, 1);
  tsl2561_cmd(TSL2561_REG_DATA1HIGH, NULL, 0, &reghigh, 1);
  return (reghigh << 8) | reglow;
}

void tsl2561_setInterrupt(tsl2561_interrupt_mode mode,
                          tsl2561_interrupt_persist persist) {
  uint8_t reg = (mode << TSL2561_INTERRUPT_INTR_POS)
    | (persist << TSL2561_INTERRUPT_PERSIST_POS);
  tsl2561_cmd(0x40 | TSL2561_REG_INTERRUPT, &reg, 1, NULL, 0);
}

void tsl2561_clearInterrupt(void) {
  tsl2561_cmd(0x40, NULL, 0, NULL, 0);
}
