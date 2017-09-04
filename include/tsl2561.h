
#ifndef _TSL2561_H
#define _TSL2561_H

#include <stdint.h>
#include "ch.h"
#include "hal.h"

#define TSL2561_SLAVE_ADDR            0x39

#define TSL2561_REG_CONTROL           0x0
#define TSL2561_POWER_MASK            0x3
#define TSL2561_REG_TIMING            0x1
#define TSL2561_TIMING_GAIN_POS       4
#define TSL2561_TIMING_MANUAL_POS     0
#define TSL2561_TIMING_INTEG_POS      0
#define TSL2561_TIMING_RESERVED_MASK  0xe4
#define TSL2561_REG_THRESHLOWLOW      0x2
#define TSL2561_REG_THRESHLOWHIGH     0x3
#define TSL2561_REG_THRESHHIGHLOW     0x4
#define TSL2561_REG_THRESHHIGHHIGH    0x5
#define TSL2561_REG_INTERRUPT         0x6
#define TSL2561_INTERRUPT_INTR_POS    4
#define TSL2561_INTERRUPT_PERSIST_POS 0
#define TSL2561_REG_ID                0xa
#define TSL2561_REG_DATA0LOW          0xc
#define TSL2561_REG_DATA0HIGH         0xd
#define TSL2561_REG_DATA1LOW          0xe
#define TSL2561_REG_DATA1HIGH         0xf

/* must implement */
static inline void tsl2561_cmd(uint8_t addr, uint8_t *txbuf, uint8_t txbytes,
                              uint8_t *rxbuf, uint8_t rxbytes) {
  // txbytes must be less than 7
  uint8_t buf[8];
  buf[0] = 0x80 | addr;
  for (int i = 0; i < txbytes; i++)
    buf[i+1] = txbuf[i];
  txbytes += 1;

  i2cAcquireBus(&I2CD1);
  i2cMasterTransmit(&I2CD1, TSL2561_SLAVE_ADDR, buf, txbytes, rxbuf, rxbytes);
  i2cReleaseBus(&I2CD1);
}

void tsl2561_powerOn(void);
void tsl2561_powerOff(void);
uint8_t tsl2561_getPower(void);

typedef enum {
  TSL2561_LOWGAIN = 0,
  TSL2561_HIGHGAIN = 1
} tsl2561_gain;

typedef enum {
  TSL2561_INTEG_14 = 0, // 13.7 ms
  TSL2561_INTEG_101 = 1, // 101 ms
  TSL2561_INTEG_402 = 2, // 402 ms
  TSL2561_INTEG_MANUAL = 3 // manual integration time
} tsl2561_integ;

void tsl2561_setGainInteg(tsl2561_gain, tsl2561_integ);
void tsl2561_setGain(tsl2561_gain);
uint8_t tsl2561_getGain(void);
void tsl2561_setInteg(tsl2561_integ);
uint8_t tsl2561_getInteg(void);
void tsl2561_startInteg(void);
void tsl2561_stopInteg(void);

uint8_t tsl2561_readID(void);

uint16_t tsl2561_getChannel0(void);
uint16_t tsl2561_getChannel1(void);

typedef enum {
  TSL2561_INTMODE_DISABLED = 0,
  TSL2561_INTMODE_LEVEL = 1,
  TSL2561_INTMODE_SMBALERT = 2,
  TSL2561_INTMODE_TEST = 3
} tsl2561_interrupt_mode;

typedef uint8_t tsl2561_interrupt_persist;

void tsl2561_setInterrupt(tsl2561_interrupt_mode, tsl2561_interrupt_persist);
void tsl2561_clearInterrupt(void);

#endif /* _TSL2561_H */ 
