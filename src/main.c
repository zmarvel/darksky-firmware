/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
    Copyright (C) 2017 Zack Marvel

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "tsl2561.h"


struct tsl_reading {
  uint32_t seconds;
  uint32_t suseconds;
  uint16_t channel0;
  uint16_t channel1;
};


static bool tslReady = false;
static binary_semaphore_t tslSem;
static void tslInit(void);

void rtcConvertDateTimeToTimestamp(RTCDateTime *dateTime, uint32_t *seconds, uint32_t *ms) {
  struct tm time;
  rtcConvertDateTimeToStructTm(dateTime, &time, ms);
  *seconds = mktime(&time);
}


/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}
/*
 * Light sensor reader thread.
 */
static struct tsl_reading readings[166];
static int readingsPos = 0;
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("reader");
  RTCDateTime dateTime;
  struct tsl_reading *reading;
  while (true) {
    if (tslReady && readingsPos < 166) {
      chBSemWait(&tslSem);
      reading = &readings[readingsPos];
      rtcGetTime(&RTCD1, &dateTime);
      rtcConvertDateTimeToTimestamp(&dateTime, &reading->seconds,
                                    &reading->suseconds);
      reading->channel0 = tsl2561_getChannel0();
      reading->channel1 = tsl2561_getChannel1();
      readingsPos += 1;
      chprintf((BaseSequentialStream *)&SD2, "%u %u\r\n", reading->channel0, reading->channel1);
      tsl2561_clearInterrupt();
    } else {
      tslInit();
    }
    chThdSleepMilliseconds(1);
  }
}

/*
 * Light sensor interrupt handler
 */
static void tslHandler(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  if (tslReady) {
    chBSemSignalI(&tslSem);
  }
}

static EXTConfig extcfg = {
  {
    { EXT_MODE_GPIOB | EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, tslHandler },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
    { EXT_CH_MODE_DISABLED, NULL },
  }
};

static void tslInit(void) {
  chBSemObjectInit(&tslSem, true);

  tsl2561_powerOff();
  chThdSleepMilliseconds(2);
  tsl2561_powerOn();
  while (tsl2561_getPower() != 0x03) {
    chThdSleepMilliseconds(1);
  }
  tsl2561_setGainInteg(TSL2561_HIGHGAIN, TSL2561_INTEG_402);

  palSetLineMode(LINE_ARD_A3, PAL_MODE_INPUT_PULLUP);

  tsl2561_setInterrupt(TSL2561_INTMODE_LEVEL, 0);
  tslReady = true;
}


static inline void tsl2561_printGain(BaseSequentialStream *chp, tsl2561_gain gain) {
  switch (gain) {
    case TSL2561_LOWGAIN:
      chprintf(chp, "low");
      break;
    case TSL2561_HIGHGAIN:
      chprintf(chp, "high");
      break;
  }
}

static inline void tsl2561_printInteg(BaseSequentialStream *chp, tsl2561_integ integ) {
  switch (integ) {
    case TSL2561_INTEG_14:
      chprintf(chp, "13.7 ms");
      break;
    case TSL2561_INTEG_101:
      chprintf(chp, "101 ms");
      break;
    case TSL2561_INTEG_402:
      chprintf(chp, "402 ms");
      break;
    case TSL2561_INTEG_MANUAL:
      chprintf(chp, "manual");
      break;
  }
}

static const char *tsl_usage = "tsl <command>\r\n"
"command:\r\n"
"\tid\r\n"
"\tread\r\n"
"\tpower [on | off]\r\n"
"\tgain [low | high]\r\n"
"\tinteg [13.7 | 101 | 402 | manual]\r\n";
static void cmd_tsl(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc < 1) {
    chprintf(chp, tsl_usage);
    return;
  }

  if (strcmp("read", argv[0]) == 0) {
    uint16_t data0 = tsl2561_getChannel0();
    uint16_t data1 = tsl2561_getChannel1();
    chprintf(chp, "data0: %u\r\ndata1: %u\r\n", data0, data1);
  } else if (strcmp("id", argv[0]) == 0) {
    uint8_t id = tsl2561_readID();
    chprintf(chp, "id: 0x%x\r\n", id);
  } else if (strcmp("power", argv[0]) == 0) {
    if (argc < 2) {
      uint8_t reg = tsl2561_getPower();
      chprintf(chp, "power: 0x%x\r\n", reg);
    } else if (strcmp("on", argv[1]) == 0) {
      tsl2561_powerOn();
    } else if (strcmp("off", argv[1]) == 0) {
      tsl2561_powerOff();
    } else {
      chprintf(chp, tsl_usage);
    }
  } else if (strcmp("gain", argv[0]) == 0) {
    if (argc < 2) {
      tsl2561_gain gain = tsl2561_getGain();
      chprintf(chp, "gain: ");
      tsl2561_printGain(chp, gain);
      chprintf(chp, "\r\n");
    } else if (strcmp("low", argv[1]) == 0) {
      tsl2561_setGain(TSL2561_LOWGAIN);
    } else if (strcmp("high", argv[1]) == 0) {
      tsl2561_setGain(TSL2561_HIGHGAIN);
    } else {
      chprintf(chp, tsl_usage);
    }
  } else if (strcmp("integ", argv[0]) == 0) {
    if (argc < 2) {
      tsl2561_integ integ = tsl2561_getInteg();
      chprintf(chp, "integ: ");
      tsl2561_printInteg(chp, integ);
      chprintf(chp, "\r\n");
    } else if (strcmp("13.7", argv[1]) == 0) {
      tsl2561_setInteg(TSL2561_INTEG_14);
    } else if (strcmp("101", argv[1]) == 0) {
      tsl2561_setInteg(TSL2561_INTEG_101);
    } else if (strcmp("402", argv[1]) == 0) {
      tsl2561_setInteg(TSL2561_INTEG_402);
    } else if (strcmp("manual", argv[1]) == 0) {
      tsl2561_setInteg(TSL2561_INTEG_MANUAL);
    } else {
      chprintf(chp, tsl_usage);
    }
   } else {
    chprintf(chp, tsl_usage);
  }
}


#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] = {
  {"tsl", cmd_tsl},
  {NULL, NULL},
};

static const ShellConfig shellcfg = {
  (BaseSequentialStream *)&SD2,
  commands,
};


static const I2CConfig i2ccfg = {
  .timingr = STM32_TIMINGR_PRESC(16)
    | STM32_TIMINGR_SCLDEL(15) | STM32_TIMINGR_SDADEL(15)
    | STM32_TIMINGR_SCLH(64) | STM32_TIMINGR_SCLL(64),
  .cr1 = 0,
  .cr2 = 0,
};

/*
 * Application entry point.
 */
int main(void) {
  

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  thread_t *shelltp = NULL;
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  palSetLineMode(LINE_ARD_D14, PAL_STM32_OTYPE_OPENDRAIN | PAL_MODE_ALTERNATE(4));
  palSetLineMode(LINE_ARD_D15, PAL_STM32_OTYPE_OPENDRAIN | PAL_MODE_ALTERNATE(4));

  i2cStart(&I2CD1, &i2ccfg);

  extStart(&EXTD1, &extcfg);
  shellInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    if (shelltp == NULL) {
      shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO+2,
                                    shellThread, (void *)&shellcfg);
    }

    chThdSleepMilliseconds(500);
  }
}
