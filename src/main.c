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
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"


/*
 * - Reset: System initialization not yet complete.
 * - Idle: System init complete, waiting for trigger.
 * - Triggered: User button pressed, set up sensor for data collection,
 *   interrupts.
 * - Running: Collecting data and writing it to flash.
 * - Finished: Flash is full or the target number of readings/experiment time
 *   has been achieved.
 */

enum experiment_state {
  EXP_RESET,
  EXP_IDLE,
  EXP_TRIGGERED,
  EXP_RUNNING,
  EXP_FINISHED,
};

/*
 * 2048 - sizeof(uint32_t)*4 = 2032
 * 2032 / sizeof(struct tsl_reading) = 508
 */
#define MAX_PAGE (FLASH_BANK_SIZE / FLASH_PAGE_SIZE)
#define MAX_READINGS_PER_PAGE 508

struct tsl_reading {
  uint16_t channel0;
  uint16_t channel1;
};

struct page_record {
  uint32_t start_seconds, start_suseconds;
  uint32_t end_seconds, end_suseconds;
  struct tsl_reading readings[MAX_READINGS_PER_PAGE];
};

enum experiment_state sysState = EXP_RESET;
struct page_record page;
int readingsPos = 0;
uint32_t *pagePos = NULL;
static binary_semaphore_t tslSem;
static void tslInit(void);

void rtcConvertDateTimeToTimestamp(RTCDateTime *dateTime, uint32_t *seconds, uint32_t *ms) {
  struct tm time;
  rtcConvertDateTimeToStructTm(dateTime, &time, ms);
  *seconds = mktime(&time);
}


/*
 * Flash utility functions
 */

uint32_t *flashData = (uint32_t *)(FLASH_BASE + 0x80000);

/*
 * Erase `flash_data` so it can be written. Returns 0xffffffff on success.
 */
int flashEraseData(void) {
  uint32_t pageerr;

  /* If flashData spans two banks, erase them separately. */
  FLASH_EraseInitTypeDef eraseinit = {
    FLASH_TYPEERASE_PAGES,
    FLASH_BANK_2,
    0,
    FLASH_BANK_SIZE / FLASH_PAGE_SIZE,
  };
  chSysLock();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  HAL_FLASHEx_Erase(&eraseinit, &pageerr);
  HAL_FLASH_Lock();
  chSysUnlock();

  return pageerr;
}

int flashWritePage(uint32_t addr, uint8_t *data, uint32_t nbytes) {
  int err, rc;

  if (nbytes > FLASH_PAGE_SIZE) {
    /* Only write one page */
    nbytes = FLASH_PAGE_SIZE;
  }

  for (unsigned int i = 0; i < nbytes; i += 8) {
    uint64_t dword = *((uint64_t *)&data[i]);
    err = 0;
    chSysLock();
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dword);
    if (rc != HAL_OK)
      err = HAL_FLASH_GetError();
    HAL_FLASH_Lock();
    chSysUnlock();
    if (err != 0)
      return err;
  }

  return rc;
}

int flashFindNextEmptySlot(void) {
  for (unsigned int i = 0; i < FLASH_BANK_SIZE / FLASH_PAGE_SIZE; i++) {
    if (flashData[i*(FLASH_PAGE_SIZE/sizeof(uint32_t))] == 0xffffffff)
      return i;
  }

  return -1;
}

/*
 * Initialize flash.
 *
 * Check if it's already been initialized. If yes, find the next empty slot
 * and return its index. If no, erase the flash page and fill the first slot
 * with the magic value. Returns negative value on failure.
 */
int flashInit(void) {
  if (flashData[0] != 0xffffffff)
    return -1;
  int empty = flashFindNextEmptySlot();
  if (empty < 0)
    return -1;
  pagePos = &flashData[empty*(FLASH_PAGE_SIZE/sizeof(uint32_t))];
  return 0;
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

static inline void reset(void) {
  sysState = EXP_IDLE;
}

static inline void idle(void) {
  if (palReadLine(LINE_BUTTON) == 0) {
    sysState = EXP_TRIGGERED;
  }
}

static inline void triggered(void) {
  chprintf((BaseSequentialStream *)&SD2, "Triggered\r\n");

  if (flashInit() < 0) {
    chprintf((BaseSequentialStream *)&SD2, "flashInit failed\r\n");
  }
  chprintf((BaseSequentialStream *)&SD2, "initial pagePos: 0x%x\r\n", (uint32_t)pagePos);
  chprintf((BaseSequentialStream *)&SD2, "flashData: 0x%x\r\n", (uint32_t)flashData);

  tslInit();
  
  sysState = EXP_RUNNING;
}

static inline void running(void) {
  if (pagePos == (flashData + MAX_PAGE*FLASH_PAGE_SIZE)) {
    sysState = EXP_FINISHED;
    return;
  }

  if (readingsPos == 0) {
    RTCDateTime dateTime;
    rtcGetTime(&RTCD1, &dateTime);
    rtcConvertDateTimeToTimestamp(&dateTime, &page.start_seconds,
                                  &page.start_suseconds);
  }

  if (readingsPos < MAX_READINGS_PER_PAGE) {
    chBSemWait(&tslSem);
    uint16_t chan0 = tsl2561_getChannel0();
    uint16_t chan1 = tsl2561_getChannel1();
    page.readings[readingsPos].channel0 = chan0;
    page.readings[readingsPos].channel1 = chan1;
    readingsPos += 1;
    chprintf((BaseSequentialStream *)&SD2, "%u %u\r\n", chan0, chan1);
    tsl2561_clearInterrupt();
  } else {
    if (pagePos != NULL) {
      flashWritePage((uint32_t)pagePos, (uint8_t *)&page, sizeof(struct page_record));
      pagePos += (FLASH_PAGE_SIZE / sizeof(uint32_t));
      chprintf((BaseSequentialStream *)&SD2, "Wrote to page %u\r\n",
               ((uint32_t)pagePos - FLASH_BASE) / (FLASH_PAGE_SIZE / sizeof(uint32_t)));
      readingsPos = 0;
    }
  }

  if (readingsPos == (MAX_READINGS_PER_PAGE)) {
    RTCDateTime dateTime;
    rtcGetTime(&RTCD1, &dateTime);
    rtcConvertDateTimeToTimestamp(&dateTime, &page.end_seconds,
                                  &page.end_suseconds);
  }
}

static inline void finished(void) {
  chprintf((BaseSequentialStream *)&SD2, "Finished\r\n");
}

/*
 * Light sensor reader thread.
 */
static THD_WORKING_AREA(waThread2, 256);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;
  chRegSetThreadName("reader");
  while (true) {
    switch (sysState) {
      case EXP_RESET:
        reset();
        break;
      case EXP_IDLE:
        idle();
        break;
      case EXP_TRIGGERED:
        triggered();
        break;
      case EXP_RUNNING:
        running();
        break;
      case EXP_FINISHED:
        finished();
        break;
      default:
        break;
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
  if (sysState == EXP_RUNNING) {
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

static const char *flash_usage = "flash <command>\r\n"
"command:\r\n"
"\terase\r\n"
"\tempty\r\n"
"\tdump\r\n";
static void cmd_flash(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc < 1) {
    chprintf(chp, flash_usage);
    return;
  }

  if (strcmp("erase", argv[0]) == 0) {
    flashEraseData();
  } else if (strcmp("empty", argv[0]) == 0) {
    int empty = flashFindNextEmptySlot();
    chprintf(chp, "empty: ");
    if (empty < 0) {
      chprintf(chp, "none");
    } else {
      chprintf(chp, "%d", empty);
    }
    chprintf(chp, "\r\n");
  } else if (strcmp("dump", argv[0]) == 0) {
    for (uint32_t *page = flashData; page < pagePos; page += (FLASH_PAGE_SIZE/sizeof(uint32_t))) {
    }
  } else {
    chprintf(chp, flash_usage);
  }
}

//static const char *tsl237_usage = "tsl237";
static void cmd_tsl237(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  systime_t start = chVTGetSystemTime();
  systime_t end = start + MS2ST(100);
  uint32_t ticks = 0, count = 0;
  while (chVTIsSystemTimeWithin(start, end)) {
    icuStartCapture(&ICUD1);
    icuWaitCapture(&ICUD1);
    ticks += icuGetPeriodX(&ICUD1);
    count++;
  }
  ticks /= count;
  chprintf(chp, "count: %u, ticks: %u\r\n", count, ticks);
}


#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] = {
  {"tsl", cmd_tsl},
  {"flash", cmd_flash},
  {"tsl237", cmd_tsl237},
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

/* In mcuconf.h, STM32_ICU_USE_TIM1 is TRUE.
 * TSL237 min/max freq: 500/1000 kHz
 * TIM1CH2 = PA9 AF1
 */
static const ICUConfig icucfg = {
  .mode = ICU_INPUT_ACTIVE_LOW,
  .frequency = 2000000,
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU_CHANNEL_2
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

  /* PA9 AF1 */
  palSetLineMode(LINE_ARD_D8, PAL_MODE_ALTERNATE(1));
  icuStart(&ICUD1, &icucfg);

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
