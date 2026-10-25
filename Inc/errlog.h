/**
  ******************************************************************************
  * @file    errlog.h
  * @brief   Append-only error log on the SD card.
  *
  *          Format: one line per event, "[ms] module: text\n". The file name
  *          embeds the firmware's compile date so multiple boots on different
  *          firmware revisions don't clobber each other:
  *              Error_Log_Pump_Tsueri_DD.MM.YYYY.log
  ******************************************************************************
  */
#ifndef PL_ERRLOG_H
#define PL_ERRLOG_H

#include <stdint.h>

/* Must be called once after SDFat_Mount(). Opens / appends to the log file. */
int  ErrLog_Init(void);

/* Append one line of text. The function prepends "[<ms> ms] " and appends
   '\n'. No-op when SD isn't mounted. */
void ErrLog_Write(const char *s);

/* printf-style sister. */
void ErrLog_Writef(const char *fmt, ...);

/* Wall-clock anchor (the box has no RTC). Logs one
   "clock: YYYY-MM-DD HH:MM:SS UTC (src=host|gps) boot=… UTC" line and from
   then on the per-minute heartbeat carries "utc=…". Host SET_TIME re-anchors
   on every connect; the GPS variant only anchors once, and only if no host
   has yet. */
void ErrLog_SetWallClock(uint64_t epoch_ms);
void ErrLog_SetWallClockGps(unsigned year, unsigned mon, unsigned day,
                            unsigned hh, unsigned mm, unsigned ss);

/* Heartbeat dump — called once per minute by main. */
void ErrLog_Heartbeat(void);

/* Flush whatever is buffered to SD. */
void ErrLog_Flush(void);

/* Warning latch — set to 1 the first time ErrLog_Write is called with a
   line starting with '***' (the convention for prominent errlog
   markers). Main loop reads this to drive the red LED on solid, so an
   operator sees "something significant happened, check the errlog"
   without needing to pull the SD card during a session. One-shot per
   boot — once latched, stays latched until reset. */
uint8_t ErrLog_WarningLatched(void);

#endif
