/**
  ******************************************************************************
  * @file    errlog.c
  * @brief   Append-only error log.
  ******************************************************************************
  */
#include "main.h"
#include "errlog.h"
#include "sd_fatfs.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static PL_File g_log;
static int     g_inited;

/* Build "Error_Log_Pump_Tsueri_DD.MM.YYYY.log" from __DATE__ which is
   "Mmm dd yyyy". Truncated to 8.3 — minimal FAT writer is 8.3 only.
   So we choose a stable, sortable short name on the card: ERRLOG.LOG.
   The pretty name with date is embedded in the first line of the file. */
#define LOG_FILENAME  "ERRLOG.LOG"

static const char *month3_to_num(const char *m)
{
  static const char *names[] = {"Jan","Feb","Mar","Apr","May","Jun",
                                "Jul","Aug","Sep","Oct","Nov","Dec"};
  static const char *nums[]  = {"01","02","03","04","05","06",
                                "07","08","09","10","11","12"};
  for (int i = 0; i < 12; i++) {
    if (m[0]==names[i][0] && m[1]==names[i][1] && m[2]==names[i][2]) return nums[i];
  }
  return "00";
}

static void format_date(char *out, size_t len)
{
  /* __DATE__ = "Mmm dd yyyy" */
  const char *d = __DATE__;
  const char *mm = month3_to_num(d);
  /* Day might have leading space "Mmm  5 yyyy" — handle. */
  char dd[3];
  dd[0] = (d[4] == ' ') ? '0' : d[4];
  dd[1] = d[5];
  dd[2] = '\0';
  snprintf(out, len, "%s.%s.%c%c%c%c", dd, mm, d[7], d[8], d[9], d[10]);
}

static void decode_reset(char *out, size_t len, uint32_t csr)
{
  out[0] = '\0';
  /* STM32U5 RCC_CSR reset flags (RM0456 / stm32u585xx.h):
       bit25 = OBLRSTF  (option-byte-loader launch — every FOTA bank swap)
       bit26 = PINRSTF  (NRST pin — accompanies most internal resets)
       bit27 = BORRSTF  (brown-out — power was cut/ramped: magnet cycle)
       bit28 = SFTRSTF  (NVIC_SystemReset)
       bit29 = IWDGRSTF   bit30 = WWDGRSTF   bit31 = LPWRRSTF
     Until v0.0.47 this table was shifted (assumed bit23=LPWR…29=OBL), so
     every cause in older field logs is MISLABELED: PIN+BOR (magnet
     power-cycle) printed as "SOFTWARE+BOR", PIN+OBL (FOTA swap) as
     "IWDG+SOFTWARE", SFT+PIN as "SOFTWARE+PIN", IWDG+PIN (real watchdog)
     as "SOFTWARE+POR", and a bare PIN reset (NRST glitch!) as "SOFTWARE".
     Translate accordingly when reading logs older than this fix. */
  const struct { uint32_t msk; const char *name; } flags[] = {
    { RCC_CSR_OBLRSTF_Msk,  "OBL"      },
    { RCC_CSR_PINRSTF_Msk,  "PIN"      },
    { RCC_CSR_BORRSTF_Msk,  "BOR"      },
    { RCC_CSR_SFTRSTF_Msk,  "SOFTWARE" },
    { RCC_CSR_IWDGRSTF_Msk, "IWDG"     },
    { RCC_CSR_WWDGRSTF_Msk, "WWDG"     },
    { RCC_CSR_LPWRRSTF_Msk, "LPWR"     },
  };
  int any = 0;
  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
    if (csr & flags[i].msk) {
      if (any) strncat(out, "+", len - strlen(out) - 1);
      strncat(out, flags[i].name, len - strlen(out) - 1);
      any = 1;
    }
  }
  if (!any) strncpy(out, "UNKNOWN", len);
}

extern uint32_t BootResetCsr;  /* set by main() right after HAL_Init */

int ErrLog_Init(void)
{
  if (!SDFat_IsMounted()) return -1;
  if (SDFat_OpenAppend(&g_log, LOG_FILENAME) != PL_FX_OK) return -1;
  g_inited = 1;

  /* Boot banner. */
  char hdr[160];
  char date[16];
  char reset[40];
  format_date(date, sizeof(date));
  decode_reset(reset, sizeof(reset), BootResetCsr);

  /* PL_BUILD_NUM is always defined by the Makefile (auto-incrementing
     .build_counter) — banner always carries it. */
  snprintf(hdr, sizeof(hdr),
           "\n--- Boot Error_Log_Pump_Tsueri_%s --- "
           PL_FW_NAME " " PL_FW_VERSION " build #%d\n",
           date, (int)PL_BUILD_NUM);
  SDFat_Append(&g_log, hdr, (uint32_t)strlen(hdr));

  snprintf(hdr, sizeof(hdr),
           "fw: build %s %s | GPS 1-%uHz | flash~?\n",
           __DATE__, __TIME__, (unsigned)GPS_RATE_HZ);
  SDFat_Append(&g_log, hdr, (uint32_t)strlen(hdr));

  snprintf(hdr, sizeof(hdr), "reset: %s (CSR=0x%08lX)\n",
           reset, (unsigned long)BootResetCsr);
  SDFat_Append(&g_log, hdr, (uint32_t)strlen(hdr));

  SDFat_Flush(&g_log);
  return 0;
}

/* Wall-clock anchor. The box has no RTC, so ERRLOG lines only carry the
   ms-since-boot tick until a host pushes SET_TIME (every connect: iPhone,
   Android, desktop) or the GPS delivers a valid UTC date+time. From then on
   wall = tick + g_wall_off_ms, and every heartbeat carries a readable UTC
   date. Lines written before the anchor are dated via the `boot=` field of
   the `clock:` line (boot = wall at tick 0). */
static uint64_t g_wall_off_ms;     /* epoch_ms - HAL_GetTick() at anchor */
static uint8_t  g_wall_src;        /* 0 = none, 1 = gps, 2 = host        */

/* Howard Hinnant's civil_from_days, unsigned-only (epoch ≥ 1970). */
static void civil_from_days(uint32_t z, unsigned *y, unsigned *m, unsigned *d)
{
  z += 719468u;
  uint32_t era = z / 146097u;
  uint32_t doe = z - era * 146097u;
  uint32_t yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
  uint32_t doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
  uint32_t mp  = (5u * doy + 2u) / 153u;
  *d = (unsigned)(doy - (153u * mp + 2u) / 5u + 1u);
  *m = (unsigned)(mp < 10u ? mp + 3u : mp - 9u);
  *y = (unsigned)(yoe + era * 400u + (*m <= 2u ? 1u : 0u));
}

static uint32_t days_from_civil(unsigned y, unsigned m, unsigned d)
{
  y -= (m <= 2u) ? 1u : 0u;
  uint32_t era = y / 400u;
  uint32_t yoe = y - era * 400u;
  uint32_t doy = (153u * (m > 2u ? m - 3u : m + 9u) + 2u) / 5u + d - 1u;
  uint32_t doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097u + doe - 719468u;
}

/* "YYYY-MM-DD HH:MM:SS" (UTC). */
static void format_epoch(char *out, size_t len, uint64_t epoch_ms)
{
  uint32_t secs = (uint32_t)(epoch_ms / 1000u);
  unsigned y, mo, d;
  civil_from_days(secs / 86400u, &y, &mo, &d);
  uint32_t sod = secs % 86400u;
  snprintf(out, len, "%04u-%02u-%02u %02u:%02u:%02u", y, mo, d,
           (unsigned)(sod / 3600u), (unsigned)((sod / 60u) % 60u),
           (unsigned)(sod % 60u));
}

static void wall_anchor(uint64_t epoch_ms, uint8_t src)
{
  uint32_t tick = HAL_GetTick();
  if (epoch_ms < (uint64_t)tick) return;           /* nonsense epoch */
  g_wall_off_ms = epoch_ms - tick;
  g_wall_src    = src;
  char now_s[24], boot_s[24];
  format_epoch(now_s,  sizeof(now_s),  epoch_ms);
  format_epoch(boot_s, sizeof(boot_s), g_wall_off_ms);
  ErrLog_Writef("clock: %s UTC (src=%s) boot=%s UTC", now_s,
                src == 2 ? "host" : "gps", boot_s);
}

void ErrLog_SetWallClock(uint64_t epoch_ms)
{
  if (!g_inited) return;
  wall_anchor(epoch_ms, 2);
}

void ErrLog_SetWallClockGps(unsigned year, unsigned mon, unsigned day,
                            unsigned hh, unsigned mm, unsigned ss)
{
  /* First GPS anchor per boot only — a host SET_TIME (or an earlier GPS
     anchor) already dated the log; re-anchoring 10x/s would just spam. */
  if (!g_inited || g_wall_src != 0) return;
  if (year < 2024u || mon < 1u || mon > 12u || day < 1u || day > 31u) return;
  uint64_t secs = (uint64_t)days_from_civil(year, mon, day) * 86400u
                + hh * 3600u + mm * 60u + ss;
  wall_anchor(secs * 1000u, 1);
}

int ErrLog_IsLogFile(const char *name)
{
  return g_inited && SDFat_HandleIs(&g_log, name);
}

int ErrLog_Clear(void)
{
  if (!g_inited) return SDFat_Delete(LOG_FILENAME);
  /* v0.0.63: the card belongs to a USB host — every write would fail and
     the reopen below would leave the log dead until reboot. */
  if (SDFat_WritesBlocked()) return PL_FX_ERR_IO;
  g_inited = 0;                          /* no writes while the handle is down */
  SDFat_Close(&g_log);
  int s = SDFat_Delete(LOG_FILENAME);
  if (SDFat_OpenAppend(&g_log, LOG_FILENAME) != PL_FX_OK) return s;
  g_inited = 1;

  char hdr[120];
  snprintf(hdr, sizeof(hdr),
           "\n--- Log cleared (BLE DELETE) --- " PL_FW_NAME " " PL_FW_VERSION
           " build #%d\n", (int)PL_BUILD_NUM);
  SDFat_Append(&g_log, hdr, (uint32_t)strlen(hdr));
  if (g_wall_src) {
    /* Re-state the anchor so the fresh file is dated from its first line. */
    char now_s[24], boot_s[24];
    format_epoch(now_s,  sizeof(now_s),  g_wall_off_ms + HAL_GetTick());
    format_epoch(boot_s, sizeof(boot_s), g_wall_off_ms);
    ErrLog_Writef("clock: %s UTC (src=%s) boot=%s UTC", now_s,
                  g_wall_src == 2 ? "host" : "gps", boot_s);
  }
  SDFat_Flush(&g_log);
  return s;
}

static volatile uint8_t g_warning_latched = 0;

uint8_t ErrLog_WarningLatched(void) { return g_warning_latched; }

/* v0.0.63: lines written while a USB host owns the card. Kept small — a
   host window is minutes of gps_diag at most; overflow is counted, not
   wrapped, so the oldest (most diagnostic) lines survive. */
static char     g_hold[2048];
static uint16_t g_hold_len;
static uint16_t g_hold_dropped;

static void hold_line(const char *line, int n)
{
  if (g_hold_len + n > (int)sizeof(g_hold)) { g_hold_dropped++; return; }
  memcpy(&g_hold[g_hold_len], line, (size_t)n);
  g_hold_len += (uint16_t)n;
}

void ErrLog_ReopenAfterHost(void)
{
  /* Still blocked (host released, battery lock still on, or vice versa):
     keep holding — the other lock's release calls this again. */
  if (!g_inited || SDFat_WritesBlocked()) return;
  /* Discard the old handle without SDFat_Close: its flush would patch a
     directory slot and write a sector at positions the host may have
     reused. The host may also have deleted or truncated the file. */
  memset(&g_log, 0, sizeof(g_log));
  if (SDFat_OpenAppend(&g_log, LOG_FILENAME) != PL_FX_OK) {
    g_inited = 0;                        /* nowhere to write — stay silent */
    return;
  }
  if (g_hold_len) SDFat_Append(&g_log, g_hold, g_hold_len);
  if (g_hold_dropped) {
    char line[96];
    int n = snprintf(line, sizeof(line),
                     "[%lu ms] sd: %u errlog lines dropped while SD writes were blocked\n",
                     (unsigned long)HAL_GetTick(), (unsigned)g_hold_dropped);
    if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
    if (n > 0) SDFat_Append(&g_log, line, (uint32_t)n);
  }
  g_hold_len = 0;
  g_hold_dropped = 0;
  SDFat_Flush(&g_log);
}

void ErrLog_Write(const char *s)
{
  if (!g_inited || !s) return;
  /* Latch the warning flag the first time we see a '***' marker —
     the convention used for prominent errlog lines (GPS not decoding,
     SD almost full, etc.). Main loop reads the latch to drive the
     red LED so the operator notices without needing to pull SD. */
  if (!g_warning_latched && s[0] == '*' && s[1] == '*' && s[2] == '*') {
    g_warning_latched = 1;
  }
  char line[200];
  int n = snprintf(line, sizeof(line), "[%lu ms] %s\n",
                   (unsigned long)HAL_GetTick(), s);
  if (n <= 0) return;
  if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
  if (SDFat_WritesBlocked()) { hold_line(line, n); return; }
  SDFat_Append(&g_log, line, (uint32_t)n);
}

void ErrLog_Writef(const char *fmt, ...)
{
  if (!g_inited) return;
  char body[180];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(body, sizeof(body), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  ErrLog_Write(body);
}

void ErrLog_Flush(void)
{
  if (!g_inited || SDFat_WritesBlocked()) return;
  SDFat_Flush(&g_log);
}

void ErrLog_Heartbeat(void)
{
  if (!g_inited) return;
  uint64_t free_b = SDFat_FreeBytes();
  if (g_wall_src) {
    char now_s[24];
    format_epoch(now_s, sizeof(now_s), g_wall_off_ms + HAL_GetTick());
    ErrLog_Writef("hb: free=%lu MB utc=%s", (unsigned long)(free_b / (1024*1024)),
                  now_s);
  } else {
    ErrLog_Writef("hb: free=%lu MB", (unsigned long)(free_b / (1024*1024)));
  }
  ErrLog_Flush();
}
