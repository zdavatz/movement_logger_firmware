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

static volatile uint8_t g_warning_latched = 0;

uint8_t ErrLog_WarningLatched(void) { return g_warning_latched; }

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
  if (!g_inited) return;
  SDFat_Flush(&g_log);
}

void ErrLog_Heartbeat(void)
{
  if (!g_inited) return;
  uint64_t free_b = SDFat_FreeBytes();
  ErrLog_Writef("hb: free=%lu MB", (unsigned long)(free_b / (1024*1024)));
  ErrLog_Flush();
}
