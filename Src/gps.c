/**
  ******************************************************************************
  * @file    gps.c
  * @brief   u-blox MAX-M10S on UART4 — byte-by-byte IRQ RX, UBX-native.
  *
  *          Since v0.0.41 the module is configured per boot (RAM layer only —
  *          nothing persists on the M10; it always cold-starts at factory
  *          9600 baud NMEA) with Peter's 2026-07-13 recipe: GPS+Galileo only
  *          (BeiDou/GLONASS off), full power, UBX NAV-PVT every nav epoch +
  *          NAV-SAT every 10th, NMEA silenced, nav rate 10 Hz. Since v0.0.42
  *          the baud raise to 230400 is the FIRST command, not the last — at
  *          9600 the module's own factory NMEA already saturates the line, so
  *          anything that adds output before the raise drowns its own ACKs.
  *          The ACK arrives at the new baud, so the local UART is switched
  *          immediately after the command drains. Every un-ACK'd config
  *          command leaves a *** errlog entry. The parser
  *          handles both UBX frames (normal mode) and NMEA lines (fallback
  *          when the box→module TX path is dead and no config could land —
  *          the module then keeps its factory 9600/1 Hz NMEA output).
  *
  *          The DMA approach (Builds #1-#9) proved unreliable: after a single
  *          framing error UART4 latched and DMA stayed stuck. One byte per
  *          IRQ, push to ring buffer, re-arm; error callback clears flags and
  *          re-arms. GPS_Tick drains the ring at main-loop priority and runs
  *          the UBX/NMEA parsing outside IRQ context. Configured UBX traffic
  *          is ~1.4 KB/s (PVT@10Hz + SAT@1Hz) → ~1.4k IRQs/s, each <10 µs
  *          → ~1.5% CPU. Bounded; F-ARCH-6 exception documented in
  *          stm32u5xx_it.c.
  ******************************************************************************
  */
#include "main.h"
#include "gps.h"
#include "sd_fatfs.h"
#include "errlog.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static UART_HandleTypeDef g_huart4;
static uint8_t            g_rx_byte;           /* single-byte landing zone for HAL */

/* GPS_RX_RING_SIZE (config.h, 2048 B) covers a worst-case full-line-rate
   burst at 230400 baud (23 KB/s × 50 ms ≈ 1.15 KB) between two 50 ms
   GPS_Tick drains — ~1.8× margin. */
static volatile uint8_t  g_rx_ring[GPS_RX_RING_SIZE];
static volatile uint16_t g_rx_head;            /* written by IRQ, read by thread */
static volatile uint16_t g_rx_tail;            /* written by thread, read by IRQ */
static volatile uint32_t g_rx_dropped;
/* Wiring report (v0.0.90): every byte the module's TX pad delivered, every
   ACK it sent back to a command, and the RXD pull-up result. */
static volatile uint32_t g_rx_total;
static uint32_t g_wire_acks;
static int8_t   g_wire_pullup = -1;    /* -1 untested, 0 none, 1 present */

#define NMEA_LINE_MAX  96
static char     g_linebuf[NMEA_LINE_MAX];
static uint16_t g_linelen;

static PL_GpsFix g_latest;
static volatile uint8_t g_updated;

/* Diagnostic counters — exposed via GPS_GetStats. Read at Logger_Tick rate
   and dumped to ErrLog every 5 s for bring-up. */
static volatile uint32_t g_diag_bytes;
static volatile uint32_t g_diag_lines_good;
static volatile uint32_t g_diag_lines_bad;
static volatile uint32_t g_diag_rmc;
static volatile uint32_t g_latest_valid_tick;   /* HAL_GetTick() of last valid fix (PVT gnssFixOK / RMC 'A'); 0 = never */
static volatile uint32_t g_diag_gga;
static volatile uint32_t g_diag_errors;        /* UART RX error callbacks invoked */

/* Baud the module ended up on in GPS_Init (230400 normally; the detected
   baud if the switch couldn't be confirmed). Logged by the bridge. */
static uint32_t g_locked_baud = GPS_UART_BAUDRATE;

/* TX diagnostics (v0.0.79). Every byte to the module goes through gps_tx so
   the HAL result is counted instead of silently dropped — "module never
   ACKs" must be split into "MCU never sent" vs "sent but nothing arrived". */
static volatile uint32_t g_tx_ok, g_tx_busy, g_tx_tout, g_tx_bytes;

static void gps_tx(const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef rc = HAL_UART_Transmit(&g_huart4, (uint8_t *)data, len, 200);
  if (rc == HAL_OK)        { g_tx_ok++; g_tx_bytes += len; }
  else if (rc == HAL_BUSY) { g_tx_busy++; }
  else                     { g_tx_tout++; }
}

/* One line with the TX counters, the PA0 pin config (MODER 2 = AF, AFRL 8 =
   AF8/UART4), UART4 enable bits, and the idle PA0 level sampled 64× —
   an idle UART TX line must read high; low means the pin is being pulled
   down externally (short / loaded line), not a firmware problem. */
static void gps_tx_diag(const char *where)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  uint32_t highs = 0;
  for (int i = 0; i < 64; i++) {
    if (GPIOA->IDR & GPIO_PIN_0) highs++;
    for (volatile int d = 0; d < 200; d++) { }
  }
  ErrLog_Writef("gps: txdiag %s ok=%lu busy=%lu tout=%lu bytes=%lu | PA0 moder=%lu af=%lu od=%lu idle_high=%lu/64 | uart4 UE=%d TE=%d RE=%d brr=%lu",
                where, (unsigned long)g_tx_ok, (unsigned long)g_tx_busy,
                (unsigned long)g_tx_tout, (unsigned long)g_tx_bytes,
                (unsigned long)(GPIOA->MODER & 3U), (unsigned long)(GPIOA->AFR[0] & 0xFU),
                (unsigned long)(GPIOA->OTYPER & 1U),
                (unsigned long)highs,
                (UART4->CR1 & USART_CR1_UE) ? 1 : 0, (UART4->CR1 & USART_CR1_TE) ? 1 : 0,
                (UART4->CR1 & USART_CR1_RE) ? 1 : 0, (unsigned long)UART4->BRR);
}

/* Persisted GPS power state (battery-save). GPSPWR.CFG on the SD root: first
   byte 'f'/'F' ("off") = backup mode, anything else = on. g_power is the
   cache: -1 = not read yet, 0 = off (in backup), 1 = on. GPS_Tick early-exits
   while off so no stale NMEA is parsed. */
#define GPSPWR_CFG_NAME "GPSPWR.CFG"
static int g_power = -1;
/* v0.0.67: module held in backup by the low-battery lock (GPS_Suspend),
   independent of the user's persisted g_power choice. */
static int g_suspended = 0;

/* ---------- Adaptive nav rate (v0.0.46, issue #10) ----------------------- */
/* Cold acquisition happens at 1 Hz; only once a valid fix is in hand is the
   module raised to GPS_RATE_HZ, and it drops back to 1 Hz for re-acquisition
   after GPS_REACQ_DOWNSHIFT_MS without a fix. Background: 10-Hz-from-cold
   never produced a single fix in the field (Peter's ERRLOG: 26'583 PVT
   epochs in 44.5 min, rmc=0) while the same module at the factory 1 Hz fixes
   on the same board — acquisition sensitivity at high nav rates is far worse
   than at the 1 Hz the u-blox TTFF specs assume. Tracking at 10 Hz is fine
   (Peter's u-center measurement: 15 sats / 3D fix at 10 Hz).

   The switch VALSET is fire-and-forget from the superloop (polling-only —
   no blocking ACK wait outside GPS_Init); its ACK-ACK is spotted by
   gps_rx_byte and consumed by gps_rate_manage, with bounded resends and a
   backoff so a NAK-ing module can't cause an errlog flood. */
static uint8_t  g_rate_armed;         /* switcher active (UBX path @230400 up) */
static uint8_t  g_rate_fast;          /* module rate now: 0 = 1 Hz, 1 = 10 Hz */
static uint8_t  g_rate_pending;       /* switch VALSET in flight, ACK awaited */
static uint8_t  g_rate_pending_fast;  /* rate the in-flight VALSET asks for */
static uint8_t  g_rate_tries;         /* sends so far for the current switch */
static uint32_t g_rate_sent_tick;     /* HAL_GetTick() of the in-flight send */
static uint32_t g_rate_backoff_until; /* no new switch attempts before this */
static volatile uint8_t g_ack_valset; /* async ACK-ACK for CFG-VALSET seen */

/* ---------- Periodic RF/signal health errlog line (v0.0.52) -------------- */
/* One `gps_rf:` line every GPS_RF_LOG_INTERVAL_MS while the UBX path is up:
   Peter's assembly metrics (2026-07-17) — fixType, numSV used, avg/min/max
   C/N0 of the 6 strongest GPS+Galileo satellites (from the NAV-SAT frames
   already flowing every 10th epoch) — plus the MON-RF EMI set (noisePerMS,
   agcCnt, jamInd, jammingState, antStatus) from a fire-and-forget poll.
   Makes every logged session an EMI record for issue #10 without a live
   survey. Deliberately NEVER a `***` line: the desktop errlog_check grades
   `***` markers as FAIL; RF health is trend data, graded host-side. */
static uint32_t g_rf_next_poll;       /* tick of the next MON-RF poll */
static uint32_t g_rf_poll_tick;       /* tick of the in-flight poll; 0 = none */
static uint8_t  g_rf_fast;            /* 1 = 1 Hz MON-RF polls (quiet window) */
static uint8_t  g_rf_noreply_logged;  /* one-shot no-reply marker per boot */
static uint8_t  g_fix_type;           /* raw NAV-PVT fixType (0/2D/3D scale) */
static uint8_t  g_top6[6];            /* strongest GPS+Galileo C/N0s, desc */
static uint8_t  g_top6_n;
static uint32_t g_top6_tick;          /* HAL_GetTick() of the last NAV-SAT */
/* Cached MON-RF reply for the SensorStream RF extension (v0.0.55): the
   Live tabs show Peter's assembly metrics without the GPS-Debug survey.
   Polls run every GPS_RF_POLL_MS; the errlog line keeps its own slower
   GPS_RF_LOG_INTERVAL_MS cadence. */
static uint8_t  g_rf_jam_state, g_rf_ant, g_rf_jam_ind;
static uint16_t g_rf_noise, g_rf_agc;
static uint32_t g_rf_seen_tick;       /* last MON-RF reply; 0 = never */
static uint32_t g_rf_last_log;        /* last gps_rf errlog line */

/* Mid-session link watchdog (v0.0.56) — see GPS_LINK_* in config.h. */
static uint32_t g_link_last_ok;       /* tick of last checksum-valid frame/line; 0 = not armed yet */
static uint32_t g_link_next_attempt;  /* earliest tick for the next recover; 0 = no backoff */
static uint16_t g_link_recoveries;    /* successful mid-session recovers this boot */

/* ---------- BLE GPS bridge state ---------------------------------------- */
static volatile uint8_t g_bridge;                 /* survey bridge active */

/* Relay ring: complete UBX reply frames captured from the RX stream, drained
   by the BLE layer (GPS_BridgeRead). Sized to hold a couple of the largest
   poll replies (NAV-SIG with many signals ≈ 650 B). */
#define GPS_UBX_RING_SIZE 2048
static uint8_t  g_ubx_ring[GPS_UBX_RING_SIZE];
static uint16_t g_ubx_head;                       /* push (GPS_Tick) */
static uint16_t g_ubx_tail;                       /* pop  (BLE tick) */
static uint32_t g_ubx_dropped;

/* UBX frame-capture state machine, run on the RX stream while bridging.
   Stages the current frame in g_cap, then bulk-pushes 8+len bytes to the
   ring. Mirrors the desktop UbxParser sync hunt; NMEA bytes never trigger
   the 0xB5 sync so they stay out of the relay. */
#define GPS_CAP_MAX 1024
static uint8_t  g_cap[GPS_CAP_MAX];
static uint16_t g_cap_st;
static uint16_t g_cap_len;
static uint16_t g_cap_got;

/* ---------- UBX builder (only used in Init, polling TX) ----------------- */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
  uint8_t hdr[6] = { 0xB5, 0x62, cls, id,
                     (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ck[2] = { 0, 0 };
  for (int i = 2; i < 6; i++) { ck[0] += hdr[i]; ck[1] += ck[0]; }
  for (int i = 0; i < len; i++) { ck[0] += payload[i]; ck[1] += ck[0]; }
  gps_tx(hdr, 6);
  if (len) gps_tx(payload, len);
  gps_tx(ck, 2);
}

/* ---------- UBX-CFG-VALSET: M10-native config (RAM layer only) ----------- */

/* On the MAX-M10S (M10 generation) the legacy $PUBX,41 / UBX-CFG-PRT output
   protocol mask is unreliable — the port keeps emitting NMEA-only even after
   we ask for UBX+NMEA, so the survey's poll requests never get a UBX reply
   (the "no NAV-PVT reply" symptom over the BLE bridge). The M10-native config
   interface (UBX-CFG-VALSET, key/value) does take. We flip the UBX in/out
   protocol on UART1 in the RAM layer only (non-persistent — the module reverts
   to its saved NMEA-only state on the next power cycle, keeping the "never
   reconfigure the receiver persistently" invariant). */
#define CFG_UART1INPROT_UBX   0x10730001UL
#define CFG_UART1OUTPROT_UBX  0x10740001UL

/* CFG-MSGOUT-NMEA_ID_*_UART1 rate keys (U1: emit every Nth nav epoch; 0 =
   off). In normal (UBX-native) operation GPS_Init silences ALL of these —
   the logger's fix comes from UBX NAV-PVT. They only stay at the module's
   factory defaults when the config phase couldn't land (dead box→module TX),
   in which case the NMEA parser below is the fallback data path. */
#define CFG_MSGOUT_NMEA_GGA_UART1  0x209100bbUL
#define CFG_MSGOUT_NMEA_RMC_UART1  0x209100acUL
#define CFG_MSGOUT_NMEA_GSV_UART1  0x209100c5UL
#define CFG_MSGOUT_NMEA_GSA_UART1  0x209100c0UL
#define CFG_MSGOUT_NMEA_VTG_UART1  0x209100b1UL
#define CFG_MSGOUT_NMEA_GLL_UART1  0x209100caUL
#define CFG_MSGOUT_NMEA_ZDA_UART1  0x209100d9UL

/* CFG-MSGOUT-UBX_NAV_*_UART1 — the v0.0.41 primary data path. PVT every nav
   epoch (position/speed/course/time/fix/pDOP in one 92-byte frame), NAV-SAT
   every 10th epoch (per-satellite C/N0 → cn0_max / sats_in_view). */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007UL
#define CFG_MSGOUT_UBX_NAV_SAT_UART1  0x20910016UL

/* CFG-SIGNAL constellation enables. Peter's recipe (2026-07-13): GPS + Galileo
   only; BeiDou + GLONASS off. Measured result: 15 sats used / 3D fix at a
   window with ~40 % sky view — where the 4-constellation default never fixed. */
#define CFG_SIGNAL_GPS_ENA        0x1031001fUL
#define CFG_SIGNAL_GPS_L1CA_ENA   0x10310001UL
#define CFG_SIGNAL_GAL_ENA        0x10310021UL
#define CFG_SIGNAL_GAL_E1_ENA     0x10310007UL
#define CFG_SIGNAL_BDS_ENA        0x10310022UL
#define CFG_SIGNAL_BDS_B1_ENA     0x1031000dUL
#define CFG_SIGNAL_BDS_B1C_ENA    0x1031000fUL
#define CFG_SIGNAL_GLO_ENA        0x10310025UL
#define CFG_SIGNAL_GLO_L1_ENA     0x10310018UL

/* Nav rate, power mode, port baud. */
#define CFG_RATE_MEAS             0x30210001UL   /* U2, ms per nav epoch */
#define CFG_RATE_NAV              0x30210002UL   /* U2, epochs per solution */
#define CFG_PM_OPERATEMODE        0x20d00001UL   /* E1, 0 = full power */
#define CFG_UART1_BAUDRATE        0x40520001UL   /* U4 */

static int ubx_send_retry(uint8_t cls, uint8_t id, const uint8_t *payload,
                          uint16_t len, int retries);

/* Set one 1-byte config key (L-type bool or U1 rate) in the RAM layer,
   ACK-verified. The value is written raw: 0/1 for the protocol-enable bools,
   an epoch-divider for the CFG-MSGOUT rate keys. Returns 0 on ACK, -1 on
   timeout/NAK after retries. */
static int gps_cfg_valset_bool(uint32_t key, uint8_t val)
{
  uint8_t p[9];
  p[0] = 0x00;                    /* version 0 */
  p[1] = 0x01;                    /* layers: RAM only (non-persistent) */
  p[2] = 0x00; p[3] = 0x00;       /* reserved */
  p[4] = (uint8_t)(key);
  p[5] = (uint8_t)(key >> 8);
  p[6] = (uint8_t)(key >> 16);
  p[7] = (uint8_t)(key >> 24);
  p[8] = val;
  return ubx_send_retry(0x06, 0x8A, p, sizeof(p), 2);
}

/* Multi-key CFG-VALSET builder. vs_begin resets the staging buffer (version 0,
   RAM layer), vs_u1/u2/u4 append key+value pairs, vs_send transmits with
   3 ACK-verified retries and logs the outcome — a *** entry for every un-ACK'd
   command (Peter's rule, 2026-07-13: "Jeder Befehl an das GPS Modul, der nicht
   bestätigt wird, muss im Errorlog einen Eintrag machen"). Returns 0 on ACK,
   1 on failure so callers can sum a failure count. */
#define VS_BUF_MAX (4 + 9 * 8)              /* header + 9 keys à (4+4) worst case */
static uint8_t  g_vs_buf[VS_BUF_MAX];
static uint16_t g_vs_len;

static void vs_begin(void)
{
  g_vs_buf[0] = 0x00;                       /* version 0 */
  g_vs_buf[1] = 0x01;                       /* layers: RAM only */
  g_vs_buf[2] = 0x00;
  g_vs_buf[3] = 0x00;
  g_vs_len    = 4;
}

static void vs_key(uint32_t key)
{
  g_vs_buf[g_vs_len++] = (uint8_t)(key);
  g_vs_buf[g_vs_len++] = (uint8_t)(key >> 8);
  g_vs_buf[g_vs_len++] = (uint8_t)(key >> 16);
  g_vs_buf[g_vs_len++] = (uint8_t)(key >> 24);
}

static void vs_u1(uint32_t key, uint8_t v)
{
  if (g_vs_len + 5 > VS_BUF_MAX) return;
  vs_key(key);
  g_vs_buf[g_vs_len++] = v;
}

static void vs_u2(uint32_t key, uint16_t v)
{
  if (g_vs_len + 6 > VS_BUF_MAX) return;
  vs_key(key);
  g_vs_buf[g_vs_len++] = (uint8_t)(v);
  g_vs_buf[g_vs_len++] = (uint8_t)(v >> 8);
}

static void vs_u4(uint32_t key, uint32_t v)
{
  if (g_vs_len + 8 > VS_BUF_MAX) return;
  vs_key(key);
  g_vs_buf[g_vs_len++] = (uint8_t)(v);
  g_vs_buf[g_vs_len++] = (uint8_t)(v >> 8);
  g_vs_buf[g_vs_len++] = (uint8_t)(v >> 16);
  g_vs_buf[g_vs_len++] = (uint8_t)(v >> 24);
}

static int vs_send(const char *what)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  int rc = ubx_send_retry(0x06, 0x8A, g_vs_buf, g_vs_len, 3);
  if (rc == 0) {
    ErrLog_Writef("gps: cfg %s ACK", what);
    return 0;
  }
  ErrLog_Writef("*** gps: cfg %s NOT ACK'd ***", what);
  return 1;
}

/* Stamp a checksum-valid frame/line for the mid-session link watchdog, and
   log the outage length when a > GPS_LINK_GAP_LOG_MS gap self-heals before
   the watchdog fires (Peter, 2026-07-20: antenna data "manchmal sichtbar,
   manchmal nicht" — these lines are the forensic trace of a flaky UART/
   supply joint). Called from both protocol paths' good-frame sites only;
   garbage bytes never refresh the stamp. */
static void link_mark_ok(void)
{
  uint32_t now = HAL_GetTick();
  if (g_link_last_ok != 0 && (now - g_link_last_ok) > GPS_LINK_GAP_LOG_MS) {
    extern void ErrLog_Writef(const char *fmt, ...);
    ErrLog_Writef("gps: link gap %lu ms (self-healed)",
                  (unsigned long)(now - g_link_last_ok));
  }
  g_link_last_ok = now;
}

/* ---------- NMEA parser ------------------------------------------------- */

static uint8_t nmea_checksum_ok(const char *line)
{
  if (line[0] != '$') return 0;
  uint8_t sum = 0;
  const char *p = line + 1;
  while (*p && *p != '*') { sum ^= (uint8_t)*p; p++; }
  if (*p != '*') return 0;
  uint8_t expected = (uint8_t)strtoul(p + 1, NULL, 16);
  return (sum == expected);
}

static double nmea_latlon(const char *val, char hem)
{
  if (!val || !*val) return 0.0;
  double raw = strtod(val, NULL);
  double deg = (int)(raw / 100.0);
  double min = raw - deg * 100.0;
  double dec = deg + min / 60.0;
  if (hem == 'S' || hem == 'W') dec = -dec;
  return dec;
}

static int nmea_split(char *line, char *fields[], int max_fields)
{
  int n = 0;
  char *p = line;
  if (*p != '$') return 0;
  p++;
  fields[n++] = p;
  while (*p && *p != '*' && n < max_fields) {
    if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
    p++;
  }
  if (*p == '*') *p = '\0';
  return n;
}

/* Per-satellite C/N0 roll-up, fed by UBX NAV-SAT (normal mode, 1 frame per
   burst) or NMEA GSV (fallback mode, multi-sentence burst). Either source
   fills the accumulators + burst flag; the epoch parser (parse_nav_pvt /
   parse_gga) then commits them to g_latest. Bursts arrive ~1/s while epochs
   run at 10 Hz, so most epochs see no fresh burst — the commit holds the last
   value between bursts and only decays to 0 ("no data") after GSV_STALE_MS of
   silence (source disabled, module gone quiet). GSV parsing is RX-only — it
   works even when the box→module command line is dead. */
static uint8_t  g_gsv_cn0_max;    /* strongest C/N0 since the last commit (dB-Hz) */
static uint8_t  g_gsv_nsat_sig;   /* satellites with a C/N0 since the last commit */
static uint8_t  g_gsv_burst;      /* ≥1 burst parsed since the last commit */
static uint32_t g_gsv_seen_tick;  /* HAL_GetTick() of the last burst */
#define GSV_STALE_MS 3000U

/* Commit the roll-up at an epoch boundary (NAV-PVT or GGA). A burst whose
   satellites all lack a C/N0 commits a genuine 0/0. */
static void commit_cn0_rollup(void)
{
  if (g_gsv_burst) {
    g_latest.cn0_max      = g_gsv_cn0_max;
    g_latest.sats_in_view = g_gsv_nsat_sig;
    g_gsv_cn0_max  = 0;
    g_gsv_nsat_sig = 0;
    g_gsv_burst    = 0;
  } else if ((uint32_t)(HAL_GetTick() - g_gsv_seen_tick) > GSV_STALE_MS) {
    g_latest.cn0_max      = 0;
    g_latest.sats_in_view = 0;
  }
}

/* $xxGSV,numMsg,msgNum,numSV,{svid,elev,azim,cno}...  (up to 4 sats/sentence;
   an optional trailing signalId in NMEA 4.11 is naturally ignored — a lone
   field never completes a 4-tuple). Every talker (GP/GL/GA/GB/GQ) maps to type
   "GSV" via nmea_handle_line's tag+2, so one branch covers all constellations.
   Empty cno = satellite in view but not tracked. */
static void parse_gsv(char *fields[], int n)
{
  if (n < 4) return;
  g_gsv_burst     = 1;
  g_gsv_seen_tick = HAL_GetTick();
  for (int i = 4; i + 3 < n; i += 4) {
    const char *cno_s = fields[i + 3];
    if (!cno_s || !*cno_s) continue;         /* satellite not tracked */
    int cno = atoi(cno_s);
    if (cno <= 0) continue;
    if (cno > 99) cno = 99;
    if (g_gsv_nsat_sig < 255) g_gsv_nsat_sig++;
    if ((uint8_t)cno > g_gsv_cn0_max) g_gsv_cn0_max = (uint8_t)cno;
  }
}

static void parse_rmc(char *fields[], int n)
{
  if (n < 10) return;
  const char *utc    = fields[1];
  const char *status = fields[2];
  const char *lat    = fields[3];
  const char *ns     = fields[4];
  const char *lon    = fields[5];
  const char *ew     = fields[6];
  const char *spd    = fields[7];
  const char *cog    = fields[8];
  if (status[0] != 'A') return;
  g_diag_rmc++;
  /* Wall-clock stamp of the latest valid RMC. GPS_LastFixQuality uses
     this to detect signal loss mid-session — if no fresh-valid RMC has
     arrived in ~3 s, the LED pattern falls back to "no fix" instead of
     latching the previous good state forever. */
  g_latest_valid_tick = HAL_GetTick();

  strncpy(g_latest.utc, utc, sizeof(g_latest.utc) - 1);
  g_latest.utc[sizeof(g_latest.utc) - 1] = '\0';
  g_latest.lat       = nmea_latlon(lat, ns[0]);
  g_latest.lon       = nmea_latlon(lon, ew[0]);
  g_latest.speed_kmh = (float)(strtod(spd, NULL) * 1.852);
  g_latest.course    = (float)strtod(cog, NULL);
  g_latest.tick_ms   = HAL_GetTick();
  g_latest.valid     = 1;
  g_updated          = 1;
}

static void parse_gga(char *fields[], int n)
{
  if (n < 10) return;
  g_diag_gga++;
  commit_cn0_rollup();
  g_latest.fix_q   = (uint8_t)atoi(fields[6]);
  g_latest.num_sat = (uint8_t)atoi(fields[7]);
  g_latest.hdop    = (float)strtod(fields[8], NULL);
  g_latest.alt_m   = (float)strtod(fields[9], NULL);
  g_updated        = 1;
}

static void nmea_handle_line(char *line)
{
  if (!nmea_checksum_ok(line)) {
    g_diag_lines_bad++;
    return;
  }
  g_diag_lines_good++;
  link_mark_ok();
  char *fields[24];
  int n = nmea_split(line, fields, 24);
  if (n < 1) return;
  const char *tag = fields[0];
  if (strlen(tag) < 5) return;
  const char *type = tag + 2;
  if      (strncmp(type, "RMC", 3) == 0) parse_rmc(fields, n);
  else if (strncmp(type, "GGA", 3) == 0) parse_gga(fields, n);
  else if (strncmp(type, "GSV", 3) == 0) parse_gsv(fields, n);
}

static void process_byte(uint8_t b)
{
  if (b == '\n' || b == '\r') {
    if (g_linelen > 0 && g_linelen < NMEA_LINE_MAX) {
      g_linebuf[g_linelen] = '\0';
      nmea_handle_line(g_linebuf);
    }
    g_linelen = 0;
    return;
  }
  if (b == '$') g_linelen = 0;
  if (g_linelen < NMEA_LINE_MAX - 1) {
    g_linebuf[g_linelen++] = (char)b;
  } else {
    g_linelen = 0;
  }
}

/* ---------- UBX parser (NAV-PVT / NAV-SAT — the v0.0.41 primary path) ---- */

static uint32_t g_diag_ubx_frames;   /* checksum-valid UBX frames (any class) */

static int32_t rd_i32(const uint8_t *p)
{
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static uint16_t rd_u16(const uint8_t *p)
{
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/* UBX-NAV-PVT (0x01 0x07, 92 B): the one-frame-per-epoch solution. Fills the
   same PL_GpsFix the NMEA RMC+GGA pair used to fill:
     fix_q   — 1 on a valid fix (flags.gnssFixOK && fixType 2/3/4), else 0.
               Deliberately mapped to the NMEA-GGA "1 = GPS fix" scale every
               host/CSV consumer was built for, not the raw fixType.
     hdop    — carries pDOP×0.01 (PVT has no hDOP; scale + meaning are close
               enough for the quality gating the tools do — see DESIGN.md).
     utc     — hhmmss.ss from the PVT UTC fields (only when validDate+validTime).
   Epochs count into g_diag_gga, valid fixes into g_diag_rmc, so the gps_diag
   errlog line and the desktop's errlog_check keep their exact 8-key format. */
static void parse_nav_pvt(const uint8_t *p, uint16_t len)
{
  if (len < 92) return;
  g_diag_gga++;
  commit_cn0_rollup();

  uint8_t  fixtype = p[20];
  uint8_t  flags   = p[21];
  uint8_t  ok      = (flags & 0x01) && fixtype >= 2 && fixtype <= 4;
  g_fix_type       = fixtype;                    /* raw scale for gps_rf line */

  g_latest.fix_q   = ok ? 1 : 0;
  g_latest.num_sat = p[23];
  g_latest.hdop    = (float)rd_u16(p + 76) * 0.01f;      /* pDOP */
  g_latest.alt_m   = (float)rd_i32(p + 36) / 1000.0f;    /* hMSL mm → m */

  if (ok) {
    g_diag_rmc++;
    g_latest_valid_tick = HAL_GetTick();
    g_latest.lat = (double)rd_i32(p + 28) * 1e-7;
    g_latest.lon = (double)rd_i32(p + 24) * 1e-7;
    g_latest.speed_kmh = (float)rd_i32(p + 60) * 0.0036f;  /* mm/s → km/h */
    float crs = (float)rd_i32(p + 64) * 1e-5f;             /* headMot */
    if (crs < 0.0f) crs += 360.0f;
    g_latest.course = crs;
    if ((p[11] & 0x03) == 0x03) {                          /* validDate+Time */
      int32_t nano = rd_i32(p + 16);
      int cs = (nano > 0) ? (int)(nano / 10000000) : 0;    /* clamp neg nano */
      snprintf(g_latest.utc, sizeof(g_latest.utc), "%02u%02u%02u.%02u",
               (unsigned)p[8], (unsigned)p[9], (unsigned)p[10], (unsigned)cs);
      ErrLog_SetWallClockGps(rd_u16(p + 4), p[6], p[7], p[8], p[9], p[10]);
    }
    g_latest.tick_ms = HAL_GetTick();
    g_latest.valid   = 1;
  }
  g_updated = 1;
}

/* UBX-NAV-SAT (0x01 0x35): per-satellite C/N0 table, one frame per burst
   (every 10th nav epoch). Overwrites the roll-up accumulators — unlike the
   multi-sentence GSV path there is nothing to accumulate across. */
static void parse_nav_sat(const uint8_t *p, uint16_t len)
{
  if (len < 8) return;
  uint16_t n = p[5];
  if ((uint16_t)(8 + 12 * n) > len) n = (uint16_t)((len - 8) / 12);
  uint8_t cn0max = 0, nsig = 0;
  uint8_t t6[6] = {0}, t6n = 0;              /* strongest GPS+GAL, descending */
  for (uint16_t i = 0; i < n; i++) {
    uint8_t gnss = p[8 + 12 * i + 0];
    uint8_t cno  = p[8 + 12 * i + 2];
    if (cno == 0) continue;                  /* in view but not tracked */
    if (cno > 99) cno = 99;
    if (nsig < 255) nsig++;
    if (cno > cn0max) cn0max = cno;
    /* Top-6 insert for the gps_rf line — Peter's constellation filter:
       GPS (gnssId 0) + Galileo (2) only. */
    if (gnss == 0 || gnss == 2) {
      for (uint8_t k = 0; k < 6; k++) {
        if (k >= t6n || cno > t6[k]) {
          for (uint8_t m = (t6n < 6) ? t6n : 5; m > k; m--) t6[m] = t6[m - 1];
          t6[k] = cno;
          if (t6n < 6) t6n++;
          break;
        }
      }
    }
  }
  memcpy((void *)g_top6, t6, sizeof(t6));
  g_top6_n        = t6n;
  g_top6_tick     = HAL_GetTick();
  g_gsv_cn0_max   = cn0max;
  g_gsv_nsat_sig  = nsig;
  g_gsv_burst     = 1;
  g_gsv_seen_tick = HAL_GetTick();
}

/* UBX fixType rendered the way Peter reads it (0/2D/3D…), not a raw enum.
   Mirrors the desktop gps-debug `fix_type_name`. */
static const char *fix_name(uint8_t t)
{
  switch (t) {
    case 0:  return "0";
    case 1:  return "DR";
    case 2:  return "2D";
    case 3:  return "3D";
    case 4:  return "3D+DR";
    case 5:  return "TIME";
    default: return "?";
  }
}

/* UBX-MON-RF (0x0A 0x38): RF block 0 → the periodic `gps_rf:` errlog line.
   Only consumed while OUR poll is in flight — during a BLE survey the host
   polls MON-RF ~1/s and logging every relayed reply would flood the log. */
static void parse_mon_rf(const uint8_t *p, uint16_t len)
{
  if (g_rf_poll_tick == 0) return;
  if (len < 4 + 24 || p[1] < 1) return;      /* header + ≥1 RF block */
  g_rf_poll_tick = 0;

  uint8_t  jam_state = p[4 + 1] & 0x03;
  uint8_t  ant_stat  = p[4 + 2];
  uint16_t noise     = rd_u16(p + 4 + 12);
  uint16_t agc       = rd_u16(p + 4 + 14);
  uint8_t  jam_ind   = p[4 + 16];

  /* Cache for the SensorStream RF extension (read by GPS_GetRfLive). */
  g_rf_jam_state = jam_state;
  g_rf_ant       = ant_stat;
  g_rf_jam_ind   = jam_ind;
  g_rf_noise     = noise;
  g_rf_agc       = agc;
  g_rf_seen_tick = HAL_GetTick();

  /* The errlog line keeps its slower cadence even though polls now run
     every GPS_RF_POLL_MS for the live stream. */
  if ((HAL_GetTick() - g_rf_last_log) < GPS_RF_LOG_INTERVAL_MS) return;
  g_rf_last_log = HAL_GetTick();

  /* Top-6 roll-up; zeroed when the last NAV-SAT is stale (> 15 s: signal
     lost or module quiet — don't report a dead sky view as healthy). */
  unsigned avg10 = 0, mn = 0, mx = 0, tn = g_top6_n;
  if (tn > 0 && (HAL_GetTick() - g_top6_tick) <= 15000U) {
    unsigned sum = 0;
    for (unsigned k = 0; k < tn; k++) sum += g_top6[k];
    avg10 = sum * 10U / tn;
    mx = g_top6[0];
    mn = g_top6[tn - 1];
  }
  static const char *js[4] = { "unk", "ok", "warn", "crit" };
  static const char *as[5] = { "init", "?", "ok", "SHORT", "OPEN" };
  ErrLog_Writef("gps_rf: fix=%s used=%u avg6=%u.%u min6=%u max6=%u "
                "noise=%u agc=%u jam=%u state=%s ant=%s",
                fix_name(g_fix_type), (unsigned)g_latest.num_sat,
                avg10 / 10U, avg10 % 10U, mn, mx,
                (unsigned)noise, (unsigned)agc, (unsigned)jam_ind,
                js[jam_state], (ant_stat <= 4) ? as[ant_stat] : "?");
}

/* Protocol router: assembles UBX frames (sync 0xB5 0x62, Fletcher checksum)
   out of the RX stream and hands everything else to the NMEA line assembler.
   Oversized frames (> UBX_PAY_MAX, e.g. a long NAV-SIG during a survey) are
   checksum-validated but not stored. Checksum-valid frames count into
   g_diag_lines_good, corrupt ones into g_diag_lines_bad — so the existing
   gps_diag line and the logger's 30 s "no data" marker keep working with
   unchanged semantics ("units decoded / units corrupt"). */
#define UBX_PAY_MAX 600
static uint8_t  g_ux_st;
static uint8_t  g_ux_cls, g_ux_id, g_ux_cka, g_ux_ckb, g_ux_skip;
static uint16_t g_ux_len, g_ux_got;
static uint8_t  g_ux_pay[UBX_PAY_MAX];

static void ux_ck(uint8_t b) { g_ux_cka += b; g_ux_ckb += g_ux_cka; }

static void gps_rx_byte(uint8_t b)
{
  switch (g_ux_st) {
    case 0:
      if (b == 0xB5) { g_ux_st = 1; return; }
      process_byte(b);
      return;
    case 1:
      if (b == 0x62) { g_ux_st = 2; g_ux_cka = 0; g_ux_ckb = 0; return; }
      g_ux_st = (b == 0xB5) ? 1 : 0;         /* repeated 0xB5 stays armed */
      if (g_ux_st == 0) process_byte(b);
      return;
    case 2: g_ux_cls = b; ux_ck(b); g_ux_st = 3; return;
    case 3: g_ux_id  = b; ux_ck(b); g_ux_st = 4; return;
    case 4: g_ux_len = b; ux_ck(b); g_ux_st = 5; return;
    case 5:
      g_ux_len |= (uint16_t)b << 8; ux_ck(b);
      g_ux_got  = 0;
      g_ux_skip = (g_ux_len > UBX_PAY_MAX) ? 1 : 0;
      g_ux_st   = g_ux_len ? 6 : 7;
      return;
    case 6:
      if (!g_ux_skip) g_ux_pay[g_ux_got] = b;
      ux_ck(b);
      if (++g_ux_got >= g_ux_len) g_ux_st = 7;
      return;
    case 7:
      if (b == g_ux_cka) { g_ux_st = 8; return; }
      g_diag_lines_bad++;
      g_ux_st = 0;
      return;
    case 8:
      if (b == g_ux_ckb) {
        g_diag_lines_good++;
        g_diag_ubx_frames++;
        link_mark_ok();
        if (!g_ux_skip) {
          if      (g_ux_cls == 0x01 && g_ux_id == 0x07) parse_nav_pvt(g_ux_pay, g_ux_len);
          else if (g_ux_cls == 0x01 && g_ux_id == 0x35) parse_nav_sat(g_ux_pay, g_ux_len);
          else if (g_ux_cls == 0x0A && g_ux_id == 0x38) parse_mon_rf(g_ux_pay, g_ux_len);
          /* ACK-ACK for a CFG-VALSET: the adaptive-nav-rate switch is sent
             fire-and-forget from the superloop, so its confirmation arrives
             here instead of through the blocking init-time ubx_wait_ack. */
          else if (g_ux_cls == 0x05 && g_ux_id == 0x01 && g_ux_len >= 2 &&
                   g_ux_pay[0] == 0x06 && g_ux_pay[1] == 0x8A)
            g_ack_valset = 1;
        }
      } else {
        g_diag_lines_bad++;
      }
      g_ux_st = 0;
      return;
    default:
      g_ux_st = 0;
      return;
  }
}

/* ---------- BLE GPS bridge ---------------------------------------------- */

/* Push a complete captured UBX frame into the relay ring. Whole-frame or
   nothing: if the ring can't hold it, drop it (the host re-polls next
   second) rather than relay a truncated frame the host would reject. */
static void bridge_push(const uint8_t *buf, uint16_t n)
{
  uint16_t free_sp = (uint16_t)((g_ubx_tail - g_ubx_head - 1u) % GPS_UBX_RING_SIZE);
  if (n > free_sp) { g_ubx_dropped++; return; }
  for (uint16_t i = 0; i < n; i++) {
    g_ubx_ring[g_ubx_head] = buf[i];
    g_ubx_head = (uint16_t)((g_ubx_head + 1u) % GPS_UBX_RING_SIZE);
  }
}

/* Feed one RX byte to the UBX frame extractor while bridging. On a complete
   frame, stage 8+len bytes (sync..checksum) and hand them to bridge_push.
   The host re-verifies the checksum, so we relay the frame as-is. */
static void bridge_capture(uint8_t b)
{
  switch (g_cap_st) {
    case 0:
      if (b == 0xB5) { g_cap[0] = 0xB5; g_cap_st = 1; }
      break;
    case 1:
      if (b == 0x62)      { g_cap[1] = 0x62; g_cap_st = 2; }
      else if (b != 0xB5) { g_cap_st = 0; }   /* repeated 0xB5 stays armed */
      break;
    case 2: g_cap[2] = b; g_cap_st = 3; break;                 /* class */
    case 3: g_cap[3] = b; g_cap_st = 4; break;                 /* id */
    case 4: g_cap[4] = b; g_cap_len = b; g_cap_st = 5; break;  /* len lo */
    case 5: g_cap[5] = b; g_cap_len |= (uint16_t)b << 8;       /* len hi */
            g_cap_got = 0;
            if (g_cap_len > (uint16_t)(GPS_CAP_MAX - 8)) g_cap_st = 0;
            else g_cap_st = g_cap_len ? 6 : 7;
            break;
    case 6: g_cap[6 + g_cap_got] = b;
            if (++g_cap_got >= g_cap_len) g_cap_st = 7;
            break;
    case 7: g_cap[6 + g_cap_len] = b; g_cap_st = 8; break;     /* ck_a */
    case 8: g_cap[6 + g_cap_len + 1] = b;                      /* ck_b */
            bridge_push(g_cap, (uint16_t)(8 + g_cap_len));
            g_cap_st = 0;
            break;
    default: g_cap_st = 0; break;
  }
}

void GPS_BridgeSet(uint8_t on)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  if (on) {
    /* Fresh capture + relay state for this survey. */
    g_cap_st = 0; g_cap_len = 0; g_cap_got = 0;
    g_ubx_head = g_ubx_tail = 0; g_ubx_dropped = 0;
    g_bridge = 1;
    /* Since v0.0.41 the port already runs UBX-native (in+out enabled, NMEA
       silenced by GPS_Init), so there is no protocol to flip. Just throttle
       our own periodic NAV-PVT to every 10th epoch so the survey's poll
       replies don't compete with a 10 Hz stream on the BLE relay ring —
       the logger still gets ~1 fix/s while the survey runs. */
    int rc = gps_cfg_valset_bool(CFG_MSGOUT_UBX_NAV_PVT_UART1, 10);
    ErrLog_Writef("gps: bridge ON @%lu baud (pvt throttle %s)",
                  (unsigned long)g_locked_baud, (rc == 0) ? "ACK" : "FAIL");
    gps_tx_diag("bridge-on");
  } else {
    g_bridge = 0;
    /* Restore the 10 Hz PVT stream. RAM-only, best-effort — a power cycle
       would clear it anyway. */
    int rc = gps_cfg_valset_bool(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
    ErrLog_Writef("gps: bridge off @%lu baud (pvt restore %s)",
                  (unsigned long)g_locked_baud, (rc == 0) ? "ACK" : "FAIL");
    gps_tx_diag("bridge-off");
  }
}

uint8_t GPS_BridgeActive(void) { return g_bridge; }

void GPS_BridgeTx(const uint8_t *data, uint16_t len)
{
  if (!data || !len) return;
  gps_tx(data, len);
}

uint16_t GPS_BridgeRead(uint8_t *out, uint16_t max)
{
  uint16_t n = 0;
  while (n < max && g_ubx_tail != g_ubx_head) {
    out[n++] = g_ubx_ring[g_ubx_tail];
    g_ubx_tail = (uint16_t)((g_ubx_tail + 1u) % GPS_UBX_RING_SIZE);
  }
  return n;
}

/* ---------- HW init ----------------------------------------------------- */

static int uart4_init_at(uint32_t baud)
{
  memset(&g_huart4, 0, sizeof(g_huart4));
  g_huart4.Instance        = UART4;
  g_huart4.Init.BaudRate   = baud;
  g_huart4.Init.WordLength = UART_WORDLENGTH_8B;
  g_huart4.Init.StopBits   = UART_STOPBITS_1;
  g_huart4.Init.Parity     = UART_PARITY_NONE;
  g_huart4.Init.Mode       = UART_MODE_TX_RX;
  g_huart4.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  g_huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  g_huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  g_huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  g_huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  return (HAL_UART_Init(&g_huart4) == HAL_OK) ? 0 : -1;
}

/* Re-open UART4 at `baud`: tear down whatever is configured, re-init, disable
   the overrun error (we drain a ring by polling — a dropped byte must not park
   the peripheral in an error state), clear stale flags, and empty the ring.
   Every baud transition in GPS_Init goes through here. */
static int gps_uart_reopen(uint32_t baud)
{
  if (g_huart4.Instance != NULL) HAL_UART_DeInit(&g_huart4);
  if (uart4_init_at(baud) != 0) return -1;
  g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
  __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                 | UART_CLEAR_FEF  | UART_CLEAR_PEF);
  g_rx_head = g_rx_tail = 0;
  return 0;
}

/* Listen for `ms_window` milliseconds and count the NMEA newlines ('\n') and
   UBX frame syncs (0xB5 0x62 pairs) that land in the ring. Used by GPS_Init
   to verify that the currently-configured UART baud matches the module's
   output — a cold-booted module streams factory NMEA at 9600, a module that
   survived an MCU-only reset streams this session's UBX at 230400; either
   traffic pattern confirms the framing. Returns the combined count; the
   per-protocol counts land in *nl / *ubx when non-NULL. Re-arms the RX IRQ
   each entry so calling this right after a baud-rate change works. */
static int listen_traffic(uint32_t ms_window, int *nl_out, int *ubx_out)
{
  /* Robust re-arm: after multiple DeInit/Init cycles the HAL's RxState
     can land in BUSY_RX or RESET leftovers, and a fresh HAL_UART_Receive_IT
     then returns HAL_BUSY silently. Abort first, then arm, then check.
     If arming truly fails we log it — otherwise "silence" gets
     mis-attributed to "wrong baud" when the real reason was a stuck
     UART driver. */
  extern void ErrLog_Write(const char *msg);
  if (nl_out)  *nl_out  = 0;
  if (ubx_out) *ubx_out = 0;
  HAL_UART_AbortReceive_IT(&g_huart4);
  g_rx_head = g_rx_tail = 0;
  HAL_StatusTypeDef rs = HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1);
  if (rs != HAL_OK) {
    ErrLog_Write("gps: listen: HAL_UART_Receive_IT FAIL");
    return 0;
  }

  uint32_t newlines = 0, syncs = 0;
  uint8_t  prev = 0;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms_window) {
    while (g_rx_head != g_rx_tail) {
      uint8_t b = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
      if (b == '\n') newlines++;
      if (prev == 0xB5 && b == 0x62) syncs++;
      prev = b;
    }
  }
  if (nl_out)  *nl_out  = (int)newlines;
  if (ubx_out) *ubx_out = (int)syncs;
  return (int)(newlines + syncs);
}

/* Watch the ring buffer for a UBX-ACK-ACK (or ACK-NAK) frame matching the
   given class+id. Returns 0 on ACK, -2 on NAK, -1 on timeout. Scans
   bytewise via a small state machine — NMEA bytes (which start with '$')
   never trigger the 0xB5 sync, so non-UBX traffic flows through harmless.
   Per UBX spec the ACK frame is fixed length:
     B5 62 05 01 02 00 <cls> <id> <ck0> <ck1>   (ACK-ACK)
     B5 62 05 00 02 00 <cls> <id> <ck0> <ck1>   (ACK-NAK)
   The checksum bytes are validated implicitly by message length. */
static int ubx_wait_ack(uint8_t cls, uint8_t id, uint32_t timeout_ms)
{
  enum { S_B5, S_62, S_C, S_I, S_L0, S_L1, S_RCLS, S_RID } s = S_B5;
  uint8_t  rcls = 0;
  uint8_t  ack_kind = 0;                 /* 1 = ACK, 0 = NAK */
  uint32_t t0 = HAL_GetTick();
  HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1);

  while ((HAL_GetTick() - t0) < timeout_ms) {
    while (g_rx_head != g_rx_tail) {
      uint8_t b = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
      switch (s) {
        case S_B5:  s = (b == 0xB5) ? S_62 : S_B5; break;
        case S_62:  s = (b == 0x62) ? S_C  : S_B5; break;
        case S_C:   s = (b == 0x05) ? S_I  : S_B5; break;
        case S_I:
          if      (b == 0x01) { ack_kind = 1; s = S_L0; }
          else if (b == 0x00) { ack_kind = 0; s = S_L0; }
          else                  s = S_B5;
          break;
        case S_L0:  s = (b == 0x02) ? S_L1   : S_B5; break;
        case S_L1:  s = (b == 0x00) ? S_RCLS : S_B5; break;
        case S_RCLS: rcls = b; s = S_RID; break;
        case S_RID:
          if (rcls == cls && b == id) { g_wire_acks++; return ack_kind ? 0 : -2; }
          s = S_B5;
          break;
      }
    }
  }
  return -1;
}

/* Send a UBX command and wait for its ACK-ACK. Retries up to `retries`
   times on timeout or NAK. Returns 0 on success, -1 if all retries
   failed. Each attempt has a 300 ms ACK budget — plenty even at 9600
   baud (ACK is 10 bytes = ~11 ms airtime). */
static int ubx_send_retry(uint8_t cls, uint8_t id, const uint8_t *payload,
                          uint16_t len, int retries)
{
  for (int i = 0; i < retries; i++) {
    ubx_send(cls, id, payload, len);
    int rc = ubx_wait_ack(cls, id, 300);
    if (rc == 0) return 0;
    HAL_Delay(50);                       /* let any garbage drain */
  }
  return -1;
}

/* ---------- TX-path test: NMEA poll + $PUBX,41 (v0.0.71) ---------------- */

/* Send "$<body>*CS\r\n" — NMEA XOR checksum over everything between $ and *. */
static void nmea_send(const char *body)
{
  uint8_t cs = 0;
  for (const char *c = body; *c; c++) cs ^= (uint8_t)*c;
  char line[96];
  int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
  if (n > 0 && n < (int)sizeof(line))
    gps_tx((const uint8_t *)line, (uint16_t)n);
}

/* Watch the RX ring for `token` (plain bytes, e.g. "ZDA,") for up to `ms`.
   1 = seen, 0 = not. Same abort-then-arm pattern as listen_traffic. */
static int wait_token(const char *token, uint32_t ms)
{
  size_t tl = strlen(token), m = 0;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms) {
    while (g_rx_head != g_rx_tail) {
      uint8_t b = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
      if (b == (uint8_t)token[m]) { if (++m == tl) return 1; }
      else m = (b == (uint8_t)token[0]) ? 1 : 0;
    }
  }
  return 0;
}

/* Every UBX command went un-ACK'd although the module streams NMEA. Two
   causes look identical from the logs: (a) the box→module TX wire is dead,
   (b) the module's UBX *input* is disabled (inProtoMask NMEA-only — kept in
   BBR while V_BCKP stays powered, or in flash on older modules) and it
   swallows every UBX frame. Tell them apart with something the module must
   answer even in case (b): a standard NMEA poll for ZDA, a sentence the
   M10 does NOT output by default — so a ZDA line can only be the reply.
     answered → TX works, UBX input blocked → try $PUBX,41 (NMEA-borne port
                config: UBX+NMEA in/out, 230400) to re-enable UBX.
     silent   → TX path dead: hardware (JP2 pin 14 → module RXD).
   Returns the new baud if $PUBX,41 took, else 0 (UART left at `baud`). */
/* MCU supply measured via VREFINT on ADC1 (v0.0.81) — PA0's push-pull "high"
   is VDD, so this says whether the box drives 1.8 V or 3.3 V into the
   module's RXD. Bare registers (the ADC HAL isn't in the build), every wait
   bounded, ADC returned to deep power-down afterwards. 0 = measurement failed. */
static uint32_t gps_measure_vdda_mv(uint32_t *raw_out, uint32_t *cal_out)
{
  uint32_t t0;
  uint32_t step = 0;
  /* U5: VDDA is an independent supply — the ADC stays isolated until it is
     declared valid (PWR_SVMCR.ASV). */
  RCC->AHB3ENR |= RCC_AHB3ENR_PWREN;
  (void)RCC->AHB3ENR;
  PWR->SVMCR |= PWR_SVMCR_ASV;
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_ADC12EN;
  (void)RCC->AHB2ENR1;
  ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_PRESC)
                    | (4UL << ADC_CCR_PRESC_Pos)          /* HCLK/8 = 20 MHz */
                    | ADC_CCR_VREFEN;
  ADC1->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN;
  t0 = HAL_GetTick();
  while (!(ADC1->ISR & ADC_ISR_LDORDY)) if (HAL_GetTick() - t0 > 10) { step = 1; goto fail; }
  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  t0 = HAL_GetTick();
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) if (HAL_GetTick() - t0 > 10) { step = 2; goto fail; }
  ADC1->PCSEL |= ADC_PCSEL_PCSEL_0;                     /* VREFINT = channel 0 */
  ADC1->SQR1   = 0;                                     /* 1 conversion, ch 0 */
  ADC1->SMPR1  = (ADC1->SMPR1 & ~ADC_SMPR1_SMP0) | ADC_SMPR1_SMP0;  /* longest */
  HAL_Delay(2);                                         /* VREFINT start-up */
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) {
    ADC1->ISR = ADC_ISR_EOC;
    ADC1->CR |= ADC_CR_ADSTART;
    t0 = HAL_GetTick();
    while (!(ADC1->ISR & ADC_ISR_EOC)) if (HAL_GetTick() - t0 > 10) { step = 3; goto fail; }
    acc += ADC1->DR;
  }
  uint32_t raw = acc / 8;
  const volatile uint8_t *c = (const volatile uint8_t *)VREFINT_CAL_ADDR;
  uint32_t cal = (uint32_t)c[0] | ((uint32_t)c[1] << 8);  /* unaligned in sysflash */
  ADC1->CR |= ADC_CR_ADDIS;
  t0 = HAL_GetTick();
  while ((ADC1->CR & ADC_CR_ADEN) && HAL_GetTick() - t0 < 10) { }
  ADC12_COMMON->CCR &= ~ADC_CCR_VREFEN;
  ADC1->CR &= ~ADC_CR_ADVREGEN;
  ADC1->CR |= ADC_CR_DEEPPWD;
  if (raw_out) *raw_out = raw;
  if (cal_out) *cal_out = cal;
  return raw ? (VREFINT_CAL_VREF * cal) / raw : 0;
fail:
  ADC12_COMMON->CCR &= ~ADC_CCR_VREFEN;
  if (raw_out) *raw_out = 0;
  if (cal_out) *cal_out = step;       /* failed step: 1 LDO, 2 ADRDY, 3 EOC */
  return 0;
}

/* PA0 output type (v0.0.80). The STEVAL-MKBOXPRO MCU I/O runs at 1.8 V
   (JP2 pin 1 = the 1V8 rail), so push-pull PA0 drives a 1.8 V "high" — below
   the 3.3 V MAX-M10S's ~0.7·VCC input threshold, so the module never sees a
   clean start bit while its own 3.3 V TX into 5 V-tolerant PA1 works fine
   (exactly the "NMEA arrives, nothing ever ACKs" signature). Open-drain lets
   the module-side RXD pull-up lift the line to 3.3 V; PA0 is FT so that is
   safe. Survives gps_uart_reopen (HAL_UART_DeInit leaves GPIO alone). */
static uint8_t g_pa0_od;

static void gps_pa0_set_od(uint8_t od)
{
  GPIO_InitTypeDef g = {0};
  g.Mode      = od ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_UART4;
  g.Pin       = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &g);
  g_pa0_od = od;
}

/* What is attached to PA0? (v0.0.83). PA0 as a plain input, sampled with no
   pull, the MCU pull-up and the MCU pull-down (~40 kΩ):
     edges > 0 (no pull)      → something drives PA0 — e.g. soldered to the
                                module's TX instead of its RX;
     low even with pull-up    → held low externally (short to GND / low output);
     high even with pull-down → an external pull-up / driver: the wire reaches
                                a pulled-up input (the module RXD);
     follows the MCU pull     → open / high-impedance line.
   Restores PA0 to UART4 AF afterwards (push-pull or open-drain per g_pa0_od). */
static void gps_pa0_mode(uint32_t mode, uint32_t pull)
{
  GPIO_InitTypeDef g = {0};
  g.Mode      = mode;
  g.Pull      = pull;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_UART4;
  g.Pin       = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &g);
}

static void gps_pa0_sample(uint32_t ms, uint32_t *highs, uint32_t *total, uint32_t *edges)
{
  uint32_t h = 0, n = 0, e = 0;
  uint32_t last = (GPIOA->IDR & GPIO_PIN_0) ? 1U : 0U;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms) {
    uint32_t v = (GPIOA->IDR & GPIO_PIN_0) ? 1U : 0U;
    h += v; n++;
    if (v != last) { e++; last = v; }
  }
  *highs = h; *total = n; *edges = e;
}

/* PA0 load + crosstalk (v0.0.88). Rise/fall time through the MCU's internal
   ~40 kΩ pull from the opposite rail, DWT-timed: t ≈ 0.69·R·C → rough
   attached capacitance (pin + box trace ≈ 10 pF, + 30 cm ribbon core
   ≈ 25-30 pF, + SparkFun trace + u-blox RXD a little more). */
static uint32_t pa0_edge_cycles(int up)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t best[15];
  for (int r = 0; r < 15; r++) {
    GPIOA->BSRR = up ? (GPIO_PIN_0 << 16) : GPIO_PIN_0;          /* opposite level */
    GPIOA->PUPDR &= ~3U;
    GPIOA->MODER = (GPIOA->MODER & ~3U) | 1U;                    /* output */
    for (volatile int d = 0; d < 800; d++) { }                   /* ~20 µs */
    __disable_irq();
    GPIOA->PUPDR = (GPIOA->PUPDR & ~3U) | (up ? 1U : 2U);
    uint32_t t0 = DWT->CYCCNT;
    GPIOA->MODER &= ~3U;                                         /* input */
    uint32_t dt;
    for (;;) {
      dt = DWT->CYCCNT - t0;
      int v = (GPIOA->IDR & GPIO_PIN_0) ? 1 : 0;
      if (v == up || dt > 160000U) break;                        /* 1 ms cap */
    }
    __enable_irq();
    best[r] = dt;
  }
  for (int i = 1; i < 15; i++) {                                 /* median */
    uint32_t x = best[i]; int j = i - 1;
    while (j >= 0 && best[j] > x) { best[j + 1] = best[j]; j--; }
    best[j + 1] = x;
  }
  return best[7];
}

static void pa0_load_and_crosstalk(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  uint32_t cu = pa0_edge_cycles(1), cd = pa0_edge_cycles(0);
  /* pF ≈ t / (0.69 · 40 kΩ); t = cycles / 160 MHz → pF ≈ cycles · 0.226 */
  uint32_t pfu = (cu * 226U) / 1000U, pfd = (cd * 226U) / 1000U;
  /* Crosstalk: PA0 floating (input, no pull), PA1 = module TX (NMEA).
     Count PA0 edges while PA1 toggled in the last 2 ms vs. while PA1 idle. */
  GPIOA->PUPDR &= ~3U;
  GPIOA->MODER &= ~3U;
  uint32_t last_p1 = 0, p1prev = (GPIOA->IDR & GPIO_PIN_1) ? 1U : 0U,
           p0prev = (GPIOA->IDR & GPIO_PIN_0) ? 1U : 0U;
  uint32_t e_act = 0, e_idle = 0, n_act = 0, n_idle = 0, p1edges = 0;
  uint32_t t_end = HAL_GetTick() + 2000U;
  while ((int32_t)(HAL_GetTick() - t_end) < 0) {
    uint32_t idr = GPIOA->IDR, now = DWT->CYCCNT;
    uint32_t p1 = (idr & GPIO_PIN_1) ? 1U : 0U, p0 = (idr & GPIO_PIN_0) ? 1U : 0U;
    if (p1 != p1prev) { last_p1 = now; p1prev = p1; p1edges++; }
    int active = (now - last_p1) < 320000U;                      /* 2 ms */
    if (active) n_act++; else n_idle++;
    if (p0 != p0prev) { if (active) e_act++; else e_idle++; p0prev = p0; }
  }
  /* edges per million samples, active vs idle */
  uint32_t r_act  = n_act  ? (uint32_t)(((uint64_t)e_act  * 1000000U) / n_act)  : 0;
  uint32_t r_idle = n_idle ? (uint32_t)(((uint64_t)e_idle * 1000000U) / n_idle) : 0;
  ErrLog_Writef("gps: pa0 load ~%lu/%lu pF (%lu/%lu cyc) | xtalk nmea=%lu/%lu (%lu ppm) idle=%lu/%lu (%lu ppm) pa1=%lu -> %s",
                (unsigned long)pfu, (unsigned long)pfd, (unsigned long)cu, (unsigned long)cd,
                (unsigned long)e_act, (unsigned long)n_act, (unsigned long)r_act,
                (unsigned long)e_idle, (unsigned long)n_idle, (unsigned long)r_idle,
                (unsigned long)p1edges,
                (r_act > 4 * r_idle + 5) ? "PA0 picks up the NMEA"
                                         : "no crosstalk");
}

/* Does PA0 reach the u-blox RXD? (v0.0.89). The MAX-M10S RXD has an always-on
   internal pull-up (datasheet UBX-20035208: R_pu typ 17 kΩ, 6..72 kΩ). With
   PA0 released (input, no MCU pull) that pull-up must lift the ~25 pF line
   to VIO within microseconds. Charge PA0 low, release, time the rise (cap
   50 ms); then charge high, release, time the fall (cap 50 ms). Pull-up
   present ⇒ fast rise, no fall. No pull-up ⇒ the line keeps its charge and
   only leakage moves it. Tolerance-independent, unlike the 40 kΩ divider. */
static uint32_t pa0_release_time(int from_high, uint32_t cap_cycles)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  GPIOA->BSRR = from_high ? GPIO_PIN_0 : (GPIO_PIN_0 << 16);
  GPIOA->PUPDR &= ~3U;
  GPIOA->MODER = (GPIOA->MODER & ~3U) | 1U;                      /* output */
  for (volatile int d = 0; d < 2000; d++) { }                    /* ~50 µs */
  uint32_t t0 = DWT->CYCCNT, dt;
  GPIOA->MODER &= ~3U;                                           /* release */
  for (;;) {
    dt = DWT->CYCCNT - t0;
    int v = (GPIOA->IDR & GPIO_PIN_0) ? 1 : 0;
    if (v != from_high || dt > cap_cycles) break;
  }
  return dt;
}

static int pa0_rxd_pullup_check(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  const uint32_t cap = 160000U * 50U;                            /* 50 ms */
  uint32_t rise = pa0_release_time(0, cap);                      /* low → high? */
  uint32_t fall = pa0_release_time(1, cap);                      /* high → low? */
  int pullup = (rise < 160000U) && (fall >= cap);                /* rise < 1 ms, never falls */
  ErrLog_Writef("gps: pa0 rxd-pullup low->high %s%lu us, high->low %s%lu us -> %s",
                (rise >= cap) ? ">" : "", (unsigned long)(rise / 160U),
                (fall >= cap) ? ">" : "", (unsigned long)(fall / 160U),
                pullup ? "pull-up present: PA0 reaches the u-blox RXD"
                       : "NO pull-up: pin 14 not on u-blox RXD (6-72k pull-up)");
  g_wire_pullup = (int8_t)pullup;
  return pullup;
}

static void gps_pa0_probe(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  uint32_t h0, n0, e0, hu, nu, eu, hd, nd, ed;
  gps_pa0_mode(GPIO_MODE_INPUT, GPIO_NOPULL);
  HAL_Delay(5);
  gps_pa0_sample(1000, &h0, &n0, &e0);          /* NMEA burst at 1 Hz lands here */
  gps_pa0_mode(GPIO_MODE_INPUT, GPIO_PULLUP);
  HAL_Delay(5);
  gps_pa0_sample(100, &hu, &nu, &eu);
  gps_pa0_mode(GPIO_MODE_INPUT, GPIO_PULLDOWN);
  HAL_Delay(5);
  gps_pa0_sample(100, &hd, &nd, &ed);
  pa0_load_and_crosstalk();                       /* v0.0.88 */
  pa0_rxd_pullup_check();                         /* v0.0.89 */
  gps_pa0_mode(g_pa0_od ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP, GPIO_NOPULL);
  const char *v =
      (e0 > 500)                          ? "PA0 is DRIVEN (toggles) - wired to an output, e.g. the module TX?" :
      (hu * 10 < nu)                      ? "PA0 held LOW externally (short to GND / low output)" :
      (hd * 10 > nd * 9)                  ? "PA0 has an external PULL-UP - wire reaches a pulled-up input" :
      (hu * 10 > nu * 9 && hd * 10 < nd)  ? "PA0 follows the MCU pull - open / high-impedance line" :
                                            "PA0 ambiguous";
  ErrLog_Writef("gps: pa0 probe nopull high=%lu/%lu edges=%lu | pullup high=%lu/%lu | pulldown high=%lu/%lu -> %s",
                (unsigned long)h0, (unsigned long)n0, (unsigned long)e0,
                (unsigned long)hu, (unsigned long)nu,
                (unsigned long)hd, (unsigned long)nd, v);
}

/* Count `token` in the RX stream for `ms`; also the longest silence between
   received bytes (ms). Same thread-side ring read as wait_token. */
static int gps_count_token(const char *token, uint32_t ms, uint32_t *max_gap_ms)
{
  size_t tl = strlen(token), m = 0;
  int hits = 0;
  uint32_t t0 = HAL_GetTick(), last = t0, gap = 0;
  while ((HAL_GetTick() - t0) < ms) {
    while (g_rx_head != g_rx_tail) {
      uint32_t now = HAL_GetTick();
      if (now - last > gap) gap = now - last;
      last = now;
      uint8_t b = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
      if (b == (uint8_t)token[m]) { if (++m == tl) { hits++; m = 0; } }
      else m = (b == (uint8_t)token[0]) ? 1 : 0;
    }
  }
  uint32_t tail = HAL_GetTick() - last;
  if (tail > gap) gap = tail;
  if (max_gap_ms) *max_gap_ms = gap;
  return hits;
}

/* Does the module hear PA0 at all, even with its replies switched off?
   (v0.0.85) Every earlier test needed a *reply*; a module whose polled/ACK
   output is disabled could accept commands silently. Watch for side effects
   in the NMEA stream we already receive instead:
     1. $PUBX,40 + UBX VALSET (RAM) GGA rate 0 → GGA sentences stop (GGA,
        not GSV: a module without satellites may emit no GSV at all);
     2. UBX-CFG-RST controlled software reset → output pauses, boot TXT
        banner appears.
   Either effect = the module receives PA0 and the fix can go over the UART. */
static void gps_input_effect_test(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  uint32_t g0, g1, gap_base, gap_rst;
  listen_traffic(50, NULL, NULL);                       /* re-arm RX IRQ */
  int gsv_before = gps_count_token("GGA,", 4000, &g0);
  nmea_send("PUBX,40,GGA,0,0,0,0,0,0");
  vs_begin();
  vs_u1(CFG_MSGOUT_NMEA_GGA_UART1, 0);
  ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);
  (void)gps_count_token("GGA,", 1500, &g1);             /* let it apply */
  int gsv_after = gps_count_token("GGA,", 4000, &g1);
  (void)gps_count_token("TXT,", 500, &gap_base);        /* baseline gap */
  const uint8_t rst[4] = { 0x00, 0x00, 0x01, 0x00 };    /* hot, controlled SW reset */
  ubx_send(0x06, 0x04, rst, sizeof(rst));
  int txt = gps_count_token("TXT,", 4000, &gap_rst);
  int gsv_effect = (gsv_before >= 3 && gsv_after * 4 <= gsv_before);
  int rst_effect = (txt > 0) || (gap_rst > 1500);
  ErrLog_Writef("gps: input-effect GGA before=%d after=%d | reset txt=%d max_gap=%lu ms (base %lu) -> %s",
                gsv_before, gsv_after, txt, (unsigned long)gap_rst, (unsigned long)gap_base,
                (gsv_effect || rst_effect)
                  ? "MODULE HEARS PA0 (replies off) - fixable over UART"
                  : "no effect - module receives nothing on RX");
}

/* Is the module RXD on another JP2 pin? (v0.0.87). The MCU-drivable JP2 pins
   besides 14 (PA0) are the debug pins 4 = SWDIO/PA13, 6 = SWCLK/PA14,
   8 = SWO/PB3, 10 = JTDI/PA15. A ribbon/IDC miscount puts the SparkFun RX
   pad on one of those while PA0 (pin 14) ends nowhere — exactly the
   "PA0 far end high-Z" probe result. Bit-bang the ZDA poll at 9600 out of
   each (DWT-timed, IRQs off only while a sentence is clocked out), listen
   for the reply on the working PA1 path, then restore the pin's registers
   byte-for-byte (SWD/JTAG usable again). */
static void bb_send(GPIO_TypeDef *port, uint16_t pin, const char *s)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t bit = SystemCoreClock / 9600U;
  __disable_irq();
  for (const char *c = s; *c; c++) {
    uint16_t frame = (uint16_t)(((uint16_t)(uint8_t)*c << 1) | 0x200U);  /* start 0, 8 data, stop 1 */
    uint32_t t = DWT->CYCCNT;
    for (int i = 0; i < 10; i++) {
      port->BSRR = (frame & (1U << i)) ? pin : ((uint32_t)pin << 16);
      t += bit;
      while ((int32_t)(DWT->CYCCNT - t) < 0) { }
    }
  }
  __enable_irq();
}

static void gps_alt_pin_scan(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  static const struct { GPIO_TypeDef *port; uint16_t pin; uint8_t idx; const char *name; } c[] = {
    { GPIOA, GPIO_PIN_13, 13, "JP2-4 SWDIO PA13" },
    { GPIOA, GPIO_PIN_14, 14, "JP2-6 SWCLK PA14" },
    { GPIOB, GPIO_PIN_3,   3, "JP2-8 SWO PB3"    },
    { GPIOA, GPIO_PIN_15, 15, "JP2-10 JTDI PA15" },
  };
  char zda[24];
  {
    uint8_t cs = 0;
    for (const char *b = "EIGNQ,ZDA"; *b; b++) cs ^= (uint8_t)*b;
    snprintf(zda, sizeof(zda), "$EIGNQ,ZDA*%02X\r\n", cs);
  }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  for (unsigned k = 0; k < sizeof(c) / sizeof(c[0]); k++) {
    GPIO_TypeDef *P = c[k].port;
    uint32_t i = c[k].idx;
    uint32_t moder = P->MODER, otyper = P->OTYPER, ospeedr = P->OSPEEDR,
             pupdr = P->PUPDR, odr = P->ODR, afr = P->AFR[i >> 3];
    P->BSRR = c[k].pin;                                          /* idle high */
    P->OTYPER &= ~(1U << i);
    P->PUPDR  &= ~(3U << (2 * i));
    P->MODER   = (P->MODER & ~(3U << (2 * i))) | (1U << (2 * i)); /* output */
    HAL_Delay(5);
    int answered = 0;
    for (int t = 0; t < 2 && !answered; t++) {
      listen_traffic(50, NULL, NULL);                            /* flush + arm RX */
      bb_send(P, c[k].pin, zda);
      answered = wait_token("ZDA,", 1200);
    }
    P->AFR[i >> 3] = afr;
    P->PUPDR = pupdr; P->OSPEEDR = ospeedr; P->OTYPER = otyper;
    P->ODR = odr;     P->MODER = moder;                          /* restore */
    ErrLog_Writef("gps: alt-pin %s -> %s", c[k].name,
                  answered ? "ANSWERED - the module RXD is wired to THIS JP2 pin, not pin 14"
                           : "no reply");
    if (answered) break;
  }
}

/* Toggle UART4 TX polarity (TXINV) — UE must be off while CR2 changes. */
static void gps_uart_txinv(uint8_t on)
{
  UART4->CR1 &= ~USART_CR1_UE;
  if (on) UART4->CR2 |= USART_CR2_TXINV;
  else    UART4->CR2 &= ~USART_CR2_TXINV;
  UART4->CR1 |= USART_CR1_UE;
}

/* ZDA poll ×3 (see gps_tx_path_test). 1 = the module answered. */
static int gps_zda_probe(void)
{
  for (int i = 0; i < 3; i++) {
    listen_traffic(50, NULL, NULL);              /* flush + arm RX IRQ */
    nmea_send("EIGNQ,ZDA");
    if (wait_token("ZDA,", 1500)) { g_wire_acks++; return 1; }
  }
  return 0;
}

/* One line per boot: which of the SparkFun board's four wires work, judged
   from what the firmware saw (v0.0.90). Pad names are the module's labels.
     3V3 + BLK  — the module talks (TX bytes) or its RXD pull-up is there
                  (the pull-up is inside the module, so it needs power).
     TX pad     — module -> box, JP2 pin 13 / PA1: bytes received, and a baud
                  lock means they decoded as NMEA/UBX.
     RX pad     — box -> module, JP2 pin 14 / PA0: proven by any ACK / poll
                  reply; else the pull-up says wired-but-ignored vs missing.
   Any broken wire puts *** on the line so the ERRLOG check flags the boot. */
static void gps_wiring_report(int locked)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  uint32_t rx = g_rx_total;
  int pwr = (rx > 0) || (g_wire_pullup == 1);
  const char *tx = (rx == 0) ? "NO DATA" : locked ? "ok" : "garbage (no baud lock)";
  const char *rxs = g_wire_acks ? "ok"
                  : (g_wire_pullup == 1) ? "wired, no reply"
                  : (g_wire_pullup == 0) ? "MISSING (no pull-up)"
                  : "no reply";
  int bad = !pwr || rx == 0 || !locked || g_wire_acks == 0;
  ErrLog_Writef("%sgps: wiring 3V3+BLK %s | TX->JP2-13 %s (%lu B) | RX<-JP2-14 %s (%lu ACK)%s",
                bad ? "*** " : "", pwr ? "ok" : "NO POWER?",
                tx, (unsigned long)rx, rxs, (unsigned long)g_wire_acks,
                bad ? " ***" : "");
}

/* ---------- Second path to the module: u-blox DDC (I2C) on I2C3 (v0.0.84) --
   The MAX-M10S also speaks UBX over I2C (DDC, 7-bit address 0x42). On the
   STEVAL-MKBOXPRO, I2C3 (PG7 = SCL, PG8 = SDA, AF4) is the DIL24/extension
   bus in the VDDIO2 domain (set to 3 V for the GPS), so wiring the SparkFun
   Qwiic SDA/SCL there gives the firmware a path to the module that does not
   depend on the UART RX line at all — to read why it ignores the UART and to
   switch UART input back on. Probe costs a few ms when nothing is wired.
   DDC reads: reg 0xFD/0xFE = bytes waiting (big-endian), 0xFF = data. */
#define UBLOX_DDC_ADDR        (0x42U << 1)
#define CFG_UART1INPROT_NMEA  0x10730002UL
#define CFG_UART1OUTPROT_NMEA 0x10740002UL
#define CFG_UART1_ENABLED     0x10520005UL

static I2C_HandleTypeDef g_hi2c3;

static int gps_i2c_init(void)
{
  __HAL_RCC_GPIOG_CLK_ENABLE();
  RCC_PeriphCLKInitTypeDef clk = {0};
  clk.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
  clk.I2c3ClockSelection   = RCC_I2C3CLKSOURCE_PCLK3;      /* 160 MHz */
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) return -1;
  __HAL_RCC_I2C3_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;                              /* Qwiic board has its own too */
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF4_I2C3;
  g.Pin       = GPIO_PIN_7 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOG, &g);
  memset(&g_hi2c3, 0, sizeof(g_hi2c3));
  g_hi2c3.Instance             = I2C3;
  g_hi2c3.Init.Timing          = 0xA040184A;              /* same as I2C1/I2C4, slow */
  g_hi2c3.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  g_hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  g_hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  g_hi2c3.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&g_hi2c3) != HAL_OK) return -1;
  HAL_I2CEx_ConfigAnalogFilter(&g_hi2c3, I2C_ANALOGFILTER_ENABLE);
  return 0;
}

static int gps_i2c_send_ubx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
  uint8_t f[128];
  if (len + 8U > sizeof(f)) return -1;
  f[0] = 0xB5; f[1] = 0x62; f[2] = cls; f[3] = id;
  f[4] = (uint8_t)(len & 0xFF); f[5] = (uint8_t)(len >> 8);
  if (len) memcpy(&f[6], payload, len);
  uint8_t a = 0, b = 0;
  for (uint16_t i = 2; i < 6U + len; i++) { a += f[i]; b += a; }
  f[6 + len] = a; f[7 + len] = b;
  return (HAL_I2C_Master_Transmit(&g_hi2c3, UBLOX_DDC_ADDR, f, (uint16_t)(len + 8U), 100) == HAL_OK) ? 0 : -1;
}

/* Collect DDC output until a frame cls/id appears or `ms` elapse. Returns the
   payload length (copied to out) or -1. NMEA in the stream is skipped. */
static int gps_i2c_wait_ubx(uint8_t cls, uint8_t id, uint8_t *out, uint16_t out_max, uint32_t ms)
{
  static uint8_t buf[600];
  uint16_t n = 0;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms) {
    uint8_t cnt[2];
    if (HAL_I2C_Mem_Read(&g_hi2c3, UBLOX_DDC_ADDR, 0xFD, I2C_MEMADD_SIZE_8BIT, cnt, 2, 50) != HAL_OK) return -1;
    uint16_t avail = (uint16_t)((cnt[0] << 8) | cnt[1]);
    if (avail == 0 || avail == 0xFFFF) { HAL_Delay(10); continue; }
    if (avail > sizeof(buf) - n) {
      if (n > 0) { memmove(buf, buf + n / 2, n - n / 2); n = (uint16_t)(n - n / 2); }
      if (avail > sizeof(buf) - n) avail = (uint16_t)(sizeof(buf) - n);
    }
    if (HAL_I2C_Mem_Read(&g_hi2c3, UBLOX_DDC_ADDR, 0xFF, I2C_MEMADD_SIZE_8BIT, buf + n, avail, 200) != HAL_OK) return -1;
    n = (uint16_t)(n + avail);
    for (uint16_t i = 0; i + 8 <= n; i++) {
      if (buf[i] == 0xB5 && buf[i + 1] == 0x62 && buf[i + 2] == cls && buf[i + 3] == id) {
        uint16_t len = (uint16_t)(buf[i + 4] | (buf[i + 5] << 8));
        if (i + 8U + len > n) break;                      /* incomplete, read more */
        uint16_t c = (len < out_max) ? len : out_max;
        memcpy(out, &buf[i + 6], c);
        return (int)len;
      }
    }
  }
  return -1;
}

/* VALGET one key from `layer` (0 RAM, 1 BBR, 2 Flash, 7 default). -1 = no
   answer / key absent. */
static int32_t gps_i2c_valget(uint32_t key, uint8_t layer)
{
  uint8_t p[8] = { 0x00, layer, 0, 0,
                   (uint8_t)key, (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24) };
  if (gps_i2c_send_ubx(0x06, 0x8B, p, sizeof(p)) != 0) return -1;
  uint8_t r[16];
  int len = gps_i2c_wait_ubx(0x06, 0x8B, r, sizeof(r), 600);
  if (len < 9) return -1;                                  /* 4 hdr + 4 key + ≥1 value */
  uint32_t sz = (key >> 28) & 7U;                          /* 1/2: 1 B, 3: 2 B, 4: 4 B */
  if (sz <= 2) return r[8];
  if (sz == 3) return (int32_t)(r[8] | (r[9] << 8));
  return (int32_t)(r[8] | (r[9] << 8) | (r[10] << 16) | ((uint32_t)r[11] << 24));
}

static void gps_i2c_session(int uart_ok)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  if (gps_i2c_init() != 0) {
    ErrLog_Writef("gps-i2c: I2C3 init failed");
    return;
  }
  if (HAL_I2C_IsDeviceReady(&g_hi2c3, UBLOX_DDC_ADDR, 3, 20) != HAL_OK) {
    ErrLog_Writef("gps-i2c: no u-blox at 0x42 on I2C3 (PG7 SCL / PG8 SDA) — Qwiic SDA/SCL not wired there");
    HAL_I2C_DeInit(&g_hi2c3);
    return;
  }
  ErrLog_Writef("gps-i2c: u-blox ACKs at 0x42 on I2C3");

  uint8_t v[64];
  memset(v, 0, sizeof(v));
  if (gps_i2c_send_ubx(0x0A, 0x04, NULL, 0) == 0 &&
      gps_i2c_wait_ubx(0x0A, 0x04, v, sizeof(v) - 1, 1000) >= 40) {
    v[29] = 0; v[39] = 0;                                  /* swVersion[30], hwVersion[10] */
    ErrLog_Writef("gps-i2c: MON-VER sw='%s' hw='%s'", (char *)v, (char *)&v[30]);
  } else {
    ErrLog_Writef("gps-i2c: MON-VER no reply over I2C");
  }

  static const struct { uint32_t key; const char *name; } keys[] = {
    { CFG_UART1_ENABLED,     "uart1_en" },
    { CFG_UART1_BAUDRATE,    "baud"     },
    { CFG_UART1INPROT_UBX,   "in_ubx"   },
    { CFG_UART1INPROT_NMEA,  "in_nmea"  },
    { CFG_UART1OUTPROT_UBX,  "out_ubx"  },
    { CFG_UART1OUTPROT_NMEA, "out_nmea" },
  };
  for (uint8_t layer = 0; layer <= 1; layer++) {
    char line[160];
    int o = snprintf(line, sizeof(line), "gps-i2c: %s", layer ? "BBR" : "RAM");
    for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]) && o < (int)sizeof(line); i++) {
      int32_t val = gps_i2c_valget(keys[i].key, layer);
      o += snprintf(line + o, sizeof(line) - (size_t)o, " %s=%ld", keys[i].name, (long)val);
    }
    ErrLog_Writef("%s", line);
  }

  if (!uart_ok) {
    /* Switch UART1 input (UBX + NMEA) and UBX output back on, RAM + BBR. */
    uint8_t p[4 + 3 * 5];
    uint8_t *q = p;
    *q++ = 0x00; *q++ = 0x03; *q++ = 0; *q++ = 0;          /* version, layers RAM|BBR */
    const uint32_t k[3] = { CFG_UART1INPROT_UBX, CFG_UART1INPROT_NMEA, CFG_UART1OUTPROT_UBX };
    for (int i = 0; i < 3; i++) {
      *q++ = (uint8_t)k[i]; *q++ = (uint8_t)(k[i] >> 8);
      *q++ = (uint8_t)(k[i] >> 16); *q++ = (uint8_t)(k[i] >> 24);
      *q++ = 1;
    }
    uint8_t ack[4];
    int rc = (gps_i2c_send_ubx(0x06, 0x8A, p, (uint16_t)(q - p)) == 0)
               ? gps_i2c_wait_ubx(0x05, 0x01, ack, sizeof(ack), 800) : -1;
    ErrLog_Writef("gps-i2c: re-enable UART1 input (RAM+BBR) %s", (rc >= 2) ? "ACK" : "no ACK");
    if (rc >= 2) {
      int heard = gps_zda_probe();
      ErrLog_Writef("gps-i2c: UART ZDA probe after re-enable: %s",
                    heard ? "ANSWERED — UART input works now; full config on next boot"
                          : "still no reply — the module ignores the RX line itself");
    }
  }
}

static uint32_t gps_tx_path_test(uint32_t baud)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  /* v0.0.90: no RXD pull-up on PA0 = no wire to talk to. The probes below
     took ~45 s, held the GPS out of GPS_Tick and fired the 30 s
     "GPS NO NMEA" alarm on a module that was streaming fine. Skip them and
     log at the factory 9600 / 1 Hz straight away. */
  if (g_wire_pullup == 0) {
    ErrLog_Writef("gps: tx-test skipped - no RXD pull-up on PA0, staying @%lu",
                  (unsigned long)baud);
    return 0;
  }
  int answered = gps_zda_probe();
  if (!answered) {
    gps_tx_diag("tx-test");
    /* v0.0.80: retry with PA0 open-drain (1.8 V MCU I/O vs 3.3 V module). */
    gps_pa0_set_od(1);
    answered = gps_zda_probe();
    if (!answered) {
      /* idle_high here = does the module side pull the open-drain line up
         at all? 0/64 → no pull-up, so this attempt could not work. */
      gps_tx_diag("tx-test-od");
      gps_pa0_set_od(0);
      /* v0.0.83: what is on the wire, and an inverted-TX attempt. */
      gps_pa0_probe();
      gps_input_effect_test();
      gps_alt_pin_scan();
      gps_uart_txinv(1);
      answered = gps_zda_probe();
      gps_uart_txinv(0);
      ErrLog_Writef("gps: tx-test @%lu with TX inverted: %s",
                    (unsigned long)baud, answered ? "ANSWERED" : "no reply");
      if (answered) {
        gps_uart_txinv(1);   /* keep it — the path inverts */
        return 0;            /* config at 9600 follows; baud raise left as-is */
      }
      ErrLog_Writef("*** gps: tx-test @%lu — no reply to NMEA poll (ZDA), push-pull "
                    "and open-drain: box->module TX dead, check JP2 pin 14 -> module RXD ***",
                    (unsigned long)baud);
      return 0;
    }
    ErrLog_Writef("gps: tx-test @%lu — answered with PA0 open-drain (1.8 V MCU vs 3.3 V module); keeping open-drain",
                  (unsigned long)baud);
    gps_tx_diag("tx-test-od");
    /* The module hears us now; its UBX input is most likely fine, so retry
       the normal baud raise before the $PUBX,41 last resort. */
    vs_begin();
    vs_u4(CFG_UART1_BAUDRATE, GPS_UART_BAUDRATE);
    ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);
    if (gps_uart_reopen(GPS_UART_BAUDRATE) != 0) return 0;
    if (ubx_wait_ack(0x06, 0x8A, 700) == 0) {
      ErrLog_Writef("gps: cfg baud %lu ACK (from %lu, open-drain)",
                    (unsigned long)GPS_UART_BAUDRATE, (unsigned long)baud);
      return GPS_UART_BAUDRATE;
    }
    int nl = 0, ux = 0;
    listen_traffic(600, &nl, &ux);
    if (nl + ux > 0) {
      /* Module switched (its TX reaches us at 230400) but the ACK was lost —
         open-drain rise time is the suspect at the high rate. Keep going at
         230400; the config steps report what lands. */
      ErrLog_Writef("*** gps: baud %lu un-ACK'd but module streams there (nmea=%d ubx=%d) — open-drain edge at high baud? ***",
                    (unsigned long)GPS_UART_BAUDRATE, nl, ux);
      return GPS_UART_BAUDRATE;
    }
    if (gps_uart_reopen(baud) != 0) return 0;
  }
  ErrLog_Writef("*** gps: tx-test @%lu — NMEA poll answered: TX works, "
                "UBX input blocked — trying $PUBX,41 ***", (unsigned long)baud);

  char body[48];
  snprintf(body, sizeof(body), "PUBX,41,1,0003,0003,%lu,0",
           (unsigned long)GPS_UART_BAUDRATE);
  nmea_send(body);
  HAL_Delay(100);                                /* let it switch */
  if (gps_uart_reopen(GPS_UART_BAUDRATE) != 0) return 0;
  ubx_send(0x0A, 0x04, NULL, 0);                 /* MON-VER: needs UBX in+out */
  int nl = 0, ux = 0;
  listen_traffic(800, &nl, &ux);
  if (ux > 0) {
    ErrLog_Writef("gps: $PUBX,41 OK — @%lu, UBX input re-enabled (nmea=%d ubx=%d)",
                  (unsigned long)GPS_UART_BAUDRATE, nl, ux);
    return GPS_UART_BAUDRATE;
  }
  ErrLog_Writef("*** gps: $PUBX,41 ignored @%lu (nmea=%d ubx=%d) — staying @%lu ***",
                (unsigned long)GPS_UART_BAUDRATE, nl, ux, (unsigned long)baud);
  (void)gps_uart_reopen(baud);
  return 0;
}

/* ---------- GPS power (backup / wake) ----------------------------------- */

/* Drop the receiver into UBX-RXM-PMREQ *backup* mode (~tens of µA). Wakes on
   the next UART-RX activity (wakeupSources = uartrx). PMREQ is NOT ACK'd, so
   this is fire-and-forget. Software backup keeps the module's RAM powered
   (VCC stays on — only the MCU commands the sleep), so this session's RAM
   config survives and a later wake resumes UBX at the locked baud without a
   reconfigure. Only a real supply cut loses the config — and that path goes
   back through the full GPS_Init anyway. */
static void gps_pmreq_backup(void)
{
  uint8_t p[16] = {0};
  p[0]  = 0x00;                 /* version 0 (16-byte form)          */
  /* duration @4 = 0 → sleep until woken                            */
  p[8]  = 0x02;                 /* flags[bit1] = backup              */
  p[12] = 0x08;                 /* wakeupSources[bit3] = uartrx      */
  ubx_send(0x02, 0x41, p, sizeof(p));
}

/* Wake a sleeping module: any edge on its RX pin (our TX) wakes it. A short
   0xFF burst (junk the module discards) provides the edges; then it hot-starts.
   Harmless when the module is already awake. */
static void gps_wake_pulse(void)
{
  uint8_t junk[16];
  memset(junk, 0xFF, sizeof(junk));
  gps_tx(junk, sizeof(junk));
  HAL_Delay(120);
}

/* Steps 2-6 of Peter's per-boot recipe (constellations → power mode → UBX
   output → NMEA off → 1 Hz acquisition rate), on whatever baud the UART is
   currently locked to. Shared by GPS_Init and the mid-session link watchdog
   (gps_link_recover) — a module that rebooted from a supply glitch is back
   at its factory defaults and needs the full replay. Returns the failure
   count; *pvt_ok_out (may be NULL) reports whether the UBX-output step
   ACK'd — the gate for both the NMEA silencing and the 1↔10 Hz switcher. */
static int gps_apply_config(int *pvt_ok_out)
{
  extern void ErrLog_Write(const char *msg);
  int fails = 0;

  /* 2. Constellations: GPS + Galileo on, BeiDou + GLONASS off. */
  vs_begin();
  vs_u1(CFG_SIGNAL_GPS_ENA,      1);
  vs_u1(CFG_SIGNAL_GPS_L1CA_ENA, 1);
  vs_u1(CFG_SIGNAL_GAL_ENA,      1);
  vs_u1(CFG_SIGNAL_GAL_E1_ENA,   1);
  vs_u1(CFG_SIGNAL_BDS_ENA,      0);
  vs_u1(CFG_SIGNAL_BDS_B1_ENA,   0);
  vs_u1(CFG_SIGNAL_BDS_B1C_ENA,  0);
  vs_u1(CFG_SIGNAL_GLO_ENA,      0);
  vs_u1(CFG_SIGNAL_GLO_L1_ENA,   0);
  fails += vs_send("signal GPS+GAL -BDS -GLO");
  HAL_Delay(500);   /* the receiver restarts its GNSS subsystem on a
                       constellation change — give it a moment before the
                       next command (u-blox M10 integration manual) */

  /* 3. Full-power mode (no power-save duty cycling). */
  vs_begin();
  vs_u1(CFG_PM_OPERATEMODE, 0);
  fails += vs_send("pm full-power");

  /* 4. UBX protocol + periodic output: NAV-PVT every epoch, NAV-SAT every
     10th. In/out protocol enables included — UBX *output* is off in the
     M10 factory default, and after a power cut that default is what we get.
     Safe to enable now: we are on the 230400 line, so the added output
     cannot back up the module's TX and drown the ACKs (Peter, step-1 note). */
  vs_begin();
  vs_u1(CFG_UART1INPROT_UBX,             1);
  vs_u1(CFG_UART1OUTPROT_UBX,            1);
  vs_u1(CFG_MSGOUT_UBX_NAV_PVT_UART1,    1);
  vs_u1(CFG_MSGOUT_UBX_NAV_SAT_UART1,   10);
  int pvt_rc = vs_send("ubx out pvt=1 sat=10");
  fails += pvt_rc;

  /* 5. Silence NMEA — but only once the UBX feed is confirmed, so a partial
     failure can never leave the module emitting nothing at all. */
  if (pvt_rc == 0) {
    vs_begin();
    vs_u1(CFG_MSGOUT_NMEA_GGA_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_RMC_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_GSV_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_GSA_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_VTG_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_GLL_UART1, 0);
    vs_u1(CFG_MSGOUT_NMEA_ZDA_UART1, 0);
    fails += vs_send("nmea off");
  } else {
    ErrLog_Write("gps: keeping NMEA (ubx out not ACK'd) — fallback parser active");
  }

  /* 6. Nav rate — ACQUISITION rate, i.e. 1 Hz (v0.0.46, issue #10). The
     module always cold-starts fix-less here, and acquiring at 10 Hz is what
     kept the field boxes fix-less forever (26'583 epochs, rmc=0) while the
     factory 1 Hz fixes on the same board. gps_rate_manage() raises the rate
     to GPS_RATE_HZ once the first valid fix is in — tracking at 10 Hz is
     proven fine (Peter's u-center: 15 sats / 3D at 10 Hz). On the slow
     fallback line 1 Hz additionally keeps the link readable (5 Hz NAV-PVT
     alone would be ~52 % of a 960 B/s line). */
  vs_begin();
  vs_u2(CFG_RATE_MEAS, (uint16_t)GPS_ACQ_RATE_MS);
  vs_u2(CFG_RATE_NAV,  1);
  fails += vs_send("rate 1Hz acq");

  if (pvt_ok_out) *pvt_ok_out = (pvt_rc == 0);
  return fails;
}

/* ---------- Listen to the u-blox firmware from power-on (v0.0.86) --------
   The module boots on the same rail as the MCU and prints its $GxTXT banner
   (firmware version, PROTVER, module, sometimes config notes) within the
   first second — long before GPS_Init (~0.45 s) and its baud handling. So the
   UART is opened RX-only-in-practice at the module's factory 9600 straight
   after SystemClock_Config, the byte IRQ fills the ring, and GPS_Init first
   reports what arrived: every TXT line plus a histogram of the NMEA sentence
   types, i.e. which (non-factory?) message config the module runs from its
   battery-backed memory. Only a real power-on shows the banner; an MCU-only
   reset (FOTA, IWDG) just shows the running stream. */
static uint32_t g_early_tick;          /* HAL tick when EarlyListen armed; 0 = not */

void GPS_EarlyListen(void)
{
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_UART4;
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &g);
  if (gps_uart_reopen(9600) != 0) return;
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  if (HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1) != HAL_OK) return;
  g_early_tick = HAL_GetTick();
  if (g_early_tick == 0) g_early_tick = 1;
}

static void gps_early_report(void)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  if (!g_early_tick) return;
  /* Give the module the first 3 s after power-on (its banner + 2 epochs). */
  while ((HAL_GetTick() - g_early_tick) < 3000) {
    uint16_t used = (uint16_t)((g_rx_head + GPS_RX_RING_SIZE - g_rx_tail) % GPS_RX_RING_SIZE);
    if (used > GPS_RX_RING_SIZE - 64) break;
  }
  static char buf[GPS_RX_RING_SIZE + 1];
  uint16_t n = 0;
  while (g_rx_tail != g_rx_head && n < GPS_RX_RING_SIZE) {
    buf[n++] = (char)g_rx_ring[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
  }
  buf[n] = 0;
  char types[12][4];
  uint16_t counts[12];
  int nt = 0, txt = 0, lines = 0;
  memset(counts, 0, sizeof(counts));
  for (uint16_t i = 0; i < n; i++) {
    if (buf[i] != '$') continue;
    uint16_t j = i;
    while (j < n && buf[j] != '\r' && buf[j] != '\n') j++;
    uint16_t len = (uint16_t)(j - i);
    if (len >= 7) {
      lines++;
      const char *t = &buf[i + 3];                 /* "$GNTXT" → "TXT" */
      if (memcmp(t, "TXT", 3) == 0 && txt < 14) {
        char line[96];
        uint16_t c = (len < sizeof(line) - 1) ? len : (uint16_t)(sizeof(line) - 1);
        memcpy(line, &buf[i], c); line[c] = 0;
        ErrLog_Writef("gps: early %s", line);
        txt++;
      }
      int k;
      for (k = 0; k < nt; k++) if (memcmp(types[k], t, 3) == 0) break;
      if (k == nt && nt < 12) { memcpy(types[nt], t, 3); types[nt][3] = 0; nt++; }
      if (k < 12) counts[k]++;
    }
    i = j;
  }
  char mix[160];
  int o = 0;
  for (int k = 0; k < nt && o < (int)sizeof(mix); k++)
    o += snprintf(mix + o, sizeof(mix) - (size_t)o, " %s=%u", types[k], counts[k]);
  ErrLog_Writef("gps: early capture %u B, %d lines in %lu ms (dropped %lu) | txt=%d | mix:%s%s",
                n, lines, (unsigned long)(HAL_GetTick() - g_early_tick),
                (unsigned long)g_rx_dropped, txt, nt ? mix : " none",
                txt ? "" : " (no banner: MCU-only reset, or module booted before the UART opened)");
  g_early_tick = 0;
}

int GPS_Init(void)
{
  gps_early_report();                 /* v0.0.86: what the module said since power-on */
  memset(&g_latest, 0, sizeof(g_latest));
  g_updated   = 0;
  g_linelen   = 0;
  g_ux_st     = 0;
  g_rx_head   = 0;
  g_rx_tail   = 0;

  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA0 = TX, PA1 = RX, AF8 */
  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_UART4;
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &g);

  extern void ErrLog_Write(const char *msg);
  extern void ErrLog_Writef(const char *fmt, ...);

  /* Before the UART is opened: idle level + pin config of PA0 (v0.0.79). */
  gps_tx_diag("pre-init");
  /* Is JP2 pin 14 on the module's RXD? (v0.0.90: every boot, for the wiring
     line.) Briefly drives PA0 as GPIO; the module at most sees one framing
     error. UART4 AF is restored before anything is sent. */
  g_wire_acks = 0;
  (void)pa0_rxd_pullup_check();
  gps_pa0_mode(GPIO_MODE_AF_PP, GPIO_NOPULL);
  {
    uint32_t vr = 0, vc = 0;
    uint32_t mv = gps_measure_vdda_mv(&vr, &vc);
    ErrLog_Writef("gps: mcu vdda=%lu mV (vrefint raw=%lu cal=%lu) — PA0 push-pull high = this level",
                  (unsigned long)mv, (unsigned long)vr, (unsigned long)vc);
  }

  /* Enable UART4 IRQ early — we need it armed during the baud-detection
     listen window so bytes accumulate in the ring. Priority 6: on Cortex-M
     a LOWER number preempts, so the <10 µs single-byte ring push preempts
     every other app IRQ (OTG_FS at 13, SysTick at 15; SDMMC is polled, its
     IRQ never enabled) — nothing at IRQ level can starve the UART into
     silent byte loss (OVRDIS discards overruns uncounted). */
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  /* --- GPSRAW.CFG config bypass (v0.0.47, issue #10 A/B test) -------------
     If a file named GPSRAW.CFG exists on the SD root, GPS_Init sends ZERO
     bytes to the module: no wake pulse, no baud raise, no CFG-* writes. The
     module stays exactly in its factory state (9600 baud, NMEA, 1 Hz, all
     constellations) — the same state as Peter's bootloader-mode experiment
     where the uputronics LED blinks — while the REST of the firmware runs at
     full tilt (SD logging, BLE, sensors, 160 MHz). The NMEA fallback parser
     (GGA/RMC/GSV @9600) keeps the logger fed if a fix arrives.

     Discriminator: LED blinks with the bypass → our configuration is what
     kills acquisition (then bisect CFG-SIGNAL etc.). LED stays dark → the
     running system (EMI / supply under load) is the problem, not the config.
     File content is ignored; delete the file to restore the normal per-boot
     configuration. The *** entry is deliberate — a box in bypass mode must
     never read as a clean production boot. */
  if (SDFat_IsMounted()) {
    PL_File raw;
    if (SDFat_OpenRead(&raw, "GPSRAW.CFG") == PL_FX_OK) {
      SDFat_Close(&raw);
      if (gps_uart_reopen(9600) != 0) {
        ErrLog_Write("gps: uart_init@9600 FAIL");
        return -1;
      }
      /* Arm the RX byte IRQ ourselves (v0.0.49). On the normal path the IRQ
         is armed as a side-effect of listen_traffic/ubx_wait_ack and then
         kept alive by HAL_UART_RxCpltCallback re-arms — the bypass path runs
         neither, and v0.0.47/48 shipped with the ring stone-dead here: even
         a fixing module would have logged bytes=0/rmc=0 and read as a
         false-negative A/B result. Same abort-first pattern as
         listen_traffic (HAL RxState can hold BUSY_RX leftovers). */
      HAL_UART_AbortReceive_IT(&g_huart4);
      if (HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1) != HAL_OK) {
        ErrLog_Write("*** gps: bypass: HAL_UART_Receive_IT FAIL ***");
      }
      g_locked_baud = 9600;
      g_rate_armed  = 0;
      ErrLog_Write("*** gps: GPSRAW.CFG — factory config untouched (A/B bypass) ***");
      ErrLog_Write("gps: ready @9600 baud, factory NMEA 1Hz, RX-only bypass");
      return 0;
    }
  }

  /* Peter's per-boot config (2026-07-13), RAM layer only, every command
     ACK-verified with 3 retries; every un-ACK'd command leaves a *** errlog
     entry (vs_send). Order: BAUD FIRST, then everything else on the fast line
     — v0.0.41 raised the baud last, so constellations / power / UBX-output all
     ran at 9600 while the module was still streaming its factory NMEA (960 B/s,
     mostly consumed). Adding NAV-PVT-every-epoch + NAV-SAT on top backed up the
     module's TX, its ACKs missed the 300 ms budget, and every later command
     reported FAIL. Raising the baud first buys 23 kB/s of headroom (~6 % utilisation at PVT@10Hz + SAT@1Hz), so no
     command can starve its own ACK. */
  int fails = 0;

  /* 1. Baud → 230400, assuming 9600. No baud scan.
     Peter (2026-07-13): "Es braucht keinen Baudratentest am Anfang. Nach dem
     Booten ist es immer 9600 Baud." Right for a cold boot — the hall switch
     cuts the whole rail, the module comes back at its factory 9600 NMEA
     default, and nothing we write ever persists (the M10 never ACKs the legacy
     CFG-CFG save, verified across 15 field boots).

     But NO MCU pin switches the GPS supply — GPS-off is software only
     (UBX-RXM-PMREQ backup). So on an MCU-only reset (FOTA's SWAP_BANK reboot,
     an IWDG reset, a software reset) the module keeps its power AND this
     session's RAM config: it is still at 230400. FOTA is the primary update
     path, so assuming 9600 *unconditionally* would leave the GPS dead after
     every wireless update until someone power-cycles with the magnet.

     Resolution: assume 9600 and send the baud command blind (no listen window
     — that is Peter's ask, and it saves 1.5 s on every cold boot), then CONFIRM
     at 230400. The confirm covers both worlds for free:
       - module @9600   → it accepts the command; the ACK arrives at 230400.
       - module @230400 → it sees our 9600 bytes as framing garbage and drops
                          them (UBX checksums make a false positive impossible),
                          but the MON-VER poll we then send at 230400 answers.
     Only if NEITHER confirms do we fall back to a real probe. */
  if (gps_uart_reopen(9600) != 0) {
    ErrLog_Write("gps: uart_init@9600 FAIL");
    return -1;
  }
  /* A module parked in PMREQ backup by a persisted-OFF session (its VCC
     survived an MCU-only reset) is asleep and would ignore the command. The
     junk burst wakes it; harmless when it is already awake. */
  gps_wake_pulse();

  vs_begin();
  vs_u4(CFG_UART1_BAUDRATE, GPS_UART_BAUDRATE);
  ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);        /* blind, at 9600 */

  if (gps_uart_reopen(GPS_UART_BAUDRATE) != 0) {
    ErrLog_Write("gps: uart_reinit@230400 FAIL");
    return -1;
  }

  uint32_t final_baud = 0;
  if (ubx_wait_ack(0x06, 0x8A, 700) == 0) {
    final_baud = GPS_UART_BAUDRATE;
    ErrLog_Writef("gps: cfg baud %lu ACK (from 9600)",
                  (unsigned long)GPS_UART_BAUDRATE);
  } else {
    ubx_send(0x0A, 0x04, NULL, 0);                 /* MON-VER poll */
    int ux = 0;
    listen_traffic(400, NULL, &ux);
    if (ux > 0) {
      final_baud = GPS_UART_BAUDRATE;
      ErrLog_Write("gps: already @230400 (MCU-only reset) — baud step skipped");
    }
  }

  /* Last resort: the module answered at neither 9600 nor 230400. Probe the
     remaining plausible rates, then retry the raise from wherever it is:
       115200 — left by v0.0.44, the one release that used 115200 as the session
                baud. A box updated by FOTA *from* v0.0.44 reboots MCU-only, so
                the module keeps power AND its 115200 setting. Without this
                candidate the GPS would be dead after exactly the update that
                brings this version in. Keep it until no v0.0.44 box is left.
       38400  — left by pre-v0.0.41 firmware.
       9600   — alive but ignored our command (e.g. a half-dead TX line). */
  if (final_baud == 0) {
    static const uint32_t cands[3] = { 115200U, 38400U, 9600U };
    uint32_t found = 0;
    for (int i = 0; i < 3 && found == 0; i++) {
      if (gps_uart_reopen(cands[i]) != 0) {
        ErrLog_Writef("gps: uart_init@%lu FAIL", (unsigned long)cands[i]);
        return -1;
      }
      int nl = 0, ux = 0;
      if (listen_traffic(1500, &nl, &ux) >= 2) {
        found = cands[i];
        ErrLog_Writef("*** gps: baud fallback — module @%lu (nmea=%d ubx=%d) ***",
                      (unsigned long)found, nl, ux);
      }
    }
    if (found == 0) {
      ErrLog_Write("*** GPS: no baud lock — silent at 9600/230400/115200/38400 ***");
      ErrLog_Write("*** check module supply + RX wiring (JP2 pin 13/14) ***");
      gps_wiring_report(0);
      return -1;
    }

    vs_begin();
    vs_u4(CFG_UART1_BAUDRATE, GPS_UART_BAUDRATE);
    ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);
    if (gps_uart_reopen(GPS_UART_BAUDRATE) != 0) {
      ErrLog_Write("gps: uart_reinit@230400 FAIL");
      return -1;
    }
    if (ubx_wait_ack(0x06, 0x8A, 700) == 0) {
      final_baud = GPS_UART_BAUDRATE;
      ErrLog_Writef("gps: cfg baud %lu ACK (from %lu)",
                    (unsigned long)GPS_UART_BAUDRATE, (unsigned long)found);
    } else {
      final_baud = found;
      ErrLog_Writef("*** gps: cfg baud %lu NOT confirmed — staying @%lu ***",
                    (unsigned long)GPS_UART_BAUDRATE, (unsigned long)found);
      if (gps_uart_reopen(found) != 0) {
        ErrLog_Write("gps: uart_reinit@fallback FAIL");
        return -1;
      }
      uint32_t nb = gps_tx_path_test(found);
      if (nb) final_baud = nb;
      else    fails++;
      listen_traffic(200, NULL, NULL);             /* re-arm the RX IRQ */
    }
  }

  /* Steps 2-6: constellation / power / UBX-out / NMEA-off / 1 Hz acq rate
     (shared with the mid-session link watchdog — gps_apply_config above). */
  int pvt_ok = 0;
  fails += gps_apply_config(&pvt_ok);

  /* Arm the fix-driven 1↔10 Hz switcher only when the configured UBX path
     is actually up — on the NMEA-fallback/slow line the rate must stay 1 Hz
     (and un-ACK'd VALSETs would only spam the errlog). */
  g_rate_armed = (final_baud == GPS_UART_BAUDRATE && pvt_ok) ? 1 : 0;
  g_rate_fast  = 0;
  g_rate_pending = 0;
  g_rate_tries = 0;
  g_rate_backoff_until = 0;

  g_locked_baud = final_baud;

  /* RX IRQ stays armed across all the listen/ack calls. */
  ErrLog_Writef("gps: ready @%lu baud, nav=1Hz acq%s, %s%s",
                (unsigned long)final_baud,
                g_rate_armed ? " (10Hz after first fix)" : "",
                pvt_ok ? "UBX NAV-PVT" : "NMEA fallback",
                (fails > 0) ? " (config incomplete)" : "");
  gps_wiring_report(final_baud != 0);

  /* v0.0.84: second path to the module over DDC/I2C3 (no-op if not wired). */
  gps_i2c_session(pvt_ok && final_baud == GPS_UART_BAUDRATE);

  /* Apply the persisted GPS-power choice. If the user turned GPS off to save
     battery, drop the just-configured module straight into backup mode (its
     RAM config survives software backup, so a later wake resumes cleanly). */
  if (GPS_GetPower() == 0) {
    gps_pmreq_backup();
    g_latest.valid = 0;
    g_latest.fix_q = 0;
    ErrLog_Write("gps: persisted OFF → backup mode");
  }

  return 0;
}

/* ---------- GPS power on/off (persisted, applied to the module) --------- */

int GPS_GetPower(void)
{
  if (g_power >= 0) return g_power;          /* cached */

  g_power = 1;                               /* default ON */
  if (SDFat_IsMounted()) {
    PL_File f;
    if (SDFat_OpenRead(&f, GPSPWR_CFG_NAME) == PL_FX_OK) {
      char c = 0;
      uint32_t got = 0;
      if (SDFat_Read(&f, &c, 1, &got) == PL_FX_OK && got == 1 &&
          (c == 'f' || c == 'F')) {
        g_power = 0;
      }
      SDFat_Close(&f);
    }
  }
  ErrLog_Writef("gps: power = %s", g_power ? "on" : "off");
  return g_power;
}

/* Put the module into backup / wake it, without touching the persisted
   choice. Shared by GPS_SetPower and GPS_Suspend. */
static void gps_apply_power(int on, int was_on)
{
  if (on) {
    /* Wake only if it was actually asleep. The RAM config survives software
       backup, so a wake resumes UBX at the locked baud with no reconfigure. */
    if (!was_on) {
      gps_wake_pulse();
      int nl = 0, ux = 0;
      listen_traffic(1000, &nl, &ux);
      ErrLog_Writef("gps: power ON (wake, nmea=%d ubx=%d)", nl, ux);
    } else {
      ErrLog_Write("gps: power ON (already on)");
    }
  } else {
    gps_pmreq_backup();
    g_latest.valid = 0;
    g_latest.fix_q = 0;
    ErrLog_Write("gps: power OFF → backup mode");
  }
}

void GPS_Suspend(int suspend)
{
  suspend = suspend ? 1 : 0;
  if (suspend == g_suspended) return;
  g_suspended = suspend;
  if (GPS_GetPower() == 0) return;   /* user has it off anyway */
  gps_apply_power(!suspend, suspend);
}

int GPS_SetPower(int on)
{
  on = on ? 1 : 0;

  /* Persist by delete + recreate (the SD layer is append-only, no truncate).
     NOT_FOUND on delete is fine the first time. */
  SDFat_Delete(GPSPWR_CFG_NAME);
  PL_File f;
  if (SDFat_OpenAppend(&f, GPSPWR_CFG_NAME) == PL_FX_OK) {
    const char *txt = on ? "on\n" : "off\n";
    SDFat_Append(&f, txt, (uint32_t)strlen(txt));
    SDFat_Flush(&f);
    SDFat_Close(&f);
  } else {
    ErrLog_Write("gps: SetPower persist open fail (applying anyway)");
  }

  int was = g_power;
  g_power = on;

  /* While suspended the module stays in backup; the new choice applies
     when GPS_Suspend(0) resumes. */
  if (g_suspended) {
    ErrLog_Writef("gps: power %s recorded (suspended by battery lock)",
                  on ? "ON" : "OFF");
    return 0;
  }
  gps_apply_power(on, was != 0);
  return 0;
}

/* ---------- IRQ + callbacks --------------------------------------------- */

void GPS_UART_IRQHandler(void)
{
  HAL_UART_IRQHandler(&g_huart4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4) return;
  uint16_t next = (uint16_t)((g_rx_head + 1u) % GPS_RX_RING_SIZE);
  if (next != g_rx_tail) {
    g_rx_ring[g_rx_head] = g_rx_byte;
    g_rx_head = next;
    g_rx_total++;
  } else {
    g_rx_dropped++;  /* ring full — main loop fell behind */
  }
  HAL_UART_Receive_IT(huart, &g_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4) return;
  /* Clear all RX error flags and re-arm. Framing errors (likely during
     baud-switch transitions) are expected — we don't want them to halt
     the byte stream the way the DMA path did. */
  __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF
                             | UART_CLEAR_FEF  | UART_CLEAR_PEF);
  g_diag_errors++;
  HAL_UART_Receive_IT(huart, &g_rx_byte, 1);
}

/* Fire the 1↔10 Hz CFG-RATE VALSET without blocking (single ~22 B frame,
   <1 ms of TX time at 230400). The ACK is picked up asynchronously by
   gps_rx_byte → g_ack_valset; gps_rate_manage() below resolves it. */
static void gps_rate_request(uint8_t fast)
{
  vs_begin();
  vs_u2(CFG_RATE_MEAS, fast ? (uint16_t)(1000U / GPS_RATE_HZ)
                            : (uint16_t)GPS_ACQ_RATE_MS);
  vs_u2(CFG_RATE_NAV, 1);
  g_ack_valset = 0;
  ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);
  g_rate_pending      = 1;
  g_rate_pending_fast = fast;
  g_rate_sent_tick    = HAL_GetTick();
}

/* Fix-driven nav-rate state machine, run once per GPS_Tick while the
   configured UBX path is up. Non-blocking by design (F-ARCH: no busy ACK
   waits in the superloop): a switch is sent fire-and-forget, confirmed by
   the async ACK flag, resent up to 3× on a 500 ms timeout, and backed off
   for 60 s after a triple failure so a wedged module can't flood the errlog.

   Caveat (accepted): any other CFG-VALSET ACK arriving while a switch is
   pending (GPS-Debug survey toggling its throttle) can be misattributed to
   the switch. Harmless — the next fix/loss edge re-syncs the state. */
static void gps_rate_manage(void)
{
  if (!g_rate_armed) return;
  uint32_t now = HAL_GetTick();

  if (g_rate_pending) {
    if (g_ack_valset) {
      g_rate_fast    = g_rate_pending_fast;
      g_rate_pending = 0;
      g_rate_tries   = 0;
      ErrLog_Writef("gps: nav rate → %s ACK",
                    g_rate_fast ? "10Hz (fix acquired)" : "1Hz (re-acq)");
    } else if ((now - g_rate_sent_tick) > 500U) {
      if (++g_rate_tries >= 3) {
        ErrLog_Writef("*** gps: nav rate switch (%s) NOT ACK'd x3 ***",
                      g_rate_pending_fast ? "10Hz" : "1Hz");
        g_rate_pending       = 0;
        g_rate_tries         = 0;
        g_rate_backoff_until = now + 60000U;
      } else {
        gps_rate_request(g_rate_pending_fast);
      }
    }
    return;
  }

  if (g_rate_backoff_until != 0 && (int32_t)(now - g_rate_backoff_until) < 0)
    return;

  /* Same 3 s freshness window GPS_LastFixQuality uses. */
  uint8_t have_fix = (g_latest_valid_tick != 0) &&
                     (now - g_latest_valid_tick) <= 3000U;

  if (!g_rate_fast && have_fix) {
    gps_rate_request(1);                      /* first fix → tracking rate */
  } else if (g_rate_fast && !have_fix && g_latest_valid_tick != 0 &&
             (now - g_latest_valid_tick) > GPS_REACQ_DOWNSHIFT_MS) {
    gps_rate_request(0);                      /* fix long gone → re-acquire @1 Hz */
  }
}

/* Fire-and-forget MON-RF poll every GPS_RF_LOG_INTERVAL_MS, feeding the
   `gps_rf:` errlog line via parse_mon_rf. Skipped while the survey bridge
   owns the channel (the host polls MON-RF itself) and on the NMEA-fallback
   line (UBX out never landed — the poll could not be answered). A lost
   reply retries at the next interval; one plain (non-***) marker per boot
   satisfies Peter's every-unanswered-command-logs rule without spamming. */
static void gps_rf_manage(void)
{
  if (!g_rate_armed || g_bridge) return;
  uint32_t now = HAL_GetTick();
  if (g_rf_poll_tick != 0) {
    if ((now - g_rf_poll_tick) <= 2000U) return;   /* reply still pending */
    if (!g_rf_noreply_logged) {
      g_rf_noreply_logged = 1;
      ErrLog_Write("gps_rf: MON-RF poll unanswered (module without MON-RF?)");
    }
    g_rf_poll_tick = 0;
  }
  if ((int32_t)(now - g_rf_next_poll) < 0) return;
  /* 1 Hz while a BLE quiet window samples per-second RF values (v0.0.57);
     the normal 5 s cadence otherwise. */
  g_rf_next_poll = now + (g_rf_fast ? 1000U : GPS_RF_POLL_MS);
  g_rf_poll_tick = now | 1U;                       /* never 0 (= idle) */
  ubx_send(0x0A, 0x38, NULL, 0);
}

/* Mid-session link recovery (v0.0.56): the module went silent while the UBX
   path was armed. By far the most likely field cause is a supply glitch on
   the hand-soldered 3.3 V line — the module reboots into its factory
   9600/NMEA state while our UART stays at 230400, so nothing parseable
   ever arrives again (before v0.0.56 the baud was negotiated ONLY in
   GPS_Init; the box stayed deaf until a magnet power-cycle). Same
   assume-9600-then-confirm sequence as GPS_Init, minus the multi-baud
   probe: mid-session there are exactly two plausible module states
   (rebooted → 9600, alive-but-unheard → 230400); if neither answers, the
   link itself is dead and no probing helps.

   Blocking by design, like GPS_Init: worst case ~1.6 s when the module is
   gone, ~7 s if every config ACK needs its full retry budget — both under
   the 8 s IWDG (fresh-fed at the top of this superloop pass), and the
   GPS_LINK_RETRY_MS backoff keeps a permanently dead link from stalling
   the superloop more than once per interval. */
static void gps_link_recover(void)
{
  extern void ErrLog_Write(const char *msg);
  extern void ErrLog_Writef(const char *fmt, ...);

  ErrLog_Writef("*** gps: link silent %lu s — reconfiguring module ***",
                (unsigned long)((HAL_GetTick() - g_link_last_ok) / 1000U));

  uint8_t confirmed = 0;
  if (gps_uart_reopen(9600) == 0) {
    gps_wake_pulse();
    vs_begin();
    vs_u4(CFG_UART1_BAUDRATE, GPS_UART_BAUDRATE);
    ubx_send(0x06, 0x8A, g_vs_buf, g_vs_len);        /* blind, at 9600 */
    if (gps_uart_reopen(GPS_UART_BAUDRATE) == 0) {
      if (ubx_wait_ack(0x06, 0x8A, 700) == 0) {
        confirmed = 1;
        ErrLog_Write("gps: link recover — module was @9600 (module reboot)");
      } else {
        ubx_send(0x0A, 0x04, NULL, 0);               /* MON-VER poll */
        int ux = 0;
        listen_traffic(400, NULL, &ux);
        if (ux > 0) {
          confirmed = 1;
          ErrLog_Write("gps: link recover — module alive @230400");
        }
      }
    }
  }

  if (!confirmed) {
    /* Silent at both bauds: not a config problem — RX joint or supply.
       Restore the session UART (re-arms the RX IRQ) and back off. */
    if (gps_uart_reopen(g_locked_baud) == 0)
      listen_traffic(50, NULL, NULL);
    ErrLog_Write("*** gps: link recover FAIL — module silent @9600+230400 (check supply/RX joint) ***");
    g_link_next_attempt = HAL_GetTick() + GPS_LINK_RETRY_MS;
    g_link_last_ok = HAL_GetTick();
    return;
  }

  /* Module is talking again — replay the full config (a rebooted module is
     back at factory defaults; a merely-unheard one just re-ACKs) and reset
     every state machine that assumed the old session. */
  int pvt_ok = 0;
  int fails = gps_apply_config(&pvt_ok);
  g_locked_baud        = GPS_UART_BAUDRATE;
  g_rate_armed         = pvt_ok ? 1 : 0;
  g_rate_fast          = 0;
  g_rate_pending       = 0;
  g_rate_tries         = 0;
  g_rate_backoff_until = 0;
  g_rf_poll_tick       = 0;      /* any in-flight MON-RF poll is void now */
  g_link_recoveries++;
  g_link_last_ok      = HAL_GetTick();
  g_link_next_attempt = 0;
  ErrLog_Writef("gps: link recovered #%u @%lu baud%s",
                (unsigned)g_link_recoveries,
                (unsigned long)GPS_UART_BAUDRATE,
                (fails > 0) ? " (config incomplete)" : "");
}

/* Mid-session link watchdog (v0.0.56) — see GPS_LINK_* in config.h for the
   field story. Armed only on the configured UBX path (g_rate_armed): the
   NMEA-fallback line stays best-effort as before, and the GPSRAW.CFG bypass
   must never see a config write. Paused while the survey bridge owns the
   channel (the host is mid-conversation with the module; its relayed
   replies keep the stamp fresh anyway). */
static void gps_link_manage(void)
{
  if (!g_rate_armed || g_bridge) return;
  uint32_t now = HAL_GetTick();
  if (g_link_last_ok == 0) { g_link_last_ok = now; return; }
  if ((now - g_link_last_ok) < GPS_LINK_SILENCE_MS) return;
  if (g_link_next_attempt != 0 && (int32_t)(now - g_link_next_attempt) < 0) return;
  gps_link_recover();
}

/* Live RF/signal health for the SensorStream extension (v0.0.55). All
   fields zero when unknown; `fresh` is 1 only when a MON-RF reply landed
   within the last 15 s (the top-6 roll-up has its own 15 s staleness
   guard, mirroring the gps_rf errlog line). */
void GPS_GetRfLive(PL_GpsRfLive *out)
{
  if (!out) return;
  memset(out, 0, sizeof(*out));
  uint32_t now = HAL_GetTick();
  out->fix_type = g_fix_type;
  out->used_sv  = g_latest.num_sat;
  if (g_top6_n > 0 && (now - g_top6_tick) <= 15000U) {
    unsigned sum = 0;
    for (unsigned k = 0; k < g_top6_n; k++) sum += g_top6[k];
    out->avg6_x10 = (uint16_t)(sum * 10U / g_top6_n);
    out->max6     = g_top6[0];
    out->min6     = g_top6[g_top6_n - 1];
  }
  if (g_rf_seen_tick != 0 && (now - g_rf_seen_tick) <= 15000U) {
    out->noise_per_ms = g_rf_noise;
    out->agc_cnt      = g_rf_agc;
    out->jam_ind      = g_rf_jam_ind;
    out->jam_state    = g_rf_jam_state;
    out->ant_status   = g_rf_ant;
    out->fresh        = 1;
  }
}

void GPS_RfFastPoll(uint8_t on)
{
  /* BLE quiet window (v0.0.57): per-second RF samples need per-second
     MON-RF replies. Arming also makes the next poll due immediately so
     the first sample isn't up to 5 s stale. No-op on the NMEA-fallback
     line or with GPS off — gps_rf_manage keeps its own g_rate_armed
     gate and simply never polls there. */
  g_rf_fast = on ? 1 : 0;
  if (g_rf_fast) g_rf_next_poll = HAL_GetTick();
}

void GPS_Tick(void)
{
  /* GPS powered off (backup mode): no NMEA is arriving. Discard anything that
     trickled in (e.g. wake-up junk) so the ring can't wrap, and skip parsing. */
  if (g_power == 0 || g_suspended) {
    g_rx_tail = g_rx_head;
    /* Slide the link-watchdog stamp: a deliberately sleeping module is not
       a dead link, and the silence window must not start until power-on. */
    g_link_last_ok = HAL_GetTick();
    return;
  }

  /* Drain the IRQ-filled ring buffer through the protocol router (UBX frames
     → NAV-PVT/NAV-SAT parsers; everything else → NMEA line assembler for the
     fallback mode). While the BLE survey bridge is active, the same bytes
     also feed the UBX frame extractor so poll replies get relayed — the
     local parsing is unaffected. */
  while (g_rx_head != g_rx_tail) {
    uint8_t b = g_rx_ring[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
    gps_rx_byte(b);
    if (g_bridge) bridge_capture(b);
    g_diag_bytes++;
  }

  gps_rate_manage();
  gps_rf_manage();
  gps_link_manage();
}

void GPS_GetStats(uint32_t *bytes, uint32_t *lines_good, uint32_t *lines_bad,
                  uint32_t *rmc, uint32_t *gga, uint32_t *errors,
                  uint32_t *rx_dropped, uint32_t *ubx_dropped)
{
  if (bytes)       *bytes       = g_diag_bytes;
  if (lines_good)  *lines_good  = g_diag_lines_good;
  if (lines_bad)   *lines_bad   = g_diag_lines_bad;
  if (rmc)         *rmc         = g_diag_rmc;
  if (gga)         *gga         = g_diag_gga;
  if (errors)      *errors      = g_diag_errors;
  /* rx_dropped: STM32 RX ring (GPS_RX_RING_SIZE, 2 KB) overflowed (main
     loop fell behind).
     ubx_dropped: 2048 B UBX relay ring overflowed (BLE drain fell behind).
     Both near 0 while the survey still drops replies ⇒ the MODULE never sent
     them ⇒ UART bandwidth saturation (baud/NMEA), not an STM32-side loss. */
  if (rx_dropped)  *rx_dropped  = g_rx_dropped;
  if (ubx_dropped) *ubx_dropped = g_ubx_dropped;
}

pl_gps_quality_t GPS_LastFixQuality(void)
{
  if (g_latest_valid_tick == 0) return PL_GPS_QUALITY_NONE;
  /* Staleness: if no fresh status='A' RMC in the last 3 s, treat as
     no-fix (signal lost). At 10 Hz fix rate that's 30 missed RMCs —
     conservatively long enough to ride out one or two cycle drops
     without flickering the LED pattern. */
  if ((HAL_GetTick() - g_latest_valid_tick) > 3000U) return PL_GPS_QUALITY_NONE;
  if (g_latest.num_sat >= 7) return PL_GPS_QUALITY_GOOD;
  if (g_latest.num_sat >= 4) return PL_GPS_QUALITY_WEAK;
  return PL_GPS_QUALITY_NONE;
}

int GPS_GetLatestFix(PL_GpsFix *out)
{
  if (!out) return -1;
  *out = g_latest;
  return g_latest.valid ? 0 : -1;
}

uint8_t GPS_FixUpdated(void)
{
  uint8_t r = g_updated;
  g_updated = 0;
  return r;
}
