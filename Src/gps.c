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

/* Persisted GPS power state (battery-save). GPSPWR.CFG on the SD root: first
   byte 'f'/'F' ("off") = backup mode, anything else = on. g_power is the
   cache: -1 = not read yet, 0 = off (in backup), 1 = on. GPS_Tick early-exits
   while off so no stale NMEA is parsed. */
#define GPSPWR_CFG_NAME "GPSPWR.CFG"
static int g_power = -1;

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
  HAL_UART_Transmit(&g_huart4, hdr, 6, 200);
  if (len) HAL_UART_Transmit(&g_huart4, (uint8_t *)payload, len, 200);
  HAL_UART_Transmit(&g_huart4, ck, 2, 200);
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
  for (uint16_t i = 0; i < n; i++) {
    uint8_t cno = p[8 + 12 * i + 2];
    if (cno == 0) continue;                  /* in view but not tracked */
    if (cno > 99) cno = 99;
    if (nsig < 255) nsig++;
    if (cno > cn0max) cn0max = cno;
  }
  g_gsv_cn0_max   = cn0max;
  g_gsv_nsat_sig  = nsig;
  g_gsv_burst     = 1;
  g_gsv_seen_tick = HAL_GetTick();
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
        if (!g_ux_skip) {
          if      (g_ux_cls == 0x01 && g_ux_id == 0x07) parse_nav_pvt(g_ux_pay, g_ux_len);
          else if (g_ux_cls == 0x01 && g_ux_id == 0x35) parse_nav_sat(g_ux_pay, g_ux_len);
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
  } else {
    g_bridge = 0;
    /* Restore the 10 Hz PVT stream. RAM-only, best-effort — a power cycle
       would clear it anyway. */
    int rc = gps_cfg_valset_bool(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
    ErrLog_Writef("gps: bridge off @%lu baud (pvt restore %s)",
                  (unsigned long)g_locked_baud, (rc == 0) ? "ACK" : "FAIL");
  }
}

uint8_t GPS_BridgeActive(void) { return g_bridge; }

void GPS_BridgeTx(const uint8_t *data, uint16_t len)
{
  if (!data || !len) return;
  HAL_UART_Transmit(&g_huart4, (uint8_t *)data, len, 200);
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
          if (rcls == cls && b == id) return ack_kind ? 0 : -2;
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
  HAL_UART_Transmit(&g_huart4, junk, sizeof(junk), 200);
  HAL_Delay(120);
}

int GPS_Init(void)
{
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
      fails++;
      final_baud = found;
      ErrLog_Writef("*** gps: cfg baud %lu NOT confirmed — staying @%lu ***",
                    (unsigned long)GPS_UART_BAUDRATE, (unsigned long)found);
      if (gps_uart_reopen(found) != 0) {
        ErrLog_Write("gps: uart_reinit@fallback FAIL");
        return -1;
      }
      listen_traffic(200, NULL, NULL);             /* re-arm the RX IRQ */
    }
  }

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

  /* Arm the fix-driven 1↔10 Hz switcher only when the configured UBX path
     is actually up — on the NMEA-fallback/slow line the rate must stay 1 Hz
     (and un-ACK'd VALSETs would only spam the errlog). */
  g_rate_armed = (final_baud == GPS_UART_BAUDRATE && pvt_rc == 0) ? 1 : 0;
  g_rate_fast  = 0;
  g_rate_pending = 0;
  g_rate_tries = 0;
  g_rate_backoff_until = 0;

  g_locked_baud = final_baud;

  /* RX IRQ stays armed across all the listen/ack calls. */
  ErrLog_Writef("gps: ready @%lu baud, nav=1Hz acq%s, %s%s",
                (unsigned long)final_baud,
                g_rate_armed ? " (10Hz after first fix)" : "",
                (pvt_rc == 0) ? "UBX NAV-PVT" : "NMEA fallback",
                (fails > 0) ? " (config incomplete)" : "");

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

  if (on) {
    /* Wake only if it was actually asleep. The RAM config survives software
       backup, so a wake resumes UBX at the locked baud with no reconfigure. */
    if (was == 0) {
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

void GPS_Tick(void)
{
  /* GPS powered off (backup mode): no NMEA is arriving. Discard anything that
     trickled in (e.g. wake-up junk) so the ring can't wrap, and skip parsing. */
  if (g_power == 0) {
    g_rx_tail = g_rx_head;
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
