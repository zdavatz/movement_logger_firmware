/**
  ******************************************************************************
  * @file    gps.c
  * @brief   u-blox MAX-M10S NMEA parser on UART4 — byte-by-byte IRQ RX.
  *
  *          The DMA approach (Builds #1-#9) proved unreliable: after a single
  *          framing error UART4 latched and DMA stayed stuck (ERRLOG showed
  *          bytes=1 over 6 minutes). The IRQ approach is lifted from the
  *          original SDDataLogFileX::gps_nmea.c — one byte per IRQ, push to
  *          ring buffer, re-arm; error callback clears flags and re-arms.
  *          GPS_Tick drains the ring at main-loop priority and runs the
  *          NMEA line-assembly + parser (atof/strtod heavy work) outside
  *          IRQ context.
  *
  *          IRQ load at 38400 baud ≈ 4800 bytes/s = one IRQ every ~210 µs,
  *          each <10 µs → ~5% CPU. Bounded; F-ARCH-6 exception explicitly
  *          documented in stm32u5xx_it.c.
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

#define GPS_RX_RING_SIZE  512
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
static volatile uint32_t g_latest_valid_tick;   /* HAL_GetTick() of last status='A' RMC; 0 = never */
static volatile uint32_t g_diag_gga;
static volatile uint32_t g_diag_errors;        /* UART RX error callbacks invoked */

/* Baud the module locked to in GPS_Init — needed by the bridge's $PUBX,41
   reconfigure (it must keep the same baud, only flip the output protocol). */
static uint32_t g_locked_baud = GPS_UART_BAUDRATE;

/* GSV emit rate chosen by GPS_Init ("every Nth nav epoch"; 0 = off). 10 at
   38400/10 Hz = one burst/s for the C/N0 telemetry; 0 at the 9600/5 Hz
   fallback (no UART headroom for GSV bursts there). The survey's NMEA
   trim/restore (gps_survey_nmea) restores GSV to this rate, not to the
   module default of every-epoch. */
static uint8_t g_gsv_rate;

/* Persisted GPS power state (battery-save). GPSPWR.CFG on the SD root: first
   byte 'f'/'F' ("off") = backup mode, anything else = on. g_power is the
   cache: -1 = not read yet, 0 = off (in backup), 1 = on. GPS_Tick early-exits
   while off so no stale NMEA is parsed. */
#define GPSPWR_CFG_NAME "GPSPWR.CFG"
static int g_power = -1;

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

static void gps_cfg_port_uart1(uint32_t baud)
{
  uint8_t p[20] = {0};
  p[0]  = 0x01;
  p[4]  = 0xC0; p[5] = 0x08;
  p[8]  = (uint8_t)(baud);
  p[9]  = (uint8_t)(baud >> 8);
  p[10] = (uint8_t)(baud >> 16);
  p[11] = (uint8_t)(baud >> 24);
  p[12] = 0x03;
  p[14] = 0x02;
  ubx_send(0x06, 0x00, p, sizeof(p));
}

/* ---------- UBX-CFG-VALSET: M10-native protocol enable ------------------ */

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
   off). At 9600 baud the UART is oversubscribed by 5 Hz NMEA, so the survey's
   UBX poll replies (NAV-PVT / MON-RF) were delayed or dropped — the "ant=?/?"
   / "used=0" survey gaps. While the bridge is active we silence the heavy /
   non-essential sentences (GSV is the big one — multi-sentence per satellite)
   to free bandwidth for the UBX answers; GGA + RMC stay on for the SD logger's
   fix. Bridge off restores the post-GPS_Init state (GSV at g_gsv_rate, rest
   off) — see gps_survey_nmea. RAM-only, ACK-verified via gps_cfg_valset_bool
   (its 1-byte value matches these U1 keys). */
#define CFG_MSGOUT_NMEA_GSV_UART1  0x209100c5UL
#define CFG_MSGOUT_NMEA_GSA_UART1  0x209100c0UL
#define CFG_MSGOUT_NMEA_VTG_UART1  0x209100b1UL
#define CFG_MSGOUT_NMEA_GLL_UART1  0x209100caUL
#define CFG_MSGOUT_NMEA_ZDA_UART1  0x209100d9UL

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

/* $PUBX,41 — u-blox proprietary NMEA sentence that configures a UART
   port (baudrate + inProtoMask + outProtoMask). Critically, this
   travels as NMEA, so it's accepted by the module even when its
   persisted inProtoMask is set to NMEA-only (= our previous Build #45
   trap, where every UBX command silently dropped). After this lands,
   the module is at the new baud with UBX input re-enabled, and all
   subsequent UBX commands ACK normally.

   Wire format: $PUBX,41,1,0003,0002,<baud>,0*<XOR>\r\n
     port 1     = UART
     inMask 0x0003 = UBX + NMEA
     outMask 0x0002 = NMEA only
     autobaud 0  = off
   The XOR checksum runs over everything between '$' and '*'. */
static void send_pubx_port_cfg(uint32_t baud_out)
{
  char body[64];
  int n = snprintf(body, sizeof(body), "PUBX,41,1,0003,0002,%lu,0",
                   (unsigned long)baud_out);
  if (n <= 0) return;
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)body[i];
  char full[80];
  int m = snprintf(full, sizeof(full), "$%s*%02X\r\n", body, cs);
  if (m > 0) HAL_UART_Transmit(&g_huart4, (uint8_t *)full,
                               (uint16_t)m, 200);
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

/* GSV per-satellite C/N0 roll-up. GSV arrives as a multi-sentence burst (one
   group per constellation); we accumulate the strongest C/N0 and the count of
   tracked (C/N0-bearing) satellites, then parse_gga commits them to g_latest
   at the GGA boundary. Since v0.0.38 GSV is throttled to ~1 burst/s while GGA
   runs at 10 Hz, so most GGA epochs see no fresh burst — parse_gga holds the
   last committed value between bursts and only decays to 0 ("no data") after
   GSV_STALE_MS of GSV silence (legacy module with GSV off, module gone quiet).
   Parsing is RX-only — works even when the box->module command line is dead
   (the exact case that blocks the UBX survey). */
static uint8_t  g_gsv_cn0_max;    /* strongest C/N0 since the last commit (dB-Hz) */
static uint8_t  g_gsv_nsat_sig;   /* satellites with a C/N0 since the last commit */
static uint8_t  g_gsv_burst;      /* ≥1 GSV sentence parsed since the last commit */
static uint32_t g_gsv_seen_tick;  /* HAL_GetTick() of the last GSV sentence */
#define GSV_STALE_MS 3000U

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
  /* Commit the C/N0 rolled up from the GSV bursts. GSV is throttled to
     ~1 burst/s (v0.0.38) while GGA arrives at 10 Hz, so most epochs carry no
     fresh burst — hold the last committed value instead of flapping to 0, and
     decay to 0 ("no data") only after GSV_STALE_MS of GSV silence. A burst
     whose satellites all lack a C/N0 commits a genuine 0/0. */
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

/* ---------- BLE GPS bridge ---------------------------------------------- */

/* $PUBX,41 to flip the port's output protocol mask while keeping baud and
   input mask. `ubx_on` → outMask 0x0003 (UBX+NMEA) so the survey's poll
   replies are emitted alongside the NMEA the SD logger needs; off → 0x0002
   (NMEA only), the normal logging state. Travels as NMEA so it lands even if
   UBX input were ever disabled (same rationale as send_pubx_port_cfg). */
static void gps_set_ubx_output(uint8_t ubx_on)
{
  const char *outmask = ubx_on ? "0003" : "0002";
  char body[64];
  int n = snprintf(body, sizeof(body), "PUBX,41,1,0003,%s,%lu,0",
                   outmask, (unsigned long)g_locked_baud);
  if (n <= 0) return;
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)body[i];
  char full[80];
  int m = snprintf(full, sizeof(full), "$%s*%02X\r\n", body, cs);
  if (m > 0) HAL_UART_Transmit(&g_huart4, (uint8_t *)full, (uint16_t)m, 200);
}

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

/* Silence (on=0) or restore (on=1) the heavy NMEA sentences on UART1 so the
   survey's UBX poll replies aren't starved by the NMEA stream. GGA + RMC are
   left untouched for the SD logger's fix. Restore means the post-GPS_Init
   state — GSV at its throttled g_gsv_rate, GSA/VTG/GLL/ZDA off — NOT the old
   uniform rate-1, which re-enabled sentences the logger never wants and
   oversubscribed the UART at 10 Hz nav until the next module power cycle. */
static void gps_survey_nmea(uint8_t on)
{
  (void)gps_cfg_valset_bool(CFG_MSGOUT_NMEA_GSV_UART1, on ? g_gsv_rate : 0);
  (void)gps_cfg_valset_bool(CFG_MSGOUT_NMEA_GSA_UART1, 0);
  (void)gps_cfg_valset_bool(CFG_MSGOUT_NMEA_VTG_UART1, 0);
  (void)gps_cfg_valset_bool(CFG_MSGOUT_NMEA_GLL_UART1, 0);
  (void)gps_cfg_valset_bool(CFG_MSGOUT_NMEA_ZDA_UART1, 0);
}

void GPS_BridgeSet(uint8_t on)
{
  extern void ErrLog_Writef(const char *fmt, ...);
  if (on) {
    /* Fresh capture + relay state for this survey. */
    g_cap_st = 0; g_cap_len = 0; g_cap_got = 0;
    g_ubx_head = g_ubx_tail = 0; g_ubx_dropped = 0;
    g_bridge = 1;
    /* Step 1: $PUBX,41 (travels as NMEA) re-enables UBX *input* even if the
       persisted config locked the port to NMEA-only input — otherwise the
       CFG-VALSET below would itself be dropped by the module. */
    gps_set_ubx_output(1);
    HAL_Delay(50);
    /* Step 2: M10-native, ACK-verified — force UBX in+out ON on UART1 (RAM).
       This is the lever $PUBX,41 alone doesn't reliably pull on the M10S; the
       ACK/FAIL below in the errlog tells us definitively whether it took. */
    int in_rc  = gps_cfg_valset_bool(CFG_UART1INPROT_UBX, 1);
    int out_rc = gps_cfg_valset_bool(CFG_UART1OUTPROT_UBX, 1);
    /* Free UART bandwidth for the UBX poll replies by silencing the heavy
       NMEA sentences (GSV/GSA/VTG/GLL/ZDA) while the survey runs. */
    gps_survey_nmea(0);
    ErrLog_Writef("gps: bridge ON @%lu baud (valset in=%s out=%s, nmea trimmed)",
                  (unsigned long)g_locked_baud,
                  (in_rc == 0) ? "ACK" : "FAIL",
                  (out_rc == 0) ? "ACK" : "FAIL");
  } else {
    g_bridge = 0;
    /* Restore NMEA-only output for the SD logger. RAM-only, best-effort — a
       power cycle would clear it anyway; the ACK doesn't matter here. */
    (void)gps_cfg_valset_bool(CFG_UART1OUTPROT_UBX, 0);
    gps_set_ubx_output(0);
    gps_survey_nmea(1);   /* restore the NMEA sentences we silenced */
    ErrLog_Writef("gps: bridge off @%lu baud (nmea restored)", (unsigned long)g_locked_baud);
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

/* Listen for `ms_window` milliseconds and return the count of '\n' bytes
   that landed in the ring during the window. Used by GPS_Init to verify
   that the currently-configured UART baud matches the GPS module's
   output rate — if we see real NMEA newlines, we're decoding; if not,
   the framing is wrong. Re-arms the RX IRQ each entry so calling this
   right after a baud-rate change works. */
static int listen_newlines(uint32_t ms_window)
{
  /* Robust re-arm: after multiple DeInit/Init cycles the HAL's RxState
     can land in BUSY_RX or RESET leftovers, and a fresh HAL_UART_Receive_IT
     then returns HAL_BUSY silently. Abort first, then arm, then check.
     If arming truly fails we log it — otherwise "0 newlines" gets
     mis-attributed to "wrong baud" when the real reason was a stuck
     UART driver. */
  extern void ErrLog_Write(const char *msg);
  HAL_UART_AbortReceive_IT(&g_huart4);
  g_rx_head = g_rx_tail = 0;
  HAL_StatusTypeDef rs = HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1);
  if (rs != HAL_OK) {
    ErrLog_Write("gps: listen: HAL_UART_Receive_IT FAIL");
    return 0;
  }

  uint32_t newlines = 0;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ms_window) {
    while (g_rx_head != g_rx_tail) {
      uint8_t b = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
      if (b == '\n') newlines++;
    }
  }
  return (int)newlines;
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
   this is fire-and-forget. The module's config is retained in BBR (saved by
   the CFG-CFG-SAVE in GPS_Init), so a later wake resumes NMEA at the locked
   baud without a full reconfigure. */
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
     listen window so bytes accumulate in the ring. Priority 6 (below
     SDMMC1 at ~14 so SD writes aren't preempted; above SysTick at 15
     so a single-byte ring push finishes in <10 µs). */
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  /* Listen-first baud detection. We don't know what state the module is
     in (could be at factory 9600 or persisted at 38400 from a previous
     SDDataLogFileX session). Try the most likely first, listen for
     NMEA newlines, fall back if nothing comes through. Deliberately
     NO blind UBX-CFG-PRT switch attempt — that path silently failed
     for weeks and made post-mortems hard. */
  uint32_t locked_baud = 0;
  uint16_t meas_ms     = (uint16_t)(1000U / GPS_RATE_HZ);   /* 100 ms = 10 Hz default */

  /* Attempt 1: 38400 (our preferred rate; what the module persists to). */
  if (uart4_init_at(GPS_UART_BAUDRATE) != 0) {
    ErrLog_Write("gps: uart_init@38400 FAIL"); return -1;
  }
  g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
  __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                 | UART_CLEAR_FEF  | UART_CLEAR_PEF);
  /* Wake pulse before the first listen: if the module was left in backup by a
     previous persisted-OFF session and its VCC survived an MCU-only reset, it
     would be asleep and emit no NMEA — baud detection would then fail. A short
     junk burst wakes it (harmless when already awake); it hot-starts NMEA. */
  gps_wake_pulse();
  int nl = listen_newlines(1500);

  if (nl >= 3) {
    locked_baud = GPS_UART_BAUDRATE;
    ErrLog_Writef("gps: locked @38400 (%d newlines in 1500ms)", nl);
  } else {
    /* Attempt 2: 9600 (u-blox factory default). */
    ErrLog_Writef("gps: no NMEA @38400 (newlines=%d) — trying 9600", nl);
    HAL_UART_DeInit(&g_huart4);
    if (uart4_init_at(9600) != 0) {
      ErrLog_Write("gps: uart_init@9600 FAIL"); return -1;
    }
    g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
    __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                   | UART_CLEAR_FEF  | UART_CLEAR_PEF);
    nl = listen_newlines(1500);
    if (nl >= 3) {
      ErrLog_Writef("gps: locked @9600 (%d newlines in 1500ms), upgrading via $PUBX,41…", nl);
      /* We're at 9600 — ask the module to switch to 38400 via the
         u-blox proprietary NMEA sentence $PUBX,41. We used to send
         UBX-CFG-PRT here, but Build #45's errlog showed all UBX
         commands silently dropped — symptom of a persisted config
         with inProtoMask = NMEA only. $PUBX,41 travels as NMEA so
         it always reaches the module regardless of UBX-input state,
         and ALSO re-enables UBX input as part of its payload —
         unsticking the module from the UBX-locked-out state for
         the rest of this boot. Can't be ACK-verified because the
         baud changes mid-stream (reply comes back on the new baud
         while we're still on the old). Verify by listen at 38400. */
      send_pubx_port_cfg(GPS_UART_BAUDRATE);
      HAL_Delay(250);
      HAL_UART_DeInit(&g_huart4);
      if (uart4_init_at(GPS_UART_BAUDRATE) != 0) {
        ErrLog_Write("gps: uart_reinit@38400 FAIL after upgrade"); return -1;
      }
      g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
      __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                     | UART_CLEAR_FEF  | UART_CLEAR_PEF);
      nl = listen_newlines(1500);
      if (nl >= 3) {
        locked_baud = GPS_UART_BAUDRATE;
        ErrLog_Writef("gps: upgrade @9600 → @38400 succeeded (%d newlines)", nl);
      } else {
        /* Upgrade didn't take — fall back to 9600 / 5 Hz. */
        ErrLog_Writef("gps: upgrade failed (newlines=%d), staying @9600 / 5Hz", nl);
        HAL_UART_DeInit(&g_huart4);
        if (uart4_init_at(9600) != 0) {
          ErrLog_Write("gps: uart_reinit@9600 FAIL"); return -1;
        }
        g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
        __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                       | UART_CLEAR_FEF  | UART_CLEAR_PEF);
        listen_newlines(500);             /* re-arm IRQ + let bytes flow */
        locked_baud = 9600;
        meas_ms     = 200;                /* 5 Hz */
      }
    } else {
      ErrLog_Write("*** GPS: no baud lock — neither 38400 nor 9600 produced NMEA ***");
      ErrLog_Write("*** factory-reset the module via u-center if the baud isn't standard ***");
      return -1;
    }
  }

  /* From here on we have a confirmed baud + working RX. All subsequent
     UBX commands are ACK-verified with 3 retries each — same pattern
     SDDataLogFileX used. The status of each lands in the errlog so a
     post-mortem can tell exactly which step worked/failed. */

  /* CFG-RATE — the most important one, set first. */
  uint8_t rate_p[6];
  rate_p[0] = (uint8_t)(meas_ms & 0xFF);
  rate_p[1] = (uint8_t)(meas_ms >> 8);
  rate_p[2] = 0x01; rate_p[3] = 0x00;     /* navRate = 1 */
  rate_p[4] = 0x01; rate_p[5] = 0x00;     /* timeRef = GPS */
  int rate_rc = ubx_send_retry(0x06, 0x08, rate_p, sizeof(rate_p), 3);
  ErrLog_Writef("gps: cfg-rate %luHz %s",
                (unsigned long)(1000U / meas_ms),
                (rate_rc == 0) ? "ACK" : "FAIL");

  /* Disable noisy NMEA sentences (GLL/GSA/VTG). GSV is NOT disabled anymore —
     it feeds the v0.0.19 C/N0 telemetry (parse_gsv → cn0_max/sats_in_view),
     which the old GLL/GSA/GSV/VTG disable list starved to a permanent 0. */
  const uint8_t off[3][2] = { {0xF0,0x01}, {0xF0,0x02}, {0xF0,0x05} };
  int msg_acks = 0;
  for (int i = 0; i < 3; i++) {
    uint8_t p[3] = { off[i][0], off[i][1], 0 };
    if (ubx_send_retry(0x06, 0x01, p, sizeof(p), 3) == 0) msg_acks++;
  }
  ErrLog_Writef("gps: cfg-msg disable %d/3 ACK'd", msg_acks);

  /* GSV on, throttled (v0.0.38). Rate byte = "emit every Nth nav epoch":
     10 @ 10 Hz nav = one burst/s — plenty for antenna/signal telemetry, and
     it keeps the 38400 UART comfortably undersubscribed (a full-rate GSV
     burst per 100 ms epoch would exceed the line rate). At the 9600/5 Hz
     fallback there is no headroom at all → explicit 0, which also covers a
     factory-fresh module that boots with GSV at every epoch. */
  g_gsv_rate = (locked_baud == GPS_UART_BAUDRATE) ? 10 : 0;
  uint8_t gsv_p[3] = { 0xF0, 0x03, g_gsv_rate };
  int gsv_rc = ubx_send_retry(0x06, 0x01, gsv_p, sizeof(gsv_p), 3);
  ErrLog_Writef("gps: cfg-msg gsv rate=%u %s",
                (unsigned)g_gsv_rate, (gsv_rc == 0) ? "ACK" : "FAIL");

  /* CFG-CFG-SAVE — persist baud + rate + msg config to BBR + Flash +
     EEPROM so the next boot starts at the configured baud and Build #44's
     listen-first lock happens on the first try (no 1.5 s fallback). */
  uint8_t save_p[13] = {0};
  save_p[4] = 0xFF; save_p[5] = 0xFF;
  save_p[12] = 0x17;
  int save_rc = ubx_send_retry(0x06, 0x09, save_p, sizeof(save_p), 3);
  ErrLog_Writef("gps: cfg-cfg-save %s", (save_rc == 0) ? "ACK" : "FAIL");

  /* Remember the locked baud for the BLE bridge's $PUBX,41 reconfigure. */
  g_locked_baud = locked_baud;

  /* RX IRQ stays armed across all the listen/ack calls. */
  ErrLog_Writef("gps: ready @%lu baud, rate=%luHz",
                (unsigned long)locked_baud,
                (unsigned long)(1000U / meas_ms));

  /* Apply the persisted GPS-power choice. If the user turned GPS off to save
     battery, drop the just-configured module straight into backup mode (its
     config is now saved in BBR, so a later wake resumes cleanly). */
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
    /* Wake only if it was actually asleep. Config is retained in BBR, so a
       wake resumes NMEA at the locked baud with no reconfigure. */
    if (was == 0) {
      gps_wake_pulse();
      int nl = listen_newlines(1000);
      ErrLog_Writef("gps: power ON (wake, %d newlines)", nl);
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

void GPS_Tick(void)
{
  /* GPS powered off (backup mode): no NMEA is arriving. Discard anything that
     trickled in (e.g. wake-up junk) so the ring can't wrap, and skip parsing. */
  if (g_power == 0) {
    g_rx_tail = g_rx_head;
    return;
  }

  /* Drain the IRQ-filled ring buffer through the NMEA parser. While the BLE
     survey bridge is active, the same bytes also feed the UBX frame
     extractor so poll replies get relayed — NMEA logging is unaffected. */
  while (g_rx_head != g_rx_tail) {
    uint8_t b = g_rx_ring[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
    process_byte(b);
    if (g_bridge) bridge_capture(b);
    g_diag_bytes++;
  }
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
  /* rx_dropped: STM32 512 B RX ring overflowed (main loop fell behind).
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
