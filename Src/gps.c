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

  /* Disable noisy NMEA sentences (GLL/GSA/GSV/VTG). */
  const uint8_t off[4][2] = { {0xF0,0x01}, {0xF0,0x02}, {0xF0,0x03}, {0xF0,0x05} };
  int msg_acks = 0;
  for (int i = 0; i < 4; i++) {
    uint8_t p[3] = { off[i][0], off[i][1], 0 };
    if (ubx_send_retry(0x06, 0x01, p, sizeof(p), 3) == 0) msg_acks++;
  }
  ErrLog_Writef("gps: cfg-msg disable %d/4 ACK'd", msg_acks);

  /* CFG-CFG-SAVE — persist baud + rate + msg config to BBR + Flash +
     EEPROM so the next boot starts at the configured baud and Build #44's
     listen-first lock happens on the first try (no 1.5 s fallback). */
  uint8_t save_p[13] = {0};
  save_p[4] = 0xFF; save_p[5] = 0xFF;
  save_p[12] = 0x17;
  int save_rc = ubx_send_retry(0x06, 0x09, save_p, sizeof(save_p), 3);
  ErrLog_Writef("gps: cfg-cfg-save %s", (save_rc == 0) ? "ACK" : "FAIL");

  /* RX IRQ stays armed across all the listen/ack calls. */
  ErrLog_Writef("gps: ready @%lu baud, rate=%luHz",
                (unsigned long)locked_baud,
                (unsigned long)(1000U / meas_ms));
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
  /* Drain the IRQ-filled ring buffer through the NMEA parser. */
  while (g_rx_head != g_rx_tail) {
    uint8_t b = g_rx_ring[g_rx_tail];
    g_rx_tail = (uint16_t)((g_rx_tail + 1u) % GPS_RX_RING_SIZE);
    process_byte(b);
    g_diag_bytes++;
  }
}

void GPS_GetStats(uint32_t *bytes, uint32_t *lines_good, uint32_t *lines_bad,
                  uint32_t *rmc, uint32_t *gga, uint32_t *errors)
{
  if (bytes)      *bytes      = g_diag_bytes;
  if (lines_good) *lines_good = g_diag_lines_good;
  if (lines_bad)  *lines_bad  = g_diag_lines_bad;
  if (rmc)        *rmc        = g_diag_rmc;
  if (gga)        *gga        = g_diag_gga;
  if (errors)     *errors     = g_diag_errors;
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
