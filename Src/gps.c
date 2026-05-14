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
  /* Step 1: 9600 baud (factory default) → ask GPS to switch to 38400 */
  if (uart4_init_at(9600) != 0) { ErrLog_Write("gps: uart_init@9600 FAIL"); return -1; }
  ErrLog_Write("gps: uart@9600 ok");
  gps_cfg_port_uart1(GPS_UART_BAUDRATE);
  HAL_Delay(200);

  /* Step 2: re-init UART4 at the new baud */
  HAL_UART_DeInit(&g_huart4);
  if (uart4_init_at(GPS_UART_BAUDRATE) != 0) { ErrLog_Write("gps: uart_reinit FAIL"); return -1; }
  /* Disable the overrun detection — if our main loop falls behind on draining
     the ring, OVRDIS=1 lets the UART silently drop bytes rather than latching
     ORE and halting the next RX. */
  g_huart4.Instance->CR3 |= USART_CR3_OVRDIS;
  __HAL_UART_CLEAR_FLAG(&g_huart4, UART_CLEAR_OREF | UART_CLEAR_NEF
                                 | UART_CLEAR_FEF  | UART_CLEAR_PEF);
  ErrLog_Write("gps: uart@38400 ok");
  HAL_Delay(50);

  /* Step 3: send UBX-CFG-RATE for the requested rate. Best-effort, no ACK
     wait here (we don't have polling RX while DMA is being set up). */
  uint8_t rate_p[6];
  uint16_t meas_ms = (uint16_t)(1000U / GPS_RATE_HZ);
  rate_p[0] = (uint8_t)(meas_ms & 0xFF);
  rate_p[1] = (uint8_t)(meas_ms >> 8);
  rate_p[2] = 0x01; rate_p[3] = 0x00;   /* navRate = 1 */
  rate_p[4] = 0x01; rate_p[5] = 0x00;   /* timeRef = GPS */
  ubx_send(0x06, 0x08, rate_p, sizeof(rate_p));
  HAL_Delay(80);

  /* Disable noisy NMEA sentences. */
  const uint8_t off[4][2] = { {0xF0,0x01}, {0xF0,0x02}, {0xF0,0x03}, {0xF0,0x05} };
  for (int i = 0; i < 4; i++) {
    uint8_t p[3] = { off[i][0], off[i][1], 0 };
    ubx_send(0x06, 0x01, p, sizeof(p));
    HAL_Delay(40);
  }

  /* Persist. */
  uint8_t save_p[13] = {0};
  save_p[4] = 0xFF; save_p[5] = 0xFF;
  save_p[12] = 0x17;
  ubx_send(0x06, 0x09, save_p, sizeof(save_p));
  HAL_Delay(120);

  /* Step 4: enable UART4 IRQ at NVIC priority 6 (same as the original
     SDDataLogFileX). Below SDMMC1 (BSP default ~14) so SD writes aren't
     preempted; above SysTick (default 15) so a single-byte ring push
     finishes in <10 µs and never blocks the scheduler. */
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  /* Arm the first RX. Each completion callback re-arms. */
  if (HAL_UART_Receive_IT(&g_huart4, &g_rx_byte, 1) != HAL_OK) {
    ErrLog_Write("gps: uart_recv_it FAIL"); return -3;
  }
  ErrLog_Write("gps: rx_irq armed");
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
