/**
  ******************************************************************************
  * @file    usb_descriptors.c
  * @brief   USB descriptors for PumpLogger MSC device.
  *
  *          Single MSC interface — the microSD exposed as a USB bulk-only
  *          mass-storage drive. Phase 9 / Issue #5.
  ******************************************************************************
  */
#include <string.h>
#include "tusb.h"

/* USB Vendor / Product IDs.
 *
 * Using TinyUSB's pid.codes test VID/PID:
 *   0xCAFE / 0x4002   ← MSC variant (CDC variant was 0x4001 in the
 *                       fp-sns-stbox1 debug build; bumped to 0x4002 here
 *                       so a host that has cached the CDC descriptors
 *                       doesn't reuse them.)
 *
 * For a production product release these should be replaced with a real
 * ywesee-assigned VID/PID. Both Linux and macOS auto-attach the kernel
 * MSC driver regardless of VID; Windows ≥7 too. So enumeration works
 * everywhere with these placeholders.
 */
#define USB_VID   0xCAFEu
#define USB_PID   0x4002u
#define USB_BCD   0x0200u  /* USB 2.0 */

/* ---------------------------------------------------------------------------
 * Device descriptor — pure MSC (not composite, no IAD).
 * --------------------------------------------------------------------------*/
tusb_desc_device_t const desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  /* Class info defined per-interface (MSC), not at device level. */
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = USB_VID,
  .idProduct          = USB_PID,
  .bcdDevice          = 0x0100,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

/* ---------------------------------------------------------------------------
 * Configuration descriptor (one MSC interface)
 * --------------------------------------------------------------------------*/
enum {
  ITF_NUM_MSC = 0,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

#define EPNUM_MSC_OUT     0x01
#define EPNUM_MSC_IN      0x81

uint8_t const desc_fs_configuration[] = {
  /* Config: itf count, string idx, total len, attr (0x80 = bus-powered),
     max power 100 mA. */
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

  /* MSC: itf, string idx, EP OUT, EP IN, EP size (64 B FS). */
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 4, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_fs_configuration;
}

/* ---------------------------------------------------------------------------
 * String descriptors
 * --------------------------------------------------------------------------*/
char const * string_desc_arr[] = {
  (const char[]){ 0x09, 0x04 }, /* 0: language id (English US, 0x0409) */
  "ywesee GmbH",                /* 1: Manufacturer */
  "PumpLogger SD",              /* 2: Product */
  "0123456789ABCDEF",           /* 3: Serial — placeholder, not per-chip
                                 *    unique yet. Host doesn't need a real
                                 *    one unless multiple boxes are mounted
                                 *    at once. */
  "PumpLogger MSC",             /* 4: MSC interface name */
};

static uint16_t _desc_str[32];

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;

    const char *str = string_desc_arr[index];
    chr_count = (uint8_t) strlen(str);
    if (chr_count > 31) chr_count = 31;

    /* Convert ASCII string into UTF-16LE. */
    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = str[i];
    }
  }

  /* First byte = total length, second byte = type (string descriptor = 0x03). */
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2u * chr_count + 2u);
  return _desc_str;
}
