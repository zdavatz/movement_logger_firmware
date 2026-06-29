/**
  ******************************************************************************
  * @file    usb_host_probe.h
  * @brief   USB host-mode connect probe — diagnostic build only.
  *
  *          Answers one empirical question that the docs can only predict:
  *          "When a USB device is plugged into the box's USB-C port, does the
  *          STM32U585 OTG_FS controller see it on the bus?"
  *
  *          This is NOT a USB host stack. It does no enumeration, no
  *          transfers, no class drivers. It forces OTG_FS into host mode,
  *          powers the port, disables VBUS sensing (the board has no
  *          VBUS-sense pin on the host path), and then POLLS the host port
  *          control/status register (HPRT) for the port-connect-status bit
  *          (PCSTS). A device only pulls a data line — and thus sets PCSTS —
  *          when it is *powered*. So the probe simultaneously tests two
  *          things:
  *
  *            1. Does the box source VBUS?  Plug the dongle straight into the
  *               box. If PCSTS never asserts → the dongle is unpowered → the
  *               box does not source VBUS (matches REQUIREMENTS.md F-USB-3:
  *               the USB-C port is a sink/device port). Confirms, on real
  *               silicon, that "plug GPS into the box" cannot work without
  *               external power.
  *
  *            2. Does the host *data* path work?  Plug the dongle through a
  *               powered USB hub / Y-cable that injects VBUS while passing
  *               D+/D- to the box. If PCSTS now asserts → the box's host
  *               controller and data lines are fine and only power was
  *               missing → green light for a full "Phase 10" host stack with
  *               external VBUS injection.
  *
  *          Reporting is via the buzzer (immediate, no host needed) and the
  *          SD errlog (full HPRT register trace for read-back):
  *            - init:      4 quick low beeps  = "HOST-PROBE build is running"
  *            - connect:   3 rapid high beeps = "DEVICE DETECTED"
  *            - removal:   1 low beep
  *          The errlog gets a 1 Hz HPRT dump for the first 30 s, then 5 s.
  *
  *          Build:  make USB_HOST_PROBE=1   (production MSC build is the
  *          default, USB_HOST_PROBE=0, and is completely untouched —
  *          main.c selects at compile time on PL_USB_HOST_PROBE).
  *
  *          Polling-only: no OTG_FS interrupt is armed in this mode, so the
  *          F-ARCH-7 OTG_FS_IRQ exception does not even apply here. The probe
  *          runs entirely from the main loop, in keeping with the firmware's
  *          polling-only rule.
  ******************************************************************************
  */
#ifndef PL_USB_HOST_PROBE_H
#define PL_USB_HOST_PROBE_H

#include <stdbool.h>

/* Bring up OTG_FS in forced-host mode, power the port, start polling.
   Returns false if the OTG core never came out of soft-reset (clock/power
   not configured) — same failure surface as UsbMsc_Init. */
bool UsbHostProbe_Init(void);

/* Poll HPRT once. Call every main-loop iteration. Emits buzzer + errlog on
   connect/remove edges and periodic register dumps. Cheap (a few register
   reads) on the steady-state path. */
void UsbHostProbe_Tick(void);

/* True once a device connect has ever been seen since boot. */
bool UsbHostProbe_EverSawDevice(void);

#endif /* PL_USB_HOST_PROBE_H */
