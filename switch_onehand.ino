/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/


/*********************************************************************
MIT License

Copyright (c) 2023, 2024 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

/* This program converts a Logitech flight joystick to a Nintendo
 * Switch compatible gamepad. This allows a player to play two joystick
 * games using one hand. Some code is taken from an Adafruit example
 * program so Adafruit's copyright is included.
 *
 * - Device run on the native usb controller with type C USB connector.
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library
 *   with type A USB connector.
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set this to 0 for use with a Nintendo Switch.
#define USB_DEBUG 0

#if USB_DEBUG
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

#define DPAD_NORTH  (0)
#define DPAD_NE     (1)
#define DPAD_EAST   (2)
#define DPAD_SE     (3)
#define DPAD_SOUTH  (4)
#define DPAD_SW     (5)
#define DPAD_WEST   (6)
#define DPAD_NW     (7)

typedef struct {
  uint8_t x;
  uint8_t y;
} DPAD_COORDS_t;

const uint8_t AXIS_MIN = 0;
const uint8_t AXIS_MID = 127;
const uint8_t AXIS_MAX = 255;

#define DPAD_MAX (8)
const DPAD_COORDS_t DPAD_TABLE[DPAD_MAX] = {
  {AXIS_MID, AXIS_MIN}, // DPAD NORTH
  {AXIS_MAX, AXIS_MIN}, // DPAD NORTH EAST
  {AXIS_MAX, AXIS_MID}, // DPAD EAST
  {AXIS_MAX, AXIS_MAX}, // DPAD SOUTH EAST
  {AXIS_MID, AXIS_MAX}, // DPAD SOUTH
  {AXIS_MIN, AXIS_MAX}, // DPAD SOUTH WEST
  {AXIS_MIN, AXIS_MID}, // DPAD WEST
  {AXIS_MIN, AXIS_MIN}, // DPAD NORTH WEST
};

uint8_t DPAD_Y(uint8_t hat) {
  if (hat < DPAD_MAX) {
    return DPAD_TABLE[hat].y;
  }
  return AXIS_MID;
}

uint8_t DPAD_X(uint8_t hat) {
  if (hat < DPAD_MAX) {
    return DPAD_TABLE[hat].x;
  }
  return AXIS_MID;
}

// Logitech Extreme 3D Pro flight joystick report layout
// Large joystick X, Y, Z (twist) axes
// 16 buttons
// 8 way hat switch
// throttle slider
typedef struct __attribute__ ((packed)) {
  uint32_t x : 10;      // 0..512..1023
  uint32_t y : 10;      // 0..512..1023
  uint32_t hat : 4;
  uint32_t twist : 8;   // 0..127..255
  uint8_t buttons_a;
  uint8_t slider;       // 0..255
  uint8_t buttons_b;
} Logitech_E3DP_t ;

typedef struct {
  const uint16_t  DEADZONE;
  const uint16_t  MIN;
  const uint16_t  CENTER;
  const uint16_t  MAX;
} Axis_Control_t;

// Logitech Extreme 3D Pro flight joystick state
typedef struct {
  Logitech_E3DP_t report;
  const uint16_t USB_VID = 0x046d;
  const uint16_t USB_PID = 0xc215;
  const Axis_Control_t X_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Y_Control = { 8, 0, 511, 1023 };
  const Axis_Control_t Twist_Control = { 4, 0, 127, 255 };
  const Axis_Control_t Speed_Control = { 4, 0, 127, 255 };
  uint8_t speed = 255;
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} Logitech_E3DP_state_t;

volatile Logitech_E3DP_state_t Le3dp;

// pio-usb is required for rp2040 usb host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"
#include "switch_tinyusb.h"

// USB Device gamepad
Adafruit_USBD_HID G_usb_hid;
NSGamepad Gamepad(&G_usb_hid);

// USB Host object for Logitech joystick
Adafruit_USBH_Host USBHost;

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif
#if USB_DEBUG
  Serial.begin(115200);
#else
  Serial.end();     // Remove CDC ACM port
#endif
  TinyUSBDevice.setID(0x0f0d, 0x00ee);
  Gamepad.begin();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif

  DBG_println("Switch TinyUSB Gamepad mounted");
}

void print_LE3DP_controls()
{
  DBG_printf("X:%d,Y:%d,", Le3dp.report.x, Le3dp.report.y);
  DBG_printf("hat:%d,", Le3dp.report.hat);
  DBG_printf("twist:%d,", Le3dp.report.twist);
  DBG_printf("slider:%d,", Le3dp.report.slider);
  DBG_printf("buttons_a:0x%x,", Le3dp.report.buttons_a);
  DBG_printf("buttons_b:0x%x", Le3dp.report.buttons_b);
  DBG_println();
}

// Swap buttons 0 and 2 so the joystick trigger maps to the gamepad A button.
uint16_t remap(uint16_t buttons) {
  uint16_t b0 = buttons & (1<<0);
  uint16_t b2 = (buttons & (1<<2));
  buttons &= ~5;
  buttons |= (b2 >> 2) | (b0 << 2);
  return buttons;
}

void loop() {
  if (Le3dp.connected) {
    if (Le3dp.available) {
      if (sizeof(Le3dp.report) == Le3dp.report_len) {
        if (Le3dp.debug) {
          uint8_t *rpt = (uint8_t *)&Le3dp.report;
          DBG_printf("LE3DP report(%d): ", Le3dp.report_len);
          for (uint16_t i = 0; i < Le3dp.report_len; i++) {
            DBG_printf("0x%02X ", rpt[i]);
          }
          DBG_println();
        }
        // Remote wakeup
        if ( TinyUSBDevice.suspended() ) {
          // Wake up host if we are in suspend mode
          // and REMOTE_WAKEUP feature is enabled by host
          TinyUSBDevice.remoteWakeup();
        }
        if (Gamepad.ready()) {
          volatile Logitech_E3DP_t *rpt = &Le3dp.report;
          static uint16_t old_buttons = 0;
          uint16_t buttons = (rpt->buttons_b << 8) | rpt->buttons_a;
          int left_x_joystick = map(rpt->x, Le3dp.X_Control.MIN,
              Le3dp.X_Control.MAX, 0, 255);
          int left_y_joystick = map(rpt->y, Le3dp.Y_Control.MIN,
              Le3dp.Y_Control.MAX, 0, 255);
          int right_x_joystick = rpt->twist;
          int right_y_joystick = DPAD_Y(rpt->hat);
          if ((abs(left_x_joystick - 127) > Le3dp.X_Control.DEADZONE) ||
              (abs(left_y_joystick - 127) > Le3dp.Y_Control.DEADZONE) ||
              (abs(right_x_joystick - 127) > Le3dp.X_Control.DEADZONE) ||
              (abs(right_y_joystick - 127) > Le3dp.Y_Control.DEADZONE) ||
              (buttons != old_buttons)) {
            if (Le3dp.debug) {
              DBG_printf("x=%4d, y=%4d, hat=%2d, twist=%3d, "
                  "buttons_a=0x%02x, slider=%3d, buttons_b=0x%02x\r\n",
                  rpt->x, rpt->y, rpt->hat, rpt->twist,
                  rpt->buttons_a, rpt->slider, rpt->buttons_b);
              DBG_printf("left_x=%6d, left_y=%6d, right_x=%6d, right_y=%6d\r\n",
                  left_x_joystick, left_y_joystick,
                  right_x_joystick, right_y_joystick);
            }
            Gamepad.leftXAxis(left_x_joystick);
            Gamepad.leftYAxis(left_y_joystick);
            Gamepad.rightXAxis(right_x_joystick);
            Gamepad.rightYAxis(right_y_joystick);
            Gamepad.buttons(remap(buttons));
            Gamepad.loop();
            old_buttons = buttons;
          }
        }
      }
      Le3dp.available = false;
    }
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
#if USB_DEBUG
    while (!Serial) { delay(1); }
#endif
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while(1) delay(1);
  }

#ifdef PIN_PIO_USB_HOST_VBUSEN
  pinMode(PIN_PIO_USB_HOST_VBUSEN, OUTPUT);
  digitalWrite(PIN_PIO_USB_HOST_VBUSEN, PIN_PIO_USB_HOST_VBUSEN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1()
{
  USBHost.task();
}

extern "C" {

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  if ((vid == Le3dp.USB_VID) && (pid == Le3dp.USB_PID)) {
    DBG_printf("Logitech Extreme 3D Pro (%d) connected\r\n", instance);
    Le3dp.connected = true;
    Le3dp.available = false;
    Le3dp.dev_addr = dev_addr;
    Le3dp.instance = instance;
    memset((Logitech_E3DP_t *)&Le3dp.report, 0, sizeof(Le3dp.report));
  }
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if ((Le3dp.dev_addr == dev_addr) && (Le3dp.instance == instance)) {
    if (Le3dp.connected) {
      Le3dp.connected = false;
      Le3dp.available = false;
      DBG_printf("Logitech Extreme 3D Pro (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (Le3dp.connected && (Le3dp.dev_addr == dev_addr) && (Le3dp.instance == instance)) {
    memcpy((Logitech_E3DP_t *)&Le3dp.report, report, min(sizeof(Le3dp.report), len));
    Le3dp.report_len = len;
    Le3dp.available = true;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

} // extern "C"
