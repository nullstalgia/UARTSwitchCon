//
//  BlueCubeMod Firmware
//
//
//  Created by Nathan Reeves 2019
//
//  Expanded by nullstalgia 2020-2022
//
// Main clock 80MHz

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rmt_reg.h"

#define LED_GPIO 12
#define PIN_SEL (1ULL << LED_GPIO)

#define PRO_CON 0x03
#define JOYCON_L 0x01
#define JOYCON_R 0x02

#define CONTROLLER_TYPE JOYCON_L

// Buttons and sticks
#define A_DPAD_CENTER 0x08
#define A_DPAD_U 0x00
#define A_DPAD_U_R 0x01
#define A_DPAD_R 0x02
#define A_DPAD_D_R 0x03
#define A_DPAD_D 0x04
#define A_DPAD_D_L 0x05
#define A_DPAD_L 0x06
#define A_DPAD_U_L 0x07
// From least to most significant bits:
// (Right)  Y, X, B, A, SR, SL, R, ZR
static uint8_t but1_send = 0;
// (Shared)  -, +, Rs, Ls, H, Cap, --, Charging Grip
static uint8_t but2_send = 0;
// (Left)  D, U, R, L, SR, SL, L, ZL
static uint8_t but3_send = 0;
static uint8_t lx_send = 128;
static uint8_t ly_send = 128;
static uint8_t cx_send = 128;
static uint8_t cy_send = 128;

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t spiSemaphore;
bool connected = false;
int paired = 0;
TaskHandle_t ButtonsHandle = NULL;
TaskHandle_t SendingHandle = NULL;
TaskHandle_t BlinkHandle = NULL;
// Timer has +1 added to it every send cycle
// Apparently, it can be used to detect packet loss/excess latency
uint8_t timer = 0;

typedef enum {
  SYNCED,
  SYNC_START,
  SYNC_1,
  OUT_OF_SYNC,
  CHOCO_SYNC_1,
  CHOCO_SYNCED
} State_t;

typedef enum {
  COMMAND_NOP = 0,
  COMMAND_SYNC_1 = 0x33,
  COMMAND_SYNC_2 = 0xCC,
  COMMAND_SYNC_START = 0xFF,
  COMMAND_CHOCO_SYNC_1 = 0x44,
  COMMAND_CHOCO_SYNC_2 = 0xEE
} Command_t;

typedef enum {
  RESP_USB_ACK = 0x90,
  RESP_UPDATE_ACK = 0x91,
  RESP_UPDATE_NACK = 0x92,
  RESP_SYNC_START = 0xFF,
  RESP_SYNC_1 = 0xCC,
  RESP_SYNC_OK = 0x33,
  RESP_CHOCO_SYNC_1 = 0xEE,
  RESP_CHOCO_SYNC_OK = CONTROLLER_TYPE
} Response_t;

#define UART_TXD_PIN \
  (UART_PIN_NO_CHANGE)  // When UART2, TX GPIO_NUM_19, RX GPIO_NUM_26
#define UART_RXD_PIN \
  (UART_PIN_NO_CHANGE)  // When UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
#define UART_NUM (UART_NUM_0)

uart_config_t uart_config;

QueueHandle_t uart_queue;

#define BUF_SIZE (256)

uint8_t* uart_data;

State_t uart_state = OUT_OF_SYNC;

// https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html
uint8_t crc8_ccitt_update(uint8_t inCrc, uint8_t inData) {
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for (i = 0; i < 8; i++) {
    if ((data & 0x80) != 0) {
      data <<= 1;
      data ^= 0x07;
    } else {
      data <<= 1;
    }
  }
  return data;
}

void uart_init() {
  uart_config.baud_rate = 19200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10,
                                      &uart_queue, 0));

  uart_data = (uint8_t*)malloc(BUF_SIZE);
}

// Send a byte over UART without echoing it to ESP LOG
void send_byte_(uint8_t data) {
  char buffer[2];
  sprintf(buffer, "%c", data);
  uart_write_bytes(UART_NUM, buffer, 1);
}

void send_byte(uint8_t data) {
  send_byte_(data);
  ESP_LOGI("uart out", "%X", data);
}

static void uart_task() {
  ESP_LOGI("hi", "Hi from core %d!. uart_task\n", xPortGetCoreID());
  // uart_flush(UART_NUM);
  while (1) {
    int len = uart_read_bytes(UART_NUM, uart_data, BUF_SIZE, portTICK_RATE_MS);

    if (len > 0) {
      ESP_LOGI("uart in", "length: %d - %X", len, uart_data[0]);
      // Packet length is supposed to be 9 bytes
      // Sometimes, however, two packets can be combined due to the nature of
      // uart_read_bytes. Theoretically, I could have my own ISR for UART
      // recieving, but I'm choosing to keep it this way for simplicities sake.
      // This may change in the future, but is currently unlikely.
      if (len >= 9) {
        // If we've already passed the handshake.
        if (/*len == 9 && */
            uart_state == SYNCED || uart_state == CHOCO_SYNCED) {
          // Passing the actual packet data through a CRC8 check to make sure
          // the recieved data isn't malformed.
          uint8_t current_crc = 0;
          for (uint8_t i = 0; i < 8; i++) {
            current_crc = crc8_ccitt_update(current_crc, uart_data[i]);
          }
          // CRC check passed. Let's populate our data.
          if (current_crc == uart_data[8]) {
            // So... This is a big fork in the road between the original
            // SwitchInputEmulator and mine here on the ESP32.

            // Normally, it would just send the values as is, but now with the
            // original handshake (Master: FF 33 CC), it will go into what I am
            // naming "Chocolate" mode. Where inputs are simplified and
            // converted for sideways joycons (if it is set as a Joy-Con, that
            // is.)

            // DPad, Left Stick, and Right Stick is all converted to the main
            // stick for that controller. (Left stick for L Joy-Con and Pro Con,
            // Right stick for R Joy-Con).
            // As of writing, the order of importance is:
            // DPad, L Stick, R Stick. If one of those is off center, the first
            // one in that list is the one the stick is set off of.

            // A B X Y is set to the proper side for the sideways joycon.
            // Example: A is going to be Down on L JC, and X on R JC.

            // Shoulder buttons are simplified in this way:
            // Sending either L, ZL, or SL, will press:
            // L, ZL on Pro Con
            // SL on Joy-Cons.
            // Similar behavior is on the right side as well.
            if (uart_state == CHOCO_SYNCED) {
              // Vanilla mode, ahoy!
              // This sets it exactly as it is given, with very little thought
              // given to the input given.

              // This is used to clear the 2 most significant bits in the first
              // packet (which are mapped to SR and SL)
              // Frankly, I'm unsure if this is *required*, but better safe than
              // sorry when it comes to being sneaky about the true nature of
              // the connected device.
              if (CONTROLLER_TYPE == PRO_CON) {
                uart_data[0] &= ~(1 << 7);  // Clear SL
                uart_data[0] &= ~(1 << 6);  // Clear SR
              }

              // Clearing shared data byte.
              but2_send = 0;

              if ((CONTROLLER_TYPE & JOYCON_R) == JOYCON_R) {
                // If this is a right joycon or a procon.
                but1_send = (((uart_data[1] >> 0) & 1) << 0) +   // Y
                            (((uart_data[1] >> 3) & 1) << 1) +   // X
                            (((uart_data[1] >> 1) & 1) << 2) +   // B
                            (((uart_data[1] >> 2) & 1) << 3) +   // A
                            (((uart_data[0] >> 7) & 1) << 4) +   // SR
                            (((uart_data[0] >> 6) & 1) << 5) +   // SL
                            (((uart_data[1] >> 5) & 1) << 6) +   // R
                            (((uart_data[1] >> 7) & 1) << 7);    // ZR
                but2_send += (((uart_data[0] >> 4) & 1) << 4) +  // Home
                             (((uart_data[0] >> 1) & 1) << 1) +  // +/Start
                             (((uart_data[0] >> 3) & 1) << 2);  // R Stick Click

                cx_send = uart_data[5];
                cy_send = 255 - uart_data[6];
              }

              if ((CONTROLLER_TYPE & JOYCON_L) == JOYCON_L) {
                // If this is a left joycon or a procon.

                // This is a bit of an ugly solution, but it's the first thing
                // that came to mind.
                bool dpad_d = false;
                bool dpad_u = false;
                bool dpad_r = false;
                bool dpad_l = false;

                switch (uart_data[2]) {
                  case A_DPAD_CENTER:
                    break;
                  case A_DPAD_U:
                    dpad_u = true;
                    break;
                  case A_DPAD_R:
                    dpad_r = true;
                    break;
                  case A_DPAD_D:
                    dpad_d = true;
                    break;
                  case A_DPAD_L:
                    dpad_l = true;
                    break;
                  case A_DPAD_U_R:
                    dpad_u = true;
                    dpad_r = true;
                    break;
                  case A_DPAD_U_L:
                    dpad_u = true;
                    dpad_l = true;
                    break;
                  case A_DPAD_D_R:
                    dpad_d = true;
                    dpad_r = true;
                    break;
                  case A_DPAD_D_L:
                    dpad_d = true;
                    dpad_l = true;
                    break;

                  default:
                    break;
                }

                but3_send = ((dpad_d) << 0) +                   // Down
                            ((dpad_u) << 1) +                   // Up
                            ((dpad_r) << 2) +                   // Right
                            ((dpad_l) << 3) +                   // Left
                            (((uart_data[0] >> 7) & 1) << 4) +  // SR
                            (((uart_data[0] >> 6) & 1) << 5) +  // SL
                            (((uart_data[1] >> 4) & 1) << 6) +  // L
                            (((uart_data[1] >> 6) & 1) << 7);   // ZL

                but2_send += (((uart_data[0] >> 5) & 1) << 5) +  // Capture
                             (((uart_data[0] >> 0) & 1) << 0) +  // -/Select
                             (((uart_data[0] >> 2) & 1) << 3);  // L Stick Click
                lx_send = uart_data[3];
                ly_send = 255 - uart_data[4];
              }
              // This is also commented out on the original SwitchInputEmulator,
              // so I'm just referencing it as such. :)
              // send_byte(RESP_UPDATE_ACK);
            } else if (uart_state == SYNCED) {
              // Chocolate mode ahoy.
              // This does the simplification for all types of controllers.
              if (CONTROLLER_TYPE == PRO_CON) {
                // uart_data[0] &= ~(1 << 7);  // Clear SL
                // uart_data[0] &= ~(1 << 6);  // Clear SR
              }
              bool up_button = ((uart_data[1] >> 3) & 1);
              bool down_button = ((uart_data[1] >> 1) & 1);
              bool left_button = ((uart_data[1] >> 0) & 1);
              bool right_button = ((uart_data[1] >> 2) & 1);

              bool start_button =
                  ((uart_data[0] >> 1) & 1) || ((uart_data[0] >> 0) & 1);

              bool stickclick_button =
                  ((uart_data[0] >> 3) & 1) || ((uart_data[0] >> 2) & 1);

              bool left_shoulder = ((uart_data[0] >> 6) & 1) ||
                                   ((uart_data[1] >> 4) & 1) ||
                                   ((uart_data[1] >> 6) & 1);
              bool right_shoulder = ((uart_data[0] >> 7) & 1) ||
                                    ((uart_data[1] >> 5) & 1) ||
                                    ((uart_data[1] >> 7) & 1);

              bool home_button = ((uart_data[0] >> 4) & 1);
              bool capture_button = ((uart_data[0] >> 5) & 1);

              uint8_t stickX = 128;
              uint8_t stickY = 128;
              if (uart_data[2] != A_DPAD_CENTER) {
                switch (uart_data[2]) {
                  case A_DPAD_CENTER:
                    break;
                  case A_DPAD_U:
                    stickY = 255;
                    break;
                  case A_DPAD_R:
                    stickX = 255;
                    break;
                  case A_DPAD_D:
                    stickY = 0;
                    break;
                  case A_DPAD_L:
                    stickX = 0;
                    break;
                  case A_DPAD_U_R:
                    stickY = 255;
                    stickX = 255;
                    break;
                  case A_DPAD_U_L:
                    stickY = 255;
                    stickX = 0;
                    break;
                  case A_DPAD_D_R:
                    stickY = 0;
                    stickX = 255;
                    break;
                  case A_DPAD_D_L:
                    stickY = 0;
                    stickX = 0;
                    break;

                  default:
                    break;
                }
              } else {
                if (uart_data[3] != 128 || uart_data[4] != 128) {
                  stickX = uart_data[3];
                  stickY = 255 - uart_data[4];
                } else if (uart_data[5] != 128 || uart_data[6] != 128) {
                  stickX = uart_data[5];
                  stickY = 255 - uart_data[6];
                }
              }

              // Clearing button bytes.
              but1_send = 0;
              but2_send = 0;
              but3_send = 0;
              if (CONTROLLER_TYPE == PRO_CON) {
                // uart_data[0] &= ~(1 << 7);  // Clear SL
                // uart_data[0] &= ~(1 << 6);  // Clear SR
                lx_send = stickX;
                ly_send = stickY;
                cx_send = 128;
                cy_send = 128;
                but1_send = (left_button << 0) +     // Y
                            (up_button << 1) +       // X
                            (down_button << 2) +     // B
                            (right_button << 3) +    // A
                            (right_shoulder << 6) +  // R
                            (right_shoulder << 7);   // ZR

                but2_send += (home_button << 4) +        // Home
                             (start_button << 1) +       // +/Start
                             (stickclick_button << 3) +  // L Stick Click
                             (capture_button << 5);

                but3_send = (left_shoulder << 6) +  // L
                            (left_shoulder << 7);   // ZL

                //(select_button << 0) +  // -/Select
                //(((uart_data[0] >> 2) & 1) << 3);  // L
                // Stick Click
              }
              if (CONTROLLER_TYPE == JOYCON_R) {
                // If this is a right joycon or a procon.

                lx_send = 128;
                ly_send = 128;
                cx_send = 255 - stickY;
                cy_send = stickX;

                but1_send = (up_button << 0) +       // Y
                            (right_button << 1) +    // X
                            (left_button << 2) +     // B
                            (down_button << 3) +     // A
                            (right_shoulder << 4) +  // SR
                            (left_shoulder << 5);    // SL

                but2_send += (home_button << 4) +       // Home
                             (start_button << 1) +      // +/Start
                             (stickclick_button << 2);  // R Stick Click
              }

              if (CONTROLLER_TYPE == JOYCON_L) {
                // If this is a left joycon or a procon.

                lx_send = stickY;
                ly_send = 255 - stickX;
                cx_send = 128;
                cy_send = 128;

                but3_send = (right_button << 0) +    // Y
                            (left_button << 1) +     // X
                            (up_button << 2) +       // B
                            (down_button << 3) +     // A
                            (right_shoulder << 4) +  // SR
                            (left_shoulder << 5);    // SL

                but2_send += (start_button << 0) +       // -/Select
                             (stickclick_button << 3) +  // L Stick Click
                             (capture_button << 5);      // Capture
              }
              // send_byte(RESP_UPDATE_ACK);
            }

          } else if (uart_data[8] == COMMAND_SYNC_START) {
            // If CRC check didn't pass, but the suggested checksum is 0xFF,
            // assume that the master is trying to force a resync.
            uart_state = SYNC_START;
            send_byte(RESP_SYNC_START);
            continue;
          } else {
            // If CRC didn't pass, but wasn't 0xFF, keep assuming we want to be
            // handshook, tell the master that we didn't update, and continue
            // on.
            send_byte(RESP_UPDATE_NACK);
            ESP_LOGI(
                "CRC Error",
                "Packet specified CRC 0x%02x but calculated CRC was 0x%02x",
                uart_data[8], current_crc);
            ESP_LOGI(
                "CRC Error",
                "Packet data: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                uart_data[0], uart_data[1], uart_data[2], uart_data[3],
                uart_data[4], uart_data[5], uart_data[6], uart_data[7],
                uart_data[8]);
          }
        }
      }
      // Initial, vanilla handshake.
      // MASTER: 0xFF
      // SLAVE: 0xFF
      // MASTER: 0x33
      // SLAVE: 0xCC
      // MASTER: 0xCC
      // SLAVE: 0x33, then 0x90 with each packet sent as an HID

      // Planning on having another handshake mode for easy JoyCon usage.
      // As in, DPad/L Stick/R Stick will update the proper stick accordingly,
      // and in the direction as if it's sideways.
      // And ABXY will be the proper positioning if sideways. But if this
      // handshake is done while emulating a Pro Controller, the current thought
      // is to use the DPad, ABXY, and L/R shoulder buttons
      // (maybe both sets of LR/ZLZR like the NullWiiCon?)

      // "Chocolate" handshake.
      // MASTER: 0xFF
      // SLAVE: 0xFF
      // MASTER: 0x44
      // SLAVE: 0xEE
      // MASTER: 0xEE
      // SLAVE: CONTROLLER_TYPE, then 0x90 with each packet sent as an HID
      if (uart_state == SYNC_START) {
        if (uart_data[0] == COMMAND_SYNC_1) {
          uart_state = SYNC_1;
          send_byte(RESP_SYNC_1);
        } else if (uart_data[0] == COMMAND_CHOCO_SYNC_1) {
          uart_state = CHOCO_SYNC_1;
          send_byte(RESP_CHOCO_SYNC_1);
        } else {
          uart_state = OUT_OF_SYNC;
        }
      } else if (uart_state == SYNC_1) {
        if (uart_data[0] == COMMAND_SYNC_2) {
          uart_state = SYNCED;
          send_byte(RESP_SYNC_OK);
        } else {
          uart_state = OUT_OF_SYNC;
        }
      } else if (uart_state == CHOCO_SYNC_1) {
        if (uart_data[0] == COMMAND_CHOCO_SYNC_2) {
          // ESP_LOGI("BBBBBBBBBBBB", "BBBBBBBBBB");
          uart_state = CHOCO_SYNCED;
          send_byte(RESP_CHOCO_SYNC_OK);
        } else {
          uart_state = OUT_OF_SYNC;
        }
      }
      if (uart_state == OUT_OF_SYNC) {
        if (uart_data[0] == COMMAND_SYNC_START) {
          uart_state = SYNC_START;
          send_byte(RESP_SYNC_START);
        }
      }
    }
    // char buffer[10];ste
    // sprintf(buffer, "l:%d", len);
    // uart_write_bytes(UART_NUM, buffer, 10);
    vTaskDelay(15);
    if (but1_send || but2_send || but3_send)
      ESP_LOGI("debug", "but1: %d, but2: %d, but3: %d\n", but1_send, but2_send,
               but3_send);

    if ((lx_send != 128 && lx_send != 127) ||
        (ly_send != 128 && ly_send != 127) ||
        (cx_send != 128 && cx_send != 127) ||
        (cy_send != 128 && cy_send != 127))
      ESP_LOGI("debug", "lx: %d, ly: %d, cx: %d, cy: %d\n", lx_send, ly_send,
               cx_send, cy_send);
  }
  // ESP_LOGI("SNES", "but1: %d, but2: %d, but3: %d, upper: %d, lower: %d\n",
  //          but1_send, but2_send, but3_send, upper, lower);
}

// Switch button report example //         batlvl       Buttons Lstick Rstick
// static uint8_t report30[] = {0x30, 0x00, 0x90,   0x00, 0x00, 0x00,   0x00,
// 0x00, 0x00,   0x00, 0x00, 0x00};
// 80
static uint8_t report30[48] = {[0] = 0x00, [1] = 0x8E, [11] = 0x80};

static uint8_t dummy[11] = {0x00, 0x8E, 0x00, 0x00, 0x00,
                            0x00, 0x08, 0x80, 0x00, 0x08, 0x80};

void send_buttons() {
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  report30[0] = timer;
  dummy[0] = timer;
  // buttons
  report30[2] = but1_send;
  report30[3] = but2_send;
  report30[4] = but3_send;
  // encode left stick
  // report30[6] = 0x00;
  // report30[7] = 0x08;
  // report30[8] = 0x80;
  report30[5] = (lx_send << 4) & 0xF0;
  report30[6] = (lx_send & 0xF0) >> 4;
  report30[7] = ly_send;
  // encode right stick
  report30[8] = (cx_send << 4) & 0xF0;
  report30[9] = (cx_send & 0xF0) >> 4;
  report30[10] = cy_send;
  xSemaphoreGive(xSemaphore);
  timer += 1;
  if (timer == 255) timer = 0;

  if ((uart_state == SYNCED || uart_state == CHOCO_SYNCED) &&
      (paired || connected)) {
    // ESP_LOGI("AAAAAAAA", "AAAAAA");
    esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                               sizeof(report30), report30);
    send_byte_(RESP_USB_ACK);
    vTaskDelay(15);
  } else {
    esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30,
                               sizeof(dummy), dummy);
    vTaskDelay(100);
  }

  if (!paired || !(uart_state == SYNCED || uart_state == CHOCO_SYNCED)) {
  } else {
  }
}

/// Switch Replies

// Reply for REQUEST_DEVICE_INFO
static uint8_t reply02[] = {
    //0x21, 
0x00, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,

    0x00, 0x00, 0x00, 0x82, 0x02, 0x04, 0x00,
    CONTROLLER_TYPE,  // Controller type byte.
    // 01 - Left Joycon
    // 02 - Right Joycon
    // 03 - Pro Controller
    0x02, 0xD4, 0xF0, 0x57, 0x6E, 0xF0, 0xD7, 0x01,
#if CONTROLLER_TYPE == PRO_CON
    0x02
#else
    0x01
#endif
    ,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Reply for SET_SHIPMENT_STATE
static uint8_t reply08[] = {
    //0x21, 
0x01, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SET_INPUT_REPORT_MODE
static uint8_t reply03[] = {
    //0x21, 
0x04, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Trigger buttons elapsed time
static uint8_t reply04[] = {
    //0x21, 
0x0A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x83, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2c, 0x01, 0x2c, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Serial number and controller type (although, our code doesn't read (and as
// such, report) the controller type from here.)
static uint8_t spi_reply_address_0[] = {//0x21, 
0x02, 0x8E,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x08, 0x80,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x90, 0x10,
                                        0x00, 0x60, 0x00,
                                        0x00, 0x10, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff,
                                        0x00, 0x00, CONTROLLER_TYPE,
                                        0xA0, 0x00, 0x00,
                                        0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00};
// The CONTROLLER_TYPE is technically unused, but it makes me feel better.

// SPI Flash colors
static uint8_t spi_reply_address_0x50[] = {
    //0x21, 
0x03, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x90, 0x10, 0x50, 0x60, 0x00, 0x00, 0x0D,  // Start of colors
    0x23, 0x23, 0x23,                                      // Body color
    0xff, 0xff, 0xff,                                      // Buttons color
#if CONTROLLER_TYPE == PRO_CON
    0x95, 0x15, 0x15,  // Left Grip color (Pro Con)
    0x15, 0x15, 0x95,  // Right Grip color (Pro Con)
#else
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
#endif
    0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x80[] = {
    //0x21, 
0x0B, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x80, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x98[] = {
    //0x21, 
0x0C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x98, 0x60, 0x00, 0x00, 0x12,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// User analog stick calib
static uint8_t spi_reply_address_0x10[] = {
    //0x21, 
0x0D, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x10, 0x80, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x3d[] = {
    //0x21, 
0x0E, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x3D, 0x60, 0x00, 0x00, 0x19,
    0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00,
    0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00};

static uint8_t spi_reply_address_0x20[] = {
    //0x21, 
0x10, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x20, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for changing the status of the IMU IMU (6-Axis sensor)
static uint8_t reply4001[] = {
    //0x21, 
0x15, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t reply4801[] = {
    //0x21, 
0x1A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SubCommand.SET_PLAYER_LIGHTS
static uint8_t reply3001[] = {
    //0x21, 
0x1C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#if CONTROLLER_TYPE == JOYCON_L
// If I had to guess, the Pro Controller equivalent for SET_NFC_IR_MCU_STATE
// Joycontrol calls it SET_NFC_IR_MCU_CONFIG, so maybe its setting the IR sensor
// to OFF?
static uint8_t reply3333[] = {
    //0x21, 
0x03, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01,
    0x18, 0x80, 0x80, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//
#elif CONTROLLER_TYPE == JOYCON_R
static uint8_t reply3333[] = {
    //0x21, 
0x31, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x08, 0x80, 0x00, 0xa0, 0x21, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x00};
#else
static uint8_t reply3333[] = {
    //0x21, 
0x31, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x08, 0x80, 0x00, 0xa0, 0x21, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0x00};
#endif

// Reply for SubCommand.SET_NFC_IR_MCU_STATE
static uint8_t reply3401[] = {
    //0x21, 
0x12, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x80, 0x00, 0x80, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t hid_descriptor[] = {
0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x06, 0x01, 0xff, 0x85, 0x21, 0x09, 0x21, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02, 0x85, 0x30, 0x09, 0x30, 0x75, 0x08, 0x95, 0x30, 0x81, 0x02, 0x85, 0x31, 0x09, 0x31, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x32, 0x09, 0x32, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x33, 0x09, 0x33, 0x75, 0x08, 0x96, 0x69, 0x01, 0x81, 0x02, 0x85, 0x3f, 0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x10, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x05, 0x09, 0x75, 0x04, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x33, 0x09, 0x34, 0x16, 0x00, 0x00, 0x27, 0xff, 0xff, 0x00, 0x00, 0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0x06, 0x01, 0xff, 0x85, 0x01, 0x09, 0x01, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x10, 0x09, 0x10, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x11, 0x09, 0x11, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0x85, 0x12, 0x09, 0x12, 0x75, 0x08, 0x95, 0x30, 0x91, 0x02, 0xc0
};
int hid_descriptor_len = sizeof(hid_descriptor);

// sending bluetooth values every 15ms
void send_task(void* pvParameters) {
  const char* TAG = "send_task";
  ESP_LOGI(TAG, "Sending hid reports on core %d\n", xPortGetCoreID());
  while (1) {
    send_buttons();
  }
}

// callback for notifying when hidd application is registered or not registered
void application_cb(esp_bd_addr_t bd_addr, esp_hidd_application_state_t state) {
  const char* TAG = "application_cb";

  switch (state) {
    case ESP_HIDD_APP_STATE_NOT_REGISTERED:
      ESP_LOGI(TAG, "app not registered");
      break;
    case ESP_HIDD_APP_STATE_REGISTERED:
      ESP_LOGI(TAG, "app is now registered!");
      if (bd_addr == NULL) {
        ESP_LOGI(TAG, "bd_addr is null...");
        break;
      }
      break;
    default:
      ESP_LOGW(TAG, "unknown app state %i", state);
      break;
  }
}
// LED blink
void startBlink() {
  while (1) {
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 0);
    vTaskDelay(150);
    gpio_set_level(LED_GPIO, 1);
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}
// callback for hidd connection changes
void connection_cb(esp_bd_addr_t bd_addr, esp_hidd_connection_state_t state) {
  const char* TAG = "connection_cb";

  switch (state) {
    case ESP_HIDD_CONN_STATE_CONNECTED:
      ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", bd_addr[0],
               bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
      ESP_LOGI(TAG, "setting bluetooth non connectable");
      esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

      // clear blinking LED - solid
      vTaskDelete(BlinkHandle);
      BlinkHandle = NULL;
      gpio_set_level(LED_GPIO, 1);
      // start solid
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      connected = true;
      xSemaphoreGive(xSemaphore);
      // restart send_task
      if (SendingHandle != NULL) {
        vTaskDelete(SendingHandle);
        SendingHandle = NULL;
      }
      xTaskCreatePinnedToCore(send_task, "send_task", 2048, NULL, 2,
                              &SendingHandle, 0);
      break;
    case ESP_HIDD_CONN_STATE_CONNECTING:
      ESP_LOGI(TAG, "connecting");
      break;
    case ESP_HIDD_CONN_STATE_DISCONNECTED:
      xTaskCreate(startBlink, "blink_task", 1024, NULL, 1, &BlinkHandle);
      // start blink
      ESP_LOGI(TAG, "disconnected from %02x:%02x:%02x:%02x:%02x:%02x",
               bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4],
               bd_addr[5]);
      ESP_LOGI(TAG, "making self discoverable");
      paired = 0;
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      connected = false;
      xSemaphoreGive(xSemaphore);
      break;
    case ESP_HIDD_CONN_STATE_DISCONNECTING:
      ESP_LOGI(TAG, "disconnecting");
      break;
    default:
      ESP_LOGI(TAG, "unknown connection status");
      break;
  }
}

// callback for discovering
void get_device_cb() { ESP_LOGI("hi", "found a device"); }

// callback for when hid host requests a report
void get_report_cb(uint8_t type, uint8_t id, uint16_t buffer_size) {
  const char* TAG = "get_report_cb";
  ESP_LOGI(TAG, "got a get_report request from host");
}

// callback for when hid host sends a report
void set_report_cb(uint8_t type, uint8_t id, uint16_t len, uint8_t* p_data) {
  const char* TAG = "set_report_cb";
  ESP_LOGI(TAG, "got a report from host");
}

// callback for when hid host requests a protocol change
void set_protocol_cb(uint8_t protocol) {
  const char* TAG = "set_protocol_cb";
  ESP_LOGI(TAG, "got a set_protocol request from host");
}

// callback for when hid host sends interrupt data
void intr_data_cb(uint8_t report_id, uint16_t len, uint8_t* p_data) {
  const char* TAG = "intr_data_cb";
  // switch pairing sequence
  // Length seems to be quite important here, as the length of [49]
  // Is used for many replies, as well.
  //デバッグ用の受信データ表示 ( GT: Displaying received data for debugging )
  esp_log_buffer_hex(TAG, p_data, len);
  //必要であればコメントアウト外しても良いかも ( GT: You may uncomment if necessary )
  
  //if (len == 49) {
    if (p_data[9] == 2) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply02), reply02);
      //これもそう ( GT: This is also the case )
      ESP_LOGI(TAG, "reply02");
    }
    if (p_data[9] == 8) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply08), reply08);
      ESP_LOGI(TAG, "reply08");
    }
    if (p_data[9] == 16 && p_data[10] == 0 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0),
                                 spi_reply_address_0);
      ESP_LOGI(TAG, "replyspi0");
    }
    if (p_data[9] == 16 && p_data[10] == 80 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x50),
                                 spi_reply_address_0x50);
      ESP_LOGI(TAG, "replyspi50");
    }
    if (p_data[9] == 3) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply03), reply03);
      ESP_LOGI(TAG, "reply03");
    }
    if (p_data[9] == 4) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply04), reply04);
      ESP_LOGI(TAG, "reply04");
    }
    if (p_data[9] == 16 && p_data[10] == 128 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x80),
                                 spi_reply_address_0x80);
      ESP_LOGI(TAG, "replyspi80");
    }
    if (p_data[9] == 16 && p_data[10] == 152 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x98),
                                 spi_reply_address_0x98);
      ESP_LOGI(TAG, "replyspi98");
    }
    if (p_data[9] == 16 && p_data[10] == 16 && p_data[11] == 128) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x10),
                                 spi_reply_address_0x10);
      ESP_LOGI(TAG, "replyspi10");
    }
    if (p_data[9] == 16 && p_data[10] == 61 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x3d),
                                 spi_reply_address_0x3d);
      ESP_LOGI(TAG, "reply3d");
    }
    if (p_data[9] == 16 && p_data[10] == 32 && p_data[11] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(spi_reply_address_0x20),
                                 spi_reply_address_0x20);
      ESP_LOGI(TAG, "replyspi20");
    }
    if (p_data[9] == 64 /*&& p_data[11] == 1*/) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply4001), reply4001);
      ESP_LOGI(TAG, "reply4001");
    }
    if (p_data[9] == 72 /* && p_data[11] == 1*/) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply4801), reply4801);
      ESP_LOGI(TAG, "reply4801");
    }
    if (p_data[9] == 34 /*&& p_data[11] == 1*/) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3401), reply3401);
      ESP_LOGI(TAG, "reply3401");
    }
    if (p_data[9] == 48 /*&& p_data[11] == 1*/) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3001), reply3001);
      ESP_LOGI(TAG, "reply3001");
      if (CONTROLLER_TYPE == JOYCON_L) {
        paired = 1;
      }
    }

    if (p_data[9] == 33 && p_data[10] == 33) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                 sizeof(reply3333), reply3333);
      ESP_LOGI(TAG, "reply3333");
      paired = 1;
    }

  //ここらへんも必要に応じて ( GT: Here and there as needed )
  //  ESP_LOGI(
  //      TAG,
  //      "got an interrupt report from host, subcommand: %d  %d  %d Length: %d",
  //      p_data[10], p_data[11], p_data[12], len);
  //} else {
    // ESP_LOGI("heap size:", "%d", xPortGetFreeHeapSize());
    // ESP_LOGI(TAG, "pairing packet size != 49, subcommand: %d  %d  %d  Length:
    // %d", p_data[10], p_data[11], p_data[12], len);
  //}
}

// callback for when hid host does a virtual cable unplug
void vc_unplug_cb(void) {
  const char* TAG = "vc_unplug_cb";
  ESP_LOGI(TAG, "host did a virtual cable unplug");
}

void set_bt_address() {
  // store a random mac address in flash
  nvs_handle my_handle;
  esp_err_t err;
  uint8_t bt_addr[8];

  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  size_t addr_size = 0;
  err = nvs_get_blob(my_handle, "mac_addr", NULL, &addr_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  if (addr_size > 0) {
    err = nvs_get_blob(my_handle, "mac_addr", bt_addr, &addr_size);
  } else {
    //前3桁はNintendo OUI Rangeの物にする ( The first 3 segments should be in Nintendo OUI Range )
    //具体的にはD4:F0:57:XX:XX:XXになる ( For example: D4:F0:57:XX:XX:XX )
    bt_addr[0] = 0xD4;
    bt_addr[1] = 0xF0;
    bt_addr[2] = 0x57;
    for (int i = 3; i < 8; i++) bt_addr[i] = esp_random() % 255;
    addr_size = sizeof(bt_addr);
    err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);
  }

  //もしアドレスを再設定したいときはコメントアウト外す ( GT: If you want to reset the address, uncomment it )
  //bt_addr[0] = 0xD4;
  //bt_addr[1] = 0xF0;
  //bt_addr[2] = 0x57;
  //for (int i = 3; i < 8; i++) bt_addr[i] = esp_random() % 255;
  //addr_size = sizeof(bt_addr);
  //err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);

  err = nvs_commit(my_handle);
  nvs_close(my_handle);
  esp_base_mac_addr_set(bt_addr);
}

void print_bt_address() {
  const char* TAG = "bt_address";
  const uint8_t* bd_addr;

  bd_addr = esp_bt_dev_get_address();
  ESP_LOGI(TAG, "my bluetooth address is %02X:%02X:%02X:%02X:%02X:%02X",
           bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4],
           bd_addr[5]);
}

#define SPP_TAG "tag"
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t* param) {
  switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
      esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
      break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
      break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
      ESP_LOGI(SPP_TAG, "%d", param->rmt_srvcs.num_uuids);
      break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
      ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
      break;
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(SPP_TAG, "authentication success: %s",
                 param->auth_cmpl.device_name);
        esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      } else {
        ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
                 param->auth_cmpl.stat);
      }
      break;
    }

    default:
      break;
  }
}
void app_main() {
  // SNES Contoller reading init
  // spi_init();
  // xTaskCreatePinnedToCore(get_buttons, "gbuttons", 2048, NULL, 1,
  //                         &ButtonsHandle, 1);
  if (CONTROLLER_TYPE != PRO_CON) {
    //report30[2] += (0x3 << 1);
    //dummy[2] += (0x3 << 1);
  }
  uart_init();
  xTaskCreatePinnedToCore(uart_task, "uart_task", 2048, NULL, 1, &ButtonsHandle,
                          1);
  // flash LED
  vTaskDelay(100);
  gpio_set_level(LED_GPIO, 0);
  vTaskDelay(100);
  gpio_set_level(LED_GPIO, 1);
  vTaskDelay(100);
  gpio_set_level(LED_GPIO, 0);
  vTaskDelay(100);
  gpio_set_level(LED_GPIO, 1);
  vTaskDelay(100);
  gpio_set_level(LED_GPIO, 0);
  const char* TAG = "app_main";
  esp_err_t ret;
  static esp_hidd_callbacks_t callbacks;
  static esp_hidd_app_param_t app_param;
  static esp_hidd_qos_param_t both_qos;
  static esp_bt_cod_t class;

  xSemaphore = xSemaphoreCreateMutex();

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
  // gap_callbacks = get_device_cb;

  //一応名前とプロバイダーを純正と一緒にする ( For now, set these the same as a genuine product )
  app_param.name = "Wireless Gamepad";
  app_param.description = "Gamepad";
  app_param.provider = "Nintendo";
  // app_param.subclass = 0x002508;
  app_param.subclass = 0x8;
  app_param.desc_list = hid_descriptor;
  app_param.desc_list_len = hid_descriptor_len;
  memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));

  callbacks.application_state_cb = application_cb;
  callbacks.connection_state_cb = connection_cb;
  callbacks.get_report_cb = get_report_cb;
  callbacks.set_report_cb = set_report_cb;
  callbacks.set_protocol_cb = set_protocol_cb;
  callbacks.intr_data_cb = intr_data_cb;
  callbacks.vc_unplug_cb = vc_unplug_cb;

  class.minor = 2;
  class.major = 5;
  class.service = 1;

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  set_bt_address();

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_mem_release(ESP_BT_MODE_BLE);
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "initialize controller failed: %s\n", esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(TAG, "enable controller failed: %s\n", esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_init()) != ESP_OK) {
    ESP_LOGE(TAG, "initialize bluedroid failed: %s\n", esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(TAG, "enable bluedroid failed: %s\n", esp_err_to_name(ret));
    return;
  }
  esp_bt_gap_register_callback(esp_bt_gap_cb);
  ESP_LOGI(TAG, "setting hid parameters");
  esp_hid_device_register_app(&app_param, &both_qos, &both_qos);

  ESP_LOGI(TAG, "starting hid device");
  esp_hid_device_init(&callbacks);

  ESP_LOGI(TAG, "setting device name");

  if (CONTROLLER_TYPE == JOYCON_L)
    esp_bt_dev_set_device_name("Joy-Con (L)");
  else if (CONTROLLER_TYPE == JOYCON_R)
    esp_bt_dev_set_device_name("Joy-Con (R)");
  else
    esp_bt_dev_set_device_name("Pro Controller");

  esp_bt_gap_set_cod(class, ESP_BT_SET_COD_ALL);
  ESP_LOGI(TAG, "setting hid device class");

  ESP_LOGI(TAG, "setting to connectable, discoverable");
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  //xTaskCreatePinnedToCore(send_task, "send_task", 2048, NULL, 2,
  //                            &SendingHandle, 0);

  // esp_hid_device_connect
  // start blinking
  xTaskCreate(startBlink, "blink_task", 1024, NULL, 2, &BlinkHandle);
}
//
