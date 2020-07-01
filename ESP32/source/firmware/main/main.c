//
//  BlueCubeMod Firmware
//
//
//  Created by Nathan Reeves 2019
//
// Main clock 80MHz

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
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

// Buttons and sticks
static uint8_t but1_send = 0;
static uint8_t but2_send = 0;
static uint8_t but3_send = 0;
static uint8_t lx_send = 0;
static uint8_t ly_send = 0;
static uint8_t cx_send = 0;
static uint8_t cy_send = 0;
static uint8_t lt_send = 0;
static uint8_t rt_send = 0;

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t spiSemaphore;
bool connected = false;
int paired = 0;
TaskHandle_t ButtonsHandle = NULL;
TaskHandle_t SendingHandle = NULL;
TaskHandle_t BlinkHandle = NULL;
uint8_t timer = 0;

#define SPI_MASTER_FREQ_83 (APB_CLK_FREQ / 960)  // 83.33kHz
#define PIN_NUM_LATCH 19
#define PIN_NUM_MISO 18
#define PIN_NUM_MOSI 05
#define PIN_NUM_CLK 33
#define PIN_LATCH (1ULL << PIN_NUM_LATCH)

// Init spi module and return spi handle
spi_device_handle_t spi;
esp_timer_handle_t latch_timer;
void spi_init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {.miso_io_num = PIN_NUM_MISO,
                             .mosi_io_num = -1,
                             .sclk_io_num = PIN_NUM_CLK,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 16};

  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = SPI_MASTER_FREQ_83,  // Clock out at 83.33k, T=12 us
      .mode = 2,                             // SPI mode 2: CPOL = 1, CPHA = 1
      .spics_io_num = -1,                    // CS pin
      .queue_size = 7};

  // Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
  ESP_ERROR_CHECK(ret);
  // Attach the SNES controller to the SPI bus
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  // Data latch gpio config
  gpio_config_t SNES_gpio_conf = {.intr_type = GPIO_PIN_INTR_DISABLE,
                                  .mode = GPIO_MODE_OUTPUT,
                                  .pin_bit_mask = PIN_LATCH,
                                  .pull_down_en = 0,
                                  .pull_up_en = 0};
  gpio_config(&SNES_gpio_conf);
}

static spi_transaction_t trans = {
    .flags = SPI_TRANS_USE_RXDATA, .length = 16, .rxlength = 16};

// Main loop for polling SNES controller
// TODO: Use interrupts (semaphores are too slow)

// bool test = 1;

static void get_buttons() {
  ESP_LOGI("hi", "Hello world from core %d!. SPI_get_buttons\n",
           xPortGetCoreID());
  while (1) {
    // SNES polling using SPI
    gpio_set_level(PIN_NUM_LATCH, 1);
    ets_delay_us(12);
    gpio_set_level(PIN_NUM_LATCH, 0);
    ets_delay_us(6);
    spi_device_transmit(spi, &trans);

    // Button mapping
    uint8_t upper = ~trans.rx_data[0];
    uint8_t lower = ~trans.rx_data[1];
    // if(test == 128){
    //     test = 0;
    // } else if (test == 0 ){
    //     test = 254;
    // } else {
    //     test = 128;
    // }
    // test = !test;
    //                                          SNES | SWITCH
    //     but1_send = (((upper >> 7) & 1) << 2)   //  B      B
    //               + (((upper >> 6) & 1) << 0)   //  Y      Y
    //               + (((lower >> 6) & 1) << 1)   //  X      X
    //               + (((lower >> 7) & 1) << 3)   //  A      A
    //               + (((lower >> 4) & 1) << 6);  //  R      R
    //     but2_send = (((upper >> 4) & 1) << 1)   //  START  +
    //               + (((upper >> 5) & 1) << 0);  //  SEL    -
    //     but3_send = (((lower >> 5) & 1) << 6)   //  L      L
    //               + (((upper >> 1) & 1) << 3)   //  ←      ←
    //               + (((upper >> 0) & 1) << 2)   //  →      →
    //               + (((upper >> 3) & 1) << 1)   //  ↑      ↑
    //               + (((upper >> 2) & 1) << 0);  //  ↓      ↓

    //         but1_send = (((upper >> 7) & 1) << 2)   //  B      B
    //             + (((upper >> 6) & 1) << 0)   //  Y      Y
    //             + (((lower >> 6) & 1) << 1)   //  X      X
    //             + (((lower >> 7) & 1) << 3)   //  A      A
    //             + (((lower >> 4) & 1) << 6);  //  R      R
    //         but2_send = (((upper >> 4) & 1) << 1)   //  START  +
    //             + (((upper >> 5) & 1) << 0);  //  SEL    -
    but2_send = (((upper >> 4) & 1) << 0)   // 
                 + (((upper >> 5) & 1) << 5)  // 
                 + ((((lower >> 4) & 1) && (lower >> 5) & 1) << 3);  // 
    but3_send = (((lower >> 7) & 1) << 0)     //
                + (((upper >> 6) & 1) << 1)   //
                + (((lower >> 6) & 1) << 2)   //
                + (((upper >> 7) & 1) << 3)   //
                + (((lower >> 4) & 1) << 4)   //
                + (((lower >> 5) & 1) << 5);  //
    if (((upper >> 3) & 1)) {
      lx_send = 254;  // Up
    } else if (((upper >> 2) & 1)) {
      lx_send = 0;  // Down
    } else {
      lx_send = 128;
    }

    if (((upper >> 1) & 1)) {
      ly_send = 254;  // Left
    } else if (((upper >> 0) & 1)) {
      ly_send = 0;  // Right
    } else {
      ly_send = 128;
    }
    // ESP_LOGI("SNES", "but1: %d, but2: %d, but3: %d, upper: %d, lower: %d\n",
    //          but1_send, but2_send, but3_send, upper, lower);

    vTaskDelay(15);
  }
}

// Switch button report example //         batlvl       Buttons Lstick Rstick
// static uint8_t report30[] = {0x30, 0x00, 0x90,   0x00, 0x00, 0x00,   0x00,
// 0x00, 0x00,   0x00, 0x00, 0x00};
// 80
static uint8_t report30[49] = {[0] = 0x30, [1] = 0x00, [2] = 0x8e, [12] = 0x80};

static uint8_t dummy[2] = {0, 0};

void send_buttons() {
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  report30[1] = timer;
  // buttons
  report30[3] = 0x00;
  report30[4] = but2_send;
  report30[5] = but3_send;
  // encode left stick
  // report30[6] = 0x00;
  // report30[7] = 0x08;
  // report30[8] = 0x80;
  report30[6] = (lx_send << 4) & 0xF0;
  report30[7] = (lx_send & 0xF0) >> 4;
  report30[8] = ly_send;
  // encode right stick
  report30[9] = 0x00;
  report30[10] = 0x00;
  report30[11] = 0x00;
  xSemaphoreGive(xSemaphore);
  timer += 1;
  if (timer == 255) timer = 0;

  if (!paired) {
    dummy[1] = timer;
    esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                               sizeof(dummy), dummy);
    vTaskDelay(100);
  } else {
    esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                               sizeof(report30), report30);
    vTaskDelay(15);
  }
}

/// Switch Replies
// 21008e00000000088000000000820204000102001a7dda711301010000000000000000000000000000000000000000000000

// Reply for REQUEST_DEVICE_INFO
static uint8_t reply02[] = {
    0x21, 0x00, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,

    0x00, 0x00, 0x00, 0x82, 0x02, 0x04, 0x00,
    0x01,  // Controller type byte. 01 - L JC. 02 - R JC. 03 - PROCON.
    0x02, 0x00, 0x1A, 0x7D, 0xDA, 0x71, 0x13, 0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Reply for SET_SHIPMENT_STATE
static uint8_t reply08[] = {
    0x21, 0x01, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Reply for SET_INPUT_REPORT_MODE
static uint8_t reply03[] = {
    0x21, 0x04, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply04[] = {
    0x21, 0x0A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x83, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2c, 0x01, 0x2c, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1060[] = {
    0x21, 0x02, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x90, 0x10, 0x00, 0x60, 0x00, 0x00, 0x10, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1050[] = {
    0x21, 0x03, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x90, 0x10, 0x50, 0x60, 0x00, 0x00, 0x0D,  // Start of colors
    0xff, 0x00, 0x00,                                // Joy-Con (L?) Body Color
    0x00, 0xff, 0xff,  // Joy-Con (L?) Buttons color
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1080[] = {
    0x21, 0x0B, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x80, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1098[] = {
    0x21, 0x0C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x98, 0x60, 0x00, 0x00, 0x12,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// User analog stick calib
static uint8_t reply1010[] = {
    0x21, 0x0D, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x10, 0x80, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply103D[] = {
    0x21, 0x0E, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x3D, 0x60, 0x00, 0x00, 0x19,
    0x00, 0x07, 0x70, 0x00, 0x08, 0x80, 0x00, 0x07, 0x70, 0x00,
    0x08, 0x80, 0x00, 0x07, 0x70, 0x00, 0x07, 0x70, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply1020[] = {
    0x21, 0x10, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x90, 0x10, 0x20, 0x60, 0x00, 0x00, 0x18,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply4001[] = {
    0x21, 0x15, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t reply4801[] = {
    0x21, 0x1A, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SubCommand.SET_NFC_IR_MCU_STATE
static uint8_t reply3401[] = {
    0x21, 0x12, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x80, 0x00, 0x80, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Reply for SubCommand.SET_PLAYER_LIGHTS
static uint8_t reply3001[] = {
    0x21, 0x1C, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// If I had to guess, the Pro Controller equivalent for SET_NFC_IR_MCU_STATE
// Joycontrol calls it SET_NFC_IR_MCU_CONFIG, so maybe its setting the IR sensor
// to OFF?
static uint8_t reply3333[] = {
    0x21, 0x03, 0x8E, 0x84, 0x00, 0x12, 0x01, 0x18, 0x80, 0x01,
    0x18, 0x80, 0x80, 0x80, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
  if (len == 49) {
    if (p_data[10] == 2) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply02), reply02);
    }
    if (p_data[10] == 8) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply08), reply08);
    }
    if (p_data[10] == 16 && p_data[11] == 0 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1060), reply1060);
    }
    if (p_data[10] == 16 && p_data[11] == 80 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1050), reply1050);
    }
    if (p_data[10] == 3) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply03), reply03);
    }
    if (p_data[10] == 4) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply04), reply04);
    }
    if (p_data[10] == 16 && p_data[11] == 128 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1080), reply1080);
    }
    if (p_data[10] == 16 && p_data[11] == 152 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1098), reply1098);
    }
    if (p_data[10] == 16 && p_data[11] == 16 && p_data[12] == 128) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1010), reply1010);
    }
    if (p_data[10] == 16 && p_data[11] == 61 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply103D), reply103D);
    }
    if (p_data[10] == 16 && p_data[11] == 32 && p_data[12] == 96) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply1020), reply1020);
    }
    if (p_data[10] == 64 && p_data[11] == 1) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply4001), reply4001);
    }
    if (p_data[10] == 72 && p_data[11] == 1) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply4801), reply4801);
    }
    if (p_data[10] == 34 && p_data[11] == 1) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply3401), reply3401);
    }
    if (p_data[10] == 48 /*&& p_data[11] == 1*/) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply3001), reply3001);
      paired = 1;
    }

    if (p_data[10] == 33 && p_data[11] == 33) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply3333), reply3333);
      paired = 1;
    }
    if (p_data[10] == 64 && p_data[11] == 2) {
      esp_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xa1,
                                 sizeof(reply4001), reply4001);
    }

    ESP_LOGI(
        TAG,
        "got an interrupt report from host, subcommand: %d  %d  %d Length: %d",
        p_data[10], p_data[11], p_data[12], len);
  } else {
    // ESP_LOGI("heap size:", "%d", xPortGetFreeHeapSize());
    // ESP_LOGI(TAG, "pairing packet size != 49, subcommand: %d  %d  %d  Length:
    // %d", p_data[10], p_data[11], p_data[12], len);
  }
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
    for (int i = 0; i < 8; i++) bt_addr[i] = esp_random() % 255;
    size_t addr_size = sizeof(bt_addr);
    err = nvs_set_blob(my_handle, "mac_addr", bt_addr, addr_size);
  }

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
  spi_init();
  xTaskCreatePinnedToCore(get_buttons, "gbuttons", 2048, NULL, 1,
                          &ButtonsHandle, 1);
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

  xSemaphore = xSemaphoreCreateMutex();

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
  // gap_callbacks = get_device_cb;

  app_param.name = "BlueCubeMod";
  app_param.description = "BlueCubeMod Example";
  app_param.provider = "ESP32";
  app_param.subclass = 0x002508;
  memset(&both_qos, 0, sizeof(esp_hidd_qos_param_t));

  callbacks.application_state_cb = application_cb;
  callbacks.connection_state_cb = connection_cb;
  callbacks.get_report_cb = get_report_cb;
  callbacks.set_report_cb = set_report_cb;
  callbacks.set_protocol_cb = set_protocol_cb;
  callbacks.intr_data_cb = intr_data_cb;
  callbacks.vc_unplug_cb = vc_unplug_cb;

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
  esp_bt_dev_set_device_name("Joy-Con (L)");

  ESP_LOGI(TAG, "setting to connectable, discoverable");
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  // esp_hid_device_connect
  // start blinking
  xTaskCreate(startBlink, "blink_task", 1024, NULL, 2, &BlinkHandle);
}
//
