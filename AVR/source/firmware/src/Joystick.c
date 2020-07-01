/*
Nintendo Switch Fightstick - Proof-of-Concept

Based on the LUFA library's Low-Level Joystick Demo
    (C) Dean Camera
Based on the HORI's Pokken Tournament Pro Pad design
    (C) HORI

This project implements a modified version of HORI's Pokken Tournament Pro Pad
USB descriptors to allow for the creation of custom controllers for the
Nintendo Switch. This also works to a limited degree on the PS3.

Since System Update v3.0.0, the Nintendo Switch recognizes the Pokken
Tournament Pro Pad as a Pro Controller. Physical design limitations prevent
the Pokken Controller from functioning at the same level as the Pro
Controller. However, by default most of the descriptors are there, with the
exception of Home and Capture. Descriptor modification allows us to unlock
these buttons for our use.
*/

#include "Joystick.h"

#include <stdio.h>

typedef enum {
  SYNCED,
  SYNC_START,
  SYNC_1,
  OUT_OF_SYNC,
  CHOCO_SYNC_1,
  CHOCO_SYNCED
} State_t;

typedef struct {
  uint8_t input[8];
  uint8_t crc8_ccitt;
  uint8_t received_bytes;
} USB_Input_Packet_t;

USB_Input_Packet_t usbInput;
USB_JoystickReport_Input_t buffer;
USB_JoystickReport_Input_t defaultBuf;
State_t state = OUT_OF_SYNC;

uint8_t upperButtons;
uint8_t lowerButtons;
uint8_t stickX = 128;
uint8_t stickY = 128;
bool leftShoulder;
bool rightShoulder;
bool stickClick;

ISR(USART1_RX_vect) {
  uint8_t b = recv_byte();
  if (state == SYNC_START) {
    if (b == COMMAND_SYNC_1) {
      state = SYNC_1;
      send_byte(RESP_SYNC_1);
    } else if (b == COMMAND_CHOCO_SYNC_1) {
      state = CHOCO_SYNC_1;
      send_byte(RESP_CHOCO_SYNC_1);
    } else {
      state = OUT_OF_SYNC;
    }
  } else if (state == SYNC_1) {
    if (b == COMMAND_SYNC_2) {
      state = SYNCED;
      send_byte(RESP_SYNC_OK);
    } else {
      state = OUT_OF_SYNC;
    }
  } else if (state == CHOCO_SYNC_1) {
    if (b == COMMAND_CHOCO_SYNC_2) {
      state = CHOCO_SYNCED;
      send_byte(RESP_CHOCO_SYNC_OK);
    } else {
      state = OUT_OF_SYNC;
    }
  } else if (state == SYNCED || state == CHOCO_SYNCED) {
    if (usbInput.received_bytes < 8) {
      // Still filling up the buffer
      usbInput.input[usbInput.received_bytes++] = b;
      usbInput.crc8_ccitt = _crc8_ccitt_update(usbInput.crc8_ccitt, b);

    } else {
      if (usbInput.crc8_ccitt != b) {
        if (b == COMMAND_SYNC_START) {
          // Start sync
          state = SYNC_START;
          send_byte(RESP_SYNC_START);
        } else {
          // Mismatched CRC
          send_byte(RESP_UPDATE_NACK);
          PRINT_DEBUG(
              "Packet specified CRC 0x%02x but calculated CRC was 0x%02x\n", b,
              usbInput.crc8_ccitt);
          PRINT_DEBUG(
              "Packet data: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
              usbInput.input[0], usbInput.input[1], usbInput.input[2],
              usbInput.input[3], usbInput.input[4], usbInput.input[5],
              usbInput.input[6], usbInput.input[7], b);
        }

      } else {
        // Everything is ok
        /*
        typedef enum {
    SWITCH_Y       = 0x01,
    SWITCH_B       = 0x02,
    SWITCH_A       = 0x04,
    SWITCH_X       = 0x08,
    SWITCH_L       = 0x10,
    SWITCH_R       = 0x20,
    SWITCH_ZL      = 0x40,
    SWITCH_ZR      = 0x80,
    SWITCH_MINUS   = 0x100,
    SWITCH_PLUS    = 0x200,
    SWITCH_LCLICK  = 0x400,
    SWITCH_RCLICK  = 0x800,
    SWITCH_HOME    = 0x1000,
    SWITCH_CAPTURE = 0x2000,
} JoystickButtons_t;*/

        if (state == CHOCO_SYNCED) {
          usbInput.input[0] &= ~(1 << 7);  // Clear SL
          usbInput.input[0] &= ~(1 << 6);  // Clear SR
          buffer.Button = (usbInput.input[0] << 8) | usbInput.input[1];
          buffer.HAT = usbInput.input[2];
          buffer.LX = usbInput.input[3];
          buffer.LY = usbInput.input[4];
          buffer.RX = usbInput.input[5];
          buffer.RY = usbInput.input[6];
          buffer.VendorSpec = usbInput.input[7];
        } else if (state == SYNCED) {
          stickX = 128;
          stickY = 128;

          leftShoulder = ((usbInput.input[0] >> 6) & 1) ||
                         ((usbInput.input[1] >> 4) & 1) ||
                         ((usbInput.input[1] >> 6) & 1);
          rightShoulder = ((usbInput.input[0] >> 7) & 1) ||
                          ((usbInput.input[1] >> 5) & 1) ||
                          ((usbInput.input[1] >> 7) & 1);

          lowerButtons = (((usbInput.input[1] >> 0) & 1) << 0) +  // Y
                         (((usbInput.input[1] >> 1) & 1) << 1) +  // B
                         (((usbInput.input[1] >> 2) & 1) << 2) +  // A
                         (((usbInput.input[1] >> 3) & 1) << 3) +  // X
                         (leftShoulder << 4) +                    // L
                         (rightShoulder << 5) +                   // R
                         (leftShoulder << 6) +                    // ZL
                         (rightShoulder << 7);                    // ZR

          upperButtons = ((((usbInput.input[0] >> 1) & 1) ||
                           ((usbInput.input[0] >> 0) & 1))
                          << 1) +  // Plus
                         ((((usbInput.input[0] >> 2) & 1) ||
                           ((usbInput.input[0] >> 3) & 1))
                          << 2) +                                 // L Click
                         (((usbInput.input[0] >> 4) & 1) << 4) +  // Home
                         (((usbInput.input[0] >> 5) & 1) << 5);   // Capture

          buffer.Button = (upperButtons << 8) | lowerButtons;
          buffer.HAT = A_DPAD_CENTER;
          if (usbInput.input[2] != A_DPAD_CENTER) {
            switch (usbInput.input[2]) {
              case A_DPAD_CENTER:
                break;
              case A_DPAD_U:
                stickY = 0;
                break;
              case A_DPAD_R:
                stickX = 255;
                break;
              case A_DPAD_D:
                stickY = 255;
                break;
              case A_DPAD_L:
                stickX = 0;
                break;
              case A_DPAD_U_R:
                stickY = 0;
                stickX = 255;
                break;
              case A_DPAD_U_L:
                stickY = 0;
                stickX = 0;
                break;
              case A_DPAD_D_R:
                stickY = 255;
                stickX = 255;
                break;
              case A_DPAD_D_L:
                stickY = 255;
                stickX = 0;
                break;

              default:
                break;
            }
          } else {
            if (usbInput.input[3] != 128 || usbInput.input[4] != 128) {
              stickX = usbInput.input[3];
              stickY = usbInput.input[4];
            } else if (usbInput.input[5] != 128 || usbInput.input[6] != 128) {
              stickX = usbInput.input[5];
              stickY = usbInput.input[6];
            }
          }
          buffer.LX = stickX;
          buffer.LY = stickY;
          buffer.RX = 0x80;
          buffer.RY = 0x80;
          buffer.VendorSpec = usbInput.input[7];
        }
        // send_byte(RESP_UPDATE_ACK);
      }
      usbInput.received_bytes = 0;
      usbInput.crc8_ccitt = 0;
    }
  }
  if (state == OUT_OF_SYNC) {
    if (b == COMMAND_SYNC_START) {
      state = SYNC_START;
      send_byte(RESP_SYNC_START);
    }
  }
}

// Main entry point.
int main(void) {
  // We also need to initialize the initial input reports.
  memset(&defaultBuf, 0, sizeof(USB_JoystickReport_Input_t));
  defaultBuf.LX = STICK_CENTER;
  defaultBuf.LY = STICK_CENTER;
  defaultBuf.RX = STICK_CENTER;
  defaultBuf.RY = STICK_CENTER;
  defaultBuf.HAT = HAT_CENTER;
  memcpy(&buffer, &defaultBuf, sizeof(USB_JoystickReport_Input_t));

  memset(&usbInput, 0, sizeof(USB_Input_Packet_t));

  // We'll start by performing hardware and peripheral setup.
  SetupHardware();
  // We'll then enable global interrupts for our use.
  GlobalInterruptEnable();
  // Once that's done, we'll enter an infinite loop.
  for (;;) {
    // We need to run our task to process and deliver data for our IN and OUT
    // endpoints.
    HID_Task();
    // We also need to run the main USB management task.
    USB_USBTask();
  }
}

// Configures hardware and peripherals, such as the USB peripherals.
void SetupHardware(void) {
  // We need to disable watchdog if enabled by bootloader/fuses.
  disable_watchdog();

  // We need to disable clock division before initializing the USB hardware.
  clock_prescale_set(clock_div_1);
  // We can then initialize our hardware and peripherals, including the USB
  // stack.
  USART_Init(19200);

  // The USB stack should be initialized last.
  USB_Init();
}

// Fired to indicate that the device is enumerating.
void EVENT_USB_Device_Connect(void) {
  // We can indicate that we're enumerating here (via status LEDs, sound, etc.).
}

// Fired to indicate that the device is no longer connected to a host.
void EVENT_USB_Device_Disconnect(void) {
  // We can indicate that our device is not ready (via status LEDs, sound,
  // etc.).
}

// Fired when the host set the current configuration of the USB device after
// enumeration.
void EVENT_USB_Device_ConfigurationChanged(void) {
  bool ConfigSuccess = true;

  // We setup the HID report endpoints.
  ConfigSuccess &= Endpoint_ConfigureEndpoint(
      JOYSTICK_OUT_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
  ConfigSuccess &= Endpoint_ConfigureEndpoint(
      JOYSTICK_IN_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);

  // We can read ConfigSuccess to indicate a success or failure at this point.
}

// Process control requests sent to the device from the USB host.
void EVENT_USB_Device_ControlRequest(void) {
  // We can handle two control requests: a GetReport and a SetReport.

  // Not used here, it looks like we don't receive control request from the
  // Switch.
}

// Process and deliver data from IN and OUT endpoints.
void HID_Task(void) {
  // If the device isn't connected and properly configured, we can't do anything
  // here.
  if (USB_DeviceState != DEVICE_STATE_Configured) return;

  // We'll start with the OUT endpoint.
  Endpoint_SelectEndpoint(JOYSTICK_OUT_EPADDR);
  // We'll check to see if we received something on the OUT endpoint.
  if (Endpoint_IsOUTReceived()) {
    // If we did, and the packet has data, we'll react to it.
    if (Endpoint_IsReadWriteAllowed()) {
      // We'll create a place to store our data received from the host.
      USB_JoystickReport_Output_t JoystickOutputData;

      // We'll then take in that data, setting it up in our storage.
      while (Endpoint_Read_Stream_LE(&JoystickOutputData,
                                     sizeof(JoystickOutputData),
                                     NULL) != ENDPOINT_RWSTREAM_NoError)
        ;

      // At this point, we can react to this data.

      // However, since we're not doing anything with this data, we abandon it.
    }
    // Regardless of whether we reacted to the data, we acknowledge an OUT
    // packet on this endpoint.
    Endpoint_ClearOUT();
  }

  // We'll then move on to the IN endpoint.
  Endpoint_SelectEndpoint(JOYSTICK_IN_EPADDR);
  // We first check to see if the host is ready to accept data.
  if (Endpoint_IsINReady()) {
    // We'll create an empty report.
    USB_JoystickReport_Input_t JoystickInputData;

    // We'll then populate this report with what we want to send to the host.
    disable_rx_isr();
    if (state == SYNCED || state == CHOCO_SYNCED) {
      memcpy(&JoystickInputData, &buffer, sizeof(USB_JoystickReport_Input_t));
      send_byte(RESP_USB_ACK);
    } else {
      memcpy(&JoystickInputData, &defaultBuf,
             sizeof(USB_JoystickReport_Input_t));
    }
    enable_rx_isr();

    // Once populated, we can output this data to the host. We do this by first
    // writing the data to the control stream.
    while (Endpoint_Write_Stream_LE(&JoystickInputData,
                                    sizeof(JoystickInputData),
                                    NULL) != ENDPOINT_RWSTREAM_NoError)
      ;

    // We then send an IN packet on this endpoint.
    Endpoint_ClearIN();
  }
}
