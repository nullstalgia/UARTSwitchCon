# UARTSwitchCon protocol.

### (and to an extent, SwitchInputEmulator protocol)

## Baud: 19200

## Handshake:

### Vanilla:

Goes into Chocolate mode... (I'm gonna need to rename this...)

Will simplify inputs according to the following comments.

      // Initial, vanilla handshake.
      // MASTER: 0xFF
      // SLAVE: 0xFF
      // MASTER: 0x33
      // SLAVE: 0xCC
      // MASTER: 0xCC
      // SLAVE: 0x33, then 0x90 with each packet sent as an HID
      
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
            
### Chocolate:

      // "Chocolate" handshake.
      // MASTER: 0xFF
      // SLAVE: 0xFF
      // MASTER: 0x44
      // SLAVE: 0xEE
      // MASTER: 0xEE
      // SLAVE: CONTROLLER_TYPE, then 0x90 with each packet sent as an HID

Goes into original mode.

Inputs are sent exactly as sent.

      
### Packets

| Byte | 0           | 1           | 2        | 3  | 4  | 5  | 6  | 7                     | 8    |
|------|-------------|-------------|----------|----|----|----|----|-----------------------|------|
| Data | MSB Buttons | LSB Buttons | HAT/DPad | LX | LY | RX | RY | Vendor Spec (unused?) | CRC8 |


Byte 0:

| Bit    | 0 | 1 | 2 | 3 | 4 | 5 | 6  | 7  |
|--------|---|---|---|---|---|---|----|----|
| Button | Y | B | A | X | L | R | ZL | ZR |

Byte 1:

| Bit    | 0     | 1    | 2     | 3     | 4    | 5       | 6  | 7  |
|--------|-------|------|-------|-------|------|---------|----|----|
| Button | Minus | Plus | L Clk | R Clk | Home | Capture | SL | SR |

Byte 2:

	A_DPAD_CENTER    = 0x08
	A_DPAD_U         = 0x00
	A_DPAD_U_R       = 0x01
	A_DPAD_R         = 0x02
	A_DPAD_D_R       = 0x03
	A_DPAD_D         = 0x04
	A_DPAD_D_L       = 0x05
	A_DPAD_L         = 0x06
	A_DPAD_U_L       = 0x07
	
Also can be visualized with:


| -- | TOP | -- |
| --:|:----:|:-- |
| 7  |  0   |  1 |
| 6  |  8   |  2 |
| 5  |  4   |  3 |
	
Byte 8:

CRC8 is calculated with this:

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