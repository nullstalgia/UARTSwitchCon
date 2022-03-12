# UARTSwitchCon
ESP32 and Arduino (AVR) compatible, UART controlled Nintendo Switch controller emulators.

![](https://i.imgur.com/HjPuPFi.png) ![](https://i.imgur.com/OnOKpHm.png)

This is an amalgamation of the amazing works of [Nathan Reeves](https://github.com/NathanReeves/BlueCubeMod) and [wchill](https://github.com/wchill/SwitchInputEmulator).

Huge thanks to [mizuyoukan-ao](https://github.com/mizuyoukanao) for figuring out v12 Switch Firmware support!

This gives users an easy to use UART interface to automate/control their Nintendo Switch with their computer/anything with a configurable UART port.

The ESP32 can act as a Left or Right Joy-Con, or a Pro Controller. (Wireless only)

- Left Joy-Con is the recommended usage, as it has the least amount of fiddlyness.

A compatible USB-enabled AVR (Arduino Micro/ATMega32u4, ATMega16u2 (HoodLoader2? :3)) can act as a "HORIPAD S", which in turn acts like a Wired Pro Controller. (Wired only)

The protocol is described in Protocol.md. It is a carbon copy of SwitchInputEmulator's protocol, with the addition of SL and SR, and is used in both the ESP32 and AVR implementations. 

This was made for [ClubchatGames](https://github.com/nullstalgia/ClubchatGames), and has a few compromises for that project in mind. However, this should be more than compatible with many other games on the Nintendo Switch.
