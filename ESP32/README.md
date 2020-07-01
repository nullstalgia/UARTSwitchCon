The ESP32 interface is based off of [Nathan Reeves' BlueCubeMod](https://github.com/NathanReeves/BlueCubeMod) (and a fork of it, [OpenSwitchPad](https://github.com/agustincampeny/OpenSwitchPad)).

Recommended UART Pins:

- Breakout board's USB-UART bridge
    - UART0
    - TX - TX
    - RX - RX
- UART2
    - UART2
    - TX - 19
    - RX - 26
    
Additional packets for the BT conversation was gotten from the logs of: [JoyControl](file:///home/tony/git/UARTSwitchCon/ESP32/source/firmware/build/old) for Linux machines.

More fundamental bits of information for BT conversation was gotten from the amazing work from [dekuNukem and team at Nintendo\_Switch\_Reverse\_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/).