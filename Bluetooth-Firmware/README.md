# Bluetooth-Firmware:

The Bluetooth interface (BT) is controlled entirely trough a UART serial port with Rx and Tx lines. The Rx line also controls the BT power state: when the line is permanently low the module enters Shut Down Sleep, and when high the module is Active. When Active, BT will enter one of three modes:
 - Unpaired: BT will search for any device named "ICOM BT(IC-705)", the defalt device name for IC-705. On the IC-705 you then need to select <<Pairing Reception>> in the MENU/SET/Bluetooth Set menu, and BT will then connect and pair to this device.
 - Paired: BT will try to connect as long as it is Active.
 - Connected: BT will receive Frequency, SWR and other information from the Tranceiver and send relevant info to the Tuner.
 
 The BT interface includes a 2nd serial port connector for programming and debugging - this port also requires the RTS and CTS signals, in addition to Rx and Tx. When programmed it is possible to use a serial terminal application and connect to the serial port, 115200 baud. To connect, press the Reset button on the BT board, then hit 'r' or 'linefeed' on the terminal, then 'CLI' to enable Command Line commands - 'h' for help. From there you can Unpair BT to start a new pairing sequence, display some status info and not least connect to the Tuner which also have some CLI commands.
 
Programming the Bluetooth interface can be done using Cypress ModusToolbox. 
