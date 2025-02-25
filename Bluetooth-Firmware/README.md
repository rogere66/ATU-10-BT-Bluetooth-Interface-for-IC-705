# Bluetooth-Firmware:

The Bluetooth interface (BT) is controlled entirely trough a UART serial port with Rx and Tx lines. The Rx line also controls the BT power state: when the line is permanently low the module enters Shut Down Sleep, and when high the module is Active. When Active, BT will enter one of three modes:
 - **Unpaired**: BT will search for any device named `ICOM BT(IC-705)`, the default device name for IC-705. On the IC-705 you then need to select `<<Pairing Reception>>` in the MENU/SET/Bluetooth Set menu, and BT will then connect and pair to this device.
 - **Paired**: BT will try to connect as long as it is Active.
 - **Connected**: BT will receive Frequency, SWR and other information from the Transceiver and send relevant info to the Tuner.

The BT interface includes a 2nd serial port connector for programming and debugging - this port also requires the RTS and CTS signals, in addition to Rx and Tx. When programmed it is possible to use a serial terminal application and connect to the serial port at 115200 baud. To connect, press the Reset button on the BT board, then hit `r` or `linefeed` on the terminal, then `CLI` to enable Command Line commands (`h` for help). From there you can Unpair BT to start a new pairing sequence, display some status info and not least connect to the Tuner, which also have some CLI commands.

### Building and Programming:
Programming the Bluetooth interface requires a serial port with RX, TX, RTS and CTS connected. Programming can be done using Infineon ModusToolbox:
- Download ModusToolbox from the https://www.infineon.com/ website and follow the installation instructions for Eclipse based IDE.
- Open the ModusToolbox IDE and select `New Application` on the `Quick Panel/Eclipse based IDE for ModusToolbox` tab to open the Project Creator window.
- Search for `CYBT-413061-EVAL` board, select `Bluetooth/RFCOMM Serial Port` sample project and hit `Create` to create the project.
- Select the new project on the Project tab (default name RFCOMM_Serial_Port) and hit `Build Application` on the Quick Panel tab.
- Locate the `build/CYBT-413061-EVAL/Debug` directory for the project and replace `BT_SPP_download.hex` with the release file for IC-705-Tuner-BT.
- Push and hold the RECOVERY button on the ATU-10-BT board, push and release the RESET button and then release the RECOVERY button after 1 second.
- Open a Terminal widow and CD into the RFCOMM_Serial_Port project directory.
- Enter command `make qprogram` to program the module (option `UART=<PORT>` can be added to select a specific port).

There are probably easier methods for just programming the module, but now you also have the tools to build the code:
- Delete existing source files `spp.c` and `COMPONENT_btstack_v1/wiced_bt_cfg.c`
- Copy the IC-705-Tuner-BT source files into the project directory.
- Select `Clean` and then `Build Application`.
