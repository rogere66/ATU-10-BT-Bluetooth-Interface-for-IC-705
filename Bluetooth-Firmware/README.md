# Bluetooth-Firmware:

The Bluetooth interface (BT) is controlled entirely trough a UART serial port with Rx and Tx lines. The Rx line also controls the BT power state: when the line is permanently low the module enters Shut Down Sleep, and when high the module is Active. When Active, BT will enter one of three modes:
 - **Unpaired**: BT will search for any device named `ICOM BT(IC-705)`, the default device name for IC-705. On the IC-705 you then need to select `<<Pairing Reception>>` in the MENU/SET/Bluetooth Set menu, and BT will then connect and pair to this device.
 - **Paired**: BT will try to connect as long as it is Active.
 - **Connected**: BT will receive Frequency, SWR and other information from the Transceiver and send relevant info to the Tuner.

The BT interface includes a 2nd serial port connector for programming and debugging - this port also requires the RTS and CTS signals, in addition to Rx and Tx. When programmed it is possible to use a serial terminal application and connect to the serial port at 115200 baud. To connect, press the Reset button on the BT board, then hit `r` or `linefeed` on the terminal, then `CLI` to enable Command Line commands (`h` for help). From there you can Unpair BT to start a new pairing sequence, display some status info and not least connect to the Tuner, which also have some CLI commands.

### Programming the BT Interface:
Programming the Bluetooth interface requires a serial port with RX, TX, RTS and CTS connected. Programming can be done using the included AIROC_MOD_Prog programmer on a Windows PC:
- Unzip the file and run the AIROC_MOD_Programmer.
- Select Module Name `CYBT-413061-02`.
- Select serial port on one of the DUT lines.
- Select the Firmware File.
- Tick the `Program BD Address` and `Gen a random address` boxes to get a unique local BD address.
- Put the ATU-10-BT board in download mode: Push and hold the RECOVERY button, push and release the RESET button and then release the RECOVERY button after 1 second.
- Hit `Program` and wait for completion.

### Programming and Building the code on any platform:
Programming can also be done using Infineon ModusToolbox:
- Download ModusToolbox from the https://www.infineon.com/ website and follow the installation instructions for Eclipse based IDE.
- Open the ModusToolbox IDE and select `New Application` on the `Quick Panel/Eclipse based IDE for ModusToolbox` tab to open the Project Creator window.
- Search for `CYBT-413061-EVAL` board, select `Bluetooth/RFCOMM Serial Port` sample project and hit `Create` to create the project.
- Select the new project on the Project tab (default name RFCOMM_Serial_Port) and hit `Build Application` on the Quick Panel tab.
- Locate the `build/CYBT-413061-EVAL/Debug` directory for the project and replace `BT_SPP_download.hex` with the release file for IC-705-Tuner-BT (name need to be changed).
- Put the ATU-10-BT board in download mode: Push and hold the RECOVERY button, push and release the RESET button and then release the RECOVERY button after 1 second.
- Open a Terminal widow and CD into the RFCOMM_Serial_Port project directory.
- Enter command `make qprogram` to program the module (option `UART=<PORT>` can be added to select a specific port). This will program the module, but it will NOT get a unique BD address, which may be a problem if 2 or more modules are used near each other.

A full ModusToolbox IDE for just programming the module mau be overkill, but now you also have the tools to build the code:
- Delete existing source files `spp.c` and `COMPONENT_btstack_v1/wiced_bt_cfg.c`
- Copy the IC-705-Tuner-BT source files into the project directory.
- Open the `Library Manager` on the Quick Panel tab and change the CYW20721B2 library to release 4.2.1. If the CYW20721B2 library is not listed, use the `Add Library` tab to add it. Update and Close when done.
- Open the `Device Configurator` on the Quick Panel tab and untick all pins and devices _except_ the main `Pins` box. Save and exit when done.
- Select `Clean` and then `Build Application`.

**NOTE:** The reason for changing the CYW20721B2 library to release 4.2.1 is that later releases have problems with Shut-Down-Sleep. The same problem arise when using the Device Configurator for pin allocation, thus all allocations are done in the source code. If compiling the code it is thus advisable to check the current consumption of the BT module when in Shut-Down-Sleep, i.e. when power is on and the Rx pin pin is Low - it should be less than 5 uA.

Also note that the changes to library and pin selection is done on the BSP for the CYBT-413061-EVAL board and if a new application is created using this board, the BSP will be overwritten and the changes lost. This can be avoided by making a local copy of the BSP with a different name (also on the .mk file inside the BSP) and updating the makefile to enable and use this BSP.
