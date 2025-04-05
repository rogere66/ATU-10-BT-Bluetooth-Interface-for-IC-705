# Tuner Firmware
The ATU-10-BT code is based on ATU-10 FW version 1.6 and is ported from microC to the free MPLAB XC8 compiler. When not connected via Bluetooth to an IC-705, the Tuner will mainly operate as with the original FW v1.6, with additional features. The main purpose of the Bluetooth interface is to connect the Tuner to the IC-705 in order to change the Tuner relay setting on band change to previously tuned bands. 

Main New Features
- Bluetooth interface for connection to the IC-705 transceiver:
  - Receive frequency/band and SWR information from the transceiver.
  - Save and restore relay settings for each band.
  - Update the Tuner SWR reading to match the transceiver SWR reading.
  - Optionally add a HF-VHF/UHF antenna switch and additional BNC connector.
- Show current relay setting and band on the OLED display.
- Extensive OLED Settings menu for Cell Parameters, BT unpair, relay testing and more.
- Tune Memory providing fast tuning also when BT is not connected to transceiver.
- Slightly more persistent tuning algorithm that may tune some previously difficult antennas.

Note that if this firmware is used without the Bluetooth Interface it is important that LED1 is NOT installed - otherwise it will light up and drain the battery, also when power is off.

### Bluetooth Connection States
When the Tuner is connected via Bluetooth to an IC-705, it will enter one of these States:
- UNPAIRED - Will try to pair for 2 minutes after power up, then Bluetooth is turned off.
- PAIRED - Will try to connect as long as power is on.
- CONNECTED - Provides BT communication between tuner and transceiver - and enter standby mode when disconnected.
- STANDBY - Will try to reconnect for number of hours defined by the standby hours setting. Will wake and exit standby on any button action.

When NOT Connected or in Standby mode the tuner will operate as a regular automatic tuner and will shut down after the time specified in the CELL setting.

### OLED Display Layout
The OLED display includes Band Relay Setting and Bluetooth State in addition to Power, SWR and Battery.

![https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-OLED.jpg](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-OLED.jpg)

When the Tune Flag is on the Tuner will tune on next TX and the flag is cleared. A long Button push will set the Tune Flag again and force a manual re-tune. It can be worthwhile to re-tune if there is a significant SWR increase right after tuning. A short button push will set the Tuner in bypass and disable tuning. The SWR Correction Flag indicates that the SWR is adjusted to match the transceiver SWR - the correction and flag is cleared on band change.

### Settings OLED Menu
The code includes a Settings OLED menu which can be activated by pushing the button for more than 3 seconds. The menu is controlled by these button inputs:
- SELECT - long button push: do some action on current menu item.
- DONE   - short button push: done with current menu item, move on to next.
- UP/DOWN - double short button push: change step direction when changing values - down is indicated by a down arrow.

#### Settings Menu
- POWER OFF  - Power off sleep
- CLR BANDS  - Clear relay settings and set Tune Flag for all bands.
- CELL PARAM - Change the CELL parameters as defined in FW v1.6 README file.
- UNPAIR BT  - Unpair Bluetooth and start new pairing sequence (may fail if BT is busy - try again).
- STANDBY    - Change standby settings.
- RELAY TEST - Tuner and Antenna Relay Tests.
- RESTORE    - Restore all settings in EEPROM to defaults.
- TUNER CONF - Tuner configuration. Show info and set various tuner settings.
  - INFO     - Show power-up greeting and battery voltage.
  - SWR CORR ON/OFF x - SWR correction enable/disable.
  - DEBUG LEVEL x - Change debug level, 0-4. Sniff the Tuner TX line for printouts (115200 baud).
  - BRIGHT xx - Change OLED brightness.
- <=DONE     - Go back to normal Tuner operation.

#### STANDBY sub-menu
- HOURS  xxx - Number of hours active standby, select 0 to turn OFF, 255 for INFINITE.
  - INFINITE - Standby Infinitely On, SELECT to change.
  -   OFF - Standby Off, SELECT to change.
- DLAY S xxx - Time between each connect retry.
- <-DONE     - Go back to main menu.

If STANDBY is enabled, the tuner will enter STANDBY state after being connected to the transceiver and then disconnected. It will then try to reconnect for the standby HOURS set, then POWER OFF at timeout. Any button action will power the Tuner up again trying to reconnect for a period - another short button push will clear standby. In standby OFF mode the Tuner will still try to reconnect, but will do normal POWER OFF according to the CELL setting timeout and require >3 seconds button push to wake. 

#### RELAY TEST sub-menu
- ANT RELAY  - Change antenna switch setting (only when not CONNECTED), SELECT to start.
  -  HF      - HF antenna connector is selected, SELECT to change (may fail if BT is busy - try again).
  -  V/UHF   - VHF/UHF antenna connector is selected, SELECT to change.
- TUNER RLY  - SELECT to run Tuner relay test - cycle all 15 relays.
- TUNE L xxx - Manual tuning of inductive tuner element, 0-127.
- TUNE C xxx - Manual tuning of capacitive tuner element 0-127.
- TUNE I x   - Set LC/CL and Tune Flag (bit 1), 0-3.
- SAVE LCI   - Save LCI relay settings in EEPROM (only if band is known).
- <-DONE     - Go back to main menu.

Note that the menu is blocking tuner operation and the tuner may be out of sync with the Bluetooth interface on exit - try a power cycle if there is any issue. The menu will time out after 10 minutes of inactivity.

#### The CELL parameters are as defined in FW 1.6 README file, but with some different default values
1) Time to display off in minutes, 2 mins by default, 0 to always on display
2) Time to power off in minutes, 30 mins by default, 0 to always power on
3) Relay's delay time, voltage applied to coiled in ms, 3 ms by default (using AXICOM IM41 3VDC relays)
4) Min power to start tuning in ten's parts of Watt, 10 by default (1.0 W). This value can not be 0
5) Max power to start tuning in watts, 15 by default
6) Delta SWR to auto start tuning in ten's parts SWR, 13 by default (SWR = 1.3)
7) Auto mode 1 for activate or 0 to off. 1 by default
8) Calibration coefficient for 1W power, 4 by default for BAT41 diodes
9) calibration coefficient for 10W power, 14 by default for BAT41 diodes
10) Peak detector time for Power measurement in tens ms, 60 (600ms)

Note particularly parameter 3, relay delay time - this may need to be increased if relays other than genuine AXICOM IM41 3VDC are used. The OLED Settings Menu has a RELAY TEST that may be useful for testing.

### Tuning
The tuner can usually auto-tune to SWR 1.2 or better, except for 80m and 160m where it is more like SWR 1.5 due to limitations in the SWR measurement precision. It is often possible to improve the transceiver SWR with manual tuning (using OLED Settings/RELAY TEST/TUNE L/C/I), although usually not needed.

After tuning, the relay setting is stored in RAM and EEPROM tune memory, either in dedicated band slots if tuner is connected to the transceiver trough BT, or in random slots if unconnected. If connected, the relay setting is restored on band change while if unconnected the tune memory is searched for best SWR at tuning start and accepted as tuned if SWR is better than specified in Cell parameter 6, Delta SWR.

Note that if the tuner is first used in connected mode and all bands are tuned, the band memory then need to be cleared in order to be available for unconnected mode (using OLED Settings/CLR BANDS). If only some band slots are used in connected mode, the other slots are available for unconnected mode without further action.

### Power Consumption
The BT Interface will not significantly increase power consumption. When BT is Connected, the Tuner is turned off whenever the OLED display is off and this may actually reduce power consumption somewhat. Standby mode use some power, but reasonably good batteries should still last 1-2 months with 90 second retry interval. If using the Tuner for extended periods without using BT, Unpairing BT will reduce power consumption somewhat since it otherwise will continuously try reconnecting.

### Compiling and Programming the Code
The code can be compiled using the free MPLAB X IDE and XC8 compiler from [Microchip](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide). Just make a project for the pic16f18877 chip and copy in the Tuner sources. Note that the free version of the XC8 compiler has limited optimization and thus tuning is slower than for FW v1.6. However, this is to a large degree compensated for by optimizing delays in the code and also, when connected, tuning is often not needed.

Programming is done by connecting to the ATU-10 USB port and copying the `ATU-10-BT_vXX.hex` file to the `ATU-10 Prog` logical disk.
