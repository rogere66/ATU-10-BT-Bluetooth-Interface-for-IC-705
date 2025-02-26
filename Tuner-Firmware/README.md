# Tuner-Firmware:
The ATU10-BT code is based on ATU-10 FW version 1.6 and is ported from microC to the free MPLAB XC8 compiler. When not connected via Bluetooth to an IC-705, the Tuner will mainly operate as with the original FW 1.6.

Main New Features:\
- Bluetooth interface for connection to the IC-705 transceiver:
  - Receive frequency/band and SWR information from the transceiver
  - Save and restore relay settings for each band
  - Update the Tuner SWR reading to match the transceiver SWR reading
- Optionally add a HF-VHF/UHF antenna switch and additional BNC connector
- Show current relay setting and band on the OLED display
- OLED Settings menu for Cell Parameters, BT unpair and more
- Slightly more persistent tuning algorithm that may tune some previously difficult antennas




### OLED Display Layout:
The OLED display now includes Band Relay Setting and Bluetooth State in addition to Power, SWR and Battery.

![https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-OLED.jpg](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-OLED.jpg)

### Bluetooth Connection States:
When the Tuner is connected via Bluetooth to an IC-705, it will enter one of these States:
- UNPAIRED - Will try to pair for 2 minutes after power up, then Bluetooth is turned off.
- PAIRED - Will try to connect as long as power is on.
- CONNECTED - Provides BT communication between tuner and transceiver - and enter standby mode when disconnected.
- STANDBY - Will try to reconnect for number of hours defined by the standby hours setting. Wake on short button push after standby period.

When NOT Connected or in Standby mode the tuner will operate as a normal automatic tuner and will shutdown after the time specified in the CELL setting.

### Settings OLED Menu:
The code now includes a Settings OLED menu which can be activated by pushing the button for more than 3 seconds. The menu is controlled by two button inputs:
- SELECT - long button push: do some action on current menu item.
- DONE   - short button push: done with current menu item, move on to next.

Settings Menu:
- POWER OFF  - Power off sleep
- CELL PARAM - Change the CELL parameters defined in FW 1.6 README file
- CLR BANDS  - Clear relay settings for all bands
- UNPAIR BT  - Unpair Bluetooth and start new pairing sequence
- STANDBY    - Change standby settings
- RELAY TEST - Tuner and Antenna Relay Tests
- BRIGHT=xxx - Change OLED brightness, 0-100%
- RESTORE    - Restore all settings in EEPROM to defaults
- TUNER INFO - Show tuner power-up info with firmware versions.
- <=DONE     - Go back to normal Tuner operation

STANDBY sub-menu:
- HOURS  xxx - Number of hours active standby, select 0 to turn OFF, 255 for INFINITE
  - INFINITE - Standby Infinitely On, SELECT to change
  -   OFF - Standby Off, SELECT to change
- DLAY S xxx - Time between each connect retry
- <-DONE     - Go back to main menu

RELAY TEST sub-menu:
- ANT RELAY? - Change antenna switch setting (only when not connected), SELECT to start
  -  HF      - HF antenna connector is selected, SELECT to change
  -  V/UHF   - VHF/UHF antenna connector is selected, SELECT to change
- TUN RELAY? - SELECT to run and repeat Tuner relay test: Cycle all 15 relays
- <-DONE     - Go back to main menu

If standby is enabled, the tuner will enter standby mode after being connected to the transceiver and then disconnected. It will then try to reconnect for the standby HOURS set, then POWER OFF at timeout. A short button push will power the Tuner up again trying to reconnect for a period, then disable standby and do a normal POWER OFF if not connecting. In standby OFF mode the Tuner will still try to reconnect, but will do normal POWER OFF according to the CELL setting timeout and require >3 seconds button push to wake. 

Note that the menu is blocking tuner operation and the tuner may be out of sync with the Bluetooth interface on exit - try a POWER OFF/ON cycle if there is any issue. The menu will time out after 10 minutes of inactivity.

The CELL parameters are as defined in FW 1.6 README file, but with some different default values:
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

Note particularly parameter 3, relay delay time - this may need to be increased if relays other than genuine AXICOM IM41 3VDC are used. The Tuner CLI menu has a Relay Test (`t`) that may be useful in testing this.
