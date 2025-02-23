# Tuner-Firmware:
The ATU10-BT code is based on ATU-10 FW version 1.6 and is ported from microC to the free MPLAB XC8 compiler. When not connected via Bluetooth to IC-705, the Tuner will mainly operate as with the original FW 1.6.

Main New Features:\
- Bluetooth interface for connection to the IC-705 transceiver:
  - Receive frequency/band and SWR information from the transceiver
  - Save and restore relay settings for each band
  - Update the Tuner SWR reading to match the transceiver SWR reading
- Optionally add a HF-VHF/UHF antenna switch and additional BNC connector
- Show current relay setting and band on the OLED display
- OLED Settings menu for Cell Parameters, BT unpair and more
- Slightly more persistent tuning algorithm that may tune some previously difficult antennas

Settings OLED Menu:\
The code now includes a Settings OLED menu which can be activated by pushing the button for more than 3 seconds. The menu is controlled by two button inputs:
 SELECT - long button push: do some action on current menu item.
 DONE   - short button push: done with current menu item, move on to next.
 
 POWER OFF  - power off sleep
 CELL PARAM - change the CELL parameters defined in FW 1.6 README file
 CLR BANDS  - clear relay settings for all bands
 UNPAIR BT  - unpair Bluetooth and start new pairing sequence
 STANDBY    - change standby settings
 ANT SWITCH - change antenna switch setting (only when not connected), SELECT to start
  "  HF     - HF antenna connector is selected, SELECT to change
  "  V/UHF  - VHF/UHF antenna connector is selected, SELECT to change
 BRIGHT=xxx - change OLED brightness, 0-100%
 RESTORE    - restore all settings in EEPROM to defaults
 TUNER INFO - show the power-up tuner info with firmware versions etc.
 <=DONE     - go back to normal Tuner opertion
 
 
STANDBY sub-menu:
 HOURS  xxx - number of hours active standby, select 0 to turn OFF
        OFF - standby off, SELECT to change
   INFINITE - standby infinitely on, SELECT to change
 DLAY S xxx - time between each connect retry
 <-DONE     - go back to main menu
 
If standby is enabled, the tuner will enter stanby mode after beeing connected to the transceiver and then disconnected. It will then try to reconnect for the standby HOURS set, then POWER OFF at timeout. A short button push will power the Tuner up again trying to reconnect for a period, then disable standby and do a normal POWER OFF if not connecting. In standby OFF mode the Tuner will still try to reconnect, but will do normal POWER OFF according to the CELL setting timeout and require >3 sek button push to wake. 

Note that the menu is blocking tuner operation and the tuner may be out of synk with the Bluetooth interface on exit - try a POWER OFF/ON cycle if there is any issue. The menu will time out after 10 minutes of inactivity.

The CELL parameters are as defined in FW 1.6 README file, but with some different default values:

Cells description:
1) Time to display off in minutes, 2 mins by default, 0 to always on display
2) time to power off in minutes, 30 mins by default, 0 to always power on
3) relay's delay time, voltage applied to coiles in ms, 3 ms by default (using AXICOM IM41 3VDC relays)
4) min power to start tuning in ten's parts of Watt, 10 by default (1.0 W). This value can not be 0
5) max power to start tuning in watts, 15 by default
6) Delta SWR to auto start tuning in ten's parts SWR, 13 by default (SWR = 1.3)
7) Auto mode 1 for activate or 0 to off. 1 by default
8) Calibration coefficient for 1W power, 4 by default for BAT41 diodes
9) calibration coefficient for 10W power, 14 by default for BAT41 diodes
10) Peak detector time for Power meassurement in tens ms, 60 (600ms)

Note particularly parameter 3, relay delay time - this may need to be increased if relays other than genuine AXICOM IM41 3VDC are used.



Bluetooth Modes:\

UNPAIRED: will try to pair for 2 minutes after power up, then Bluetooth is turned off.

PAIRED: will try to connect as long as power is on.

CONNECTED: provides BT communication between tuner and tranceiver. Will enter standby mode when disconnected.

STANDBY: will try to reconnect number of hours defined by the standby hours setting. Wake on short button push after standby period.

When not Connected or in Stanby modes the tuner will operate as a normal automatic tuner and will shut-down after the time specefined in the "cell" setting, 30 minutes default.














