# ATU-10-BT: ATU-10 with Bluetooth Interface for Icom IC-705

![](Pictures/ATU-10-BT.jpg)

This project adds Bluetooth interface to the ATU-10 for use with the Icom IC-705 transceiver. The code is based on ATU-10 FW version 1.6 and is ported from microC to the free MPLAB XC8 compiler. When not connected via Bluetooth to IC-705, the Tuner will mainly operate as with the original FW 1.6.

Main New Features:
- Bluetooth interface for connection to the IC-705 transceiver:
  - Receive frequency/band and SWR information from the transceiver
  - Save and restore relay settings for each band
  - Update the Tuner SWR reading to match the transceiver SWR reading
- Optionally add a HF-VHF/UHF antenna switch and additional BNC connector
- Show current relay setting and band on the OLED display
- OLED Settings menu for Cell Parameters, BT unpair and more
- Slightly more persistent tuning algorithm that may tune some previously difficult antennas

Background:\
After testing the Icom AH-705, the mAT-705 and ATU-10 tuners it appeared that all had less than desired integration level with the transceiver, i.e. they all actually needed RF transmission in order to set the relays to previously tuned bands. A Bluetooth interface for the ATU-10 then seemed like a reasonable solution - and a reasonable sized project since it was all open source.

For more information, see the README files for each part of the project.
#
README file for ATU-10 by N7DDC FW version 1.6:
# ATU-10  - The Tyny QRP Automatic Antenna Tuner

### Official conversation group - https://groups.io/g/ATU100
### Schematic and assembly instruction by VK3PE - http://carnut.info/ATU_N7DDC/ATU-10/ATU-10_by-vk3pe_build_info/ATU-10_vk3pe_V1.2_ALL_INFO_290921.pdf

###### New in FW version 1.6
1 - New better meassurement formula for Power and SWR calculation, compensation and calibration.  
2 - Setting by Cells control implementation. There are 10 cells you can find opening a FW .hex file in Notepad++ program as Intel HEX file. 
At the end by address EEE0 and EEF0 you can see them and change values. After changing any value you must to correct a checksumm for each 
changed string. Copy the needed string into Checsumm.html and get correct checksumm. It should be green color before you can save changed .hex FW file. 

[![](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/Cells.jpg)](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/Cells.jpg)

Cells description:
1) Time to display off in minutes, 5 mins by default, 0 to always on display
2) time to power off in minutes, 30 mins by default, 0 to always power on
3) relay's delay time, voltage applied to coiles in ms, 7 ms by default
4) min power to start tuning in ten's parts of Watt, 10 by default (1.0 W). This value can not be 0
5) max power to start tuning in watts, 15 by default
6) Delta SWR to auto start tuning in ten's parts SWR, 13 by default (SWR = 1.3)
7) Auto mode 1 for activate or 0 to off. 1 by default
8) Calibration coefficient for 1W power, 4 by default for BAT41 diodes
9) calibration coefficient for 10W power, 14 by default for BAT41 diodes
10) Peak detector time for Power meassurement in tens ms, 60 (600ms)

If you are using 1N5711 diodes in the RF detector, you can calibrate the power meassurement.
Apply known 1W of power on 7MHz and change cell 8 for correct value.
Apply known 10W of power on 7MHw and change cell 9 for correct value.
Repeat it couple times to reach a good result.

###### New in FW version 1.5
1 - Tuning algorithm improvement   
2 - Minor changes 

###### New in FW version 1.4
1 - The button glitches solved   
2 - More stability in bus transfer to OLED   
3 - No continious data transferring to the display   

###### New in FW version 1.2  
1 - Display memory feature for last SWR  added  
2 - full automatic mode error solved.

###### New in FW version 1.1  
1 - the control method has been reworked, there is no more sleep mode. Now the tuner either shines on the display for 5 minutes after it is disturbed, or turns off after 30 minutes if it is not touched and the transmitter is not turned on. For these 30 minutes, the tuner constantly monitors the power supplied and instantly lights up the display when needed. The current consumption in this monitoring mode is 4 mA.
Long press on the button now turns the device on and off.  
2 - external control using the Icom protocol is implemented, works in both directions. That is, when the tuner button is pressed, the transceiver automatically generates a carrier for tuning and when changing from band to band, the transceiver initiates tuning by the tuner. When you press the button of external tuner control on the transceiver, the tuner is automatically run if ON or reset if OFF position.  

### Description
   The tuner is assembled in an affordable Chinese case 100x71x25 mm, the front and rear panels are made as PCB, in the same way as the main printed circuit board.
On the front panel there is a control button, a small 0.91" OLED 128 * 32 display and a USB Type C connector, used to charge the tuner's built-in battery and connect to a computer to change the firmware.

[![](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_1.jpg)](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_1.jpg)

   The rear panel contains RF BNC connectors, a ground clamp and an external control interface connector that can be connected to the transceiver for more convenient control of the tuner.

[![](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_2.jpg)](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_2.jpg)

   The control button has only 3 functions - a short press resets the tuner and sets all relays to their initial state, in which all reactive elements are disabled and do not affect the signal flow through the tuner, a long press causes the tuner to enter the tuning mode and a very long press for more than 5 seconds causes the firmware version to be displayed on the display.
   The display mainly shows the current transmitter power and SWR in the transmitter cable and sometimes briefly indicates the modes.
   
   The tuner is built on IM41 bistable relays, which means that a significant current is consumed by the tuner only for a short time during tuning; in rest mode, the tuner's relays retain their state for an arbitrarily long time, without consuming power.

   The built-in battery consists of two Li-Ion 14500 batteries connected in parallel. The best examples of these (SANYO) have a capacity of 800 mAh each, the worst ones are usually 400 mAh.
    The tuner has 3 power consumption modes - the operating mode, when the display indicates the parameters, lasts 5 minutes, after which the tuner falls asleep and the display turns off. You can wake him up with a short press on the button. After 30 minutes of sleep, the tuner turns off completely and you can turn it on by holding the button for a long time (more than 3 seconds).
    
   The current consumption in the operating mode is 12 mA, in the sleep mode 170 μA, in the off state 37 μA.
   
[![](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_3.jpg)](https://github.com/Dfinitski/ATU-10-10W-QRP-antenna-tuner/blob/main/Photos/tuner_3.jpg)

   The main microprocessor of the tuner is PIC16F18877, another PIC16F1454 processor is used as an embedded programmer. It should be flashed once using the programmer with a special firmware, after which it will be possible to change the main firmware of the tuner as many times as necessary without resorting to special means. After connecting the tuner to the computer, a new logical disk with the appropriate name will appear in the explorer, in which you can find a link to the firmware repository. To flash the tuner, it is enough to copy the firmware file to this disk, the process takes a couple of seconds and the tuner is flashed at the same moment when the file copying is completed.
   
   There is also space for two dual-color LEDs with a common anode on the board, they can be installed if you do not need an OLED display.
The right LED indicates the battery charging process, the left LED indicates the operation. In operating mode, it blinks briefly in color every three seconds, depending on the battery charge level.

