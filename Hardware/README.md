# ATU-10-BT Bluetooth Interface Hardware
The Bluetooth interface (BT) use the Cypress CYBT-413061-02 Bluetooth module, based on the CYW20721 SOC. The module has fairly low power consumption which makes it suitable for battery operation. It is mounted on a mezzanine board which plugs into existing connectors on the ATU-10 board - at least that is the case for the ATU-10 used for development.

[BT Interface Schematic](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Hardware/ATU-10-Bluetooth-IF.pdf)

The BT module is mounted bottom up on the mezzanine board with the antenna over the OLED display and towards the front panel. Part of the copper layer on the inside of the front panel is removed to (hopefully) get more BT RF to the outside - seems to work quite well anyways.

![Bluetooth Interface Mezzanine Board](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-IF-Mount.jpg)

Hand soldering the BT board is not too bad with 1mm pin pitch on the BT module. The prototype use a board with a contiuous copper layer on the bottom side connected to GND - this works great and can be recommended.

BT Interface Component Side:\
![BT Board Component Side](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-IF-Top.jpg)

BT Interface Bottom Side:\
![BT Board Bottom Side](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/BT-IF-Bot.jpg)

The BT interface is controlled entirely trough a UART serial port with Rx and Tx lines. The Rx and Tx lines are connected to the pins normally driving LED1 on the ATU-10 board. LED1 is not needed on an ATU-10 with OLED display, and is usually not mounted - if mounted it has to be removed and replaced with header pins. If not already mounted, J2 also need header pins. It is also necessary to remove (or replace) C51 and C52 connected to the LED pins since the installed 10nF capacitors usually mounted will kill the Rx and Tx signals. The capacitors can be replaced with 220 pF ones if there is any issue with RF interference on the Rx and Tx lines, but this has not been an issue so far.

ATU-10 Board:\
![ATU-10 Board Component Side](https://github.com/rogere66/ATU-10-BT-Bluetooth-Interface-for-IC-705/blob/main/Pictures/ATU-10-PCB-Top.jpg)

There seems to be a few different versions of the ATU-10 available and some may be more suited than others for this BT modification. The one used for development has a few noticeable features different from other versions: the end panels are fastened with hex cap-head screws, while others may use countersunk Phillips heads, and the BNC connectors are fastened with nuts, while others don't have nuts. 

A note on the OLED display: the tuner used for development had an issue with random errors on the display, and on a replacement display. This was tracked down to be a bad LDO voltage regulator on both displays and was fixed by replacing the LDO (f.e. AP2138N-3.3) - it is the 3 pin chip on the back side of the display.

Antenna Switch:\
The project also includes an optional HF-VHF/UHF antenna switch and additional BNC connector. Mounting the BNC connector is a very thight fit and requires to remove the 3.5mm JACK1 (which is now redundant) and move T8 slightly sideways. A new back banel is also needed. The switch cause some issues with SWR on the UHF band, but keeping the coax short (like 40cm) between tranceiver and tuner bring this down to an acceptable level.


