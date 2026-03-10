// ATU-10-BT-Comms.h - ATU-10 Tuner / Bluetooth interface (BT) control message definitions

#ifndef ATU_10_BT_COMMS_H
#define ATU_10_BT_COMMS_H


// All control bytes have the T_CBIT set and consist of 1-2 bytes followed by a CRC byte.
// Each byte consist of 3 bit command code + 5 bit data/sub-command/CRC code.
// Some 1-byte commands are acknowledged by echoing the command byte + CRC.

#define T_CBIT  0x80  // command byte flag bit
#define T_CMSK  0xe0  // command field mask
#define T_DMSK  0x1f  // data field mask
#define T_DLEN  5     // data field bit count

#define T_SWR   0x80  // BT info: current SWR * 100, 100-1000 in 2 bytes, MSB first, no ACK
#define T_BAND  0xa0  // BT command: band number, 0 for below HF bands, HF_BANDS+1-3 for VHF/UHF/outside, >=ALL_BANDS if not set, ACK from Tuner
#define T_CRC   0xc0  // CRC byte: last message byte with 5-bit CRC code

#define T_TXON  0xe1  // BT info: transmit on, ACK from Tuner
#define T_TXOFF 0xe2  // BT info: transmit off, ACK from Tuner
#define T_NOPR  0xe3  // BT connection state: not paired - sent after each discovery/connection attempt, ACK from Tuner
#define T_PAIRD 0xe4  // BT connection state: paired, but not connected - sent after each connection attempt, ACK from Tuner
#define T_CONN  0xe5  // BT connection state: connected (and paired) - sent when connected, ACK from Tuner
#define T_TUNE  0xe6  // Tuner info: tuning in progress, no ACK
#define T_TEND  0xe7  // Tuner info: tuning completed or aborted, no ACK
#define T_UNPR  0xe8  // Tuner command: unpair bluetooth and start new discovery/pairing sequence, ACK from BT
#define T_ANTHF 0xe9  // Tuner command: set antenna relay to HF, used when not connected, ACK from BT
#define T_ANTVU 0xea  // Tuner command: set antenna relay to VHF/UHF, used when not connected, ACK from BT

#define HF_BANDS  11  // number of HF bands used - higher band number indicates VHF/UHF
#define ALL_BANDS 15  // total number of band codes used, numbered 0 - ALL_BANDS-1 (higher values indicates that band is unknown)
#define BAND_NAMES {"<HF  ","160m ","80m  ","60m  ","40m  ","30m  ","20m  ","17m  ","15m  ","12m  ","10m  ","6m   ","VHF  ","UHF  ",">HF  "}


#endif  // ATU_10_BT_COMMS_H
