//  BT_extension.h

#ifndef BT_EXTENSION_H
#define	BT_EXTENSION_H

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "/home/roger/MTworkspace/IC-705-Tuner-BT_v10/ATU-BT-comms.h"
#include "pic16f18877.h"

// BT timeouts:
#define BT_PAIRING_TIMEOUT_S   120 // pairing window after power-up (2 min)
#define BT_DFLT_RETRY_DELAY_S   84 // default retry delay used for timed sleep (~1.5 min)
#define BT_DEFAULT_RETRIES      40 // default numer of retries before power off/sleep (<1 hour)
#define DEFAULT_OLED_CONTRAST   30 // OLED contrast, 0-255

extern uint8_t currentBand;
extern uint8_t currentBTstate;
extern uint8_t pendingBTstate;
extern char btWasConnected;
extern int  SWRcorrection;
extern char tuneAbort;
extern char dbLevel;
extern char btExtVer[];
extern char cliActive;
extern char uartEnabled;
extern uint32_t btAwakeTimer;
extern uint32_t btPairingTimer;
extern unsigned int connRetryCntr;
extern unsigned int btConnRetries;
extern unsigned int sleepTime_s;
extern unsigned int sleepTstdby_s;
extern uint8_t oledContrast;

void show_Band_Relay (void);
char btSendCmdWithAck (uint8_t cmd);
void eraseStoredBand (char all);
void saveBandState (void);
uint8_t crc5 (int iVal);
void btCommProcess (void);
void btExtension_init (void);
void btExtension_main (void);
void uartRxIsr (void);
void uartInit (void);
void uartDeInit (void);
char uGetc (void);
uint8_t uGetCmd (void);
void uPuts (char *s);
uint8_t eepromRead (uint8_t addr);
void eepromWrite (uint8_t addr, uint8_t val);
void Delay_us (unsigned int delay);
void Delay_ms (unsigned int delay);
void ADC_Init (void);
void ADC_DeInit (void);
void ADC_Init_Advanced (char flag);
unsigned int ADC_Get_Sample (char adcChannel);
char Bcd2Dec (char bcd);
void IntToStr (int val, char *str);
void IntToHex (int val, char *str);

#define _ADC_INTERNAL_FVRH1 1
#define _ADC_INTERNAL_FVRH2 2
#define _ADC_INTERNAL_VREFH 4
#define _ADC_INTERNAL_VREFL 8

#define VDelay_ms(delay) Delay_ms((unsigned int)delay)

extern char uTxBuf[];

#define TXB_LEN 64
#define uPutc(c)       do{while(uartEnabled && !PIR3bits.TXIF);TXREG = c;}while(0)
#define uPrintf(f,...) do{snprintf(uTxBuf,TXB_LEN,f,##__VA_ARGS__);uPuts(uTxBuf);}while(0)
#define uSendCmd(cmd)  do{uPutc(cmd);uPutc(crc5(cmd)|T_CRC);}while(0)  // send command byte + CRC byte

#define deBUG_PRINTF_1 if(dbLevel>0)uPrintf
#define deBUG_PRINTF_2 if(dbLevel>1)uPrintf
#define deBUG_PRINT_1  if(dbLevel>0)uPuts
#define deBUG_PRINT_2  if(dbLevel>1)uPuts

#define DPIN LATCbits.LATC3   // debug test pin


// EEPROM address allocation:
#define EEADDR_BANDS  0                           // relay settings for each HF band
#define EEADDR_PARAMS (EEADDR_BANDS+(HF_BANDS*3)) // parameter that can be changed by OLED settings menu


// BT_oled_menu.c:
char oledMenu (void);
void eepromParamSave (void);
void eepromParamRestore (void);


#endif	// BT_EXTENSION_H
