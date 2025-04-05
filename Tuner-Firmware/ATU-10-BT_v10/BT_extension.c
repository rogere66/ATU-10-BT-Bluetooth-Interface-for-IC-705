//****************************************************
// BT_extension.c
// ATU-10-BT code for Bluetooth interface and other added functionality
// by LB1LI 2025

#define PROJECT_NAME    "ATU-10-BT"
#define PROJECT_VERSION "10"

#include "pic_init.h"
#include "main.h"
#include "oled_control.h"
#include "BT_extension.h"

// global variables:
char     btStandby      = 0;   // BT standby - try reconnecting
char     pendingTxOn    = 0;   // pending TX ON request
int      SWRcorrection  = 0;   // SWR correction value
char     SWRcorrEnable  = 1;   // SWR correction enable
char     tuneAbort      = 0;   // tuning abort flag
char     wakeCause      = 0;   // reason for waking up: 0= Button, 1= timer, 2= BT interrupt
char     btExtVer[6]    = {0}; // BT extension code version
char     crcErrorCnt    = 0;   // CRC error counter for BT communication
char     cliActive      = 0;   // CLI is activated
char     uartEnabled    = 0;   // UART enabled flag
uint8_t  dbLevel        = 0;   // debug level controlling printouts
uint8_t  currentBand    = 255; // band in use
uint8_t  pendingBand    = 255; // new band received from BT
uint8_t  currentBTstate = 0;   // BT connection state, 0=unknown, T_NOPR, T_PAIRD, T_CONN
uint8_t  pendingBTstate = 0;   // new connection state received from BT
uint8_t  lastAck        = 0;   // last command acknowledge from BT
uint32_t btConnectTimer = 0;   // keep awake timer used when paired
uint32_t btPairingTimer = 0;   // pairing timer
unsigned char Auto_cell = 0;   // persistent Auto cell parameter
unsigned int connRetryCntr  = 0;                     // standby connect retry counter
unsigned int btConnRetries  = BT_DEFAULT_RETRIES;    //  standby default connect retry count limit
unsigned int sleepTime_s    = BT_DFLT_RETRY_DELAY_S; // current sleep time in seconds
unsigned int sleepTimeMax_s = BT_DFLT_RETRY_DELAY_S; // max standby sleep time in seconds
uint8_t      oledContrast   = DEFAULT_OLED_CONTRAST;

// relay settings for each band, LCI stored in EEPROM:
char band_L[HF_BANDS];
char band_C[HF_BANDS];
char band_I[HF_BANDS];   // relay I state + bit 3/2/1 used as unconnected/unknown/tune flags
char bandUse[HF_BANDS];  // slot usage - used only when BT not connected


//****************************************************
// show band + relay setting in HEX on OLED, if known:

void show_Band_Relay (void) {
  static char *bandTab[] = BAND_NAMES;

  // show band or PWR= on upper line:
  if (currentBand < ALL_BANDS) {
    oled_wr_str (0, 0, bandTab[currentBand], 5);
    if (currentBTstate) {
      char st = 'U';      // unpaired
      if (currentBTstate == T_PAIRD) {
        if (btStandby && (btConnRetries > 0))
          st = 'S';       // standby
        else
          st = 'P';       // paired
      }
      else if (currentBTstate == T_CONN)
        st = 'C';         // connected
      oled_wr_str_s (0, 49, &st, 1);
    }
  } else {
    oled_wr_str (0, 0, "PWR  ", 5);
    oled_wr_str(0, 42, "=", 1);
  }

  // show relay setting etc. on lower line:
  if ((currentBand > HF_BANDS) && (currentBand < ALL_BANDS)) {
    oled_wr_str (2, 0, "     ", 5);
  }
  else if ((currentBand <= HF_BANDS) || ((currentBand >= ALL_BANDS) && !(rel_I & I_UNNO))) {
    char buf[] = { '0', '0', '0', '0', '0', '0', ' ', ' ' };
    IntToHex ((rel_L << 8) | rel_C, buf);
    oled_wr_str (2, 0, &buf[2], 5);
    buf[0] = '0' + (rel_I & (I_TUNE | I_REL));
    oled_wr_str_s (3, 49, buf, 1);
  }
  else {
    oled_wr_str (2, 0, "SWR  ", 5);
    oled_wr_str(2, 42, "=", 1);
  }

  disp_cnt = Disp_time;
}


//****************************************************
// send command with acknowledge to BT:

char btSendCmdWithAck (uint8_t cmd) {
  char i = 0, ret = 0;
  if (currentBTstate || pendingBTstate) {
    btPairingTimer = Tick;
    if (!uartEnabled) {
      uartInit();
      Delay_ms (200);
    }
    lastAck = 0;
    do {
      uSendCmd (cmd);
      Delay_ms (300);
    } while ((lastAck != cmd) && (i++ < 10));
    if (lastAck == cmd)
      ret = 1;
  }
  return ret;
}


//****************************************************
// band storage access functions. band relay settings are saved both
// in RAM and EEPROM and is restored on band change:

// load and verify stored relay settings from EEPROM to RAM for all bands:
void loadStoredBandsFromEEPROM (void) {
  for (char i = 0; i < HF_BANDS * 3; i++) {
    if (eepromRead (EEADDR_BANDS + i) > 127) {
      eraseStoredBand (1);
      return;
    }
  }

  for (char i = 0; i < HF_BANDS; i++) {
    band_L[i] = eepromRead (EEADDR_BANDS + i);
    band_C[i] = eepromRead (EEADDR_BANDS + i + HF_BANDS);
    band_I[i] = eepromRead (EEADDR_BANDS + i + HF_BANDS * 2);
  }
}

// erase relay settings in RAM and EEPROM for one or all bands:
void eraseStoredBand (char all) {
  char i = 0, n = HF_BANDS;
  if (!all) {
    if ((currentBand > 0) && (currentBand <= HF_BANDS)) {
      i = currentBand - 1;
      n = 1;
    } else
      return;
  }

  for (; n > 0; n--, i++) {
      eepromWrite (EEADDR_BANDS + i, 0);
      eepromWrite (EEADDR_BANDS + i + HF_BANDS, 0);
      eepromWrite (EEADDR_BANDS + i + HF_BANDS * 2, 2);
      band_L[i]  = 0;
      band_C[i]  = 0;
      band_I[i]  = 2;
      bandUse[i] = 0;
  }
  Relay_set (0, 0, I_TUNE);
}

// restore relay settings for specified band:
void reloadStoredBand (char newBand) {
  currentBand = newBand;
  if ((newBand > 0) && (newBand <= HF_BANDS)) {
    if (band_I[newBand - 1] & 0x04)
      Relay_set (0, 0, I_TUNE);
    else
      Relay_set (band_L[newBand - 1], band_C[newBand - 1], band_I[newBand - 1]);
  }
}

// scan for least used band slot (used when BT is not connected):
char findLeastUsedBandSlot (void) {
  uint8_t i, minUse = 255, maxUse = 0, minSlot = HF_BANDS;
  for (i = 0; i < HF_BANDS; i++) {
    if (band_I[i] & (I_UCON | I_TUNE)) {
      if (bandUse[i] < minUse) {
        minUse  = bandUse[i];
        minSlot = i;
      }
      if (bandUse[i] > maxUse)
        maxUse = bandUse[i];
      if (bandUse[i] == 0)
        break;
    }
  }

  if (minUse > 1)
    for (i = 0; i < HF_BANDS; i++)
      bandUse[i]--;

  bandUse[minSlot] = maxUse + 1;
  deBUG_PRINTF_3 ("saved band slot %d used %d times\n", minSlot, bandUse[minSlot]);
  return minSlot;
}

// save relay setting in band slot:
void saveBandState (void) {
  uint8_t bandSlot;
  if (currentBTstate == T_CONN)
    bandSlot = currentBand - 1;
  else {
    for (uint8_t i = 0; i < HF_BANDS; i++) { // check for duplicate
      if ((band_L[i] == rel_L) && (band_C[i] == rel_C) && ((band_I[i] & I_REL) == (rel_I & I_REL)))
        return;
    }
    bandSlot = findLeastUsedBandSlot();
  }

  if (bandSlot < HF_BANDS) {
    band_L[bandSlot] = rel_L;
    band_C[bandSlot] = rel_C;
    band_I[bandSlot] = (currentBTstate == T_CONN)? rel_I : rel_I | I_UCON;
    eepromWrite (EEADDR_BANDS + bandSlot,                rel_L);
    eepromWrite (EEADDR_BANDS + bandSlot + HF_BANDS * 1, rel_C);
    eepromWrite (EEADDR_BANDS + bandSlot + HF_BANDS * 2, band_I[bandSlot]);
    if (dbLevel > 0)
      deBUG_PRINTF_2 ("LCI %3d %3d %d Stored in Band Slot %2d\n", rel_L, rel_C, rel_I, bandSlot);
  }
}

// scan tuned band slots for best tuning (used when BT is not connected):
void scanTunedBandSlots (int swrLimit) {
  if (currentBTstate == T_CONN) {
    SWR = 0;
    return;
  }
  uint8_t i, L_min, C_min, I_min, minSlot = HF_BANDS;
  int SWR_min = 999;

  for (i = 0; i < HF_BANDS; i++) {
    if (!(band_I[i] & I_TUNE)) {
      Relay_set (band_L[i], band_C[i], band_I[i]);
      get_swr();
      if (tuneAbort)
        return;

      if ((SWR < SWR_min) && (SWR >= 100))
        SWR_min = SWR, L_min = rel_L, C_min = rel_C, I_min = rel_I, minSlot = i;
    }
  }
  Relay_set (L_min, C_min, I_min);
  SWR = SWR_min;
  if ((SWR <= swrLimit) && (bandUse[minSlot] < 20))
    bandUse[minSlot]++;

  deBUG_PRINTF_3 ("loaded band slot %d used %d times, LCI %3d %3d %d\n", minSlot, bandUse[minSlot],rel_L, rel_C, rel_I);
}


//****************************************************
// calculate 5 bit CRC code for 10 bit input value:

uint8_t crc5 (int iVal) {
  uint8_t b, crc = 0x1f;
  for (int i = 0;  i < 11;  ++i) {
    b = (iVal ^ crc) & 1;
    iVal >>= 1;
    crc  >>= 1;
    if (b)
      crc ^= 0x14;
  }
  return crc ^ 0x1f;
}


//****************************************************
// handle input from BT interface - background task called from Delay_ms():

void btCommProcess (void) {
  static char c1 = 0, c2 = 0;

  // get byte from command RX FIFO:
  char c0 = uGetCmd();
  if (!c0)
    return;

  // save control bytes until CRC byte is received:
  if ((c0 & T_CMSK) != T_CRC) {
    c2 = c1;
    c1 = c0;
  }

  // handle 3-byte SWR message:
  else if (((c2 & T_CMSK) == T_SWR) && ((c1 & T_CMSK) == T_SWR)) {
    int newSWR = ((c2 & T_DMSK) << T_DLEN) | (c1 & T_DMSK);
    if ((c0 & T_DMSK) == crc5 (newSWR)) {
      disp_cnt = Disp_time;

      // update tuner SWR correction:
      if (SWRcorrEnable && !Tuning && (newSWR >= 100) && (newSWR < 300) && (SWR >= 100) && (SWR < 300) && (PWR > 10)) {
        SWRcorrection += newSWR - SWR;
      }

      if ((dbLevel >= 2) && (SWR >= 100)) { // use inline debug printout to avoid call stack depth problem:
        uPuts ("SWR BT LOCAL  ");
        uTxBuf[0] = (char)(newSWR / 100) + '0';
        char2str (&uTxBuf[1], (char)(newSWR % 100));
        uTxBuf[1] = '.';
        uTxBuf[4] = ' ';
        uTxBuf[5] = (char)((SWR - SWRcorrection) / 100) + '0';
        char2str (&uTxBuf[6], (char)((SWR - SWRcorrection) % 100));
        uTxBuf[6] = '.';
        uTxBuf[9] = '\n';
        for (char i = 0; i <= 9; i++)
          uPutc (uTxBuf[i]);
      }
    } else {
      crcErrorCnt++;
      deBUG_PRINT_1 ("BT Rx3 CRC ERROR ****\n");
    }
  }

  else if ((c0 & T_DMSK) == crc5 ((int)c1)) {  // check CRC for 2-byte messages

    // schedule band change:
    if ((c1 & T_CMSK) == T_BAND) {
      pendingBand = c1 & T_DMSK;
      if (pendingBand != currentBand)
        tuneAbort = 1;
      uSendCmd (c1);       // send ACK
    }

    // schedule state change:
    else if ((c1 == T_NOPR) || (c1 == T_PAIRD) || (c1 == T_CONN)) {
      pendingBTstate = c1;
      uSendCmd (c1);       // send ACK
      deBUG_PRINT_2 ("State Received\n");
    }

    // handle message acknowledge:
    else if ((c1 == T_UNPR) || (c1 == T_ANTHF) || (c1 == T_ANTVU)) {
      lastAck = c1;
      deBUG_PRINT_2 ("ACK Received\n");
    }

    // handle TX ON message:
    else if (c1 == T_TXON) {   // TX start
      uSendCmd (c1);           // send ACK
      pendingTxOn = 1;
      deBUG_PRINT_2 ("T_TXON Received\n");
    }

    // handle TX OFF message:
    else if (c1 == T_TXOFF) {  // TX stop
      tuneAbort = 1;
      deBUG_PRINT_2 ("T_TXOFF Received\n");
    }

    else
      deBUG_PRINT_1 ("Unknown BT Control Message\n");
  }
  else {
    crcErrorCnt++;
    deBUG_PRINT_1 ("BT Rx CRC ERROR ****\n");
  }

  return;
}


//****************************************************
// BT extension initialization and main loop:

void btExtension_init (void) {
  PMD0bits.NVMMD = 0; // enable EEPROM module
  loadStoredBandsFromEEPROM ();
  LATBbits.LATB3 = 0; // reset BT
  Delay_ms (100);
  uartInit ();
  sprintf (btExtVer, "%s/--", PROJECT_VERSION);
  deBUG_PRINTF_2 ("\n%s v%s\n", PROJECT_NAME, PROJECT_VERSION);
}


//****************************************************
// BT extension main loop:

void btExtension_main (void) {

  Delay_ms (1); // check for input from BT processor (background function in Delay_ms())

  //****************************************************
  // handle control input from BT processor:

  // handle pending band change:
  if (currentBand != pendingBand) {
    currentBand = pendingBand;
    SWRcorrection = 0;
    draw_swr (0);
    if (currentBand == 0)         // outside HF bands - set bypass:
      Relay_set (0, 0, 0);
    else
      reloadStoredBand (currentBand);

    if (OLED_PWD) {
      show_Band_Relay ();
      draw_swr (0);
    } else
      oled_start ();

    deBUG_PRINTF_2 ("T_BAND= %d  Relays= %d %d %d\n", currentBand, rel_L, rel_C, rel_I);
  }

  // handle pending BT state change:
  if ((currentBTstate != pendingBTstate) && pendingBTstate) {
    if (currentBTstate == T_CONN) {
      sleepTime_s = sleepTimeMax_s / 2;
    }
    if (pendingBTstate == T_CONN) {
      connRetryCntr = 0;
      btStandby     = 1;
      sleepTime_s   = sleepTimeMax_s;
    }
    currentBTstate = pendingBTstate;
    if (OLED_PWD)
      show_Band_Relay ();
    else
      oled_start ();

    deBUG_PRINTF_2 ("BT state %02x\n", currentBTstate);
  }

  // handle pending tune request:
  if (pendingTxOn) {
    pendingTxOn = 0;
    if ((currentBand > 0) && (currentBand <= HF_BANDS)) {
      if(!OLED_PWD)
        oled_start();

      if ((rel_I & I_TUNE) && !Tuning)
        Btn_long();
    }
  }

  // handle Auto-tune flags:
  if ((currentBand > 0) && (currentBand <= HF_BANDS))
    Auto = (rel_I & I_TUNE) != 0;
  else if (currentBand >= ALL_BANDS)
    Auto = Auto_cell;


  //****************************************************
  // handle Tuner and BT sleep for the different connection states:

  // keep power on when connected:
  if (currentBTstate == T_CONN) {
    off_cnt = Off_time;
    btConnectTimer = Tick;
    btPairingTimer = btConnectTimer;
  }

  // handle sleep modes when the power-up pairing window has closed:
  else if (((Tick - btPairingTimer) > ((uint32_t)BT_PAIRING_TIMEOUT_S * 1000)) && !cliActive) {

    // turn OFF UART and BT if not paired by now (or no BT present):
    if (((currentBTstate == T_NOPR) || !currentBTstate) && uartEnabled) {
      deBUG_PRINT_2 ("BT not paired - turning off BT and UART\n");
      Delay_us (100);
      uartDeInit();
      LATBbits.LATB3 = 0;  // BT shutdown
      Delay_ms (1);
      if (OLED_PWD)
        show_Band_Relay();
      else
        oled_start();
    }

    // standby mode: try reconnecting on lost connection:
    if ((currentBTstate == T_PAIRD) && btStandby && (btConnRetries > 0)) {
      off_cnt = Off_time;
      if (pendingBTstate || ((Tick - btConnectTimer) > 15000)) {  // check for state update, with timeout
        pendingBTstate = 0;
        if (OLED_PWD)
          Delay_ms (2000);

        // handle timed sleep:
        if ((connRetryCntr < btConnRetries) || (btConnRetries == 0xffff)) {
          if (btConnRetries != 0xffff)
            connRetryCntr++;
          deBUG_PRINTF_2 ("STANDBY sleep %d sec\n", sleepTime_s);
          Delay_us (500);
          power_off (2);
          btConnectTimer = Tick;
          if (!wakeCause)
            btPairingTimer = btConnectTimer;

          if (sleepTime_s < sleepTimeMax_s)
            sleepTime_s++;
          deBUG_PRINTF_2 ("BT connect retry # %d\n", connRetryCntr);
        }

        // infinite sleep, wake on button:
        else {
          deBUG_PRINT_2 ("STANDBY sleep end, wake on button\n");
          sleepTime_s = 0;
          power_off (2);
          off_cnt = Off_time / 2;
          sleepTime_s = sleepTimeMax_s / 2;
        }
      }
    }
  }


  //****************************************************
  // handle character input from UART/BT processor:
  char c = uGetc();
  if (!c)
    return;

  // receive version number from BT:
  if (c == 'V') {
    Delay_us (100);
    char v1 = uGetc();
    if ((v1 >= '0') && (v1 <= '9')){
      char v2 = uGetc();
      if ((v2 >= '0') && (v2 <= '9')){
        btExtVer[3] = v1;
        btExtVer[4] = v2;
        deBUG_PRINTF_2 ("BT version %s\n", btExtVer);
        uGetc();
        return;
      }
    }
  }

  // check for ASCII command unlock key 'CLI':
  if (c == 'C') {
    Delay_us (10);
    if (uGetc() == 'L') {
      if (uGetc() == 'I') {
        if (!cliActive)
          uPuts ("\nCLI enabled\n");

        cliActive = 1;
        c = 'h';
      }
    }
  }

  if (!cliActive)
    return;


  //****************************************************
  // CLI commands:

  if ((c == 'e') || (c == 'E')) {   // erase stored band settings
    eraseStoredBand (c == 'E');
    reloadStoredBand (currentBand);
    show_Band_Relay ();
    uPuts ("band setting(s) Erased\n");
  }

  else if ((c == 'n') || (c == 'N')){  // display/erase EEPROM
    uint8_t addr = 0;
    static char buf[6];
    uPuts ("EEPROM:\n");
    for (uint8_t n = 0; n < 16; n++) {
      uPrintf ("%08X ", addr);
      for (uint8_t i = 0; i < 16; i++) {
        if ((i % 4) == 0)
          uPutc (' ');

        sprintf (buf, "%02X", eepromRead (addr));
        uPutc (buf[0]);
        uPutc (buf[1]);
        uPutc (' ');
        if (c == 'N')
          eepromWrite (addr, 0xff);
        addr++;
      }
      uPutc ('\n');
    }
  }

  else if (c == 't') {             // run relay test
    uPuts ("Toggle all tuner relays\n");
    Relay_set (0, 0, 0);
    for (unsigned int rBits = 1; rBits; rBits <<= 1) {
      Relay_set ((uint8_t)(rBits >> 8), (uint8_t)(rBits >> 1), rBits & 0x01);
      show_Band_Relay ();
      Delay_ms (200);
      Relay_set (0, 0, 0);
      Delay_ms (200);
    }
  }

  else if (c == 'd') {             // change debug level
    if (++dbLevel > 4)
        dbLevel = 0;
    eepromParamSave();
    uPrintf ("Debug Level %d\n", dbLevel);
  }

  else if (c == 's') {             // print status
    uPrintf ("\nStatus %s v%s:\n", PROJECT_NAME, PROJECT_VERSION);
    uPrintf ("Battery %d.%d V\n", Voltage / 1000, (Voltage / 10) % 100);
    uPrintf ("Current BT state %x\n", currentBTstate);
    uPrintf ("CRC Error Count %d\n", crcErrorCnt);
    uPrintf ("SWR correction %d\n", SWRcorrection);

    for (char i = 0; i < HF_BANDS; i++) {
      uPrintf ("Band %02d  LCI %3d %3d %d  (%d) %s\n", i + 1, band_L[i],
        band_C[i], band_I[i], bandUse[i], (i == (currentBand - 1))? " Current":"");
    }
  }

  else if (c == 'x') {             // close CLI
    cliActive = 0;
    uPuts ("CLI Closed\n");
  }

  else if ((c == 'h') || (c == '?')){  // help menu:
    uPuts ("\n");
    uPuts (" e - erase band setting for current band\n");
    uPuts (" E - erase band setting for all bands\n");
    uPuts (" n - display EEPROM\n");
    uPuts (" N - display and erase EEPROM\n");
    uPuts (" t - run relay toggle test\n");
    uPuts (" d - change debug level\n");
    uPuts (" s - show Tuner status\n");
    uPuts (" x - close CLI\n");
  }

  else if ((c != '\n') && (c != '\r'))
    uPrintf ("0x%02x not recognized, type h|? for help\n", c);
  else
    uPuts ("Enter Tuner Command$\n");
}


//****************************************************
// UART functions:

char uTxBuf[TXB_LEN];     // UART char RX FIFO buffer
char uRxBuf[RXB_LEN];
uint8_t cRxBuf[RXB_LEN];  // UART command RX FIFO buffer
volatile uint8_t uRxRdInd = 0, uRxWrInd = 0;  // UART char RX FIFO rd/wr indexes
volatile uint8_t cRxRdInd = 0, cRxWrInd = 0;  // UART command RX FIFO rd/wr indexes

// UART RX ISR: put received byte in FIFO buffer, no buffer overflow check:
// is placed inline in main ISR to save stack depth

// initialize UART for BT communication:
void uartInit (void) {
  PMD4bits.UART1MD  = 0;    // enable UART module
  TRISBbits.TRISB4  = 1;    // RX on pin B4 (Red LED)
  WPUBbits.WPUB4    = 1;    // pull-up in case not connected
  RXPPS             = 0x0C;
  RB3PPS            = 0x10; // TX on pin B3 (Green LED)
  TRISBbits.TRISB3  = 0;
  TXSTAbits.BRGH    = 1;
  BAUDCONbits.BRG16 = 1;
  SPBRG             = 68;   // baud 115200
  RCSTAbits.SPEN    = 1;    // enable serial port
  RCSTAbits.CREN    = 1;    // enable continuous RX
  TXSTAbits.TXEN    = 1;    // enable TX
  PIR3bits.RCIF     = 0;    // clear and enable RX interrupt
  PIE3bits.RCIE     = 1;
  INTCONbits.PEIE   = 1;
  INTCONbits.GIE    = 1;
  uartEnabled = 1;
}

// suspend UART:
void uartDeInit (void) {
  TXSTAbits.TXEN   = 0;    // disable TX
  RB3PPS           = 0;    // change TX pin to GPIO
  PIE3bits.RCIE    = 0;    // disable RX interrupt
  RCSTAbits.SPEN   = 0;    // disable serial port
  PMD4bits.UART1MD = 1;    // disable UART module
  uRxRdInd = uRxWrInd;     // clear FIFO buffers
  cRxRdInd = cRxWrInd;
  uartEnabled = 0;
}

// get character from RX buffer, return 0 if none:
char uGetc (void) {
  if (uRxRdInd == uRxWrInd)
    return 0;

  uint8_t tmp = uRxRdInd;
  char    ch  = uRxBuf[tmp++];
  if (tmp >= RXB_LEN)
    tmp = 0;
  uRxRdInd = tmp;
  return ch;
}

// get byte from command RX FIFO, return 0 if none:
uint8_t uGetCmd (void) {
  if (cRxRdInd == cRxWrInd)
    return 0;

  uint8_t tmp = cRxRdInd;
  char    ch  = cRxBuf[tmp++];
  if (tmp >= RXB_LEN)
    tmp = 0;
  cRxRdInd = tmp;
  return ch;
}

// send string with tuner mark and CR on NL:
void uPuts (char *s) {
  if (uartEnabled) {
    uPutc ('>');   // tuner mark
    uPutc (' ');
    for (int i = 0; s[i] > 0;) {
      uPutc (s[i]);
      if (s[i++] == '\n') {
        uPutc ('\r');
        if (s[i] > 0) {
          uPutc ('>');
          uPutc (' ');
        }
      }
    }
  }
}


//****************************************************
// EEPROM access functions:

uint8_t eepromRead (uint8_t addr) {
  NVMCON1bits.NVMREGS = 1;
  while (NVMCON1bits.WR)
    ;
  NVMADRL = addr;
  NVMADRH = 0xf0;
  NVMCON1bits.RD = 1;
  return NVMDATL;
}

void eepromWrite (uint8_t addr, uint8_t val) {
  if (val == eepromRead (addr))
    return;

  NVMCON1bits.NVMREGS = 1;
  while (NVMCON1bits.WR)
    ;
  NVMADRL = addr;
  NVMADRH = 0xf0;
  NVMCON1bits.WREN = 1;
  NVMDATL = val;
  INTCONbits.GIE = 0;
  NVMCON2 = 0x55;
  NVMCON2 = 0xaa;
  NVMCON1bits.WR = 1;
  INTCONbits.GIE = 1;
}


//****************************************************
// replacements for microC library functions not available with MPLAB/XC8 compiler:

// Delay_us() is located in bank 0 and written in assembly for consistency,
// ~5 uS minimum delay. C source in comments:
void __at(0x0200) Delay_us (unsigned int delay) {

  // if (delay > 5)
  asm ("movlw	0");
  asm ("subwf	Delay_us@delay+1,w");
  asm ("movlw	6");
  asm ("skipnz");
  asm ("subwf	Delay_us@delay,w");
  asm ("skipc");
  asm ("goto	Delay_us_1");

  // delay -= 5;
  asm ("movlw	5");
  asm ("subwf	Delay_us@delay,f");
  asm ("movlw	0");
  asm ("subwfb	Delay_us@delay+1,f");
  asm ("goto	Delay_us_2");

  // else delay = 0;
  asm ("Delay_us_1:");
  asm ("clrf	Delay_us@delay");
  asm ("clrf	Delay_us@delay+1");

  // while (delay-- > 0);
  asm ("Delay_us_2:");
  asm ("movlw	1");
  asm ("subwf	Delay_us@delay,f");
  asm ("movlw	0");
  asm ("subwfb	Delay_us@delay+1,f");
  asm ("incf	Delay_us@delay,w");
  asm ("skipz");
  asm ("goto	Delay_us_2");
  asm ("incf	Delay_us@delay+1,w");
  asm ("btfsc	3,2");
  asm ("return");
  asm ("goto	Delay_us_2");
}

void Delay_ms (unsigned int delay) {
  while (delay-- > 0) {
    btCommProcess ();  // background task: handle BT RX
    Delay_us (993);
  }
}


//****************************************************
// ADC functions:

void ADC_Init (void) {
  PMD2bits.ADCMD    = 0;  // enable ADC module
  PMD0bits.FVRMD    = 0;  // V-ref module
  ADCON0bits.ADFRM0 = 1;  // right justify ADC result
  ADCLKbits.ADCCS   = 8;  // set ADC clock to Fosc/18
  ADREFbits.ADPREF  = 3;  // use internal reference
  FVRCONbits.ADFVR  = 1;  // 2.048V V-ref
  FVRCONbits.FVREN  = 1;  // enable V-ref
  ADCON0bits.ADON   = 1;  // enable ADC
}

void ADC_DeInit (void) {
  FVRCONbits.FVREN = 0;  // disable V-ref
  PMD0bits.FVRMD   = 1;  // disable V-ref module
  ADCON0bits.ADON  = 0;  // disable ADC
  PMD2bits.ADCMD   = 0;  // disable ADC module
}

void ADC_Init_Advanced (char refOpt)  // change reference voltage
{
  if (refOpt & _ADC_INTERNAL_VREFH) {
    ADREFbits.ADPREF = 0;             // use VDD as reference
  } else {
    ADREFbits.ADPREF = 3;             // use internal reference
    if (refOpt & _ADC_INTERNAL_FVRH1) // 1.024V
      FVRCONbits.ADFVR = 1;
    if (refOpt & _ADC_INTERNAL_FVRH2) // 2.048V
      FVRCONbits.ADFVR = 2;
  }
}

unsigned int ADC_Get_Sample (char adcChannel) {
  ADPCH = adcChannel;
  Delay_us (100);               // input + ref charge time
  ADCON0bits.ADGO = 1;          // start ADC conversion
  while (ADCON0bits.ADGO == 1)  // wait for completion
    ;

  return (unsigned int) ((ADRESH & 0x03) << 8) + ADRESL;
}


//****************************************************
// BCD/integer conversion functions:

char Bcd2Dec (char bcd) {
  return (((bcd >> 4) & 0x0f) * 10) + (bcd & 0x0f);
}

void IntToStr (int val, char *str) {
  int i = 6;
  str[i - 1] = '0';
  while (val && (i > 0)) {
    str[--i] = (val % 10) + '0';
    val /= 10;
  }
}

void IntToHex (int val, char *str) {
  int i = 6;
  str[i - 1] = '0';
  while (val && (i > 0)) {
    str[--i] = (val % 16) + '0';
    val /= 16;
    if (str[i] > '9')
      str[i] += 7;
  }
}
