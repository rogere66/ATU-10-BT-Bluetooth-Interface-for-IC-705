//****************************************************
// BT_oled_menu.c
// ATU-10-BT OLED menu for Bluetooth interface setup, cell parameters etc.
// by LB1LI 2025

#include "pic_init.h"
#include "main.h"
#include "oled_control.h"
#include "BT_extension.h"

//****************************************************
// service menu activated by extra long button press (power-off) - normal operation is blocked
// short button press to scroll menu, long button press to select:

#define MENU_HEADING oled_wr_str (0, 4, "Settings  ", 10)
#define MENU_TIMEOUT ((Tick - menuTimer) > 600000)
#define DEBOUNCE_MS  20  // button debounce time

uint32_t menuTimer = 0;
char sBuf[16];


// get button input, 0= short, 1= long, optionally with menu text and confirmation:
char menuEntry (char *msg, char *confirm) {
  if (MENU_TIMEOUT)
    return 0;
  B_short = 0;
  B_long  = 0;
  if (msg != NULL)
    oled_wr_str (2, 4, msg, 10);
  while (!B_short && !B_long && !MENU_TIMEOUT)
    Delay_ms (1);
  if (B_long) {
    menuTimer = Tick;
    if (confirm != NULL) {
      B_long = 0;
      oled_wr_str (2, 4, confirm, 10);
      while (!B_short && !B_long && !MENU_TIMEOUT)
        Delay_ms (1);
    }
    if (B_long) {
      B_long = 0;
      return 1;
    }
  }
  if (B_short)
    menuTimer = Tick;
  B_short = 0;
  return 0;
}

// display message:
void menuResult (char *msg) {
  oled_wr_str (2, 4, msg, 10);
  Delay_ms (1000);
  while (GetButton);
  Delay_ms (DEBOUNCE_MS);
}


// handle action on value change:
void menuGetValAction (uint8_t valAction, uint8_t val) {
  switch (valAction) {
    case 1:
      Relay_set (val, rel_C, rel_I);
      break;

    case 2:
      Relay_set (rel_L, val, rel_I);
      break;

    case 3:
      Relay_set (rel_L, rel_C, val);
      break;

    case 4:
        oledContrast = (val == 32)? 255 : (val - 1) * 8;
        oled_init ();
        MENU_HEADING;
  }
}

// display message and change value up or down:
char menuGetVal (char *msg, uint8_t *pVal, uint8_t minVal, uint8_t maxVal, uint8_t valAction) {
  char changed = 0;
  uint8_t val = *pVal;
  signed char step = 1;

  while (!MENU_TIMEOUT) {
    sprintf (sBuf, "%s%c%d  ", msg, (step > 0)? ' ' : 0x80, val);
    if (!menuEntry (sBuf, NULL)) {
      while (GetButton)
        B_short = 0;
      char i = 50;
      while (--i) {
        if (B_short) {
          B_short = 0;
          step = -step;
          break;
        }
        Delay_ms (10);
      }
      if (!i)
        break;
    } else {
      changed = 1;
      if (step > 0)
        val = (val < maxVal)? val + 1 : minVal;
      else
        val = (val > minVal)? val - 1 : maxVal;
      sprintf (sBuf, "%s%c%d  ", msg, (step > 0)? ' ' : 0x80, val);
      oled_wr_str (2, 4, sBuf, 10);
      for (char i = 0; (i < 50) && GetButton; i++)
        Delay_ms (10);
      Delay_ms (DEBOUNCE_MS);
      while (GetButton) {
        if ((val >= maxVal) || (val <= minVal)) {
          while (GetButton);
          break;
        } else
          val += step;
        sprintf (sBuf, "%s%c%d  ", msg, (step > 0)? ' ' : 0x80, val);
        oled_wr_str (2, 4, sBuf, 10);
      }
      Delay_ms (DEBOUNCE_MS);
      if (valAction)
        menuGetValAction (valAction, val);
      *pVal = val;
    }
  }
  return changed;
}


// change "cell" values:
void menuCells (void) {
  uint8_t val;

  while (!MENU_TIMEOUT) {
    val = (uint8_t)(Disp_time / 60000);
    if (menuGetVal ("1 TDISP", &val, 0, 99, 0)) {
      Disp_time = (uint32_t)val * 60000;
    }

    val = (uint8_t)(Off_time / 60000);
    if (menuGetVal ("2 TPOFF", &val, 0, 99, 0)) {
      Off_time = (uint32_t)val * 60000;
    }

    val = (uint8_t)Rel_Del;
    if (menuGetVal ("3 RELDT", &val, 1, 12, 0)) {
      Rel_Del = val;
    }

    val = (uint8_t)min_for_start;
    if (menuGetVal ("4 MINPW", &val, 1, 99, 0)) {
      min_for_start = val;
    }

    val = (uint8_t)(max_for_start / 10);
    if (menuGetVal ("5 MAXPW", &val, 0, 20, 0)) {
      max_for_start = (int)val * 10;
    }

    val = (uint8_t)(Auto_delta / 10);
    if (menuGetVal ("6 ADSWR", &val, 10, 30, 0)) {
      Auto_delta = (int)val * 10;
    }

    val = Auto_cell;
    if (menuGetVal ("7 AUTOT", &val, 0, 1, 0)) {
      Auto = val;
      Auto_cell = val;
    }

    val = (uint8_t)((Cal_b * 10.0f) + 0.1);
    if (menuGetVal ("8 CAL1W", &val, 0, 99, 0)) {
      Cal_b = (float)val / 10.0f;
    }

    val = (uint8_t)(((Cal_a - 1.0f) * 100.0f) + 0.1);
    if (menuGetVal ("9 CAL10", &val, 0, 99, 0)) {
      Cal_a = (float)val / 100.0f + 1.0f;
    }

    val = (uint8_t)((Peak_cnt * 6) / 10);
    if (menuGetVal ("10 PDET", &val, 0, 99, 0)) {
      Peak_cnt = (int)val * 10 / 6;
    }

    if (menuEntry ("<-DONE    ", NULL))
      break;
  }
}

// change standby settings:
void menuStandby (void) {
  uint8_t val_h, val_s = (uint8_t)sleepTimeMax_s + 6;
  if (btConnRetries == 0xffff)
    val_h = 255;
  else
    val_h = (uint8_t)(((uint32_t)btConnRetries * ((uint32_t)sleepTimeMax_s + 6)) / 3600);

  while (!MENU_TIMEOUT) {
    while (!MENU_TIMEOUT) {
      if (val_h == 255) {
        if (menuEntry ("  INFINITE", NULL))
          val_h = 0;
        else
          break;
      }
      if (val_h == 0) {
        if (menuEntry ("       OFF", NULL))
          val_h = 1;
        else
          break;
      }
      if ((val_h != 0) && (val_h != 255)) {
        if (menuGetVal ("HOURS ", &val_h, 0, 255, 0)) {
          if ((val_h != 0) && (val_h != 255))
            break;
        } else
          break;
      }
    }

    menuGetVal ("DLAY S", &val_s, 30, 255, 0);

    if (menuEntry ("<-DONE    ", NULL)) {
      sleepTimeMax_s = val_s - 6;
      sleepTime_s    = sleepTimeMax_s;
      if (val_h == 255)
        btConnRetries = 0xffff;
      else
        btConnRetries = (unsigned int)(((uint32_t)val_h * 3600 + 3599) / ((uint32_t)sleepTimeMax_s + 6));
      break;
    }
  }
}

// Tuner and Antenna Relay Tests:
void menuRelayTest (void) {
  uint8_t val, antSw = 0;
  while (!MENU_TIMEOUT) {

    // change antenna HF-VHF/UHF switch setting:
    while (!antSw && menuEntry ("ANT RELAY ", NULL)) {
      if (btSendCmdWithAck (T_ANTHF))
        antSw = T_ANTHF;
      else
        menuResult (" FAILED   ");
    }
    while (antSw && !MENU_TIMEOUT) {
      if (antSw == T_ANTHF) {
        if (menuEntry ("ANT HF    ", NULL)) {
          if (btSendCmdWithAck (T_ANTVU))
            antSw = T_ANTVU;
        } else
          break;
      } else {
        if (menuEntry ("ANT V/UHF ", NULL)) {
          if (btSendCmdWithAck (T_ANTHF))
            antSw = T_ANTHF;
        } else
          break;
      }
    }

    // run relay test: toggle all tuner relays:
    if (menuEntry ("TUNER RLY ", NULL)) {
      oled_clear();
      while (!MENU_TIMEOUT) {
        Relay_set (0, 0, 0);
        for (unsigned int rBits = 1; rBits & 0x7fff; rBits <<= 1) {
          show_Band_Relay ();
          menuEntry (NULL, NULL);
          Relay_set (0, 0, 0);
          Delay_ms (100);
          Relay_set ((uint8_t)(rBits >> 8), (uint8_t)(rBits >> 1), rBits & I_REL);
        }
        show_Band_Relay ();
        oled_wr_str (0, 0, "DONE?      ", 10);
        if (menuEntry (NULL, NULL)) {
          Relay_set (0, 0, I_TUNE);
          oled_clear();
          oled_wr_str (0, 4, "RELAY TEST", 10);
          break;
        }
      }
    }

    // set relays LCI:
    menuGetVal ("TUNE L", &rel_L, 0, 127, 1);
    menuGetVal ("TUNE C", &rel_C, 0, 127, 2);
    menuGetVal ("TUNE I", &rel_I, 0,   7, 3);

    // save LCI relay setting to EEPROM if HF band:
    if ((currentBand > 0) && (currentBand <= HF_BANDS))
      if (menuEntry ("SAVE LCI  ", NULL))
        saveBandState ();

    if (menuEntry ("<-DONE    ", NULL))
      break;
  }
}


//****************************************************
// OLED top menu:
char oledMenu (void) {
  uint8_t antSw = 0;
  menuTimer = Tick;
  oled_clear ();
  MENU_HEADING;
  while (!MENU_TIMEOUT) {

    // POWER OFF sleep mode:
    if (menuEntry ("POWER OFF ", NULL)) {
      eepromParamSave();
      return 0;
    }

    // clear relay settings for all bands and option for relay test:
    if (menuEntry ("CLR BANDS ", NULL)) {
      if (dbLevel > 0) {   // print current band slots on serial port if debug;
        uPuts ("\n");
        for (char i = 0; i < HF_BANDS; i++) {
          uPrintf ("Band %02d  LCI %3d %3d %d  (%d) %s\n", i + 1, band_L[i],
          band_C[i], band_I[i], bandUse[i], (i == (currentBand - 1))? " Current":"");
        }
      }
      if (menuEntry (" CLEAR?   ", NULL)) {
        eraseStoredBand (1);
        menuResult (" CLEARED  ");
      }
    }

    // change "cell" settings:
    if (menuEntry ("CELL PARAM", NULL)) {
      oled_wr_str (0, 4, "CELL PARAM", 10);
      menuCells();
      MENU_HEADING;
    }

    // unpair bluetooth and start new pairing sequence:
    while (menuEntry ("UNPAIR BT ", " UNPAIR?  ")) {
      btPairingTimer = Tick;
      if (btSendCmdWithAck (T_UNPR)) {
        menuResult (" UNPAIRED ");
        break;
      } else
        menuResult (" FAILED   ");
    }

    // change standby mode:
    if (menuEntry ("STANDBY   ", NULL)) {
      oled_wr_str (0, 4, "STANDBY   ", 10);
      menuStandby();
      MENU_HEADING;
    }

    // Tuner and Antenna Relay Tests:
    if (menuEntry ("RELAY TEST", NULL)) {
      oled_wr_str (0, 4, "RELAY TEST", 10);
      menuRelayTest();
      MENU_HEADING;
    }

    // restore default settings for all parameters stored in EEPROM:
    if (menuEntry ("RESTORE   ", NULL)) {
      oled_wr_str (0, 4, "RESTORE   ", 10);
      if (menuEntry ("DEFAULTS? ", NULL)) {
        eraseStoredBand (1);
        cells_reading();
        Auto_cell      = Auto;
        btConnRetries  = BT_DEFAULT_RETRIES;
        sleepTimeMax_s = BT_DFLT_RETRY_DELAY_S;
        oledContrast   = DEFAULT_OLED_CONTRAST;
        SWRcorrEnable  = 1;
        dbLevel        = 0;
        eepromParamSave();
        menuResult (" RESTORED ");
        MENU_HEADING;
      }
    }

    // tuner info and display/debug settings:
    if (menuEntry ("TUNER CONF", NULL)) {
      if (menuEntry (" INFO     ", NULL)) {
        Greating();
        menuEntry (NULL, NULL);
        oled_clear ();

        // display battery voltage:
        oled_wr_str (0, 4, " BATTERY  ", 10);
        while (!MENU_TIMEOUT && !B_short && !B_long) {
          get_batt();
          sprintf (sBuf, " %1.2f V   ", (float)Voltage / 1000.0f);
          oled_wr_str (2, 4, sBuf, 10);
          Delay_ms (200);
        }
      }

      // enable SWR correction:
      oled_wr_str (0, 4, " SWR CORR ", 10);
      menuGetVal (" ON/OFF", (uint8_t *)&SWRcorrEnable, 0, 1, 0);
      if (!SWRcorrEnable)
        SWRcorrection = 0;

      // set debug level:
      oled_wr_str (0, 4, " DEBUG    ", 10);
      menuGetVal (" LEVEL", &dbLevel, 0, 4, 0);
      MENU_HEADING;

      // change OLED brightness:
      uint8_t val = oledContrast / 8 + 1;
      menuGetVal (" BRIGHT", &val, 1, 32, 4);
    }

    // exit:
    if (menuEntry ("<=DONE    ", NULL))
      break;
  }
  // clean up a few things after blocking:
  eepromParamSave();
  oled_start();
  Voltage_show();
  btConnectTimer = Tick;
  connRetryCntr = 0;
  sleepTime_s = sleepTimeMax_s / 2;
  if (pendingBTstate != T_CONN)
    pendingBTstate = 0;
  return 1;
}


//****************************************************
// save and restore settings in EEPROM:

static struct {
  unsigned long Disp_time;
  unsigned long Off_time;
  int           Rel_Del;
  int           min_for_start;
  int           max_for_start;
  int           Auto_delta;
  unsigned char Auto_cell;
  float         Cal_b;
  float         Cal_a;
  int           Peak_cnt;
  uint16_t      btConnRetries;
  unsigned int  sleepTimeMax_s;
  unsigned char oledContrast;
  char          SWRcorrEnable;
  char          dbLevel;
} param;


// save changed parameters:
void eepromParamSave (void) {
  param.Disp_time      = Disp_time;
  param.Off_time       = Off_time;
  param.Rel_Del        = Rel_Del;
  param.min_for_start  = min_for_start;
  param.max_for_start  = max_for_start;
  param.Auto_delta     = Auto_delta;
  param.Auto_cell      = Auto_cell;
  param.Cal_b          = Cal_b;
  param.Cal_a          = Cal_a;
  param.Peak_cnt       = Peak_cnt;
  param.btConnRetries  = btConnRetries;
  param.sleepTimeMax_s = sleepTimeMax_s;
  param.oledContrast   = oledContrast;
  param.SWRcorrEnable  = SWRcorrEnable;
  param.dbLevel        = dbLevel;

  for (unsigned char i = 0; i < sizeof (param); i++)
    eepromWrite (EEADDR_PARAMS + i, ((char *)&param)[i]);
}

// validate and restore settings from EEPROM:
void eepromParamRestore (void) {
  for (unsigned char i = 0; i < sizeof (param); i++)
    ((char *)&param)[i] = eepromRead (EEADDR_PARAMS + i);

  // validate some EEPROM parameters and restore only if OK:
  if ((param.Rel_Del >= 2) && (param.Rel_Del <= 12) && param.min_for_start && param.Auto_delta &&
      (param.Auto_delta <= 300) && (param.Auto_cell <= 1) && (param.sleepTimeMax_s > 20) &&
      (param.SWRcorrEnable <= 1) && (param.dbLevel <= 4)) {
    Disp_time      = param.Disp_time;
    Off_time       = param.Off_time;
    Rel_Del        = param.Rel_Del;
    min_for_start  = param.min_for_start;
    max_for_start  = param.max_for_start;
    Auto_delta     = param.Auto_delta;
    Auto_cell      = param.Auto_cell;
    Cal_b          = param.Cal_b;
    Cal_a          = param.Cal_a;
    Peak_cnt       = param.Peak_cnt;
    btConnRetries  = param.btConnRetries;
    sleepTimeMax_s = param.sleepTimeMax_s;
    oledContrast   = param.oledContrast;
    SWRcorrEnable  = param.SWRcorrEnable;
    dbLevel        = param.dbLevel;
    Auto = Auto_cell;
  }
  else {
    Auto_cell = Auto;
    eepromParamSave();
  }
}
