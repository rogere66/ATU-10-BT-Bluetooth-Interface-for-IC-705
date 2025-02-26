//****************************************************
// BT_oled_menu.c
// ATU-10 OLED menu for Bluetooth interface setup, cell parameters etc.
// by LB1LI, 2025

#include "pic_init.h"
#include "main.h"
#include "oled_control.h"
#include "BT_extension.h"

//****************************************************
// service menu activated by extra long button press (power-off) - normal operation is blocked
// short button press to scroll menu, long button press to select:

#define MENU_HEADING oled_wr_str (0, 4, "Settings  ", 10)
#define MENU_TIMEOUT ((Tick - menuTimer) > 600000)

uint32_t menuTimer    = 0;

// display menu line and get response, optionally with confirmation:
char menuItem (char *msg, char *confirm) {
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
    B_long = 0;
    if (confirm == NULL)
      return 1;
    else {
      oled_wr_str (2, 4, confirm, 10);
      while (!B_short && !B_long && !MENU_TIMEOUT)
        Delay_ms (1);
      if (B_long) {
        B_long = 0;
        return 1;
      }
    }
  }
  if (B_short)
    menuTimer = Tick;
  B_short = 0;
  B_long  = 0;
  return 0;
}

// display message:
void menuResult (char *msg) {
  oled_wr_str (2, 4, msg, 10);
  Delay_ms (1000);
  while (GetButton);
}

// display message and change value:
char menuGetVal (char *msg, uint8_t *val, uint8_t minVal, uint8_t maxVal) {
  static char buf[14];
  char changed = 0;

  while (!MENU_TIMEOUT) {
    sprintf (buf, "%s%d  ", msg, *val);
    if (menuItem (buf, NULL)) {
      changed = 1;
      *val = (*val < maxVal)? *val + 1 : minVal;
      sprintf (buf, "%s%d  ", msg, *val);
      oled_wr_str (2, 4, buf, 10);
      for (int i = 0; (i < 50) && GetButton; i++)
        Delay_ms (10);
      while (GetButton) {
        if (*val >= maxVal) {
          while (GetButton);
          break;
        } else
          (*val)++;
        sprintf (buf, "%s%d  ", msg, *val);
        oled_wr_str (2, 4, buf, 10);
      }
    } else
      break;
  }
  return changed;
}


// change "cell" values:
void menuCells (void) {
  uint8_t val;

  while (!MENU_TIMEOUT) {
    val = (uint8_t)(Disp_time / 60000);
    if (menuGetVal ("1 TDISP ", &val, 0, 99)) {
      Disp_time = (uint32_t)val * 60000;
    }

    val = (uint8_t)(Off_time / 60000);
    if (menuGetVal ("2 TPOFF ", &val, 0, 99)) {
      Off_time = (uint32_t)val * 60000;
    }

    val = (uint8_t)Rel_Del;
    if (menuGetVal ("3 RELDT ", &val, 2, 12)) {
      Rel_Del = val;
    }

    val = (uint8_t)min_for_start;
    if (menuGetVal ("4 MINPW ", &val, 1, 99)) {
      min_for_start = val;
    }

    val = (uint8_t)(max_for_start / 10);
    if (menuGetVal ("5 MAXPW ", &val, 0, 20)) {
      max_for_start = (int)val * 10;
    }

    val = (uint8_t)(Auto_delta / 10);
    if (menuGetVal ("6 ADSWR ", &val, 10, 30)) {
      Auto_delta = (int)val * 10;
    }

    val = Auto;
    if (menuGetVal ("7 AUTOT ", &val, 0, 1)) {
      Auto = val;
    }

    val = (uint8_t)((Cal_b * 10.0f) + 0.1);
    if (menuGetVal ("8 CAL1W ", &val, 0, 99)) {
      Cal_b = (float)val / 10.0f;
    }

    val = (uint8_t)(((Cal_a - 1.0f) * 100.0f) + 0.1);
    if (menuGetVal ("9 CAL10 ", &val, 0, 99)) {
      Cal_a = (float)val / 100.0f + 1.0f;
    }

    val = (uint8_t)((Peak_cnt * 6) / 10);
    if (menuGetVal ("10 PDET ", &val, 0, 99)) {
      Peak_cnt = (int)val * 10 / 6;
    }

    if (menuItem ("<-DONE    ", NULL))
      break;
  }
}

// change standby settings:
void menuStandby (void) {
  uint8_t val_h, val_s = (uint8_t)sleepTstdby_s + 6;
  if (btConnRetries == 0xffff)
    val_h = 255;
  else
    val_h = (uint8_t)(((uint32_t)btConnRetries * (sleepTstdby_s + 6)) / 3600);

  while (!MENU_TIMEOUT) {
    while (!MENU_TIMEOUT) {
      if (val_h == 0) {
        if (menuItem ("       OFF", NULL))
          val_h = 255;
        else
          break;
      }
      if (val_h == 255) {
        if (menuItem ("  INFINITE", NULL))
          val_h = 1;
        else
          break;
      }
      if ((val_h != 0) && (val_h != 255)) {
        if (menuGetVal ("HOURS  ", &val_h, 0, 254)) {
          if (val_h != 0)
            break;
        } else
          break;
      }
    }

    menuGetVal ("DLAY S ", &val_s, 30, 255);

    if (menuItem ("<-DONE    ", NULL)) {
      sleepTstdby_s = val_s - 6;
      sleepTime_s   = sleepTstdby_s;
      if (val_h == 255)
        btConnRetries = 0xffff;
      else
        btConnRetries = (unsigned int)(((uint32_t)val_h * 3600) / (sleepTstdby_s + 6));
      break;
    }
  }
}


//****************************************************
// OLED top menu:
char oledMenu (void) {
  uint8_t antSw = 0;
  menuTimer = Tick;
  oled_clear ();
  MENU_HEADING;
  while (GetButton);
  while (!MENU_TIMEOUT) {

    // POWER OFF sleep mode:
    if (menuItem ("POWER OFF ", NULL)) {
      eepromParamSave();
      return 0;
    }

    // change "cell" settings:
    if (menuItem ("CELL PARAM", NULL)) {
      oled_wr_str (0, 4, "CELL PARAM", 10);
      menuCells();
      MENU_HEADING;
    }

    // clear relay settings for all bands:
    if (menuItem ("CLR BANDS ", " CLEAR?   ")) {
      eraseStoredBand (1);
      Relay_set (0, 0, 2);
      menuResult (" CLEARED  ");
    }

    // unpair bluetooth and start new pairing sequence:
    while (menuItem ("UNPAIR BT ", " UNPAIR?  ")) {
      btPairingTimer = Tick;
      if (btSendCmdWithAck (T_UNPR)) {
        menuResult (" UNPAIRED ");
        break;
      } else
        menuResult (" FAILED   ");
    }

    // change standby mode:
    if (menuItem ("STANDBY   ", NULL)) {
      oled_wr_str (0, 4, "STANDBY   ", 10);
      menuStandby();
      MENU_HEADING;
    }

    // change antenna HF-VHF/UHF switch setting:
    while (!antSw && menuItem ("ANT SWITCH", NULL)) {
      if (btSendCmdWithAck (T_ANTHF))
        antSw = T_ANTHF;
      else
        menuResult (" FAILED   ");
    }
    while (antSw && !MENU_TIMEOUT) {
      if (antSw == T_ANTHF) {
        if (menuItem ("ANT HF    ", NULL)) {
          if (btSendCmdWithAck (T_ANTVU))
            antSw = T_ANTVU;
        } else
          break;
      } else {
        if (menuItem ("ANT V/UHF ", NULL)) {
          if (btSendCmdWithAck (T_ANTHF))
            antSw = T_ANTHF;
        } else
          break;
      }
    }

    // change OLED brightness:
    while (!MENU_TIMEOUT) {
      uint8_t val = oledContrast / 2;
      val = (val <= 73)? val : (val - ((val - 73) / 2));
      if (menuGetVal ("BRIGHT ", &val, 0, 100)) {
        oledContrast = (val <= 73)? val * 2 : (val + (val - 73)) * 2;
        oled_init ();
        MENU_HEADING;
      }
      else
        break;
    }

    // restore default settings for all parameters stored in EEPROM:
    if (menuItem ("RESTORE   ", NULL)) {
      oled_wr_str (0, 4, "RESTORE   ", 10);
      if (menuItem ("DEFAULTS? ", NULL)) {
        eraseStoredBand (1);
        cells_reading();
        btConnRetries  = BT_DEFAULT_RETRIES;
        oledContrast   = DEFAULT_OLED_CONTRAST;
        sleepTstdby_s  = BT_DFLT_RETRY_DELAY_S;
        eepromParamSave();
        menuResult (" RESTORED ");
        MENU_HEADING;
      }
    }

    // show tuner info:
    if (menuItem ("TUNER INFO", NULL)) {
      Greating();
      menuItem (NULL, NULL);
      oled_clear ();
      MENU_HEADING;
    }

    // exit:
    if (menuItem ("<=DONE    ", NULL))
      break;
  }
  // clean up a few things after blocking:
  eepromParamSave();
  oled_start();
  Voltage_show();
  btAwakeTimer = Tick;
  connRetryCntr = 0;
  sleepTime_s = sleepTstdby_s / 2;
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
  unsigned char Auto;
  float         Cal_b;
  float         Cal_a;
  int           Peak_cnt;
  uint16_t      btConnRetries;
  unsigned int  sleepTstdby_s;
  unsigned char oledContrast;
} param;


// save changed parameters:
void eepromParamSave (void) {
  param.Disp_time     = Disp_time;
  param.Off_time      = Off_time;
  param.Rel_Del       = Rel_Del;
  param.min_for_start = min_for_start;
  param.max_for_start = max_for_start;
  param.Auto_delta    = Auto_delta;
  param.Auto          = Auto;
  param.Cal_b         = Cal_b;
  param.Cal_a         = Cal_a;
  param.Peak_cnt      = Peak_cnt;
  param.btConnRetries = btConnRetries;
  param.sleepTstdby_s = sleepTstdby_s;
  param.oledContrast  = oledContrast;

  for (unsigned char i = 0; i < sizeof (param); i++)
    eepromWrite (EEADDR_PARAMS + i, ((char *)&param)[i]);
}

// validate and restore settings from EEPROM:
void eepromParamRestore (void) {
  for (unsigned char i = 0; i < sizeof (param); i++)
    ((char *)&param)[i] = eepromRead (EEADDR_PARAMS + i);

  // validate some EEPROM parameters and restore only if OK:
  if ((param.Rel_Del >= 2) && (param.Rel_Del <= 12) && param.min_for_start && param.Auto_delta &&
      (param.Auto_delta <= 300) && (param.Auto <= 1) && (param.sleepTstdby_s > 20)) {
    Disp_time     = param.Disp_time;
    Off_time      = param.Off_time;
    Rel_Del       = param.Rel_Del;
    min_for_start = param.min_for_start;
    max_for_start = param.max_for_start;
    Auto_delta    = param.Auto_delta;
    Auto          = param.Auto;
    Cal_b         = param.Cal_b;
    Cal_a         = param.Cal_a;
    Peak_cnt      = param.Peak_cnt;
    btConnRetries = param.btConnRetries;
    sleepTstdby_s = param.sleepTstdby_s;
    oledContrast  = param.oledContrast;
  }
  else
    eepromParamSave();
}
