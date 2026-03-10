// David Fainitski, N7DDC
// 2020
//
// Updated by LB1LI 2025 to include Bluetooth interface for Icom BT-705
// ported to MPLAB X and XC8 compiler


#include "pic_init.h"
#include "main.h"
#include "oled_control.h"
#include "Soft_I2C.h"


// global variables
char txt[8], txt_2[8];
unsigned long TickCnt = 0; // ms system tick
int Voltage, Voltage_old = 0;
char btn_1_cnt = 0, btn_2_cnt = 0;
unsigned long volt_cnt = 0, watch_cnt = 0, btn_cnt = 0, off_cnt = 10, disp_cnt=10;
int PWR, SWR, SWR_ind = 0, SWR_fixed_old = 100, PWR_fixed_old = 9999, rldl;
char ind = 0, cap = 0, SW = 0;
char Overflow, B_short, B_long, B_xlong, gre, E_short, E_long;

unsigned char rel_L, rel_C;            // current relay states
unsigned char rel_I = I_UNNO | I_TUNE; // I relay and relay flags
unsigned char Tuning = 0;              // tuning in progress flag

// depending on Cells
unsigned long Disp_time, Off_time;
int Rel_Del, min_for_start, max_for_start, Auto_delta, Peak_cnt;
char Auto;
float Cal_a, Cal_b;
// Cells
const char Cells[10] __at(0x7770) = {0x02, 0x30, 0x03, 0x10, 0x15, 0x13, 0x01, 0x04, 0x14, 0x60};

#define FW_VER "1.6"


//****************************************************
// read system tick count with interrupts disabled:
unsigned long int Tick_ms (void) {
  INT = 0;
  unsigned long tmpTick = TickCnt;
  INT = 1;
  return tmpTick;
}


//****************************************************
// interrupt processing
void __interrupt() ISR (void) {
  //
  // UART RX ISR: put received byte in FIFO buffer, no buffer overflow check:
  if (PIE3bits.RCIE && PIR3bits.RCIF) {
    uint8_t c = RCREG;

    // send byte proper FIFO:
    if (c & T_CBIT) {     // command
      uint8_t tmp = cRxWrInd;
      cRxBuf[tmp++] = c;
      if (tmp >= RXB_LEN)
        tmp = 0;
      cRxWrInd = tmp;
    } else {                // character
      uint8_t tmp = uRxWrInd;
      uRxBuf[tmp++] = c;
      if (tmp >= RXB_LEN)
        tmp = 0;
      uRxWrInd = tmp;
    }
  }
  if (RCSTAbits.OERR) {   // clear any RX overflow
    RCSTAbits.CREN = 0;
    RCSTAbits.CREN = 1;
  }  // UART RX ISR end

  if(PIR0bits.TMR0IF) {   // Timer0   every 1ms
    PIR0bits.TMR0IF = 0;
    TickCnt++;
    if(disp_cnt!=0) disp_cnt--;
    if(off_cnt!=0) off_cnt--;
    TMR0H = 0xE0;
    TMR0L = 0xC0;   // 8_000 cycles to OF
    //
    if(TickCnt>=btn_cnt){  // every 10ms
      btn_cnt += 10;
      //
      if(GetButton | Start){
        disp_cnt = Disp_time;
        off_cnt = Off_time;
      }
      //
      if(GetButton){  //
        if(btn_1_cnt<250) btn_1_cnt++;
        if(btn_1_cnt==35) B_long = 1;  // long pressing detected
        if((btn_1_cnt==250) & OLED_PWD) B_xlong = 1;  // Xtra long pressing detected
      }
      else if(btn_1_cnt>2 & btn_1_cnt<35){
        B_short = 1;               // short pressing detected
        btn_1_cnt = 0;
      }
      else
        btn_1_cnt = 0;
      //  External interface
      if(Start){
        if(btn_2_cnt<25) btn_2_cnt++;
        if((btn_2_cnt==20) & Key_in) E_long = 1;
      }
      else if(btn_2_cnt>1 & btn_2_cnt<10){
        E_short = 1;
        btn_2_cnt = 0;
      }
      else
        btn_2_cnt = 0;
    }
  }
  return;
}


//****************************************************
void main() {
  pic_init();
  btExtension_init();
  Delay_ms (1500);      // wait for BT processor start
  btExtension_main();   // get BT code version
  cells_reading();
  eepromParamRestore(); // restore cells etc from EEPROM
  Key_out = 1;
  gre = 1;
  OLED_PWD = 1;
  Soft_I2C_Init();
  oled_init();
  check_reset_flags();
  oled_start();
  ADC_Init();
  Overflow = 0;
  //
  disp_cnt = Disp_time;
  off_cnt = Off_time;

  while(1) {
    btExtension_main ();

    if(Tick>=volt_cnt){ // every 3 second
      volt_cnt += 3000;
      Voltage_show();
    }
    // every 300 ms unless power off or VHF/UHF bands
    if((Tick>=watch_cnt) && ((currentBand <= HF_BANDS) || (currentBand >= ALL_BANDS))){
      watch_cnt += 300;
      watch_swr();
    }
    //
    if(Disp_time!=0 && disp_cnt==0) {  // Display off
      // power down Tuner for a period if BT is connected and CLI not in use:
      if ((currentBTstate == T_CONN) && !cliActive) {
        power_off (1);
        Voltage_show();
      } else
        OLED_PWD = 0;
    }
    //
    if(Off_time!=0 && off_cnt==0){    // Go to power off
      power_off(0);
    }
    //
    if(B_short){
      if(OLED_PWD) Btn_short();
      else oled_start();
    }
    if(B_long){
      if(OLED_PWD) Btn_long();
      else oled_start();
    }
    if(B_xlong){
      if(!OLED_PWD)
        oled_start();
      else if (!oledMenu())
        Btn_xlong();
    }
    // External interface
    if(E_short){
      if(OLED_PWD==0) oled_start();
      Btn_short();
    }
    if(E_long && ((currentBand <= HF_BANDS) || (currentBand >= ALL_BANDS))){
      if(OLED_PWD==0) { Ext_long(); oled_start(); }
      else Btn_long();
    }

  } // while(1)
} // main
//
void oled_start(){
  OLED_PWD = 1;
  Delay_us(500);
  Soft_I2C_Init();
  oled_init();
  //
  if(gre){
    Greating();
    Delay_ms(3000);
    gre = 0;
    oled_clear();
  }
  oled_bat();
  Voltage_old = 9999;
  SWR_fixed_old = 100;
  PWR_fixed_old = 9999;
  draw_swr(SWR_ind);
  volt_cnt = Tick + 1;
  watch_cnt = Tick;
  B_short = 0; B_long = 0; B_xlong = 0, E_short = 0; E_long = 0;
  disp_cnt = Disp_time;
  off_cnt = Off_time;
  show_Band_Relay ();
  return;
}
//
void watch_swr(void){
  int delta = Auto_delta - 100;
  int PWR_fixed, SWR_fixed, c;
  //
  // peak detector
  PWR_fixed = 0;
  SWR_fixed = 0;
  for(c=0; c<Peak_cnt; c++){
    get_pwr();
    if(PWR>PWR_fixed) {PWR_fixed = PWR; SWR_fixed = SWR;}
    Delay_us(1000);
  }
  //
  if(PWR_fixed>0){   // Turn on the display
    if(OLED_PWD){
      disp_cnt = Disp_time;
      off_cnt = Off_time;
    }
    else oled_start();
  };
  //
  if(PWR_fixed!=PWR_fixed_old){
    if(Overflow)
      oled_wr_str(0, 42, ">", 1);
    PWR_fixed_old = PWR_fixed;
    draw_power(PWR_fixed);
  }
  //
  if(SWR_fixed>99 && SWR_fixed!=SWR_ind){
    SWR_ind = SWR_fixed;
    if(PWR_fixed<min_for_start){
      SWR_fixed = 0;
      //SWR_ind = 0;    // Last meassured SWR on display ! not a bug
      draw_swr(SWR_ind);
      return;
    }
    else
      draw_swr(SWR_ind);
  }
  //
  if(Overflow){
    for(c=3; c!=0; c--){
      oled_wr_str(2, 6, "OVERLOAD ", 9);
      Delay_ms(500);
      oled_wr_str(2, 0, "         ", 9);
      Delay_ms(500);
    }
    draw_swr(SWR_fixed);
    show_Band_Relay ();
    Delay_ms(500);
    Overflow = 0;
  }
  //

  else if(Auto && PWR_fixed>=min_for_start && PWR_fixed<max_for_start && SWR_fixed>120) {
    if(  (SWR_fixed-SWR_fixed_old)>delta || (SWR_fixed_old-SWR_fixed)>delta || SWR_fixed>(999-delta) ) {
      Btn_long();
      return;
    }
  }
  //
  return;
}
//
void draw_swr(int s){
  if(s==0)
    oled_wr_str(2, 60, "0.00", 4);
  else {
    IntToStr(s, txt_2);
    txt[0] = txt_2[3];
    txt[1] = '.';
    txt[2] = txt_2[4];
    txt[3] = txt_2[5];
    //
    oled_wr_str(2, 60, txt, 4);
  }
  // show SWR correction flag
  if (s && SWRcorrection)
    oled_wr_str_s (2, 107, "\x81", 1);
  else
    oled_wr_str_s (2, 107, " ", 1);

  return;
}
//
void draw_power(int p){
  //
  if(p==0){
    oled_wr_str(0, 60, "0.0W", 4);
    return;
  }
  else if(p<10){  // <1 W
    IntToStr(p, txt_2);
    txt[0] = '0';
    txt[1] = '.';
    txt[2] = txt_2[5];
  }
  else if(p<100){ // <10W
    IntToStr(p, txt_2);
    txt[0] = txt_2[4];
    txt[1] = '.';
    txt[2] = txt_2[5];
  }
  else{  // >10W
    p += 5;
    IntToStr(p, txt_2);
    txt[0] = ' ';
    txt[1] = txt_2[3];
    txt[2] = txt_2[4];
  }
  txt[3] = 'W';
  oled_wr_str(0, 60, txt, 4);
  return;
}
//
void Voltage_show(){       //  4.2 - 3.4  4200 - 3400
  get_batt();
  if(Voltage != Voltage_old) {
    Voltage_old = Voltage;
    if (OLED_PWD)
      oled_voltage(Voltage);
    if(Voltage<=3800) rldl = Rel_Del + 1;
    else rldl = Rel_Del;
  }

  if(Voltage<3400){
    if (!OLED_PWD) {
      OLED_PWD = 1;
      Delay_ms(200);
      Soft_I2C_Init();
      Delay_ms(10);
      oled_init();
    }
    oled_clear();
    oled_wr_str(1, 0, "  LOW BATT ", 11);
    Delay_ms(2000);
    OLED_PWD = 0;
    power_off(0);
  }
  return;
}
//
void Btn_xlong(){
  oled_clear();
  oled_wr_str(1, 0, " POWER OFF ", 11);
  Delay_ms(2000);
  power_off(0);
  return;
}
//
void Btn_long(){
  oled_wr_str(2, 0, "TUNE     ", 9);
  while (GetButton) {
    Delay_ms(10);
    if (B_xlong) {
      B_long = 0;
      return;
    }
  }
  Key_out = 0;
  Relay_set (0, 0, I_TUNE);
  tuneAbort = 0;
  Tuning = 1;
  if (currentBTstate != T_CONN) {
    currentBand = 255;
    pendingBand = 255;
    SWRcorrection = 0;
    btStandby = 0;
  }
  uSendCmd (T_TUNE);       // send tune start to BT
  if (!B_long)
    scanTunedBandSlots (Auto_delta);
  if ((B_long || (SWR == 0) || (SWR > Auto_delta)) && !tuneAbort) {
    tune();
    if ((SWR > 120) && (SWR < 250) && !tuneAbort)
      nudge_tune (120);
  } else
    rel_I &= ~I_TUNE;
  uSendCmd (T_TEND);       // send tune stop to BT
  if ((SWR != 0) && !tuneAbort) {
    rel_I &= ~I_TUNE;
    if (SWR < 200)
      saveBandState();
  } else
    rel_I |= I_TUNE;
  deBUG_PRINTF_2 ("Tuning Done LCI %3d %3d %d SWR %d.%02d Abort %d\n", rel_L, rel_C, rel_I, SWR / 100, SWR % 100, tuneAbort);

  SWR_ind = SWR;
  SWR_fixed_old = SWR;
  show_Band_Relay ();
  draw_swr(SWR_ind);
  Key_out = 1;
  B_long = 0;
  E_long = 0;
  btn_1_cnt = 0;
  volt_cnt = Tick;
  watch_cnt = Tick;
  Tuning = 0;
  return;
}
//
void Ext_long(){
  OLED_PWD = 1;
  Key_out = 0;   //
  Relay_set (0, 0, I_TUNE);
  tuneAbort = 0;
  Tuning = 1;
  get_swr();     //
  if((SWR>99) && Auto && !tuneAbort){
    uSendCmd (T_TUNE);       // send tune start to BT
    if (!E_long)
      scanTunedBandSlots (Auto_delta);
    if (((SWR == 0) || (SWR > Auto_delta)) && !tuneAbort) {
      tune();
      if ((SWR > 120) && (SWR < 250) && !tuneAbort)
        nudge_tune (120);
    } else
      rel_I &= ~I_TUNE;
    uSendCmd (T_TEND);       // send tune stop to BT
    if ((SWR > 0) && (SWR < 200) && !tuneAbort) {
      rel_I &= ~I_TUNE;
      saveBandState ();
    }
    if ((SWR == 0) || tuneAbort)
      Relay_set (0, 0, I_TUNE);
  }
  Key_out = 1;   //
  SWR_ind = SWR;
  E_long = 0;
  Tuning = 0;
  return;
}
//
void Btn_short(){
  if (btStandby && (currentBTstate != T_CONN)) {
    btStandby = 0;
    B_short   = 0;
    show_Band_Relay ();
    return;
  }
  Relay_set (0, 0, 0);
  oled_wr_str(2, 0, "RESET    ", 9);
  Delay_ms(600);
  show_Band_Relay ();
  oled_wr_str(2, 60, "0.00", 4);
  SWR_fixed_old = 0;
  Delay_ms(300);
  B_short = 0;
  E_short = 0;
  btn_1_cnt = 0;
  volt_cnt = Tick;
  watch_cnt = Tick;
  return;
}
//
void Greating(){
  oled_clear();
  oled_wr_str_s(0, 0, " DESIGNED BY N7DDC", 18);
  oled_wr_str_s(1, 0, " FW VERSION ", 12);
  oled_wr_str_s(1, 12*6, FW_VER, 3);
  oled_wr_str_s(1, 16*6, "Based", 5);
  oled_wr_str_s(2, 0, " BT i/f by LB1LI", 16);
  oled_wr_str_s(3, 0, " PIC/BT FW Ver", 14);
  oled_wr_str_s(3, 15*6, btExtVer, 5);
  while(GetButton) asm ("NOP");
  return;
}
//
void atu_reset(){
  ind = 0;
  cap = 0;
  SW = 0;
  Relay_set(ind, cap, SW);
  return;
}
//
void Relay_set(char Li, char Ci, char I){
  if ((I & I_TUNE) != (rel_I & I_TUNE))
    rel_I = (rel_I & ~I_TUNE) | (I & I_TUNE);   // always update tune bit

  if ((Li == rel_L) && (Ci == rel_C) && ((I & I_REL) == (rel_I & I_REL)))
    return;

  rel_L = Li & 0x7f;
  rel_C = Ci & 0x7f;
  rel_I = I  & (I_TUNE | I_REL);

  // use inline debug printout to avoid call stack depth problem:
  if (dbLevel >= 4) {
    int tmpSWR = (SWR >= 100)? SWR : SWR * 10;
    uPuts ("SWR ");
    uTxBuf[0] = (char)(tmpSWR / 100) + '0';
    char2str (&uTxBuf[1], (char)(tmpSWR % 100));
    uTxBuf[1] = '.';

    memcpy (&uTxBuf[4], " : Relay LCI  ", 14);
    char2str (&uTxBuf[18], rel_L);
    uTxBuf[21] = ' ';
    char2str (&uTxBuf[22], rel_C);
    uTxBuf[25] = ' ';
    uTxBuf[26]  = rel_I + '0';
    uTxBuf[27] = '\n';
    for (char i = 0; i <= 27; i++)
      uPutc (uTxBuf[i]);
  }

  bit8t X;
  X.B = ~Li;
  L_010 = X.B0;
  L_022 = X.B1;
  L_045 = X.B2;
  L_100 = X.B3;
  L_220 = X.B4;
  L_450 = X.B5;
  L_1000 = X.B6;
  //
  X.B = ~Ci;
  C_22 = X.B0;
  C_47 = X.B1;
  C_100 = X.B2;
  C_220 = X.B3;
  C_470 = X.B4;
  C_1000 = X.B5;
  C_2200 = X.B6;
  //
  C_sw = I & 0x01;
  //
  Rel_to_gnd = 1;
  VDelay_ms(rldl);
  Rel_to_gnd = 0;
  Delay_us(10);
  Rel_to_plus_N = 0;
  VDelay_ms(rldl);
  Rel_to_plus_N = 1;
  VDelay_ms(rldl);
  //
  L_010 = 0;
  L_022 = 0;
  L_045 = 0;
  L_100 = 0;
  L_220 = 0;
  L_450 = 0;
  L_1000 = 0;
  //
  C_22 = 0;
  C_47 = 0;
  C_100 = 0;
  C_220 = 0;
  C_470 = 0;
  C_1000 = 0;
  C_2200 = 0;
  //
  C_sw = 0 & 0x01;
  return;
}

// power off (sleep) Tuner and optionally BT - power off/sleep modes:
//  0: power off both Tuner and BT, wake both on long button press
//  1: only Tuner off with timed and BT interrupt wake
//  2: both Tuner and BT off with timed wake of both
void power_off (char sleepMode) {
  char button_cnt;
  deBUG_PRINTF_3 ("power_off (%d) %d S\n", sleepMode, sleepTime_s); Delay_ms(1);
  // Disable interrupts
  INTCONbits.GIE   = 0;
  T0CON0bits.T0EN  = 0;
  PIR0bits.TMR0IF  = 0;
  PIE0bits.IOCIE   = 1;
  IOCBFbits.IOCBF5 = 0;
  IOCBNbits.IOCBN5 = 1;
  // Power saving
  OLED_PWD = 0;

  // disable ADC and UART and set up wake conditions:
  ADC_DeInit();
  uartDeInit();
  wakeCause = 0;

  if (sleepMode == 1) {    // set up wake on BT interrupt
    LATBbits.LATB3   = 1;  // keep BT enabled
    IOCBFbits.IOCBF4 = 0;
    IOCBNbits.IOCBN4 = 1;  // enable BT interrupt on UART RX pin
  } else
    LATBbits.LATB3   = 0;  // BT shutdown

  if (sleepMode != 0)      // timed wake
    timer0WakeInit (sleepTime_s);

  button_cnt = 0;
  while(1){
    if(button_cnt==0) {Delay_ms(100);IOCBFbits.IOCBF5 = 0;asm ("sleep"); }
    asm ("NOP");
    if(GetButton) {
      if (sleepMode)
        break;
      else {
        button_cnt++;
        Delay_ms(100);
      }
    }
    else button_cnt = 0;
    if(button_cnt>15) break;

    if (IOCBFbits.IOCBF4) {  // wake from UART RX pin interrupt
      IOCBFbits.IOCBF4 = 0;  // clear and disable interrupt
      IOCBNbits.IOCBN4 = 0;
      if (sleepMode == 1) {
        wakeCause = 2;
        break;
      }
    }
    if (PIR0bits.TMR0IF) {   // wake from timer
      PIR0bits.TMR0IF = 0;   // clear interrupt
      if (sleepMode != 0) {
        wakeCause = 1;
        break;
      }
    }
  }

  // power-up reset:
  if (sleepMode == 0) {
    TickCnt   = 0;  // reset Tick counter/timers to avoid rollover
    volt_cnt  = 0;
    watch_cnt = 0;
    btn_cnt   = 0;
    SWR_ind   = 0;
    cliActive = 0;
    btStandby = 0;
    pendingTxOn = 0;
    currentBand = 255;
    pendingBand = 255;
    SWRcorrection  = 0;
    currentBTstate = 0;
    pendingBTstate = 0;
    btConnectTimer = 0;
    btPairingTimer = 0;
  }

  // Enable interrupts
  PIE0bits.IOCIE = 0;
  IOCBNbits.IOCBN5 = 0;
  IOCBFbits.IOCBF5 = 0;
  timer0Init();
  ADC_Init ();
  uartInit ();
  INTCONbits.GIE = 1;

  // Return to work

  gre = (button_cnt > 15)? 1 : 0;
  if ((wakeCause == 0) || ((wakeCause == 2) && !OLED_PWD))
    oled_start();
  while(GetButton){asm ("NOP");}
  btn_1_cnt = 0;
  B_short = 0;
  B_long = 0;
  B_xlong = 0;
  return;
}
//
void check_reset_flags(void){
  if (STKOVF || STKUNF || !nRWDT)
    oled_clear();
  else
    return;

  char i = 0;
  if(STKOVF){oled_wr_str_s(0,  0, "Stack overflow",  14); i = 1;}
  if(STKUNF){oled_wr_str_s(1,  0, "Stack underflow", 15); i = 1;}
  if(!nRWDT){oled_wr_str_s(2,  0, "WDT overflow",    12); i = 1;}
  if(!nRMCLR){oled_wr_str_s(3, 0, "MCLR reset",      10); i = 1;}
  if(!nBOR){oled_wr_str_s (3, 70, "BOR reset",        9); i = 1;}
  if(i){
    while (!GetButton && (i++ < 100))
      Delay_ms(100);
    STKOVF = 0;
    STKUNF = 0;
    nRWDT  = 1;
    nRMCLR = 1;
    nBOR   = 1;
    oled_clear();
    while (GetButton);
    B_short = 0;
    B_long  = 0;
  }
  return;
}
//
int get_reverse(void){
  unsigned int v;
  volatile unsigned long d;
  ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_FVRH1);
  v = ADC_Get_Sample(REV_input);
  if(v==1023){
    ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_FVRH2);
    v = ADC_Get_Sample(REV_input) * 2;
  }
  if(v==2046){
    ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_VREFH);
    v = ADC_Get_Sample(REV_input);
    if(v==1023) Overflow = 1;
    get_batt();
    d = (unsigned long)v * (unsigned long)Voltage;
    d = d / 1024;
    v = (unsigned int)d;
  }
  return (int)v;
}
//
int get_forward(void){
  unsigned int v;
  volatile unsigned long d;
  ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_FVRH1);
  v = ADC_Get_Sample(FWD_input);
  if(v==1023){
    ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_FVRH2);
    v = ADC_Get_Sample(FWD_input) * 2;
  }
  if(v==2046){
    ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_VREFH);
    v = ADC_Get_Sample(FWD_input);
    if(v==1023) Overflow = 1;
    get_batt();
    d = (unsigned long)v * (unsigned long)Voltage;
    d = d / 1024;
    v = (unsigned int)d;
  }
  return (int)v;
}
//
void get_pwr(){
  float F, R;
  volatile float gamma;
  //
  F = get_forward();
  R = get_reverse();
  F /= 1000;  // to Volts
  R /= 1000;  // to Volts
  F = Cal_a * F * F + Cal_b * F;
  R = Cal_a * R * R + Cal_b * R;
  PWR = (int)(F * 10 + 0.5);         // 0 - 150 (0 - 15.0 Watts)
  //
  if(PWR>0){
    if(OLED_PWD){
      disp_cnt = Disp_time;
      off_cnt = Off_time;
    }
    else oled_start();
  }
  //
  if(PWR<min_for_start)  SWR = 0;      // < 1W
  else if(R >= F) SWR = 999;
  else {
    gamma = sqrt_n(R / F);
    if((1.0-gamma) == 0) gamma = 0.001;
    gamma = ((1.0f + gamma) / (1.0f - gamma));
    if(gamma<1.0)
      gamma = 1.0;
    if(gamma>9.985) SWR = 999;
    else SWR = (int)(gamma * 100 + 0.5);

    SWR += SWRcorrection;
    if (SWR < 100)
      SWR = 100;
    else if (SWR > 999)
      SWR = 999;
  }
  //
  return;
}
//
float sqrt_n(float x){   // Thanks, Newton !
  char i;
  const char n = 8;
  float a[8];
  a[0] = x/2;
  for(i=1; i<(n); i++)
    a[i] = (a[i-1] + x/a[i-1]) / 2;
  //
  return a[n-1];
}
//
void get_swr(){
  int pwr_cnt = 75, tuneoff_cnt = 100;
  int swr_1, pwr_1, PWR_max = 0;
  char cnt;
  PWR = 0;
  SWR = 0;
  PWR_max = 0;
  //
  while(PWR<min_for_start || PWR>max_for_start){   // waiting for good power
    //
    if(B_short){
      Btn_short();
      SWR = 0;
      break;
    }
    if(B_xlong){
      //Btn_xlong();
      SWR = 0;
      break;
    }
    //
    swr_1 = 1000;
    for(cnt=5; cnt>0; cnt--){
      get_pwr();
      if (tuneAbort) return;
      if(SWR<swr_1){
        swr_1 = SWR ;
        pwr_1 = PWR;
        Delay_us(500);
        btCommProcess ();  // run BT background task
      }
      else{
        SWR = swr_1;
        PWR = pwr_1;
        break;
      }
    }
    //
    if(PWR>min_for_start & PWR<max_for_start)
      break;
    //
    if(pwr_cnt>0){
      pwr_cnt --;
      if(PWR>PWR_max)
        PWR_max = PWR;
    }
    else {
      if(PWR_max!=PWR_fixed_old) draw_power(PWR_max);
      PWR_fixed_old = PWR_max;
      PWR_max = 0;
      pwr_cnt = 50;
      if(tuneoff_cnt>0) tuneoff_cnt--;
      else { SWR = 0; break; }
    }
  }
  //  good power
  return;
}
//
void get_batt(void){
  ADC_Init_Advanced(_ADC_INTERNAL_VREFL | _ADC_INTERNAL_FVRH1);
  Voltage = (int)ADC_Get_Sample(Battery_input) * 11;
  return;
}
//
void tune(void){
  int SWR_mem;
  char cap_mem, ind_mem;
  //
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  subtune();
  if (tuneAbort) return;
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  SWR_mem = SWR;
  cap_mem = cap;
  ind_mem = ind;
  if(SW==1) SW = 0;
  else SW = 1;
  subtune();
  if (tuneAbort) return;
  get_swr();
  if (tuneAbort) return;
  if(SWR>SWR_mem){
    if(SW==1) SW = 0;
    else SW = 1;
    cap = cap_mem;
    ind = ind_mem;
    Relay_set(ind, cap, SW);
    get_swr();
    if (tuneAbort) return;
  }
  if(SWR<=120) return;
  sharp_tune();
  if (tuneAbort) return;
  get_swr();
  if(SWR==999)
    atu_reset();
  return;
}
//
void subtune(void){
  cap = 0;
  ind = 0;
  Relay_set(ind, cap, SW);
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  coarse_tune();
  if (tuneAbort) return;
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  sharp_tune();
  return;
}
//
void coarse_tune(void){
  int SWR_mem1 = 10000, SWR_mem2 = 10000, SWR_mem3 = 10000;
  char ind_mem1, cap_mem1, ind_mem2, cap_mem2, ind_mem3, cap_mem3;
  coarse_cap();
  if (tuneAbort) return;
  coarse_ind();
  if (tuneAbort) return;
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  SWR_mem1 = SWR;
  ind_mem1 = ind;
  cap_mem1 = cap;
  if(cap<=2 & ind<=2){
    cap = 0;
    ind = 0;
    Relay_set(ind, cap, SW);
    coarse_ind();
    if (tuneAbort) return;
    coarse_cap();
    if (tuneAbort) return;
    get_swr();
    if((SWR<=120) || tuneAbort) return;
    SWR_mem2 = SWR;
    ind_mem2 = ind;
    cap_mem2 = cap;
  }
  //  if(cap<=2 & ind<=2){
  cap = 0;
  ind = 0;
  Relay_set(ind, cap, SW);
  coarse_ind_cap();
  if (tuneAbort) return;
  get_swr();
  if((SWR<=120) || tuneAbort) return;
  SWR_mem3 = SWR;
  ind_mem3 = ind;
  cap_mem3 = cap;
  //  }
  if(SWR_mem1<=SWR_mem2 & SWR_mem1<=SWR_mem3){
    cap = cap_mem1;
    ind = ind_mem1;
  }
  else if(SWR_mem2<=SWR_mem1 & SWR_mem2<=SWR_mem3){
    cap = cap_mem2;
    ind = ind_mem2;
  }
  else if(SWR_mem3<=SWR_mem1 & SWR_mem3<=SWR_mem2){
    cap = cap_mem3;
    ind = ind_mem3;
  }
  return;
}
//
void coarse_ind_cap(void){
  int SWR_mem;
  char ind_mem;
  ind_mem = 0;
  get_swr();
  if (tuneAbort) return;
  SWR_mem = SWR / 10;
  for(ind=1; ind<64; ind*=2){
    Relay_set(ind, ind, SW);
    get_swr();
    if (tuneAbort) break;
    SWR = SWR/10;
    if(SWR<=SWR_mem){
      ind_mem = ind;
      SWR_mem = SWR;
    }
    else if (SWR < 30)
      break;
  }
  ind = ind_mem;
  cap = ind_mem;
  Relay_set(ind, cap, SW);
  return;
}
//
void coarse_cap(void){
  int SWR_mem;
  char cap_mem;
  cap_mem = 0;
  get_swr();
  if (tuneAbort) return;
  SWR_mem = SWR / 10;
  for(cap=1; cap<64; cap*=2){
    Relay_set(ind, cap, SW);
    get_swr();
    if (tuneAbort) break;
    SWR = SWR/10;
    if(SWR<=SWR_mem){
      cap_mem = cap;
      SWR_mem = SWR;
    }
    else if (SWR < 30)
      break;
  }
  cap = cap_mem;
  Relay_set(ind, cap, SW);
  return;
}
//
void coarse_ind(void){
  int SWR_mem;
  char ind_mem;
  ind_mem = 0;
  get_swr();
  if (tuneAbort) return;
  SWR_mem = SWR / 10;
  for(ind=1; ind<64; ind*=2){
    Relay_set(ind, cap, SW);
    get_swr();
    if (tuneAbort) return;
    SWR = SWR/10;
    if(SWR<=SWR_mem){
      ind_mem = ind;
      SWR_mem = SWR;
    }
    else if (SWR < 30)
      break;
  }
  ind = ind_mem;
  Relay_set(ind, cap, SW);
  return;
}
//
void sharp_tune(void){
  char swrImproving;
  get_swr();
  if (tuneAbort) return;
  int SWR_min = SWR;
  do {
    swrImproving = 0;

    while (1) {
      sharp_ind();
      if (tuneAbort) return;
      get_swr();
      if ((SWR <= 120) || tuneAbort)
        return;
      else if (SWR < SWR_min) {
        SWR_min = SWR;
        swrImproving = 1;
      } else
        break;
    }

    while (1) {
      sharp_cap();
      if (tuneAbort) return;
      get_swr();
      if ((SWR <= 120) || tuneAbort)
        return;
      else if (SWR < SWR_min) {
        SWR_min = SWR;
        swrImproving = 1;
      } else
        break;
    }
  } while (swrImproving);
}
//
void sharp_cap(void){
  int SWR_mem;
  char step, cap_mem;
  cap_mem = cap;
  step = cap / 10;
  if(step==0) step = 1;
  get_swr();
  if (tuneAbort) return;
  SWR_mem = SWR;
  cap += step;
  Relay_set(ind, cap, SW);
  get_swr();
  if (tuneAbort) return;
  if(SWR<=SWR_mem){
    SWR_mem = SWR;
    cap_mem = cap;
    for(cap+=step; cap<=(127-step); cap+=step){
      Relay_set(ind, cap, SW);
      get_swr();
      if (tuneAbort) return;
      if(SWR<=SWR_mem){
        cap_mem = cap;
        SWR_mem = SWR;
        step = cap / 10;
        if(step==0) step = 1;
      }
      else
        break;
    }
  }
  else{
    SWR_mem = SWR;
    for(cap-=step; cap>=step; cap-=step){
      Relay_set(ind, cap, SW);
      get_swr();
      if (tuneAbort) return;
      if(SWR<=SWR_mem){
        cap_mem = cap;
        SWR_mem = SWR;
        step = cap / 10;
        if(step==0) step = 1;
      }
      else
        break;
    }
  }
  cap = cap_mem;
  Relay_set(ind, cap, SW);
  return;
}
//
void sharp_ind(void){
  int SWR_mem;
  char step, ind_mem;
  ind_mem = ind;
  step = ind / 10;
  if(step==0) step = 1;
  get_swr();
  if (tuneAbort) return;
  SWR_mem = SWR;
  ind += step;
  Relay_set(ind, cap, SW);
  get_swr();
  if (tuneAbort) return;
  if(SWR<=SWR_mem){
    SWR_mem = SWR;
    ind_mem = ind;
    for(ind+=step; ind<=(127-step); ind+=step){
      Relay_set(ind, cap, SW);
      get_swr();
      if (tuneAbort) return;
      if(SWR<=SWR_mem){
        ind_mem = ind;
        SWR_mem = SWR;
        step = ind / 10;
        if(step==0) step = 1;
      }
      else
        break;
    }
  }
  else{
    SWR_mem = SWR;
    for(ind-=step; ind>=step; ind-=step){
      Relay_set(ind, cap, SW);
      get_swr();
      if (tuneAbort) return;
      if(SWR<=SWR_mem){
        ind_mem = ind;
        SWR_mem = SWR;
        step = ind / 10;
        if(step==0) step = 1;
      }
      else
        break;
    }
  }
  ind = ind_mem;
  Relay_set(ind, cap, SW);
  return;
}
//

// trim L/C/I to get further SWR improvements:
void nudge_tune (int swrLimit) {
  int8_t L_min = (int8_t)rel_L, C_min = (int8_t)rel_C, I_min = (int8_t)rel_I & I_REL;
  int SWR_min = SWR;
  int8_t L_inc[] = {0,  0, 1, -1, 0};
  int8_t C_inc[] = {1, -1, 0,  0, 0};

  while (1) {
    deBUG_PRINTF_4 ("nudge_tune L %d C %d SWR %d\n", L_min, C_min, SWR_min);
    int8_t L_in = L_min, C_in = C_min, L_try = L_min, C_try = C_min, I_try = I_min;
    int SWR_in = SWR_min;

    for (int8_t i = 0; i < 50;) {
      L_try += L_inc[i % 5], C_try += C_inc[i % 5];
      if ((L_try >= 0) && (C_try >= 0)) {
        Relay_set ((char)L_try, (char)C_try, (char)I_try);
        get_swr();
      }
      if (tuneAbort)
        return;
      if ((SWR < SWR_min) && (SWR >= 100) && (L_try >= 0) && (C_try >= 0))
        SWR_min = SWR, L_min = L_try, C_min = C_try, I_min = I_try;
      else {
        if ((i % 5) == 0)
          C_try = C_in;
        else if ((i % 5) == 1)
          C_try = C_min;
        else {
          L_try = L_in;
          if ((i % 5) == 3) {
            L_try = L_min;
            if (C_min <= 8)
              I_try = !I_try;
            else
              i++;
          }
        }
        if ((i++ % 5) == 4)
          break;
      }
    }
    if ((SWR_min == SWR_in) || (SWR_min <= swrLimit) || tuneAbort)
      break;
  }
  Relay_set ((char)L_min, (char)C_min, (char)I_min);
  SWR = SWR_min;
}


void cells_reading(void){
  char i;
  char Cells_2[10];
  for(i=0; i<10; i++){
    Cells_2[i] = Cells[i];
  }
  //
  Disp_time = (unsigned long)Bcd2Dec(Cells_2[0]) ;
  Disp_time *= 60000;                        // minutes to ms
  Off_time = (unsigned long)Bcd2Dec(Cells_2[1]);
  Off_time *= 60000;                         // minutes to ms
  Rel_Del =  Bcd2Dec(Cells_2[2]);            // Delay in ms
  min_for_start = Bcd2Dec(Cells_2[3]);       // Power with tens parts
  max_for_start = Bcd2Dec(Cells_2[4]) * 10;  // power in Watts
  Auto_delta = Bcd2Dec(Cells_2[5]) * 10;     // SWR with tens parts
  Auto = Bcd2Dec(Cells_2[6]);
  Cal_b = (float)Bcd2Dec(Cells_2[7]) / 10.0f;
  Cal_a = (float)Bcd2Dec(Cells_2[8]) / 100.0f + 1.0f;
  Peak_cnt = Bcd2Dec(Cells_2[9]) * 10 / 6;
  rldl = Rel_Del;
  return;
}

//
