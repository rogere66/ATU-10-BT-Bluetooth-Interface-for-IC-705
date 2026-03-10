#include <htc.h>
#include "BT_extension.h"

void pic_init (void);
void Btn_long (void);
void Btn_short (void);
void Btn_xlong (void);
void check_reset_flags (void);
void Voltage_show (void);
void Voltage_check(void);
void Relay_set (char, char, char);
int get_reverse (void);
int get_forward (void);
void get_pwr (void);
void get_swr (void);
void get_batt (void);
void watch_swr (void);
void coarse_cap (void);
void tune (void);
void subtune (void);
void coarse_tune (void);
void coarse_cap (void);
void coarse_ind (void);
void coarse_ind_cap (void);
void sharp_tune (void);
void sharp_cap (void);
void sharp_ind (void);
void atu_reset (void);
void draw_swr (int);
void draw_power (int);
float sqrt_n (float);
void oled_start (void);
void power_off (char sleepMode);
void Greating (void);
void Ext_long (void);
void nudge_tune (int swrLimit);
void cells_reading (void);

#define INT       INTCONbits.GIE
#define Battery_input 9
#define FWD_input 8
#define REV_input 10
#define _AD_High  FVRCONbits.ADFVR0=0;FVRCONbits.ADFVR1=1;
#define _AD_Low   FVRCONbits.ADFVR0=1;FVRCONbits.ADFVR1=0;
#define Key_out   LATDbits.LATD2
#define Key_in    PORTDbits.RD2
#define Start_out LATDbits.LATD1
#define Start    !PORTDbits.RD1

extern int Voltage, PWR, SWR;
extern unsigned long disp_cnt, off_cnt;
extern char B_short, B_long;
extern unsigned char rel_L, rel_C, rel_I;
extern unsigned char Tuning;

extern unsigned long Disp_time, Off_time;
extern int Rel_Del, min_for_start, max_for_start, Auto_delta, Peak_cnt;
extern char Auto;
extern float Cal_a, Cal_b;

#define Tick Tick_ms()
unsigned long int Tick_ms (void);

// rel_I bit mask definitions:
#define I_REL  0x01  // I relay state, 0: C connected to BNC IN, 1: C connected to BNC OUT/Antenna
#define I_TUNE 0x02  // Tune Flag: 0: Tuned, 1: Tune Pending
#define I_UCON 0x04  // Unconnected Flag: tuned with BT not connected
#define I_UNNO 0x08  // Unknown Flag: relay setting is not known

typedef union {
  unsigned char B;
  struct {
    unsigned char B0 :1;
    unsigned char B1 :1;
    unsigned char B2 :1;
    unsigned char B3 :1;
    unsigned char B4 :1;
    unsigned char B5 :1;
    unsigned char B6 :1;
    unsigned char B7 :1;
  };
} bit8t;
