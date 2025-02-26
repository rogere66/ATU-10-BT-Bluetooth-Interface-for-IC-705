// pic_init unit for MPLAB X and XC8 compiler
// 2020

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include "main.h"

// Timer0 1 ms settings:
void timer0Init (void) {
  T0CON0bits.T0EN = 0;     // disable timer 0
  T0CON1bits.T0CS = 0x02;  // select Fosc/4
  OSCENbits.LFOEN = 0;     // disable 32kHz LFINTOSC
  T0CON1bits.T0ASYNC = 0;  // sync mode
  T0CON0bits.T016BIT = 1;  // 16 bit mode
  T0CON1bits.T0CKPS  = 0;  // pre-scaler
  TMR0H = 0xE0;            // MSB start value, counting up
  TMR0L = 0xC0;            // 8_000 cycles = 1 mS
  PIR0bits.TMR0IF = 0;     // clear interrupt flag
  T0CON0bits.T0EN = 1;     // enable timer
  PIE0bits.TMR0IE = 1;     // enable interrupt
}

// Timer0 wake from sleep settings, wake time in seconds, 0 = no timed wake:
void timer0WakeInit (unsigned int sleepTime_s) {
  T0CON0bits.T0EN = 0;          // disable timer 0
  PIR0bits.TMR0IF = 0;          // clear interrupt flag
  if (sleepTime_s) {
    OSCENbits.LFOEN    = 1;     // enable 32kHz LFINTOSC
    T0CON1bits.T0CS    = 4;     // select LFINTOSC
    T0CON1bits.T0ASYNC = 1;     // async mode
    T0CON0bits.T016BIT = 1;     // 16 bit mode
    T0CON1bits.T0CKPS  = 15;    // pre-scaler -> ~1Hz clock
    TMR0H = ~sleepTime_s >> 8;  // start value, counting up
    TMR0L = (char)~sleepTime_s;
    PIR0bits.TMR0IF = 0;        // clear interrupt flag
    T0CON0bits.T0EN = 1;        // enable timer
    PIE0bits.TMR0IE = 1;        // enable interrupt
  }
}


void pic_init (void) {
  // ports initialisation
  ANSELA = 0;         // all as digital
  ANSELB = 0b00000111; // B0, B1 and B2 as analog inputs
  ANSELC = 0;         // all as digital
  ANSELE = 0;         // all as digital
  ANSELD = 0;         // all as digital

  CM1CON0bits.C1ON = 0; // Disable comparators
  CM2CON0bits.C2ON = 0;

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  PORTE = 0;
  LATA = 0b00000000;
  LATB = 0b00000000;
  LATC = 0b00010000;
  LATD = 0b00000110;
  LATE = 0b00000000;
  TRISA = 0b00000000;
  TRISB = 0b00100111;
  TRISC = 0b00000000;
  TRISD = 0b00000000;
  TRISE = 0b00000000;

  // open drains
  ODCONAbits.ODCA2 = 1;
  ODCONAbits.ODCA3 = 1;
  ODCONDbits.ODCD1 = 1;
  ODCONDbits.ODCD2 = 1;

  timer0Init();

  // Modules disable
  PMD0 = 0b00011110; //
  PMD1 = 0b11111110;
  PMD2 = 0b01000111;
  PMD3 = 0b01111111;
  PMD4 = 0b1110111;
  PMD5 = 0b11011111;
  //interrupt setting
  INTCONbits.GIE = 1;
  Delay_ms (100);
  return;
}
