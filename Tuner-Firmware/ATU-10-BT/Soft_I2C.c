//
#include "Soft_I2C.h"
#include "main.h"
//
#define Delay_I2C Delay_us(25)
#define Delay_I2C_short Delay_us(5) // short dalay mainly used after falling edge
//
// Software I2C connections
#define Soft_I2C_Scl     LATAbits.LATA3
#define Soft_I2C_Sda     LATAbits.LATA2
#define Soft_I2C_Sda_in  PORTAbits.RA2
#define Soft_I2C_Scl_in  PORTAbits.RA3
//
void Soft_I2C_Init(void) {
  Soft_I2C_Stop();
  return;
}

void Soft_I2C_Start() {
  Soft_I2C_Scl = 1;
  Soft_I2C_Sda = 1;
  Delay_I2C;
  Soft_I2C_Sda = 0;
  Delay_I2C_short;
  Soft_I2C_Scl = 0;
  Delay_I2C_short;
  return;
}

char Soft_I2C_Write(char d) {
  char i, ack;
  for(i=0; i<8; i++) {
    if (d & 0x80) {
      Soft_I2C_Sda = 1;
      Delay_I2C_short;
    } else
      Soft_I2C_Sda = 0;
    Soft_I2C_Scl = 1;
    Delay_I2C;
    Soft_I2C_Scl = 0;
    d <<= 1;
  }
  //
  Soft_I2C_Sda = 1; //ACK
  Delay_I2C;
  Soft_I2C_Scl = 1;
  ack = Soft_I2C_Sda_in;
  Delay_I2C;
  Soft_I2C_Scl = 0;
  Delay_I2C_short;
  return ack;
}

char Soft_I2C_Read(void){
  char i, d = 0;
  for(i=0; i<8; i++){
    d = (char)(d << 1);
    Soft_I2C_Scl = 1;
    Delay_I2C;
    d = d + Soft_I2C_Sda_in;
    Soft_I2C_Scl = 0;
    Delay_I2C;
  }
  return d;
}

void Soft_I2C_ACK(void){
  Soft_I2C_Sda = 0;
  Delay_I2C;
  Soft_I2C_Scl = 1;
  Delay_I2C;
  Soft_I2C_Scl = 0;
  Soft_I2C_Sda = 1;
  Delay_I2C;
  return;
}

void Soft_I2C_NACK(void){
  Soft_I2C_Sda = 1;
  Delay_I2C;
  Soft_I2C_Scl = 1;
  Delay_I2C;
  Soft_I2C_Scl = 0;
  Delay_I2C;
  return;
}

void Soft_I2C_Stop() {
  Soft_I2C_Sda = 0;
  Delay_I2C_short;
  Soft_I2C_Scl = 1;
  Delay_I2C;
  Soft_I2C_Sda = 1;
  return;
}
