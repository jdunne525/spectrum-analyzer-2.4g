/*
Yet Another cc2500 2.4GHz Scanner by Simon Castle
Scans 256 channels over the 2.4GHz ISM band,
currently has 4 modes of operation showing the Full spectrum (256 samples)
or the Bottom, Middle or Top 128 samples limited by the small screen.

As test input I'm using a TV sender which can be switched between 4 channels.
*/

#include <SPI.h>
#include "U8glib.h"
#include "cc2500_REG.h"

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);	// Dev 0, Fast I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send ACK

#define BUT 2
#define SCAN_CS    10     // scanner Select
#define RSSI_TRESHOLD 100  // treshold of RSSI alarm level
#define RSSI_OFFSET   88 //88 // was 75//95   // offset for displayed data
#define RSSI_OVERSAMPLE 50 // number of measures for each channel to found a max RSSI value
#define MAX_CHAN_QTY  256  // max number of channel for spacing 405.456543MHz
//#define MAX_DISP_LINE 84//159  // max horizontal display resolution
#define MAX_SAMPLING 20 // was 500    // qty of samples in each iteration (1...100)

byte mode = 0;
byte inter = 0;
byte dataB;
byte cal[MAX_CHAN_QTY];
//byte rssi_data[MAX_DISP_LINE];
byte RSSI_data;
int RSSI_dbm;
int RSSI_max;
long cont;

byte data[128];
void draw(void)
{
  for(byte x = 0; x < 128; x++)
  {
//    u8g.drawVLine(x,0,data[x]);
    u8g.drawVLine(x,64-data[x],64);
  }
}

void myISR()
{
  cont = 0;
  if (millis() > 4000)
  inter = 1;
  while (digitalRead(BUT) == 0);
  delay(20);
}

void setup(void)
{
  pinMode(SCAN_CS, OUTPUT);
  pinMode(BUT, INPUT_PULLUP);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK

//  attachInterrupt(0, myISR, RISING); // INT 1 is D3, INT 0 is D2
  attachInterrupt(0, myISR, FALLING); // INT 1 is D3, INT 0 is D2

//  u8g.setRot180();
//  u8g.setColorIndex(1);         // pixel on
//  u8g.setScale2x2();

  init_CC2500();

  for (int i = 0; i < MAX_CHAN_QTY; i++)
  {
    WriteReg(CHANNR, i);		// set channel
    WriteReg(SIDLE, 0x3D);		// idle mode
    WriteReg(SCAL, 0x3D);		// start manual calibration
    delayMicroseconds(800);		// wait for calibration
    dataB = ReadReg(FSCAL1);            // read calibration value
    cal[i] = dataB;			// and store it
  }

  WriteReg(CHANNR, 0x00);		// set channel
  WriteReg(SFSTXON, 0x3D);	        // calibrate and wait
  delayMicroseconds(800);               // settling time, refer to datasheet
  WriteReg(SRX, 0x3D);			// enable rx

  displayMode();
}

void loop(void)
{
  for (int i = 0; i < MAX_CHAN_QTY; i++) data[i] = 0;

  for (int i = 0; i < MAX_CHAN_QTY && !inter; i++)
  {
    WriteReg(CHANNR, i);		// set channel
    WriteReg(FSCAL1, cal[i]);		// restore calibration value for this channel
    delayMicroseconds(300);	        // settling time, refer to datasheet

    RSSI_max = -120;

    for (int j = 0; j <= MAX_SAMPLING; j++)// collect samples for max value
    {
      digitalWrite(SCAN_CS, LOW);       // select the scanner chip
      SPI.transfer(REG_RSSI);	        // read RSSI register
      RSSI_data = SPI.transfer(0);
      digitalWrite(SCAN_CS,HIGH);

      // convert RSSI data from 2's complement to signed decimal
      if(RSSI_data >= 128)
      {
        RSSI_dbm = (RSSI_data-256) / 2-70;
      }
      else
      {
        RSSI_dbm = RSSI_data / 2-70;
      }
      if (RSSI_dbm > RSSI_max) RSSI_max = RSSI_dbm; // keep maximum   
    }

    RSSI_max += RSSI_OFFSET;
    if(RSSI_max < 0) RSSI_max = 0;
    if(RSSI_max > 110) RSSI_max = 110;
    
    switch(mode)
    {
      case 0:
      {
        int p = i / 2;
        if (RSSI_max > data[p])
        data[p] = RSSI_max;
      }
      break;
      
      case 1:
        if(i < 128) data[i] = RSSI_max;
      break;
      
      case 2:
        if(i >= 64 && i < 192) data[i - 64] = RSSI_max;
      break;
      
      case 3:
        if(i >= 128) data[i - 128] = RSSI_max;
      break;
    }
  }

  if(inter)
  {
    while(inter)
    {
    inter = 0;
    mode++;
    mode &= 3;    
    displayMode();
    }
  }
  else
  {
    u8g.firstPage();  
    do
    {
      draw();
    } while( u8g.nextPage() );
  }
//  delay(150);
}

void displayMode()
{
  cont = millis() + 2500;
  u8g.firstPage();
  do
  {
    u8g.setFont(u8g_font_6x10);
    u8g.setFontRefHeightExtendedText();
    u8g.setDefaultForegroundColor();
    u8g.setFontPosTop();
    u8g.setScale2x2();

    switch(mode)
    {
      case 0:
      u8g.drawStr(22, 10, "Full");
      u8g.undoScale();
      u8g.drawStr(48, 44, "0..255");
      break;
      
      case 1:
      u8g.drawStr(16, 10, "Bottom");
      u8g.undoScale();
      u8g.drawStr(50, 44, "0..127");
      break;
      
      case 2:
      u8g.drawStr(16, 10, "Middle");
      u8g.undoScale();
      u8g.drawStr(48, 44, "64..191");
      break;
      
      case 3:
      u8g.drawStr(24, 10, "Top");
      u8g.undoScale();
      u8g.drawStr(42, 44, "128..255");
      break;
    }
  } while( u8g.nextPage() );

  while (millis() < cont);
}

void init_CC2500()
{
  WriteReg(SRES,0x3D);       // software reset for CC2500
  WriteReg(FSCTRL1,0x0F);    // Frequency Synthesizer Control
  WriteReg(PKTCTRL0,0x12);   // Packet Automation Control
  WriteReg(FREQ2,0x5C);      // Frequency control word, high byte
  WriteReg(FREQ1,0x4E);      // Frequency control word, middle byte
  WriteReg(FREQ0,0xDE);      // Frequency control word, low byte
//  WriteReg(FREQ2,0xD7);      // Frequency control word, high byte
//  WriteReg(FREQ1,0x62);      // Frequency control word, middle byte
//  WriteReg(FREQ0,0x76);      // Frequency control word, low byte
//  WriteReg(MDMCFG4,0x0D);    // Modem Configuration 0D = 812
  WriteReg(MDMCFG4,0x6D);    // Modem Configuration 6D = 270
  WriteReg(MDMCFG3,0x3B);    // Modem Configuration
  WriteReg(MDMCFG2,0x00);    // Modem Configuration 0x30 - OOK modulation, 0x00 - FSK modulation (better sensitivity)
  WriteReg(MDMCFG1,0x23);    // Modem Configuration
//  WriteReg(MDMCFG0,0xFF);    // Modem Configuration
  WriteReg(MDMCFG0,0xA4);    // Modem Configuration
  WriteReg(MCSM1,0x0F);      // Always stay in RX mode
  WriteReg(MCSM0,0x04);      // Main Radio Control State Machine Configuration
  WriteReg(FOCCFG,0x15);     // Frequency Offset Compensation configuration
  WriteReg(AGCCTRL2,0x83);   // AGC Control
  WriteReg(AGCCTRL1,0x00);   // AGC Control
  WriteReg(AGCCTRL0,0x91);   // AGC Control
  WriteReg(FSCAL3,0xEA);     // Frequency Synthesizer Calibration
  WriteReg(FSCAL2,0x0A);     // Frequency Synthesizer Calibration 
  WriteReg(FSCAL1,0x00);     // Frequency Synthesizer Calibration 
  WriteReg(FSCAL0,0x11);     // Frequency Synthesizer Calibration
}

void WriteReg(char addr, char value)
{
  digitalWrite(SCAN_CS,LOW);
  while (digitalRead(MISO) == HIGH)
  { };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SCAN_CS, HIGH);
}

char ReadReg(char addr)
{
  addr = addr + 0x80;
  digitalWrite(SCAN_CS, LOW);
  while (digitalRead(MISO) == HIGH)
  { };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SCAN_CS,HIGH);
  return y;  
}