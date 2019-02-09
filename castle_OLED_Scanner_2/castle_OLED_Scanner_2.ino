/*
Yet Another cc2500 2.4GHz Scanner by Simon Castle
Scans 256 channels over the 2.4GHz ISM band,
currently has 4 modes of operation showing the Full spectrum (256 samples)
or the Bottom, Middle or Top 128 samples limited by the small screen.

As test input I'm using a TV sender which can be switched between 4 channels.

2/9/2019 Jdunne - Note from Simon Castle (original author) Via email:  
I don't really mind what you do with it its just that some credit should possibly go to others but I don't know who that should be any more.
In the end there are no secrets here and its all from the net in any case :-)

Jdunne notes:
Starting from Simon Castle's code I did several things for my purposes.
(I was trying to identify the source of a wifi disturbance in my house every 5 seconds
which turned out to be some kind of frequency scan coming from the router itself.)

* Switched to an esp32 with built in oled display (Hiletgo wifi kit)
* Updated from u8g lib to u8g2
* Added "spike" mode to zoom into a certain range (code configured range)
* Spike mode also shows the max rssi over a 5 second interval. (for locating the Signal source)
* Minor refactoring during development
* Added horizontal axis tick marks to aid in identifying frequencies 

*/

#include <SPI.h>
#include "U8g2lib.h"
#include "cc2500_REG.h"

//OLED setup for "Heltec Wifi Kit 32"
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);	// Dev 0, Fast I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send ACK

#define BUT 2
#define SCAN_CS    5     // scanner Select
#define RSSI_TRESHOLD 100  // treshold of RSSI alarm level
#define RSSI_OFFSET   88 //88 // was 75//95   // offset for displayed data
#define RSSI_OVERSAMPLE 50 // number of measures for each channel to found a max RSSI value
#define MAX_CHAN_QTY  256  // max number of channel for spacing 405.456543MHz
//#define MAX_DISP_LINE 84//159  // max horizontal display resolution

//original sampling:
//#define MAX_SAMPLING 20 // was 500    // qty of samples in each iteration (1...100)
#define MAX_SAMPLING 20 // was 500    // qty of samples in each iteration (1...100)

#define mode_full   0
#define mode_bottom 1
#define mode_middle 2
#define mode_top    3
#define mode_spike  4
#define mode_last   mode_spike

//range of channels to display for spike mode  (separate from scan range below):
int SpikeDisplayStart = 100;    //112;
int SpikeDisplayEnd = 163;      //128;

//range of channels to scan for spike mode:  (setting this different from display effects the samping rate which may be useful in some cases)
//reduce this as much as possible for the fastest sampling rate.  display will only show a subset of the scan range.
int SpikeScanStart = 100;
int SpikeScanEnd = 227;


byte DisplayMode = 0;
byte inter = 0;
byte dataB;
byte cal[MAX_CHAN_QTY];
//byte rssi_data[MAX_DISP_LINE];
byte RSSI_data;
int RSSI_dbm;
int RSSI_max;
int MaxPeak;
long cont;

double AvgMaxPeak = 0;

byte data[128];

#define NUM_PEAKS   16      //With a scan range of 128 channels, it takes about 5 seconds to do 16 scans.  This represents the max peak observed during that time.
int peaks[NUM_PEAKS];
int peakindex = 0;

void draw(void)
{
  int HighestPeak;
  int Height;
  for(byte x = 0; x < 128; x++)
  {
//    u8g2.drawVLine(x,0,data[x]);
    Height = data[x];
    if (Height > 60) Height = 60;
    
    if (Height > 0) {
      //u8g2.drawVLine(x,50-Height,50);
      u8g2.drawVLine(x,60-Height,Height);
    }
  }

  //draw X axis notches:
  for(byte x = 0; x < 128; x+=10)
  {
    //draw notches every 10 ticks to allow identifying the range of frequencies
    u8g2.drawVLine(x,62,2);
  }
  
  if (DisplayMode == mode_spike) {

    //Average the peak values;
    //AvgMaxPeak = (MaxPeak - AvgMaxPeak)/128 + AvgMaxPeak;

    //Store each peak value in a circular buffer of recent peak values:
    peaks[peakindex] = MaxPeak;
    peakindex++;
    if (peakindex > NUM_PEAKS) peakindex = 0;

    //sort through the circular buffer and find the highest peak value observed recently:
    HighestPeak = 0;
    for (int i = 0; i < NUM_PEAKS; i++) {
      if (peaks[i] > HighestPeak) HighestPeak = peaks[i];
    }

    //Display the highest peak observed recently:
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(100, 12);
    u8g2.print(HighestPeak);

    //print the index so I can keep track of how long an interval the peak represents:
//    u8g2.setCursor(100, 42);
//    u8g2.print(peakindex);
    
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
  //SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK

//  attachInterrupt(0, myISR, RISING); // INT 1 is D3, INT 0 is D2
  attachInterrupt(0, myISR, FALLING); // INT 1 is D3, INT 0 is D2

//  u8g2.setRot180();
//  u8g2.setColorIndex(1);         // pixel on
//  u8g2.setScale2x2();

  init_CC2500();

  //Scan all channels to calibrate:
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

  u8g2.begin();
  u8g2.clearBuffer();
  displayModeScreen();
}

void loop(void)
{
  int StartChan = 0;
  int EndChan = 255;
  
  //Clear all channel data:
  for (int i = 0; i < MAX_CHAN_QTY; i++) data[i] = 0;
  MaxPeak = 0;

  switch(DisplayMode)
  {
    case mode_full:
      StartChan = 0;
      EndChan = 255;
      break;
    case mode_bottom: 
      StartChan = 0;
      EndChan = 127;
      break;
    case mode_middle:
      StartChan = 64;
      EndChan = 191;
      break;      
    case mode_top:
      StartChan = 128;
      EndChan = 255;
      break;
    case mode_spike:
      StartChan = SpikeScanStart;   //SpikeDisplayStart;
      EndChan = SpikeScanEnd;       //SpikeDisplayEnd;
      break;
  }

  for (int i = StartChan; i <= EndChan && !inter; i++)
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

    if (RSSI_max > MaxPeak) MaxPeak = RSSI_max;
    
    switch(DisplayMode)
    {
      case mode_full:   //full
      {
        int p = i / 2;
        if (RSSI_max > data[p])
        data[p] = RSSI_max;
      }
      break;
      
      case mode_bottom:
        if(i < 128) data[i] = RSSI_max;
      break;
      
      case mode_middle:
        if(i >= 64 && i < 192) data[i - 64] = RSSI_max;
      break;
      
      case mode_top:
        if(i >= 128) data[i - 128] = RSSI_max;
        break;
      case mode_spike:

        //only store the displayed range for spike mode:
        if(i >= SpikeDisplayStart && i <= SpikeDisplayEnd) {
            if (RSSI_max < 1) RSSI_max = 1;    //minimum value.. to put a floor and show how much bandwidth is actually sampled..
            data[i - SpikeDisplayStart] = RSSI_max;
        }
        break;
    }
  }

  if(inter)
  {
    while(inter)
    {
      inter = 0;
      DisplayMode++;
      if (DisplayMode > mode_last) DisplayMode = 0;
      displayModeScreen();
    }
  }
  else
  {
    u8g2.firstPage();  
    do
    {
      draw();
    } while( u8g2.nextPage() );
  }
//  delay(150);
}

void displayModeScreen()
{
  cont = millis() + 2500;
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g_font_6x10);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);   //White
    u8g2.setFontPosTop();
    //u8g2.setScale2x2();   //doesn't exist in u8g2

    switch(DisplayMode)
    {
      case mode_full:
        u8g2.drawStr(22, 10, "Full");
        //u8g2.undoScale();
        u8g2.drawStr(48, 44, "0..255");
        break;
      case mode_bottom:
        u8g2.drawStr(16, 10, "Bottom");
        //u8g2.undoScale();
        u8g2.drawStr(50, 44, "0..127");
        break;
      case mode_middle:
        u8g2.drawStr(16, 10, "Middle");
        //u8g2.undoScale();
        u8g2.drawStr(48, 44, "64..191");
        break;
      case mode_top:
        u8g2.drawStr(24, 10, "Top");
        //u8g2.undoScale();
        u8g2.drawStr(42, 44, "128..255");
        break;
      case mode_spike:
        u8g2.drawStr(24, 10, "Spike");
        //u8g2.undoScale();
        
        u8g2.setCursor(42, 44);
        u8g2.print(SpikeDisplayStart);
        u8g2.print("..");
        u8g2.print(SpikeDisplayEnd);
        
        ///u8g2.drawStr(42, 44, "112..128");
        break;
    }
  } while( u8g2.nextPage() );

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
