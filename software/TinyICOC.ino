// ===================================================================================
// Project:   TinyICOC - AVR In-Circuit Oscillator Calibrator based on ATtiny84
// Version:   v1.0
// Year:      2021
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// Sometimes AVRs are operated without an external clock. The internal
// oscillator does a good job in most applications, but when it comes to
// precise timing, it is too inaccurate. The accuracy of the oscillator
// of an AVR is only +/-10%. Fortunately, the oscillator can be calibrated,
// increasing its accuracy to +/-2% or better. There are a few ways to
// perform this manual calibration, but several steps are required. The
// TinyICOC does this fully automatically. Simply connect the device to the
// target MCU via the ICSP header and press the button.
//
// To carry out the calibration, a program is first uploaded to the target
// MCU using the integrated in-circuit serial programmer (ICSP). This
// program applies an oscillating signal with half the clock frequency to
// the MOSI pin of the target MCU. Since the fuses were previously set so
// that the target MCU clock runs with a prescaler of 8, a signal with 1/16
// of the oscillator frequency is applied to that pin. This frequency is
// measured by the timers of the ATtiny84 and compared with the target value.
// The oscillator calibration value (OSCCAL) is then adjusted accordingly
// and written into the EEPROM of the target MCU. This value is in turn
// read by the target MCU and written to its OSCCAL register. This process
// is repeated until the OSCCAL value, which leads to the lowest frequency
// deviation, has been found.
//
// References:
// -----------
// The I²C OLED implementation is based on TinyOLEDdemo
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// The small OLED font was adapted from Neven Boyanov and Stephen Denne
// https://github.com/datacute/Tiny4kOLED
//
// The calibration function is based on TinyCalibrator
// https://github.com/wagiminator/ATtiny84-TinyCalibrator
//
// Wiring:
// -------
//                                 +-\/-+
//                           Vcc  1|°   |14  GND
// 12M CRYSTAL -------- XTAL PB0  2|    |13  PA0 ADC0 AREF --- I2C SCK OLED
// 12M CRYSTAL -------- XTAL PB1  3|    |12  PA1 ADC1 AIN0 --- I2C SDA OLED
//       RESET -------- !RST PB3  4|    |11  PA2 ADC2 AIN1 --- BUTTON
//             -------- INT0 PB2  5|    |10  PA3 ADC3 T0 ----- TGT OSC
//    TGT !RST -------- ADC7 PA7  6|    |9   PA4 ADC4 SCK ---- TGT SCK
//    TGT MOSI --- MOSI ADC6 PA6  7|    |8   PA5 ADC5 MISO --- TGT MISO
//                                 +----+
//
// Compilation Settings:
// ---------------------
// Core:    ATtinyCore (https://github.com/SpenceKonde/ATTinyCore)
// Board:   ATtiny24/44/84(a) (No bootloader)
// Chip:    ATtiny44(a) or ATtiny84(a)
// Clock:   12 MHz (external)
// Millis:  disabled
// BOD:     2.7V
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// No Arduino core functions or libraries are used. Use the makefile if 
// you want to compile without Arduino IDE.
//
// Fuse settings: -U lfuse:w:0xff:m -U hfuse:w:0xd5:m -U efuse:w:0xff:m


// ===================================================================================
// Libraries and Definitions
// ===================================================================================

// Libraries
#include <avr/io.h>         // for GPIO
#include <avr/interrupt.h>  // for interrupts
#include <avr/pgmspace.h>   // to store data in programm memory
#include <util/delay.h>     // for delays

// Pin definitions
#define I2C_SCL     PA0     // I2C SCL
#define I2C_SDA     PA1     // I2C SDA
#define BUTTON_PIN  PA2     // button pin
#define OSC_PIN     PA3     // target oscillator input
#define SCK_PIN     PA4     // ISCP SCK
#define MISO_PIN    PA5     // ICSP MISO
#define MOSI_PIN    PA6     // ICSP MOSI
#define RST_PIN     PA7     // ICSP !RST

// ===================================================================================
// I2C Implementation
// ===================================================================================

// I2C macros
#define I2C_SDA_HIGH()  DDRA &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRA |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRA &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRA |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU
#define I2C_DELAY()     asm("lpm")            // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH(); I2C_DELAY(); I2C_SCL_LOW()  // clock out

// I2C init function
void I2C_init(void) {
  DDRA  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTA &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i = 8; i; i--, data<<=1) {  // transmit 8 bits, MSB first
    (data & 0x80) ? (I2C_SDA_HIGH()) : (I2C_SDA_LOW());  // SDA HIGH if bit is 1
    I2C_CLOCKOUT();                       // clock out -> slave reads the bit
  }
  I2C_DELAY();                            // delay 3 clock cycles
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_CLOCKOUT();                         // 9th clock pulse is for the ignored ACK bit
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78    // OLED write address
#define OLED_CMD_MODE   0x00    // set command mode
#define OLED_DAT_MODE   0x40    // set data mode
#define OLED_INIT_LEN   11      // 9: no screen flip, 11: screen flip

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xA8, 0x1F,                   // set multiplex for 128x32
  0x20, 0x01,                   // set vertical memory addressing mode
  0xDA, 0x02,                   // set COM pins hardware configuration to sequential
  0x8D, 0x14,                   // enable charge pump
  0xAF,                         // switch on OLED
  0xA1, 0xC8                    // flip the screen
};

// OLED 5x8 font
const uint8_t OLED_FONT_SMALL[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7f, 0x14, 0x7f, 0x14, 0x24, 0x2a, 0x7f, 0x2a, 0x12, 0x62, 0x64, 0x08, 0x13, 0x23,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1c, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43
};

// OLED 5x16 font
const uint8_t OLED_FONT_BIG[] PROGMEM = {
  0x7C, 0x1F, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x7C, 0x1F, // 0  0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x1F, // 1  1
  0x00, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x00, // 2  2
  0x00, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 3  3
  0x7C, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7C, 0x1F, // 4  4
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 5  5
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 6  6
  0x7C, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x7C, 0x1F, // 7  7
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 8  8
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 9  9
  0x7C, 0x1F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x1F, // A 10
  0x7C, 0x1F, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x00, 0x1F, // b 11
  0x7C, 0x1F, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x00, 0x00, // C 12
  0x00, 0x1F, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x7C, 0x1F, // d 13
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x00, // E 14
  0x7C, 0x1F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x00, 0x00, // F 15
  0xFE, 0x3F, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0xFE, 0x3F, // H 16
  0xFE, 0x07, 0x00, 0x18, 0x00, 0x20, 0x00, 0x18, 0xFE, 0x07, // V 17
  0xFE, 0x3F, 0x00, 0x04, 0x00, 0x0A, 0x00, 0x11, 0x80, 0x20, // k 18
  0x80, 0x31, 0x00, 0x0A, 0x00, 0x04, 0x00, 0x0A, 0x80, 0x31, // x 19
  0x80, 0x30, 0x80, 0x28, 0x80, 0x24, 0x80, 0x22, 0x80, 0x21, // z 20
  0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, // . 21
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //   22
};

// OLED character definitions
#define DECIMAL 21
#define SPACE   22

// OLED current page
uint8_t OLED_page;

// OLED init function
void OLED_init(void) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                 // set command mode
  for(uint8_t i=0; i<OLED_INIT_LEN; i++) 
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                               // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                 // set command mode
  I2C_write(0x22);                          // command for min/max page
  I2C_write(ypos);                          // min: ypos
  (ypos > 1) ? I2C_write(3) : I2C_write(ypos); // max: depending
  I2C_write(xpos & 0x0F);                   // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));            // set high nibble of start column
  I2C_write(0xB0 | (ypos));                 // set start page
  I2C_stop();                               // stop transmission
  OLED_page = ypos;
}

// OLED clear screen
void OLED_clearScreen(void) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                 // set command mode
  I2C_write(0x22);                          // command for min/max page
  I2C_write(0x00);                          // min ypos=0
  I2C_write(0x03);                          // max ypos=3
  I2C_write(0x0F);                          // start column=0 (low nibble)
  I2C_write(0x10);                          // start column=0 (high nibble)
  I2C_write(0xB0);                          // start page=0
  I2C_stop();                               // stop transmission
  
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  uint8_t i=0;                              // clear screen ...
  do {
    I2C_write(0x00);
    I2C_write(0x00);
  } while(--i);
  I2C_stop();                               // stop transmission
}

// OLED plot a character
void OLED_plotChar(uint8_t ch) {
  if(OLED_page > 1) {                       // big character ?
    ch = (ch << 3) + (ch << 1);             // calculate position of character in font array
    I2C_write(0x00); I2C_write(0x00);       // print spacing between characters
    for(uint8_t i=10; i; i--)               // 10 bytes per character
      I2C_write(pgm_read_byte(&OLED_FONT_BIG[ch++])); // print character
  }
  else {                                    // small character ?
    uint16_t p = ch - 32;
    p += (p << 2);                          // calculate position of character in font array
    I2C_write(0x00);                        // print spacing between characters
    for(uint8_t i=5; i; i--)                // 5 bytes per character
      I2C_write(pgm_read_byte(&OLED_FONT_SMALL[p++])); // print character
  }
}

// OLED print a string from program memory
void OLED_printPrg(const char* p) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  while(char ch = pgm_read_byte(p++))       // repeat until string terminator
    OLED_plotChar(ch);                      // print character on OLED
  I2C_stop();                               // stop transmission
}

// OLED print a string
void OLED_printStr(const char* p) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  while(*p) OLED_plotChar(*p++);            // print the characters
  I2C_stop();                               // stop transmission
}

// OLED BCD conversion array
const uint16_t DIVIDER[] PROGMEM = {10000, 1000, 100, 10, 1};

// OLED print 16-bit value as 5-digit decimal (BCD conversion by substraction method)
void OLED_printDec(uint16_t value) {
  uint8_t leadflag = 0;                     // flag for leading spaces
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  for(uint8_t digit = 0; digit < 5; digit++) {  // 5 digits
    uint8_t digitval = 0;                   // start with digit value 0
    uint16_t divider = pgm_read_word(&DIVIDER[digit]);
    while(value >= divider) {               // if current divider fits into the value
      leadflag = 1;                         // end of leading spaces
      digitval++;                           // increase digit value
      value -= divider;                     // decrease value by divider
    }
    if(leadflag || (digit == 4)) OLED_plotChar(digitval); // print the digit
    else OLED_plotChar(SPACE);              // or print leading space
  }
  I2C_stop();                               // stop transmission
}

// OLED print byte as hex
void OLED_printHex(uint8_t value) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  OLED_plotChar(0); OLED_plotChar(19);      // print "0x"
  OLED_plotChar(value >> 4);                // print high nibble
  OLED_plotChar(value & 0x0F);              // print low nibble
  I2C_stop();                               // stop transmission
}

// OLED print Vcc
void OLED_printVCC(uint8_t value) {
  I2C_start(OLED_ADDR);                     // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                 // set data mode
  uint8_t digitval = 0;                     // start with digit value 0
  while(value >= 10) {                      // if current divider fits into the value
    digitval++;                             // increase digit value
    value -= 10;                            // decrease value by divider
  }
  OLED_plotChar(digitval);                  // print first digit
  OLED_plotChar(DECIMAL);                   // print decimal
  OLED_plotChar(value);                     // print second digit
  I2C_stop();                               // stop transmission
}

// ===================================================================================
// Target Chip Definitions
// ===================================================================================

// Definition of target data type
typedef struct {
  uint16_t  SIG;              // signature
  uint8_t   LFUSE;            // low fuse
  uint8_t   HFUSE;            // high fuse
  uint8_t   EFUSE;            // extended fuse
  uint8_t   PSIZE;            // page size of flash
  uint8_t   PROG_LENGTH;      // length in bytes of calibration program
  const uint8_t* PROG;        // pointer to calibration program
  uint16_t  FREQ;             // target frequency in kHz
  char      NAME[11];         // name of target MCU
} TGT_TYPE;

// Global TGT variable
TGT_TYPE TGT;

// Program for ATtiny13
const uint8_t PROG_T13[] PROGMEM = {
  0x09, 0xC0, 0x0E, 0xC0, 0x0D, 0xC0, 0x0C, 0xC0, 0x0B, 0xC0, 0x0A, 0xC0, 0x09, 0xC0, 0x08, 0xC0,
  0x07, 0xC0, 0x06, 0xC0, 0x11, 0x24, 0x1F, 0xBE, 0xCF, 0xE9, 0xCD, 0xBF, 0x02, 0xD0, 0x12, 0xC0,
  0xEF, 0xCF, 0x80, 0xE0, 0x90, 0xE0, 0x07, 0xD0, 0x81, 0xBF, 0x81, 0xE0, 0x87, 0xBB, 0x92, 0xE4,
  0x9F, 0xBD, 0x83, 0xBF, 0xFF, 0xCF, 0xE1, 0x99, 0xFE, 0xCF, 0x8E, 0xBB, 0xE0, 0x9A, 0x99, 0x27,
  0x8D, 0xB3, 0x08, 0x95, 0xF8, 0x94, 0xFF, 0xCF
};

// Program for ATtiny25/45/85
const uint8_t PROG_Tx5[] PROGMEM = {
  0x0E, 0xC0, 0x13, 0xC0, 0x12, 0xC0, 0x11, 0xC0, 0x10, 0xC0, 0x0F, 0xC0, 0x0E, 0xC0, 0x0D, 0xC0,
  0x0C, 0xC0, 0x0B, 0xC0, 0x0A, 0xC0, 0x09, 0xC0, 0x08, 0xC0, 0x07, 0xC0, 0x06, 0xC0, 0x11, 0x24,
  0x1F, 0xBE, 0xCF, 0xED, 0xCD, 0xBF, 0x02, 0xD0, 0x13, 0xC0, 0xEA, 0xCF, 0x80, 0xE0, 0x90, 0xE0,
  0x07, 0xD0, 0x81, 0xBF, 0x81, 0xE0, 0x87, 0xBB, 0x92, 0xE4, 0x9A, 0xBD, 0x83, 0xBF, 0xFF, 0xCF,
  0xE1, 0x99, 0xFE, 0xCF, 0x1F, 0xBA, 0x8E, 0xBB, 0xE0, 0x9A, 0x99, 0x27, 0x8D, 0xB3, 0x08, 0x95,
  0xF8, 0x94, 0xFF, 0xCF
};

// Program for ATtiny24
const uint8_t PROG_T24[] PROGMEM = {
  0x10, 0xC0, 0x15, 0xC0, 0x14, 0xC0, 0x13, 0xC0, 0x12, 0xC0, 0x11, 0xC0, 0x10, 0xC0, 0x0F, 0xC0,
  0x0E, 0xC0, 0x0D, 0xC0, 0x0C, 0xC0, 0x0B, 0xC0, 0x0A, 0xC0, 0x09, 0xC0, 0x08, 0xC0, 0x07, 0xC0,
  0x06, 0xC0, 0x11, 0x24, 0x1F, 0xBE, 0xCF, 0xED, 0xCD, 0xBF, 0x02, 0xD0, 0x13, 0xC0, 0xE8, 0xCF,
  0x80, 0xE0, 0x90, 0xE0, 0x07, 0xD0, 0x81, 0xBF, 0x80, 0xE4, 0x8A, 0xBB, 0x8F, 0xBD, 0x89, 0xE0,
  0x8E, 0xBD, 0xFF, 0xCF, 0xE1, 0x99, 0xFE, 0xCF, 0x1F, 0xBA, 0x8E, 0xBB, 0xE0, 0x9A, 0x99, 0x27,
  0x8D, 0xB3, 0x08, 0x95, 0xF8, 0x94, 0xFF, 0xCF
};

// Program for ATtiny44/84
const uint8_t PROG_T84[] PROGMEM = {
  0x10, 0xC0, 0x17, 0xC0, 0x16, 0xC0, 0x15, 0xC0, 0x14, 0xC0, 0x13, 0xC0, 0x12, 0xC0, 0x11, 0xC0,
  0x10, 0xC0, 0x0F, 0xC0, 0x0E, 0xC0, 0x0D, 0xC0, 0x0C, 0xC0, 0x0B, 0xC0, 0x0A, 0xC0, 0x09, 0xC0,
  0x08, 0xC0, 0x11, 0x24, 0x1F, 0xBE, 0xCF, 0xE5, 0xD2, 0xE0, 0xDE, 0xBF, 0xCD, 0xBF, 0x02, 0xD0,
  0x13, 0xC0, 0xE6, 0xCF, 0x80, 0xE0, 0x90, 0xE0, 0x07, 0xD0, 0x81, 0xBF, 0x80, 0xE4, 0x8A, 0xBB,
  0x8F, 0xBD, 0x89, 0xE0, 0x8E, 0xBD, 0xFF, 0xCF, 0xE1, 0x99, 0xFE, 0xCF, 0x9F, 0xBB, 0x8E, 0xBB,
  0xE0, 0x9A, 0x99, 0x27, 0x8D, 0xB3, 0x08, 0x95, 0xF8, 0x94, 0xFF, 0xCF
};

// Program for ATmega8
const uint8_t PROG_M8[] PROGMEM = {
  0x12, 0xC0, 0x19, 0xC0, 0x18, 0xC0, 0x17, 0xC0, 0x16, 0xC0, 0x15, 0xC0, 0x14, 0xC0, 0x13, 0xC0,
  0x12, 0xC0, 0x11, 0xC0, 0x10, 0xC0, 0x0F, 0xC0, 0x0E, 0xC0, 0x0D, 0xC0, 0x0C, 0xC0, 0x0B, 0xC0,
  0x0A, 0xC0, 0x09, 0xC0, 0x08, 0xC0, 0x11, 0x24, 0x1F, 0xBE, 0xCF, 0xE5, 0xD4, 0xE0, 0xDE, 0xBF,
  0xCD, 0xBF, 0x02, 0xD0, 0x12, 0xC0, 0xE4, 0xCF, 0x80, 0xE0, 0x90, 0xE0, 0x06, 0xD0, 0x81, 0xBF,
  0x88, 0xE0, 0x87, 0xBB, 0x8A, 0xE1, 0x85, 0xBD, 0xFF, 0xCF, 0xE1, 0x99, 0xFE, 0xCF, 0x9F, 0xBB,
  0x8E, 0xBB, 0xE0, 0x9A, 0x99, 0x27, 0x8D, 0xB3, 0x08, 0x95, 0xF8, 0x94, 0xFF, 0xCF
};

// Target data
const TGT_TYPE TGTs[] PROGMEM = {
  // SIGNATURE  LFUSE HFUSE EFUSE PSIZE PLENGTH PROG      FREQ  NAME 
  {   0x9007,   0x6A, 0xFF, 0xFF, 16,   72,     PROG_T13, 9600, "ATTINY13  "  },
  {   0x9108,   0x62, 0xDF, 0xFF, 16,   84,     PROG_Tx5, 8000, "ATTINY25  "  },
  {   0x9206,   0x62, 0xDF, 0xFF, 32,   84,     PROG_Tx5, 8000, "ATTINY45  "  },
  {   0x930B,   0x62, 0xDF, 0xFF, 32,   84,     PROG_Tx5, 8000, "ATTINY85  "  },
  {   0x910B,   0x62, 0xDF, 0xFF, 16,   88,     PROG_T24, 8000, "ATTINY24  "  },
  {   0x9207,   0x62, 0xDF, 0xFF, 32,   92,     PROG_T84, 8000, "ATTINY44  "  },
  {   0x930C,   0x62, 0xDF, 0xFF, 32,   92,     PROG_T84, 8000, "ATTINY84  "  },
  {   0x9307,   0x64, 0xDF, 0xFF, 32,   94,     PROG_M8,  8000, "ATMEGA8   "  }
};

// Number of target definitions
#define TGT_LENGTH  8

// ===================================================================================
// In-Circuit Serial Programmer Implementation
// ===================================================================================

// ICSP macros
#define ICSP_RST_HIGH()     PORTA |=  (1<<RST_PIN)
#define ICSP_RST_LOW()      PORTA &= ~(1<<RST_PIN)
#define ICSP_MOSI_HIGH()    PORTA |=  (1<<MOSI_PIN)
#define ICSP_MOSI_LOW()     PORTA &= ~(1<<MOSI_PIN)
#define ICSP_SCK_HIGH()     PORTA |=  (1<<SCK_PIN)
#define ICSP_SCK_LOW()      PORTA &= ~(1<<SCK_PIN)
#define ICSP_MISO_BIT       (PINA &   (1<<MISO_PIN))
#define ICSP_DELAY()        _delay_us(2)
#define ICSP_WAIT_WRITE()   _delay_ms(5)
#define ICSP_WAIT_ERASE()   _delay_ms(10)

// ICSP set up all control lines
void ICSP_init(void) {
  PORTA &= ~((1<<RST_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN));
  DDRA  |=   (1<<RST_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN);
  DDRA  &=  ~(1<<MISO_PIN);
}

// ICSP release all control lines (input, no pullup)
void ICSP_release(void) {
  PORTA &= ~((1<<RST_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN));
  DDRA  &= ~((1<<RST_PIN) | (1<<SCK_PIN) | (1<<MOSI_PIN));
}

// ICSP send instruction byte and receive reply (SPI bit-banging)
uint8_t ICSP_sendByte(uint8_t byte) {
  for(uint8_t i=8; i; i--) {
    (byte & 0x80) ? (ICSP_MOSI_HIGH()) : (ICSP_MOSI_LOW());
    byte<<=1;
    ICSP_DELAY();
    ICSP_SCK_HIGH();
    ICSP_DELAY();
    if(ICSP_MISO_BIT) byte |= 1;
    ICSP_SCK_LOW();
  }        
  return byte;
}

// ICSP enter the programming mode
uint8_t ICSP_enterProgMode(void) {
  ICSP_init();
  _delay_ms(20);
  ICSP_sendByte(0xAC);
  ICSP_sendByte(0x53);
  uint8_t echo = ICSP_sendByte(0x00);
  ICSP_sendByte(0x00);
  return(echo == 0x53);
}

// ICSP exit the programming mode
void ICSP_exitProgMode(void) {
  ICSP_release();
  _delay_ms(200);
}

// ICSP send complete instruction (4 bytes)
void ICSP_sendInstr(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
  ICSP_sendByte(byte1);
  ICSP_sendByte(byte2);
  ICSP_sendByte(byte3);
  ICSP_sendByte(byte4);
}

// ICSP send instruction and read reply
uint8_t ICSP_readInstr(uint8_t byte1, uint8_t byte2, uint8_t byte3) {
  ICSP_sendByte(byte1);
  ICSP_sendByte(byte2);
  ICSP_sendByte(byte3);
  return(ICSP_sendByte(0x00));
}
  
// ICSP read signature of target device
uint16_t ICSP_readSignature(void) {
  return(((uint16_t)ICSP_readInstr(0x30, 0x00, 0x01) << 8) | ICSP_readInstr(0x30, 0x00, 0x02));
}

// ICSP read calibration byte
uint8_t ICSP_readCalib(void) {
  return(ICSP_readInstr(0x38, 0x00, 0x00));
}

// ICSP write fuse settings to target device
void ICSP_writeFuses(void) {
  ICSP_sendInstr(0xAC, 0xA0, 0x00, TGT.LFUSE); ICSP_WAIT_WRITE();
  ICSP_sendInstr(0xAC, 0xA8, 0x00, TGT.HFUSE); ICSP_WAIT_WRITE();
  ICSP_sendInstr(0xAC, 0xA4, 0x00, TGT.EFUSE); ICSP_WAIT_WRITE();
}

// ICSP perform chip erase
void ICSP_eraseChip(void) {
  ICSP_sendInstr(0xAC, 0x80, 0x00, 0x00);
  ICSP_WAIT_ERASE();
}

// ICSP write to EEPROM
void ICSP_writeEEPROM(uint8_t addr, uint8_t data) {
  ICSP_sendInstr(0xC0, addr >> 8, addr, data);
  ICSP_WAIT_WRITE();
}

// ICSP write data from program memory to flash of target
void ICSP_writeFlash(void) {
  uint16_t addr     = 0;
  uint8_t  pagesize = TGT.PSIZE;
  uint16_t length   = (TGT.PROG_LENGTH + 1) >> 1;
  const uint8_t* p  = TGT.PROG;
  
  do {
    ICSP_sendInstr(0x40, addr >> 8, addr, pgm_read_byte(p++));
    ICSP_sendInstr(0x48, addr >> 8, addr, pgm_read_byte(p++));
    if((++addr & (pagesize - 1)) == 0) {
      ICSP_sendInstr(0x4C, (addr-pagesize) >> 8, addr-pagesize, 0x00);
      ICSP_WAIT_WRITE();
    }
  } while(--length);

  if(addr & (pagesize - 1)) {
    ICSP_sendInstr(0x4C, addr >> 8, addr, 0x00);
    ICSP_WAIT_WRITE();
  }
}

// ===================================================================================
// Frequency Measurement Implementation
// ===================================================================================

// Target power macros
#define FRQ_TOP   (F_CPU / 250) - 1 // 250 = 8 [prescaler] * (1000 / 32) [1/32ms]

// Global variables for frequency measurement
volatile uint8_t FRQ_highByte;      // high byte of virtual 16-bit timer0
volatile uint8_t FRQ_busy;          // sampling in progress flag

// Init frequency measurement
void FRQ_init(void) {
  OCR1A  = FRQ_TOP;                 // timer1 compare match A after 32 ms
  TIMSK0 = (1<<TOIE0);              // enable timer0 overflow interrupt
  TIMSK1 = (1<<OCIE1A);             // enable timer1 compare match A interrupt
  sei();                            // enable global interrupts
}

// Measure frequency by counting signals from target (OSC / 16) for 32ms
uint16_t FRQ_measure(void) {
  FRQ_busy = 1;                     // sampling flag; "0" when completed
  FRQ_highByte = 0;                 // high byte for virtual 16-bit counter
  TCNT0  = 0;                       // clear timer0
  TCNT1  = 0;                       // clear timer1
  TCCR1B = (1<<CS11);               // start timer1, prescaler 8
  TCCR0B = (1<<CS02) | (1<<CS01);   // start counting signals from target
  while(FRQ_busy);                  // wait for sampling complete
  uint16_t freq = (uint16_t)(FRQ_highByte << 8) | TCNT0; 
  freq >>= 1;                       // calculate frequency in kHz
  return(freq);
}

// Timer0 overflow interrupt service routine (expand timer0 to 16-bit)
ISR(TIM0_OVF_vect) {
  FRQ_highByte++;                   // increase timer high byte
}

// Timer1 compare match A interrupt service routine (fires after 32ms);
ISR(TIM1_COMPA_vect) {
  TCCR0B   = 0;                     // stop counting signals
  TCCR1B   = 0;                     // stop timer1
  FRQ_busy = 0;                     // sampling complete
}

// ===================================================================================
// ADC Implementation for Supply Voltage Measurement
// ===================================================================================

// Setup ADC
void ADC_init(void) {
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // enable ADC with prescaler 64
  ADMUX  = (1<<MUX5) | (1<<MUX0);               // set 1.1V reference against Vcc
}

// Read supply voltage in dV
uint8_t ADC_readVCC(void) {
  ADCSRA |= (1<<ADSC);              // start conversion
  while(ADCSRA & (1<<ADSC));        // wait for ADC conversion complete
  uint16_t vcc = ADC;               // get result
  vcc = 11253 / vcc;                // calculate Vcc in dV; 11253 = 1.1*1023*10
  return vcc;                       // return VCC in dV
}

// ===================================================================================
// Button Functions
// ===================================================================================

// Button init
void BUTTON_init(void) {
  PORTA |= (1<<BUTTON_PIN);         // pullup on button pin
}

// Wait for button pressed
void BUTTON_wait(void) {
  while(~PINA & (1<<BUTTON_PIN));   // wait for button released
  _delay_ms(10);                    // debounce
  while( PINA & (1<<BUTTON_PIN));   // wait for button pressed
}

// ===================================================================================
// Main Function
// ===================================================================================

// Some "strings"
const char SEP1[] PROGMEM = { 17, 22, 22,  0 };  // "V  "
const char SEP2[] PROGMEM = { 22, 22, 22,  0 };  // "   "
const char SEP3[] PROGMEM = { 18, 16, 20,  0 };  // "kHz"

// Calculate difference
uint16_t diff(uint16_t a, uint16_t b) {
  if(a > b) return(a - b);
  return(b - a);
}

// Main Function
int main(void) {
  // Local variables
  uint8_t  target;
  uint8_t  calib;
  uint16_t signature;
  uint16_t frequency;
  
  // Setup
  BUTTON_init();
  ADC_init();
  FRQ_init();
  I2C_init();
  OLED_init();
  OLED_clearScreen();
  
  // Loop
  while(1) {
    // Print start screen and wait for button
    OLED_setCursor(0,0); OLED_printPrg(PSTR("PRESS BUTTON TO START"));
    OLED_setCursor(0,1); OLED_printPrg(PSTR("  OSC CALIBRATION !  "));
    BUTTON_wait();
    
    // Read signature und identify target MCU
    ICSP_enterProgMode();
    signature = ICSP_readSignature();
    for(target=0; target<TGT_LENGTH; target++) {
      if(signature == pgm_read_word(&TGTs[target])) break;
    }
    
    // Print error message if negative detection of MCU
    if(target == TGT_LENGTH) {
      ICSP_exitProgMode();
      OLED_setCursor(0,0); OLED_printPrg(PSTR(" ERROR, NO SUPPORTED "));
      OLED_setCursor(0,1); OLED_printPrg(PSTR("  DEVICE DETECTED !  "));
      BUTTON_wait();
      continue;
    }
    
    // Copy target MCU data from flash to TGT variable 
    memcpy_P (&TGT, &TGTs[target], sizeof TGT);
    
    // Print message on OLED
    OLED_setCursor(0,0); OLED_printPrg(PSTR(" CALIBRATING OSC OF  "));
    OLED_setCursor(0,1); OLED_printPrg(PSTR("    "));
                         OLED_printStr(TGT.NAME);
                         OLED_printPrg(PSTR("...    "));
    
    // Write calibration program to flash and OSCCAL value to EEPROM of target MCU
    calib = ICSP_readCalib();
    ICSP_eraseChip();
    ICSP_writeFuses();
    ICSP_writeFlash();
    ICSP_writeEEPROM(0, calib);
    ICSP_exitProgMode();
    
    // Calibration process variables
    uint8_t  lastcalib  = calib;
    uint16_t difference = 65535;
    
    // Calibration process
    while(1) {
      // Measure frequency
      frequency = FRQ_measure();
      
      // Leave if current OSCCAL value is worse than the last one
      if(diff(frequency, TGT.FREQ) >= difference) break;

      // Print current calibration results
      OLED_setCursor(0,2);
      OLED_printVCC(ADC_readVCC()); OLED_printPrg(SEP1);
      OLED_printHex(calib);         OLED_printPrg(SEP2);
      OLED_printDec(frequency);     OLED_printPrg(SEP3);

      // Calculate next OSCCAL value
      lastcalib = calib;
      difference = diff(frequency, TGT.FREQ);
      if(difference == 0) break;
      if(frequency > TGT.FREQ) calib--;
      else calib++;

      // Program target for next calibration value
      ICSP_enterProgMode();
      ICSP_writeEEPROM(0, calib);
      ICSP_exitProgMode();
    }   
    
    // Write calibration result und wait for button
    ICSP_enterProgMode();
    ICSP_writeEEPROM(0, lastcalib);
    ICSP_exitProgMode();
    OLED_setCursor(0,0); OLED_printPrg(PSTR("CALIBRATION COMPLETE!"));
    OLED_setCursor(0,1); OLED_printPrg(PSTR("VCC  OSCCAL FREQUENCY"));
    BUTTON_wait();
  }
}
