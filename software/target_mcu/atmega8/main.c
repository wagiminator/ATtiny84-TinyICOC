#include <avr/io.h>
#include <avr/eeprom.h>

int main(void) {
  OSCCAL = eeprom_read_byte(0);   // Set OSCCAL from EEPROM
  DDRB   = 0b00001000;            // Set PB3 as output
  TCCR2  = 0b00011010;            // CTC, prescaler 8, toggle PB3
  while(1);                       // That's all
}
