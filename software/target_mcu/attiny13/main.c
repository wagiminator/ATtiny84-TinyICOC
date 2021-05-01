#include <avr/io.h>
#include <avr/eeprom.h>

int main(void) {
  OSCCAL = eeprom_read_byte(0);   // Set OSCCAL from EEPROM
  DDRB   = 0b00000001;            // Set PB0 as output
  TCCR0A = 0b01000010;            // CTC mode, toggle PB0 on compare match
  TCCR0B = 0b00000001;            // Start timer, no prescaler
  while(1);                       // That's all
}
