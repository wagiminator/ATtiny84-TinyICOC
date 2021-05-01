#include <avr/io.h>
#include <avr/eeprom.h>

int main(void) {
  OSCCAL = eeprom_read_byte(0);   // Set OSCCAL from EEPROM
  DDRA   = 0b01000000;            // Set PA6 as output
  TCCR1A = 0b01000000;            // Toggle PA6 on compare match
  TCCR1B = 0b00001001;            // Start timer CTC mode, no prescaler
  while(1);                       // That's all
}
