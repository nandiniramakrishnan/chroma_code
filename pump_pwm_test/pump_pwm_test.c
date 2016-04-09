#include <avr/io.h>

void pump_init() {
  DDRD |= (1 << DDD2);
}

void pwm_init() {
  /* Setup for our PWM output */
  /* Output enable PD5 */
  DDRD |= (1 << DDD5);
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM20);
  TCCR0B = _BV(CS02);
  OCR0B = 180;
}

int main() {
  pump_init();
  pwm_init();
  while (1) {
  }
  return 0;
}
