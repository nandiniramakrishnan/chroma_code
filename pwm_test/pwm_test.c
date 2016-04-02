/*
   TRY PHASE CORRECT PWM...!!!!

 */


#include <avr/io.h>

void pwm_init() {
  /* Setup for our PWM output */
  /* Output enable PD5 */
  DDRD |= (1 << DDD5);
  TCCR0A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR0B = _BV(CS22);
  // OCR0A = 180;
  OCR0B = 180;
  /* PWM SETUP */
  // TCCR0A |= (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
  // TCCR0B |= (1<<CS00);

  /* Variables used to write to PWM pin */
  // OCR0B = 0;

}

int main() {
  pwm_init();
  while (1) {
  }
  return 0;
}
