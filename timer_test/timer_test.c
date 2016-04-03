#include <avr/io.h>
#include <avr/interrupt.h>

//Init PD5 and PD6 pins as output

void InitPort(void)
{//initial value 0
  PORTD&=~((1<<PD6)|(1<<PD5));
  //set PD6 as output
  DDRD|=(1<<PD6)|(1<<PD5);
  //set PB0 pin as input
  DDRB&=~(1<<PB0);
  //set internal pullup
  PORTB|=(1<<PB0);
}

//Initialize Timer0
void InitTimer0(void)
{
  //Set Initial Timer value
  TCNT0=0;
  //Place TOP timer values to Output compare registers
  OCR0A=249;
  OCR0B=150;
  //Set CTC mode
  //and make toggle PD6/OC0A and PD5/OC0b pins on compare match
  TCCR0A|=(1<<COM0A0)|(1<<COM0B0)|(1<<WGM01);
  //Enable Timer0 OCF0B Interrupt
  TIMSK0|=(1<<OCIE0B);
}

void StartTimer0(void) {
  //Set prescaller 64 and start timer
  TCCR0B|=(1<<CS01)|(1<<CS00);
  //enable global interrupts
  sei();
}

//
ISR(TIMER0_COMPB_vect) {
  //if button is pressed
  if (!(PINB)&(1<<PB0))
  {
    //shift phase
    if (OCR0B<OCR0A)
      OCR0B++;
    else
      OCR0B=0;
  }
}

int main(void) {
  InitPort();
  InitTimer0();
  StartTimer0();
  while(1)
  {
    //doing nothing
  }
}
