
/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>

#define BAUD 115200
#include <util/setbaud.h>

void adc_init()
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);
 
    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
  // select the corresponding channel 0~7
  // ANDing with ’7′ will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
 
  // start single convertion
  // write ’1′ to ADSC
  ADCSRA |= (1<<ADSC);
 
  // wait for conversion to complete
  // ADSC becomes ’0′ again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
 
  return (ADC);
}

int main(void)
{
	adc_init();
	
	/* Setup for our PWM output */
	/* Output enable PD5 */
	DDRD |= (1 << DDD5);

	/* PWM SETUP */
	TCCR0A |= (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
	TCCR0B |= (1<<CS00);
	
	/* Variables used to write to PWM pin */
	uint8_t pwm = 0x00;
	OCR0B = 0;
	while(1) {
		
		/* ~~~~~ Original code ~~~~~ */
		/* Read from the sensor */
		pwm = adc_read(0) & 0xFF;	
		
		/* Control the actuator by writing to PWM */
		OCR0B = pwm;
		_delay_ms(10);
	
	}
}
