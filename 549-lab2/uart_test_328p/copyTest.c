
/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>

#define BAUD 9600
#include <util/setbaud.h>

#include <stdbool.h>			/* Added to use bools for PWM control */
#define ADC_PIN	0

void uart_init(void) {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
	loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
	UDR0 = c;
}

char uart_getchar(void) {
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

/*
Steps to output PWM signal
1. Read sensor value
2. Write 


*/

uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}

int main(void)
{
	/* Setup serial port */
	uart_init();
	stdout = &uart_output;
	stdin  = &uart_input;

	/* Enable the ADC */
        ADCSRA |= _BV(ADEN);

	char input;
	
	// Setup ports
	
	DDRB |= (1<<1) | (1<<0);
	PORTB |= (1<<0);
	PORTB &= ~(1<<1);
	

	/* Output enable PD5 */
	DDRD |= (1<<5);

	TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	TCCR0B |= _BV(CS00);
	
	char pwm = 0x00;
	bool up = true;

	/* Print hello and then echo serial
	 ** port data while blinking LED */
	printf("Hello world!\r\n");
	while(1) {
		
		input = getchar();
		printf("You wrote %c\r\n", input);
		PORTB ^= 0x01;
		

		/* Read from the sensor */
		pwm = (char)(adc_read(ADC_PIN) & 0xFF);		


		/* Control the actuator by writing to PWM */
		OCR0B = pwm;
		pwm += up ? 1 : -1;
		if (pwm == 0xff)
			up = false;
		else if (pwm == 0x00)
			up = true;
		printf("%x\n",pwm);	
		_delay_ms(10);
	
	}
}
