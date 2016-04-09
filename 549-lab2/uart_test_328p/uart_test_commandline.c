
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


int main(void)
{

   /* Setup serial port */
   uart_init();
   stdout = &uart_output;
   stdin  = &uart_input;

   char input;

	/* Setup for our PWM output */
	/* Output enable PD5 */
	DDRD |= (1 << DDD5);

	/* PWM SETUP */
	TCCR0A |= (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
	TCCR0B |= (1<<CS00);
	
	/* Variables used to write to PWM pin */
	OCR0B = 0;


   /* Print hello and then echo serial
   ** port data while blinking LED */
   printf("Enter a value between 0 and 9! (0 = very slow, 9 = very fast/r/n");
   while(1) {
	input = getchar();
	switch (input) {
case '0': 
OCR0B = 100;
break;
case '1': OCR0B = 110;
break;
case '2': OCR0B = 120;
break;
case '3':OCR0B = 130;
break;
case '4':OCR0B = 140;
break;
case '5':OCR0B = 150;
break;
case '6':OCR0B = 160;
break;
case '7':OCR0B = 170;
break;
case '8':OCR0B = 180;
break;
case '9':OCR0B = 190;
break;
default:
break;
   }

}
}
