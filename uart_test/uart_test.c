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
#define USART_BAUDRATE 9600
#define UBRR_VAL (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

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

void USART0Init(void)
{
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VAL>>8);
  UBRR0L = (uint8_t)UBRR_VAL;
  // Set frame format to 8 data bits, no parity, 1 stop bit
  UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
  //enable transmission and reception
  UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}

void USART0SendByte(uint8_t u8Data)
{
  //wait while previous byte is completed
  while(!(UCSR0A&(1<<UDRE0))){};
  // Transmit data
  UDR0 = u8Data;
}

uint8_t USART0ReceiveByte()
{
  // Wait for byte to be received
  while(!(UCSR0A&(1<<RXC0))){};
  // Return received data
  return UDR0;
}

int main (void)
{
  uint8_t u8TempData = 0x01;
  //Initialize USART0
  USART0Init();
  USART0SendByte(u8TempData);
  while(1)
  {
    // Receive data
    u8TempData = USART0ReceiveByte();
    // Increment received data
    printf("%d\n", u8TempData);
     u8TempData++;
    //Send back to terminal
    USART0SendByte(u8TempData);
  }
}
