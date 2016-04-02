#include <avr/io.h>
#include <util/delay.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void USART0Init(void)
{
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
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
  uint8_t colorVal = UDR0;
  uint8_t i = 0;
  if (colorVal > 0) {
    OCR0B = 0x7F;
  }
  while (i < colorVal) {
    _delay_ms(10);
    i++;
  }
  OCR0B = 0;
  return UDR0;
}

void pwm_init() {
  /* Setup for our PWM output */
  /* Output enable PD5 */
  DDRD |= (1 << DDD5);

  /* PWM SETUP */
  TCCR0A |= (1<<COM0B1) | (1<<WGM00) | (1<<WGM01);
  TCCR0B |= (1<<CS00);
  
  /* Variables used to write to PWM pin */
  OCR0B = 0x7F;
}

int main(void) {
  USART0Init();
  pwm_init();
  uint8_t data;
  while (1) {
    data = USART0ReceiveByte();
  }
  return 0;
}
