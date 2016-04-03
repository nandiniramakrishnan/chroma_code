#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//define max buffer size
#define BUF_SIZE 20

//type definition of buffer structure
typedef struct {
  //Array of chars
  uint8_t buffer[BUF_SIZE];
  //array element index
  uint8_t index;
} u8buf;

//declare buffer
u8buf buf;

//initialize buffer
void buffer_init(u8buf *buf) {
  //set index to start of buffer
  buf->index = 0;
}

int buf_empty(u8buf *buf) {
  if (buf->index == 0) 
    return 1;
  return 0;
}

int buf_full(u8buf *buf) {
  if (buf->index == (BUF_SIZE-1)) 
    return 1;
  return 0;
}

//write to buffer routine
void buffer_write(u8buf *buf, uint8_t u8data) {
  if (!buf_full(buf)) {
    buf->buffer[buf->index] = u8data;
    //increment buffer index
    buf->index++;
  }
}

void buffer_read(u8buf *buf, volatile uint8_t *u8data) {
  if (!buf_empty(buf)) {
    buf->index--;
    *u8data = buf->buffer[buf->index];
//    return buf->buffer[buf->index];
  }
}

void uart_init(void) {
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  // Set frame format to 8 data bits, no parity, 1 stop bit
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
  //enable reception and RC complete interrupt
  UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);
}


//RX Complete interrupt service routine
ISR(USART_RX_vect) {
  //check if period char or end of buffer
  buffer_write(&buf, UDR0);
  if (buf_full(&buf)) {
    //disable reception and RX Complete interrupt
    UCSR0B &= ~((1<<RXEN0) | (1<<RXCIE0));
    //enable transmission and UDR0 empty interrupt
    UCSR0B |= (1<<TXEN0) | (1<<UDRIE0);
  }
}

// UDR0 Empty interrupt service routine
ISR(USART_UDRE_vect) {
  buffer_read(&buf, &UDR0);
  // if index is not at start of buffer
  if (buf_empty(&buf)) {
    // start over
    // reset buffer
    buffer_init(&buf);
    // disable transmission and UDR0 empty interrupt
    UCSR0B &= ~((1<<TXEN0) | (1<<UDRIE0));
    // enable reception and RC complete interrupt
    UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);
  }
}

int main (void) {
  //Init buffer
  buffer_init(&buf);
  //set sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);
  //Initialize USART0
  uart_init();
  //enable global interrupts
  sei();
  while(1) {
    //put MCU to sleep
    sleep_mode();
  }
}
