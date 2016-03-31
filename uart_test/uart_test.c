#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//define max buffer size
#define BUF_SIZE 20

//type definition of buffer structure
typedef struct{
  //Array of chars
  uint8_t buffer[BUF_SIZE];
  //array element index
  uint8_t index;
  uint8_t colourIndex;
}u8buf;

//declare buffer
u8buf buf;

typedef struct {
  uint8_t cyan;
  uint8_t magenta;
  uint8_t yellow;
} coloursBuf;

coloursBuf colours;


//initialize buffer
void BufferInit(u8buf *buf)
{
  //set index to start of buffer
  buf->index=0;
}

//write to buffer routine
void BufferWrite(u8buf *buf, uint8_t u8data) {
  if (buf->index<BUF_SIZE) {
    buf->buffer[buf->index] = u8data;
    buf->index++;
  }
}

uint8_t BufferRead(u8buf *buf, volatile uint8_t *u8data)
{
  if(buf->index>0)
  {
    buf->index--;
    *u8data=buf->buffer[buf->index];
    return 0;
  }
  else return 1;
}

void getColours(u8buf *buf, coloursBuf *colours) {
  if (buf->index >= 2) {
    colours->yellow = buf->buffer[buf->index];
    colours->magenta = buf->buffer[buf->index - 1];
    colours->cyan = buf->buffer[buf->index - 2]; 
  }
}

void USART0Init(void)
{
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  // Set frame format to 8 data bits, no parity, 1 stop bit
  UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
  //enable reception and RC complete interrupt
  UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  //enable transmission and UDR0 empty interrupt
  UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);
}

//RX Complete interrupt service routine
ISR(USART_RX_vect)
{
  BufferWrite(&buf, UDR0);
  buf.colourIndex++;
  if (buf.colourIndex == 3) {
    getColours(&buf, &colours);
    buf.colourIndex = 0;
  } 
}

//UDR0 Empty interrupt service routine
ISR(USART_UDRE_vect)
{
  //if index is not at start of buffer
  if (BufferRead(&buf, &UDR0)==1)
  {
    //start over
    //reset buffer
    BufferInit(&buf);
    //disable transmission and UDR0 empty interrupt
    UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
    //enable reception and RC complete interrupt
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
  }
}


int main (void)
{
  //Init buffer
  BufferInit(&buf);
  //set sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);
  //Initialize USART0
  USART0Init();
  //enable global interrupts
  sei();
  while(1)
  {
    //put MCU to sleep
    sleep_mode();
  }
}
