/*
 * Team 6: Chroma Printer - Instant Makeup
 * Team members: nramakri, jmccarl, jinbingl, umallamp
 * 
 * Description:
 *
 * Pin functions:
 * Fuel injectors (PWM) -  PD6, PB1, PB2
 * Pumps - PD2, PD3, PD4
 * LED - PD5
 * Photo sensor - PC4
 * Motor - PC0, PC1, PC2, PC3
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

/* --------- MACROS -------- */

//define max buffer size
#define BUF_SIZE 20
#define DELAY 3
#define INTERVAL 35

/* --------- GLOBAL VARIABLES ---------- */

// Variables to store received bytes for each colour from UART
volatile uint8_t cyan = 0;
volatile uint8_t magenta = 0;
volatile uint8_t yellow = 0;

// Counter used for each colour in the Timer Overflow Interrupt
volatile unsigned long cyan_overflow_count = 0;
volatile unsigned long magenta_overflow_count = 0;
volatile unsigned long yellow_overflow_count = 0;

// type definition of buffer structure
typedef struct {
  // Array of chars
  uint8_t buffer[BUF_SIZE];
  // Array element index
  volatile uint8_t index;
} u8buf;

// Declare buffer
u8buf buf;

// Global to turn motor on and off
volatile uint8_t motor_on = 0;

volatile uint8_t getColours = 0;

// Scaled colour values used as timer count limits
volatile unsigned long cyanVal = 0;
volatile unsigned long magentaVal = 0;
volatile unsigned long yellowVal = 0;

// Count to keep track of how many colours are done printing
volatile uint8_t mDone = 0;
volatile uint8_t cDone = 0;
volatile uint8_t yDone = 0;

/* --------- METHODS CALLED DURING START UP --------- */

/* pwm_init: Register settings for PWM to actuate the fuel injectors */
void pwm_init() {
  DDRD |= (1 << DDD6);                              // F1
  DDRB |= (1 << DDB1) | (1 << DDB2);                // F2 and F3

  // F1 
  TCCR0A = _BV(COM0A1) | _BV(WGM00);
  TCCR0B = _BV(CS02);

  // F2 and F3 
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS12);

  // Initially the injectors are off. Duty cycle = 0 
  OCR0A = 0;
  OCR1A = 0;
  OCR1B = 0;
}

/* timer_init: Using the timer on OC2B */
void timer_init() {
  // disable timer 2 interrupts
  TIMSK2 = 0;
  // Initialize Timer 0 counter to 0
  TCNT2 = 0;
  // prescale by 1024
  TCCR2B |= (1 << CS20) | (1 << CS22);
  // clear interrupt flags
  TIFR2 = (1 << TOV2);
}

/* motor_init: Set motor pins as output and stop any rotation */
void motor_init() {
  DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
  PORTC &= 0xF0;            
}

/* pump_init: Set pump pins as output */
void pump_init() {
  DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4);
  PORTD &= 0xE3;             // 1110 0011 
}

void adc_init() {
    // AREF = AVcc
    ADMUX = (1<<REFS0);
 
    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

void led_init() {
  DDRD |= (1 << DDD5);
  PORTD |= 0x20;              // 0010 0000
}



/* --------- END START UP METHODS --------- */


/* timer_intt_on: Enable timer 2 overflow interrupt */
void timer_intt_on() {
  TIMSK2 = (1 << TOIE2);           // 0000 0001
}

/* timer_intt_off: Disable timer 2 overflow interrupt */
void timer_intt_off() {
  TIMSK2 &= 0xFE;           // 1111 1110
}

/* Methods to turn each fuel injector on */
void pwm1_on() {
  OCR0A = 127;
}

void pwm2_on() {
  OCR1A = 127;
}

void pwm3_on() {
  OCR1B = 127;
}

void injectors_on() {
  OCR0A = 127;
  OCR1A = 127;
  OCR1B = 127;
}

/* Methods to turn each fuel injector off */
void pwm1_off() {
  OCR0A = 0;
}

void pwm2_off() {
  OCR1A = 0;
}

void pwm3_off() {
  OCR1B = 0;
}

/* Methods to turn each pump on */
void pump1_on() {             // PD2
  PORTD |= 0x40;              // 0000 0100
}

void pump2_on() {             // PD3
  PORTD |= 0x08;              // 0000 1000
}

void pump3_on() {             // PD4
  PORTD |= 0x10;              // 0001 0000
}

/* Methods to turn each pump off */
void pump1_off() {            // PD2     
  PORTD &= 0xFD;              // 1111 1011
}

void pump2_off() {            // PD3
  PORTD &= 0xF7;              // 1111 0111
}

void pump3_off() {            // PD4
  PORTD &= 0xEF;              // 1110 1111
}

void pumps_on() {
  PORTD |= 0x1C;               // 0001 1100
}

/* rotate_motor: Moves the stepper motor */
void rotate_motor() {
  PORTC = 0x01;          //0001
  _delay_ms(DELAY);
  PORTC = 0x04;          //0100
  _delay_ms(DELAY);
  PORTC = 0x02;          //0010
  _delay_ms(DELAY);
  PORTC = 0x08;          //1000
  _delay_ms(DELAY);
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

/* --------- UART Methods --------- */

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

// write to buffer routine
void buffer_write(u8buf *buf, uint8_t u8data) {
  if (buf_full(buf) == 0) {
    buf->buffer[buf->index] = u8data;
    // increment buffer index
    buf->index++;
  }
}

/* Not currently being used. Needed to transmit things to app */
void buffer_read(u8buf *buf, volatile uint8_t *u8data) {
  if (buf_empty(buf) == 0) {
    buf->index--;
    *u8data = buf->buffer[buf->index];
  }
}

void uart_init() {
  buf.index = 0;
  // Set baud rate
  UBRR0H = (uint8_t)(UBRR_VALUE>>8);
  UBRR0L = (uint8_t)UBRR_VALUE;
  // Set frame format to 8 data bits, no parity, 1 stop bit
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);
  // disable transmission
  UCSR0B &= 0xF7;                               // 1111 0111
  //enable reception and RC complete interrupt
  UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);
}

/* --------- END UART --------- */

/* reset_printer: Call at startup and after finishing printing a colour */
void reset_printer() {
  // at this point receiver is still disabled and transmitter is enabled
  // send message done code 0x01
  UDR0 = 0x01;
  // Now, we reset the UART settings - enable receiver and disabled transmitter
  uart_init();
  // turn off timer interrupt
  timer_intt_off();
  // indicator to turn motor on and off in main. Initially its off
  motor_on = 0;
  // startup methods
  pump_init();
  timer_init();
  cyan_overflow_count = 0;
  magenta_overflow_count = 0;
  yellow_overflow_count = 0;
  mDone = 0;
  cDone = 0;
  yDone = 0;
  getColours = 0;
  // turn on transmitter 
}

/* ---------- Interrupt Subroutines --------- */

//RX Complete interrupt service routine
ISR(USART_RX_vect) {
  uint8_t u8temp;
  u8temp = UDR0;
  //check if period char or end of buffer
  buffer_write(&buf, u8temp);
  if (buf_full(&buf) || (u8temp == '.')) {
    getColours = 1;
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
    buf.index = 0;
    // disable transmission and UDR0 empty interrupt
    UCSR0B &= ~((1<<TXEN0) | (1<<UDRIE0));
    // enable reception and RC complete interrupt
    UCSR0B |= (1<<RXEN0) | (1<<RXCIE0);
  }
}

// Timer2 overflow interrupt
ISR(TIMER2_OVF_vect) {
  if (yellow_overflow_count > yellowVal) {
    pump1_off();
    pwm1_off();
    yDone = 1;
  }
  if (cyan_overflow_count > cyanVal) {
    pump2_off();
    pwm2_off();
    cDone = 1;
  }
  if (magenta_overflow_count > magentaVal) {
    pump3_off();
    pwm3_off();
    mDone = 1;
  }
  // all colours are done spraying
  if (mDone && cDone && yDone) {
    reset_printer(); 
  }
  cyan_overflow_count++;
  magenta_overflow_count++;
  yellow_overflow_count++;
}

/* ----- End interrupt subroutines ----- */

/* ----- Main routine. Only operating the motor here ----- */
int main() {
  // start up methods
  motor_init();
  pwm_init();
  pump_init();
  uart_init();
  timer_init();
  adc_init();
  led_init();
  // enable interrupts
  sei();			            
  while (1) {
    if (getColours == 1) {        // Transmitter is on at this point and receiver is disabled
      if (adc_read(4) > 0x7F) {
        // 0x02: transmit message for "Cannot detect substrate"
        UDR0 = 0x02;
      }
      else {
        // store the received colour value
        cyan = buf.buffer[0];
        // scale up the amount of time you need the pump to be on
        cyanVal = cyan*INTERVAL;
        magenta = buf.buffer[1];
        magentaVal = magenta*INTERVAL;
        yellow = buf.buffer[2];
        yellowVal = yellow*INTERVAL;
        injectors_on();
        pumps_on();
        // Enable timer interrupts to check completion of the three colours
        timer_intt_on();
        motor_on = 1;
        getColours = 0;  
      }
    }
    if (motor_on == 1) {
      rotate_motor();       // Turn motor on
    }
    else {
      motor_init();         // Turn motor off
    }
  }
  }
  return 0;
}
