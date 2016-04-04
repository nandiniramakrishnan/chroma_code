/*
 *
 * test_motor.c - This file contains code to make the stepper motor 
 *                run continuously.
 *
 * Date: 3/16/2016
 *
 *
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>

#define BAUD 115200
#include <util/setbaud.h>

void init() {
    DDRC = 0xFF;        // set PORT C as output
    //PORTC = 0x00;     
}

int main() {
    init();
    while(1)
    {
        PORTC = 0x01;          //0001
        _delay_ms(10);
        PORTC = 0x04;          //0100
        _delay_ms(10);
        PORTC = 0x02;          //0010
        _delay_ms(10);
        PORTC = 0x08;          //1000
        _delay_ms(10);
    }
    return 0;
}
