/*
 * ObstacleAvoidingCar.c
 *
 * Created: 9.2.2022. 14:38:37
 * Authors : Mateo Mikulic, Dominik Vicevic, Mauro Gizdulic
 */ 

#define F_CPU 7372800UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "lcd.h"

#define TRIGGER_PIN PD7

volatile int TimerOverflow = 0;
double distance[3] = {0, 0, 0};
	
ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	/* Increment Timer Overflow count */
}

void writeValueToLcd(uint8_t x, uint8_t y, double value)
{
	char string[10];
	dtostrf(value, 2, 2, string);/* distance to string */
	lcd_gotoxy(x, y);
	lcd_puts(string);
}

double calculateDistance(uint8_t echo_pin)
{	
	PORTD |= (1 << TRIGGER_PIN);
	_delay_us(10);
	PORTD &= (~(1 << TRIGGER_PIN));
	
	TCNT1 = 0;	/* Clear Timer counter */
	TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */

	while((PIND & _BV(echo_pin)) == 0); // while 0, wait for high
	
	//reset timer and overflow
	TCNT1 = 0;	/* Clear Timer counter */
	TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */
	TimerOverflow = 0;/* Clear Timer overflow count */
	
	
	while(PIND & _BV(echo_pin)); // while 1, wait for low
	long count = TCNT1 + (65535 * TimerOverflow); //calculate how many ticks the echo pin was HIGH
	
	return (double)count/427.21; // calculate distance
}

void readFromUSSensors()
{
	distance[0] = calculateDistance(PIND0);
	writeValueToLcd(0,0, distance[0]);
	_delay_ms(50);
	distance[1] = calculateDistance(PIND1);
	writeValueToLcd(8,0, distance[1]);
	_delay_ms(50);
	distance[2] = calculateDistance(PIND2);
	writeValueToLcd(0,1, distance[2]);
}

int main(void)
{
	
	DDRD = _BV(TRIGGER_PIN);		/* Make trigger pin as output */
	PORTD = _BV(PIND0) | _BV(PIND1) | _BV(PIND2);		/* Turn on Pull-up */
	
	///////////lcd///////////////
	DDRB = _BV(PB3);
	TCCR0 =  _BV(WGM01) | _BV(WGM00) | _BV(CS01) | _BV(COM01);
	OCR0 = 128;

	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	/////////////////////////////
	
	sei();			/* Enable global interrupt */
	TIMSK = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
	TCCR1A = 0;		/* Set all bit to zero Normal operation */
	TCCR1B = _BV(CS10);

	while(1)
	{
		readFromUSSensors();
		_delay_ms(500);
	}
}

