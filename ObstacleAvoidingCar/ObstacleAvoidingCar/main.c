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

void calculateDistance(uint8_t echo_pin){
	char string[10];
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
	double distance = (double)count/427.21; // calculate distance
	
	dtostrf(distance, 2, 2, string);/* distance to string */
	strcat(string, " cm   ");	/* Concat unit i.e.cm */
	lcd_gotoxy(2, 0);
	lcd_puts("Dist = ");
	lcd_gotoxy(2, 7);	/* Print distance */
	lcd_puts(string);
}



ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	/* Increment Timer Overflow count */
}

int main(void)
{
	char string[10];
	long count;
	double distance;
	
	DDRD = _BV(TRIGGER_PIN);		/* Make trigger pin as output */
	PORTD = _BV(PIND0);		/* Turn on Pull-up */
	
	///////////lcd///////////////
	DDRB = _BV(PB3);

	/*TCCR1A = _BV(COM1B1) | _BV(WGM10);
	TCCR1B = _BV(WGM12) | _BV(CS11);
	OCR1B = 128;*/
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
		calculateDistance(PIND0);	
		_delay_ms(500);
	}
}

