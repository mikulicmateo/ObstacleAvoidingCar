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
#include "defines.h"

volatile int timer_overflow = 0;
double distance[3] = {0, 0, 0}, turn_distance = 0;
uint8_t flag = 0, reversed = 0, turn_on=0, left_ticks = 0, right_ticks = 0, counter = 0, first = 1;
double right_rpm = 255, left_rpm = 255;
	
ISR(TIMER1_OVF_vect)
{
	timer_overflow++;	/* Increment Timer Overflow count */
}

ISR(INT0_vect){
	left_ticks++;
}

ISR(INT1_vect){
	right_ticks++;
}

ISR(TIMER2_COMP_vect)
{
	counter++;
	if(counter == TIMESX) // 20 times 0.05sec == 1 sec
	{
		left_rpm = (left_ticks * MINUTE)/NUM_HOLES;
		right_rpm = (right_ticks * MINUTE)/NUM_HOLES;
		left_ticks=0;
		right_ticks=0;
		counter = 0;
	}
}


void writeValueToLcd(uint8_t x, uint8_t y, double value)
{
	char string[10];
	dtostrf(value, 2, 2, string);/* distance to string */
	lcd_gotoxy(x, y);
	lcd_puts(string);
}

void rpm_sensor_init()
{
	TCCR2 = _BV(WGM21) | _BV(CS22) | _BV(CS21) | _BV(CS20);
	OCR2 = OCR_VAL;
	GICR = _BV(INT0) | _BV(INT1);
	MCUCR = _BV(ISC11) | _BV(ISC10) | _BV(ISC01) | _BV(ISC00);
	TIMSK |=  _BV(OCIE2);
}

void lcd_initialize()
{
	DDRB = _BV(PB3);
	TCCR0 =  _BV(WGM01) | _BV(WGM00) | _BV(CS01) | _BV(COM01);
	OCR0 = 128;
	
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
}

double calculateDistance(uint8_t echo_pin)
{	
	PORTD &= (~(1 << echo_pin));
	PORTD |= (1 << TRIGGER_PIN);
	_delay_us(10);
	PORTD &= (~(1 << TRIGGER_PIN));

	while((PIND & _BV(echo_pin)) == 0); // while 0, wait for high
	
	//reset timer and overflow
	TCNT1 = 0;	/* Clear Timer counter */
	TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */
	timer_overflow = 0;/* Clear Timer overflow count */	
	
	while(PIND & _BV(echo_pin))// while 1, wait for low
	{
		if(timer_overflow > 3)
			PORTD &= (~(1 << echo_pin));
	}
	long count = TCNT1 + (REGISTER16BIT_MAX * timer_overflow); //calculate how many ticks the echo pin was HIGH
	
	return (double)count/DISTCONST; // calculate distance 
}

void readFromUSSensors()
{
	distance[FRONT_SENSOR] = calculateDistance(FRONT_SENSOR);
	_delay_ms(10);
	distance[LEFT_SENSOR] = calculateDistance(LEFT_SENSOR);
	_delay_ms(10);
	distance[RIGHT_SENSOR] = calculateDistance(RIGHT_SENSOR);
	_delay_ms(10);
}

void moveCar(uint8_t value)
{
	PORTA = value;
}

uint8_t checkRPM()
{
	cli();
	if(right_rpm > (double) RPM_LOW && left_rpm > (double) RPM_LOW)
	{
		sei();
		return 1;
	}
	else
	{
		sei();
		return 0;
	}
}

void choose_left_right_back(uint8_t repetition)
{
	if(repetition == 0)
		return;
	if(distance[LEFT_SENSOR] > (double) SIDE_THRESH || distance[RIGHT_SENSOR] > (double) SIDE_THRESH){ //if not stuck left & right
			
		if(distance[LEFT_SENSOR] > distance[RIGHT_SENSOR]){ // if more room to the left
			moveCar(LEFT);
			_delay_ms(500);			
		}
		else if(distance[LEFT_SENSOR] < distance[RIGHT_SENSOR]){ //if more room to the right
			moveCar(RIGHT);
			_delay_ms(500);
		}
		else{ //if same go right
			moveCar(RIGHT);
			_delay_ms(500);
		}
	}
	else{
		moveCar(BACKWARD);
		_delay_ms(1000);
		if(!checkRPM())
			return;
		readFromUSSensors();
		choose_left_right_back(repetition--);
	}
}

void carDrive()
{		
	readFromUSSensors();
	if(distance[FRONT_SENSOR] > (double) TOO_FAR) distance[FRONT_SENSOR] = (double) TOO_FAR;
	if(distance[FRONT_SENSOR] > (double) FRONT_THRESH) //if there is room forward
	{ 
		moveCar(FORWARD);
		if(!checkRPM()){
			moveCar(BACKWARD);
			_delay_ms(1000);
			readFromUSSensors();
			choose_left_right_back(RECURSE);
		}
	}
	else
	{
		choose_left_right_back(RECURSE);
	}
}

int main(void)
{
	
	DDRD = _BV(TRIGGER_PIN);		/* Make trigger pin as output */
	PORTD = _BV(FRONT_SENSOR) | _BV(LEFT_SENSOR) | _BV(RIGHT_SENSOR);		/* Turn on Pull-up */
	DDRA = 0xff; //FOR Motors
	PORTB = _BV(0);//Start button

	
	//lcd_initialize();
	TIMSK = (1 << TOIE1);	/* Enable Timer1 & timer2 overflow interrupts */
	TCCR1A = 0;		/* Set all bit to zero Normal operation */
	TCCR1B = _BV(CS10); //prescaler
	sei();	/* Enable global interrupt */
	while(1)
	{	
		if(bit_is_clear(PINB, 0))
			turn_on = 1;
		switch(turn_on) {
			case 1:
				if(first){
					first = 0;
					rpm_sensor_init();
				}
				carDrive();
				break;
		}
	}
}
