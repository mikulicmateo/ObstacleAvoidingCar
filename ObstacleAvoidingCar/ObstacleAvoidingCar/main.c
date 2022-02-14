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
#define FL_FORWARD 0x02
#define FL_BACKWARD 0x01
#define FR_FORWARD 0x08
#define FR_BACKWARD 0x04
#define BL_FORWARD 0x20
#define BL_BACKWARD 0x10
#define BR_FORWARD 0x80
#define BR_BACKWARD 0x40
#define FORWARD 0xAA
#define BACKWARD 0x55
#define LEFT 0x99
#define RIGHT 0x66
#define FRONT_SENSOR 0
#define LEFT_SENSOR 1
#define RIGHT_SENSOR 2


volatile int TimerOverflow = 0;
double distance[3] = {0, 0, 0};
uint8_t flag = 0, reversed = 0;
	
ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	/* Increment Timer Overflow count */
}

/*ISR(INT0_vect){
	flag = 1;
	//_delay_ms(50);
}

ISR(INT1_vect){
	flag=0;
	//_delay_ms(50);
}*/


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
	distance[FRONT_SENSOR] = calculateDistance(FRONT_SENSOR);
	_delay_ms(10);
	distance[LEFT_SENSOR] = calculateDistance(LEFT_SENSOR);
	_delay_ms(10);
	distance[RIGHT_SENSOR] = calculateDistance(RIGHT_SENSOR);
	_delay_ms(10);
}

void go_forward(){
	
}

void go_backwards(){;
}

void go_left(){
		
}

void go_right(){
	
}

void moveCar(uint8_t value){
	PORTA = value;
}

void carDrive(){	
	
	if(distance[FRONT_SENSOR] > 8.00 && !reversed) //if there is room forward and did not reverse
		moveCar(FORWARD);
	else
	{
		if(!(distance[LEFT_SENSOR] < 6 && distance[RIGHT_SENSOR] < 6)){ //if not stuck left & right
			
			if(distance[LEFT_SENSOR] > distance[RIGHT_SENSOR]){ // if more room to the left
				moveCar(LEFT);
				reversed=0;
			}else if(distance[LEFT_SENSOR] < distance[RIGHT_SENSOR]){ //if more room to the right
				moveCar(RIGHT);
				reversed=0;
			}else{ //if same go right
				moveCar(RIGHT);
				reversed=0;
			}
			
		}
		else{
			
			//IF -> polako skretanje
			moveCar(BACKWARD);
			reversed = 1;
			_delay_ms(500);
		}
	}
}

int main(void)
{
	
	DDRD = _BV(TRIGGER_PIN);		/* Make trigger pin as output */
	PORTD = _BV(PIND0) | _BV(PIND1) | _BV(PIND2);		/* Turn on Pull-up */
	DDRA = 0xff;

	
	///////////lcd///////////////
	//DDRB = _BV(PB3);
	//TCCR0 =  _BV(WGM01) | _BV(WGM00) | _BV(CS01) | _BV(COM01);
	//OCR0 = 128;
	//
	//lcd_init(LCD_DISP_ON);
	//lcd_clrscr();
	/////////////////////////////
	
			
	TIMSK = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
	TCCR1A = 0;		/* Set all bit to zero Normal operation */
	TCCR1B = _BV(CS10);
	//GICR = _BV(INT0) | _BV(INT1);
	sei();	/* Enable global interrupt */
	
	while(1)
	{	
		/*if(flag==1){
			move_car();
		}*/
		calculateDistance(FRONT_SENSOR);
		if(1){
			carDrive();
		}
	}
}

//prednji lijevi PA0,PA1
//prednji desni PA2,PA3
//zadnji lijevi PA4, PA5
//zadnji desni PA6, PA7

//	_delay_ms(1); -> zavoj min