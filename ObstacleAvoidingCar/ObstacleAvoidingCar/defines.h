/*
 * defines.h
 *
 * Created: 16.2.2022. 18:52:59
 *  Author: Mateo
 */ 


#ifndef DEFINES_H_
#define DEFINES_H_

#define TRIGGER_PIN PIND7
#define FL_FORWARD 0x10
#define FL_BACKWARD 0x20
#define FR_FORWARD 0x40
#define FR_BACKWARD 0x80
#define BL_FORWARD 0x04
#define BL_BACKWARD 0x08
#define BR_FORWARD 0x01
#define BR_BACKWARD 0x02
#define FORWARD 0xAA
#define BACKWARD 0x55
#define LEFT 0x69
#define RIGHT 0x96
#define FRONT_SENSOR 0
#define LEFT_SENSOR 1
#define RIGHT_SENSOR 4
#define SIDE_THRESH 13
#define FRONT_THRESH 25
#define TOO_FAR 115
#define NUM_HOLES 20
#define MINUTE 60
#define OCR_VAL 194
#define REGISTER16BIT_MAX 65535
#define RPM_DIFF 20
#define RPM_LOW 1
#define TIMESX 20
#define RECURSE 5
#define STOP 0


#endif /* DEFINES_H_ */
