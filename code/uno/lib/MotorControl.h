/******************************************************************************
 * Library for controlling DC motors, pretty much just a worse copy of        *
 * Adaftuits library (AFMotor)                                                *
 * https://github.com/adafruit/Adafruit-Motor-Shield-library                  *
 *****************************************************************************/

#ifndef MotorControl_h
#define MotorControl_h

#include <inttypes.h>
#include <avr/io.h>

#define MOTOR34_8KHZ _BV(CS01) // divide by 8

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 0
#define MOTOR3_B 6
#define MOTOR4_A 5
#define MOTOR4_B 7

#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

class MotorControl
{
	MotorlControl(void);
	void enable(void);
	friend class DCMotor;
	void latch_tx(void);
};

class DCMotor
{
public:
	DCMotor(uint8_t motornum, uint8_t freq = MOTOR34_8KHZ);
	void run(uint8_t);
	void setSpeed(uint8_t);

private:
	uint8_t motornum, pwmfreq;
};

uint8_t getlatchstate(void);

#endif
