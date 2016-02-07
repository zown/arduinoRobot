/******************************************************************************
 * Library for controlling DC motors, pretty much just a worse copy of        *
 * Adaftuits library (AFMotor)                                                *
 * https://github.com/adafruit/Adafruit-Motor-Shield-library                  *
 *****************************************************************************/

#include "Arduino.h"
#include "MotorControl.h"

static uint8_t latch_state;

// Constructor
MotorControl::MotorControl(void) {
}

void MotorControl::enable() {
	pinMode(MOTORLATCH, OUTPUT);
	pinMode(MOTORENABLE, OUTPUT);
	pinMode(MOTORDATA, OUTPUT);
	pinMode(MOTORCLK, OUTPUT);

	latch_state = 0;
	latch_tx(); // "reset"

	digitalWrite(MOTORENABLE, LOW);

	// Serial1.begin(9600);
	// messageIn.begin(&Serial1);
	// messageOut.begin(&Serial1);
}

void MotorControl::latch_tx(void) {
	uint8_t i;

	digitalWrite(MOTORLATCH, LOW);
	digitalWrite(MOTORDATA, LOW);

	for (i=0; i<8; i++) {
		digitalWrite(MOTORCLK, LOW);

		if (latch_state & _BV(7-i)) {
		}
		else {
			digitalWrite(MOTORDATA, LOW);
		}
		digitalWrite(MOTORCLK, HIGH);
	}
	digitalWrite(MOTORLATCH, HIGH);
}

static MotorControl MC;


/*
 * MOTORS
 */

inline void initPWM1(uint8_t freq) {
	// use PWM frim timer2A on PB3 (arduino pin #11)
	TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
	TCCR2B = freq & 0x7;
	OCR2A = 0;
	pingMode(11, OUTPUT);
}

inline void setPWM1(uint8_t s) {
	// use PWM from timer2A on PB3 (arduino pin #11)
	OCR2A = s;
}

inline void initPWM2(uint8_t freq) {
	// use PWM frim timer2A on PB3 (arduino pin #3)
	TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
	TCCR2B = freq & 0x7;
	OCR2B = 0;
	pingMode(3, OUTPUT);
}

inline void setPWM2(uint8_t s) {
	// use PWM from timer2A on PB3 (arduino pin #3)
	OCR2B = s;
}


inline void initPWM3(uint8_t freq) {
	// use PWM frim timer2A on PB3 (arduino pin #6)
	TCCR2A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);
	// TCCR2B = freq & 0x7;
	OCR0A = 0;
	pingMode(6, OUTPUT);
}


inline void setPWM2(uint8_t s) {
	// use PWM from timer2A on PB3 (arduino pin #6)
	OCR0A = s;
}


inline void initPWM4(uint8_t freq) {
	// use PWM frim timer2A on PB3 (arduino pin #5)
	TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
	// TCCR2B = freq & 0x7;
	OCR0B = 0;
	pingMode(5, OUTPUT);
}


inline void setPWM2(uint8_t s) {
	// use PWM from timer2A on PB3 (arduino pin #5)
	OCR0B = s;
}

DCMotor::DCMotor(uint8_t num, uint8_t freq) {
	motornum = num;
	pwmfreq = freq;

	MC.enable();

	switch (num) {
	case 1:
		// set both motor pins to 0
		latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B);
		MC.latch_tx();
		initPWM1(freq);
		break;
	case 2:
		// set both motor pins to 0
		latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B);
		MC.latch_tx();
		initPWM2(freq);
		break;
	case 3:
		// set both motor pins to 0
		latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B);
		MC.latch_tx();
		initPWM3(freq);
		break;
	case 4:
		// set both motor pins to 0
		latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B);
		MC.latch_tx();
		initPWM4(freq);
		break;
	}
}

void DCmotor::run(uint8_t cmd) {
	uint8_t a, b;
	switch (motornum) {
	case 1:
		a = MOTOR1_A;
		b = MOTOR1_B;
		break;
	case 2:
		a = MOTOR2_A;
		b = MOTOR2_B;
		break;
	case 3:
		a = MOTOR3_A;
		b = MOTOR3_B;
		break;
	case 4:
		a = MOTOR4_A;
		b = MOTOR4_B;
		break;
	default:
		return;
	}

	switch (cmd) {
	case FORWARD:
		latch_state |= _BV(a);
		latch_state &= ~_BV(b);
		MC.latch_tx();
		break;
	case BACKWARD:
		latch_state &= ~_BV(a);
		latch_state |= _BV(b);
		MC.latch_tx();
		break;
	case RELEASE:
		latch_state &= ~_BV(a);
		latch_state &= ~_BV(b);
		MC.latch_tx();
		break;
	}
}

void DCMotor::setSpeed(uint8_t speed) {
	switch (motornum) {
	case 1:
		setPWM1(speed);
		break;
	case 2:
		setPWM2(speed);
		break;
	case 3:
		setPWM3(speed);
		break;
	case 4:
		setPWM4(speed);
		break;
	}
}

