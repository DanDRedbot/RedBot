/*UCONN ECE 3411 Spring 2018 Redbot Project by Daniel Dabkowski and John Brousseau */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/power.h>
#include <math.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"

#define F_CPU 16000000UL
#define A0  PORTC0 //left IR Sensor
#define A1  PORTC1 //middle IR Sensor
#define	A2  PORTC2 //right IR Sensor
#define L_CTRL_1   PORTD2 //for left motor
#define L_CTRL_2   PORTD4
#define PWM_L      PORTD5

#define R_CTRL_1   PORTD7 //for right motor
#define R_CTRL_2   PORTB0
#define PWM_R      PORTD6

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void leftread(void)
{
	int LeftAin;
	volatile float LeftVoltage;
	char LeftVoltageBuffer[6];
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Set ADC prescaler to 128
	ADCSRA |= (1<<ADEN) | (1 << ADATE); // Enable ADC
	ADCSRB |= (1<<ADTS0);
	// Start A to D conversion
	ADCSRA |= (1<<ADSC);
	// Wait until this conversion is completed
	while((ADCSRA & (1<<ADSC)));
	LeftAin = (ADCL); // First read lower byte
	LeftAin |= (ADCH<<8); // Then read upper byte
	// Data type conversion for reading
	// Typecast the volatile integer into floating type data,
	// divide by maximum 10-bit value, and
	// multiply by 5V for normalization
	LeftVoltage = (float)LeftAin/1024.00 * 5.00;
	
	uart_init();
	stdout = stdin = stderr = &uart_str;
	dtostrf(LeftVoltage, 3, 2, LeftVoltageBuffer);
	fprintf(stdout," Left Voltage is %s\n\r",LeftVoltageBuffer);
}
void centerread(void)
{
	int CenterAin;
	volatile float CenterVoltage;
	char CenterVoltageBuffer[6];
	ADMUX &= ~(1<<MUX1);
	ADMUX |= (1<<MUX0);
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Set ADC prescaler to 128
	ADCSRA |= (1<<ADEN) | (1 << ADATE); // Enable ADC
	ADCSRB |= (1<<ADTS0);
	// Start A to D conversion
	ADCSRA |= (1<<ADSC);
	// Wait until this conversion is completed
	while((ADCSRA & (1<<ADSC)));
	CenterAin = (ADCL); // First read lower byte
	CenterAin |= (ADCH<<8); // Then read upper byte
	// Data type conversion for reading
	// Typecast the volatile integer into floating type data,
	// divide by maximum 10-bit value, and
	// multiply by 5V for normalization
	CenterVoltage = (float)CenterAin/1024.00 * 5.00;
	
	uart_init();
	stdout = stdin = stderr = &uart_str;
	dtostrf(CenterVoltage, 3, 2, CenterVoltageBuffer);
	fprintf(stdout," Center Voltage is %s\n\r",CenterVoltageBuffer);
}
void rightread(void)
{
	int RightAin;
	volatile float RightVoltage;
	char RightVoltageBuffer[6];
	ADMUX &= ~(1<<MUX0);
	ADMUX |= (1<<MUX1);
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Set ADC prescaler to 128
	ADCSRA |= (1<<ADEN) | (1 << ADATE); // Enable ADC
	ADCSRB |= (1<<ADTS0);
	// Start A to D conversion
	ADCSRA |= (1<<ADSC);
	// Wait until this conversion is completed
	while((ADCSRA & (1<<ADSC)));
	RightAin = (ADCL); // First read lower byte
	RightAin |= (ADCH<<8); // Then read upper byte
	// Data type conversion for reading
	// Typecast the volatile integer into floating type data,
	// divide by maximum 10-bit value, and
	// multiply by 5V for normalization
	RightVoltage = (float)RightAin/1024.00 * 5.00;
	
	uart_init();
	stdout = stdin = stderr = &uart_str;
	dtostrf(RightVoltage, 3, 2, RightVoltageBuffer);
	fprintf(stdout," Right Voltage is %s\n\r",RightVoltageBuffer);
}

void RedBotMotors::leftFwd(byte spd)
{
	digitalWrite(L_CTRL_1, HIGH);
	digitalWrite(L_CTRL_2, LOW);
	analogWrite(PWM_L, spd);
	
}

void RedBotMotors::leftRev(byte spd)
{
	digitalWrite(L_CTRL_1, LOW);
	digitalWrite(L_CTRL_2, HIGH);
	analogWrite(PWM_L, spd);
	
}

void RedBotMotors::rightFwd(byte spd)
{
	digitalWrite(R_CTRL_1, HIGH);
	digitalWrite(R_CTRL_2, LOW);
	analogWrite(PWM_R, spd);
	
}

void RedBotMotors::rightRev(byte spd)
{
	digitalWrite(R_CTRL_1, LOW);
	digitalWrite(R_CTRL_2, HIGH);
	analogWrite(PWM_R, spd);

}


int main(void)
{
	DDRB |= (1<<R_CTRL_2); //Enables the necessary bits for output
	DDRC |= (1<<A0) | (1<<A1) |(1<<A2);
	DDRD |= (1<<L_CTRL_1) | (1<<L_CTRL_2) | (1<<PWM_L) | (1<<R_CTRL_1) | (1<<PWM_R);
    while (1) 
    {
		rightread();
    }
}

