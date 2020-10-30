#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>
#include <string.h>
#include "LCD_16x2_H_file.h"//Include LCD header file

#define  Trigger_pin	PA0	 //Trigger pin

int TimerOverflow = 0;

ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	// Increment Timer Overflow count
}

void LM_FORWARD(unsigned char A)  // left motor forward
{
	    OCR2=A;               //TIMER 2 PD7
	    TCCR2 |= (1<<5);      //for non inverting
		TCCR2 =(1<<3)|(1<<6); //fast PWM
		TCCR2 |= (1<<1);      //prescaler =8 to allow the timer to be clocked at the rate desired
		DDRD |=(1<<7) ;       //TIMER 2 PD7

		PORTC |=  (1<<3);     // Left motor rotates clockwise
		PORTC &= ~(1<<4);
}

void LM_backward(unsigned char A)  // left motor backward
{
	    OCR2=A;               //TIMER 2 PD7
	    TCCR2 |= (1<<5);      //for non inverting
		TCCR2 =(1<<3)|(1<<6); //FAST PWM
		TCCR2 |= (1<<1);      //prescaler =8 to allow the timer to be clocked at the rate desired
		DDRD |=(1<<7) ;       //TIMER 2 PD7

		PORTC &= ~(1<<3);     // Left motor rotates anti-clockwise
		PORTC |=  (1<<4);
}

void RM_FORWARD(unsigned char B)  // right motor forward
{
	    OCR0=B;               //TIMER 2 PD7
	    TCCR0 |= (1<<5);      //for non inverting
		TCCR0 =(1<<3)|(1<<6); //FAST PWM
		TCCR0 |= (1<<1);      //prescaler =8 to allow the timer to be clocked at the rate desired
		DDRB |=(1<<3) ;       //TIMER 0 PB3

		PORTC |= (1<<1);      // right motor rotates clockwise
		PORTC &= ~(1<2);
}

void RM_backward(unsigned char B)  // right motor backward
{
	    OCR0=B;               //TIMER 2 PD7
	    TCCR0 |= (1<<5);      //for non inverting
		TCCR0 =(1<<3)|(1<<6); //FAST PWM
		TCCR0 |= (1<<1);      //prescaler =8 to allow the timer to be clocked at the rate desired
		DDRB |=(1<<3) ;       //TIMER 0 PB3

		PORTC &= ~(1<<1);     // right motor rotates anti-clockwise
		PORTC |=  (1<<2);
}


ISR(INT0_vect)  //front IR sensor interrupt
{
	if (PIND &(1<<2))   //front IR sensor is detecting a line
	{
		LM_FORWARD(250); //the car will rotate completely in a clockwise direction at full speed as the two motors are in opposite directions
		RM_backward(250);

	}
	else{
		RM_FORWARD(250);
		LM_FORWARD(250);
		}
}


ISR(INT1_vect)  //right IR sensor interrupt
{
	if (PIND &(1<<3)) //right IR sensor is detecting a line
		{
		RM_FORWARD(250); //the car will rotate to the left slightly until the sensor is clear as the right motor is faster than the left motor
		LM_FORWARD(64);

		}
	else{
		RM_FORWARD(250);
		LM_FORWARD(250);
	}
}

ISR(INT2_vect)  //left IR sensor interrupt
{
	if (PINB &(1<<2)) //left IR sensor is detecting a line
		{
		LM_FORWARD(250); //the car will rotate to the right slightly until the sensor is clear as the left motor is faster than the right motor
		RM_FORWARD(64);

		}
	else{
		RM_FORWARD(250);
		LM_FORWARD(250);
		}
}






int main ()
{
	char string[10];
		long count;
		double distance;

		DDRA = 0x01;     //Make trigger pin as output
		PORTD |= (1<<6); // Turn on Pull-up
		LCD_Init();
		LCD_String_xy(1, 0, "Ultrasonic");
        sei();			/* Enable global interrupt */
		TIMSK = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
		TCCR1A = 0;		/* Set all bit to zero Normal operation */




	MCUCR = (1<<0) | (1<<1);   //to determine the nature of signal at (INT0)
	MCUCR = (1<<2) | (1<<3);   //to determine the nature of signal at (INT1)
    MCUCSR = (1<<6);           //to determine the nature of signal at (INT2)
    GICR = (1<<5) | (1<<6) | (1<<7); // to enable interrupts (INT0)(INT1)(INT2)
    SREG |=(1<<7);             //to use  GICR bit7 of SERG must be set to 1

	DDRB &= ~(1<<2);      //IR sensor 1 (left) as input (INT2)
	PINB &= ~(1<<2);

	DDRD &= ~(1<<2);      //IR sensor 2 (front) as input (INT0)
	PIND &= ~(1<<2);

	DDRD &= ~(1<<3);      //IR sensor 3 (right) as input (INT1)
	PIND &= ~(1<<3);

	    DDRC |=(1<<1);          //Right motor as output
		PORTC &= ~(1<<1);
	    DDRC |=(1<<2);
		PORTC &= ~(1<<2);

		DDRC |=(1<<3);          //Left motor as output
		PORTC &= ~(1<<3);
		DDRC |=(1<<4);
	    PORTC &= ~(1<<4);


	while (1)
	{    //the following code is from electricwing.com to the the distance on the LCD (in real live it will not be needed)
		/* Give 10us trigger pulse on trig. pin to HC-SR04 */
				PORTA |= (1 << Trigger_pin);
				_delay_us(10);
				PORTA &= (~(1 << Trigger_pin));

				TCNT1 = 0;	/* Clear Timer counter */
				TCCR1B = 0x41;	/* Capture on rising edge, No prescaler*/
				TIFR = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
				TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */

				/*Calculate width of Echo by Input Capture (ICP) */

				while ((TIFR & (1 << ICF1)) == 0);/* Wait for rising edge */
				TCNT1 = 0;	/* Clear Timer counter */
				TCCR1B = 0x01;	/* Capture on falling edge, No prescaler */
				TIFR = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
				TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */
				TimerOverflow = 0;/* Clear Timer overflow count */

				while ((TIFR & (1 << ICF1)) == 0);/* Wait for falling edge */
				count = ICR1 + (65535 * TimerOverflow);	/* Take count */
				/* 8MHz Timer freq, sound speed =343 m/s */
				distance = (double)count / 466.47;

				dtostrf(distance, 2, 2, string);/* distance to string */
				strcat(string, " cm   ");	/* Concat unit i.e.cm */
				LCD_String_xy(2, 0, "Dist = ");
				LCD_String_xy(2, 7, string);	/* Print distance */
				_delay_ms(200);



		if(distance > 10) //if the ultrasonic sensor is far from any object more than 10 cm the car will continue forward
		{
			RM_FORWARD(250);
			LM_FORWARD(250);
		}
		else if(distance>10) // if the ultrasonic sensor detect an object it will rotate to the right until the sensor reads otherwise
		{
			LM_FORWARD(250); //the car will rotate to the right slightly until the sensor is clear as the left motor is faster than the right motor
			RM_FORWARD(64);

		}

	}



	}

