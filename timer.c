#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "project.h"

void timer1_init()
{
	TCCR1B |= ((1 << WGM12)); //CTC
	TIMSK1 |= (1 << OCIE1A); //enable interrupt
	OCR1A = 62500; //4 second delay
}
void timer0_init()
{
	TCCR0A |= (1 << WGM01);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 120;
}
void timer2_init() 
{
	TCCR2A |= (1 << COM2A1);
	TCCR2A |= (1 << WGM21) | (1 << WGM20); //fast pwm
	TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20)); //largest prescalar for max period
	OCR2A = 23;
}