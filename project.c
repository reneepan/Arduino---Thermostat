/********************************************
 *
 *  Name: Renee Pan
 *  Email: reneepan@usc.edu
 *  Section: 3:30pm Wednesday
 *  Assignment: Project - Thermostat
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "lcd.h"
#include "ds18b20.h"
#include "timer.h"

#define EE_PROM_LOW_TEMP 0x00
#define EE_PROM_HIGH_TEMP 0x01

void play_note(); //uint16_t
void update_motor(uint8_t temp);

// Frequencies for natural notes from middle C (C4)
// up one octave to C5.
uint16_t frequency[8] =
    { 262, 294, 330, 349, 392, 440, 494, 523 };

volatile uint8_t new_state, old_state, a, b;
volatile uint8_t temp_state = 1, lowCount = 50, highCount = 90;
volatile uint8_t changed = 0; //flag for state change
volatile int16_t count = 0; //count to display
volatile int16_t changeTemp = 0;
volatile uint8_t sound = 0; //sound flag
volatile int num = 0;
volatile int buzz_count = 0, motor_count = 0, motor_temp = 1;

int main(void) {

    // Initialize DDR and PORT registers and LCD
	DDRC |= ((1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
	DDRD &= ~((1 << PD2) | (1 << PD3));
	
	DDRB |= ((1 << PB4) | (1 << PB5) | (1 << PB3));
	PORTB &= ~(1 << PB4);
	
	PORTC |= ((1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
	PORTD |= ((1 << PD2) | (1 << PD3));
	
	PCICR |= ((1 << PCIE1) | (1 << PCIE2));
	PCMSK1 |= ((1 << PCINT9) | (1 << PCINT10));
	PCMSK2 |= ((1 << PCINT18) | (1 << PCINT19));

	lcd_init();
	timer1_init();
	timer2_init();
	timer0_init();
	sei();
	ds_init(); //initialize DS18B20

    // Write a spash screen to the LCD
	lcd_writecommand(1);
    char lab[14] = "EE109 Project";
    lcd_moveto(0,0);
    lcd_stringout(lab);
    char name[10] = "Renee Pan";
    lcd_moveto(1, 0);
    lcd_stringout(name);
    _delay_ms(1000);

    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
	unsigned char t[2];
    
    if (ds_init() == 0) {    // Initialize the DS18B20
         // Sensor not responding
    }
    ds_convert();    // Start first temperature conversion
    
	new_state = old_state;

	lcd_writecommand(1);

	lowCount = eeprom_read_byte ((uint8_t*) EE_PROM_LOW_TEMP);
	highCount = eeprom_read_byte((uint8_t*)EE_PROM_HIGH_TEMP);

	if(lowCount < 50 || lowCount > 90) {
		lowCount = 50;
	}

	if(highCount > 90 || highCount < 50) {
		highCount = 90;
	}

	lcd_moveto(1,0);
	char low[8];
	snprintf(low, 8, "Low= %d", lowCount);
	lcd_stringout(low);

	lcd_moveto(1, 8);
	char high[9];
	snprintf(high, 9, "High= %d", highCount);
	lcd_stringout(high);

    while (1) { 
        // Loop forever
		if (ds_temp(t)) {    // True if conversion complete
            /*
              Process the values returned in t[0]
              and t[1] to find the temperature.
            */
        
            ds_convert();   // Start next conversion
			unsigned int whole = (t[1] & 0x0F) * 256 + t[0];
			unsigned fullNum = (whole * 18 / 16) + 320;
			num = fullNum / 10;
			int decimal = fullNum % 10;

			unsigned char buf[12];
			snprintf(buf, 12, "Temp: %2d.%1d", num, decimal);
			lcd_moveto(0,0);
			lcd_stringout(buf);
			if (!motor_count) {
				OCR2A = (-2 * num + 255) / 5;
			}
		}

		if (changed) {
			if (temp_state == 1) {
				lcd_moveto(1, 0);
				char low[8];
				snprintf(low, 8, "Low? %d", lowCount);
				lcd_stringout(low);
				char high[7] = "High= ";
				lcd_moveto(1, 8);
				lcd_stringout(high);
				if (motor_count) {
					OCR2A = (-2 * lowCount + 255) / 5;
				}
			}
			else if (temp_state == 0){
				lcd_moveto(1, 8);
				char high[9];
				snprintf(high, 9, "High? %d", highCount);
				lcd_stringout(high);
				char low[6] = "Low= ";
				lcd_moveto(1, 0);
				lcd_stringout(low);
				if (motor_count) {
					OCR2A = (-2 * highCount + 255) / 5;
				}
			}
		}
		if (num < lowCount) { //red LED
			PORTC |= (1 << PC4);
			PORTC |= (1 << PC5);
			PORTC &= ~(1 << PC3);
		}
		else if (num > highCount) { //blue LED
			PORTC |= (1 << PC3);
			PORTC |= (1 << PC4);
			PORTC &= ~(1 << PC5);
		}
		else { //green LED
			PORTC |= (1 << PC3);
			PORTC |= (1 << PC5);
			PORTC &= ~(1 << PC4);
		}

		if ((num > highCount + 3 || num < lowCount - 3) && (sound == 0)) {
			sound = 1;
			play_note();
		}
		else if (num <= highCount + 3 && num >= lowCount - 3) {
			sound = 0;
		}

    }
}

/*
  Play a tone at the frequency specified for one second
*/
void play_note() //uint16_t freq
{
	TCCR0B |= (0b100 << CS00); // Set prescaler to 64
	buzz_count = 262; // freq
}

ISR(PCINT1_vect)
{
    // In Task 6, add code to read the encoder inputs and determine the new
    // count value
	uint8_t x = PINC;
		if (x & (1 << PC1)) {
			a = 1;
		}
		else {
			a = 0;
		}
		if (x & (1 << PC2)) {
			b = 1;
		}
		else {
			b = 0;
		}
	// For each state, examine the two input bits to see if state
	// has changed, and if so set "new_state" to the new state,
	// and adjust the count value.
	if (old_state == 0) {
		// Handle A and B inputs for state 0
		if (a == 1) {
			new_state = 1;
			if (temp_state) {
				lowCount++;
			}
			else {
				highCount++;
			}
		}
		else if (b == 1) {
			new_state = 2;
			if (temp_state) {
				lowCount--;
			}
			else {
				highCount--;
			}
		}
	}
	else if (old_state == 1) {
	    // Handle A and B inputs for state 1
		if (a == 0) {
			new_state = 0;
			if (temp_state) {
				lowCount--;
			}
			else {
				highCount--;
			}
		}
		else if (b == 1) {
			new_state = 3;
			if (temp_state) {
				lowCount++;
			}
			else {
				highCount++;
			}
		}

	}
	else if (old_state == 2) {
	    // Handle A and B inputs for state 2
		if (a == 1) {
			new_state = 3;
			if (temp_state) {
				lowCount--;
			}
			else {
				highCount--;
			}
		}
		else if (b == 0) {
			new_state = 0;
			if (temp_state) {
				lowCount++;
			}
			else {
				highCount++;
			}
		}
	}
	else {   // old_state = 3
	    // Handle A and B inputs for state 3
		if (a == 0) {
			new_state = 2;
			if (temp_state) {
				lowCount++;
			}
			else {
				highCount++;
			}
		}
		else if (b == 0) {
			new_state = 1;
			if (temp_state) {
				lowCount--;
			}
			else {
				highCount--;
			}
		}

	}
	if (lowCount < 50) {
		lowCount = 50;
	}
	if (lowCount > 90) {
		lowCount = 90;
	}
	if (highCount < 50) {
		highCount = 50;
	}
	if (highCount > 90) {
		highCount = 90;
	}
	if (lowCount >= highCount) {
		if (temp_state) {
			lowCount = highCount - 1;
		}
		else {
			highCount = lowCount + 1;
		}
	}
	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    changed = 1;
		if (temp_state) {
			eeprom_update_byte((uint8_t*)EE_PROM_LOW_TEMP, lowCount);
		}
		else {
			eeprom_update_byte((uint8_t*)EE_PROM_HIGH_TEMP, highCount);
		}
	    old_state = new_state;
	}
}
ISR(TIMER0_COMPA_vect)
{
	PORTB ^= (1 << PB5);
	buzz_count--;
	if(buzz_count == 0) {
		TCCR0B &= ~(0b111 << CS00);
	}
}
ISR(TIMER1_COMPA_vect)
{
	TCCR1B &= ~((1 << CS12) | (1 << CS10)); //stop timer 
	motor_count = 0;
}
ISR(PCINT2_vect) {
	if((PIND & (1 << PD2)) == 0){
		temp_state = 1; //low temp flag
		TCCR1B |= ((1 << CS12) | (1 << CS10)); //start timer
		motor_count = 1;
	}
	if((PIND & (1 << PD3)) == 0){
		temp_state = 0; //high temp flag
		TCCR1B |= ((1 << CS12) | (1 << CS10)); //start timer
		motor_count = 1;
	}
}