/*************************************************************************
Title:    testing output to a HD44780 based LCD display.
Author:   Peter Fleury  <pfleury@gmx.ch>  http://jump.to/fleury
File:     $Id: test_lcd.c,v 1.6 2004/12/10 13:53:59 peter Exp $
Software: AVR-GCC 3.3
Hardware: HD44780 compatible LCD text display
          ATS90S8515/ATmega if memory-mapped LCD interface is used
          any AVR with 7 free I/O pins if 4-bit IO port mode is used
**************************************************************************/
#define F_CPU 8000000UL

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include "lcd.h"


//#define XTAL        8e6                 // 8MHz

#define PHASE_A     (PINB & 1<<PINB4)
#define PHASE_B     (PINB & 1<<PINB3)

volatile int8_t enc_delta;              // Drehgeberbewegung zwischen
                                        // zwei Auslesungen im Hauptprogramm
// Dekodertabelle für wackeligen Rastpunkt
// halbe Auflösung
int8_t table[16] PROGMEM = {0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0};

// Dekodertabelle für normale Drehgeber
// volle Auflösung
//int8_t table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

const char str_closelid[] PROGMEM = "Deckel schliessen";
const char str_timeleft[] PROGMEM = "Zeit: ";
const char str_duration[] PROGMEM = "Belichtungsdauer";
const char str_seconds[] PROGMEM = " Sek.  ";
const char str_tubes[] PROGMEM = "Roehren";
const char str_tubes_upper[] PROGMEM = "Oben ";
const char str_tubes_lower[] PROGMEM = "Unten";
const char str_tubes_both[] PROGMEM = "Beide";
const char str_done1[] PROGMEM = "Fertig";



char buf[18];

uint8_t time;
uint8_t sides = 0;

/*
** constant definitions
*/
static const PROGMEM unsigned char copyRightChar[] =
{
	0x07, 0x08, 0x13, 0x14, 0x14, 0x13, 0x08, 0x07,
	0x00, 0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x00
};



ISR( TIMER0_COMPA_vect )             // 1ms fuer manuelle Eingabe
{
    static int8_t last=0;           // alten Wert speichern

    last = (last << 2)  & 0x0F;
    if (PHASE_A) last |=2;
    if (PHASE_B) last |=1;
    enc_delta += pgm_read_byte(&table[last]);
}


void encode_init( void )            // nur Timer 0 initialisieren
{
  TCCR0A = 1<<WGM01;     // CTC, XTAL / 64
  TCCR0B = 1<<CS01^1<<CS00;

  OCR0A = (uint8_t)(XTAL / 64.0 * 1e-3 - 0.5);   // 1ms
  TIMSK |= 1<<OCIE0A;
}


/*int8_t encode_read( void )         // Encoder auslesen
{
  int8_t val;

  // atomarer Variablenzugriff
  cli();
  val = enc_delta;
  enc_delta = 0;
  sei();
  if (val < 0) { return -1; }
  if (val == 0) { return 0; }
  if (val > 1) { return 1; }
  return val;
}*/

int8_t encode_read( void )         // read two step encoders
{
  int8_t val;

  cli();
  val = enc_delta;
  enc_delta = val & 1;
  sei();
  return val >> 1;
}

uint8_t key_pressed () {
	return PINB & 1<<PINB1;
}

uint8_t lid_closed () {
	return PINB & 1<<PINB7;
}

void output_time_info () {
	lcd_gotoxy(0,0);
				lcd_puts_p(str_duration);
				lcd_gotoxy(0,1);
				itoa((time+1)*10, buf, 10);
				lcd_puts(buf);
				lcd_puts_p(str_seconds);
}
void read_time () {
	int8_t val;

	output_time_info();

	for (;;) {
		val = encode_read();

		if (val != 0) {
			time+=val;
			output_time_info();
		}

		if (key_pressed()) {
			// Wait until key release
						while (key_pressed()) {}
			return;
		}
	}

}

void output_side_info () {
	lcd_gotoxy(0,0);
				lcd_puts_p(str_tubes);
				lcd_gotoxy(0,1);


				switch (sides) {
				case 0:
					lcd_puts_p(str_tubes_both);
					break;
				case 1:
					lcd_puts_p(str_tubes_upper);
					break;
				case 2:
					lcd_puts_p(str_tubes_lower);
					break;
				}
}


void read_sides () {
	int8_t val;

	output_side_info();

	for (;;) {
		val = encode_read();

		if (val != 0) {

			if (!(sides == 0 && val < 0)) {
				sides += val;
			}

			if (sides > 2) {
				sides = 2;
			}

			output_side_info();
		}

		if (key_pressed()) {
			// Wait until key release
			while (key_pressed()) {}
			return;
		}
	}
}

void beep () {
	uint8_t master_timer;

	for (master_timer=0;master_timer<10;master_timer++) {
		_delay_ms(5);
		PORTB |= (1<<PB2);
		_delay_ms(5);
		PORTB &= ~(1<<PB2);
	}
}

void start_doing_it () {
	uint32_t master_timer = time;

	lcd_clrscr();

	lcd_puts_p(str_closelid);

	while (!lid_closed()) {}

	lcd_clrscr();




	switch (sides) {
		case 0:
			PORTB |= (1<<PB0);
			PORTD |= (1<<PD6);
			break;
		case 1:
			PORTB |= (1<<PB0);
			break;
		case 2:
			PORTD |= (1<<PD6);
			break;
	}

	for (master_timer = ((time+1)*10)-1;master_timer > 0;master_timer--) {
		lcd_home();
		lcd_puts_p(str_timeleft);

		itoa(master_timer, buf, 10);
		lcd_puts(buf);
		lcd_puts(" s ");
		_delay_ms(1000);
	}

	PORTB &= ~(1<<PB0);
	PORTD &= ~(1<<PD6);

	lcd_clrscr();
	lcd_puts_p(str_done1);

	for (master_timer = 0; master_timer < 2;master_timer++) {
		beep();
			_delay_ms(100);
			beep();
			_delay_ms(100);
			beep();
			_delay_ms(100);
			beep();
			_delay_ms(1000);
			beep();
			beep();
			beep();
			_delay_ms(1000);

	}



}

int main(void)
{
    /* initialize display, cursor off */
    lcd_init(LCD_DISP_ON);

    /* Eingänge */
    DDRB &= ~(1 << DDB4);
    DDRB &= ~(1 << DDB1);
    DDRB &= ~(1 << DDB3);
    DDRB &= ~(1 << DDB7);

    /* Ausgänge */
    DDRB |= (1 << DDB2);
    DDRB |= (1 << DDB0);
    DDRD |= (1 << DDD6);

    lcd_clrscr();
    //lcd_puts("LCD Test Line 1\n");

    encode_init();
    sei();

    time = 0;

    for (;;) {                           /* loop forever */
    	lcd_clrscr();
    	read_time();
    	lcd_clrscr();
    	read_sides();
    	lcd_clrscr();
    	start_doing_it();
    }
}



