/*
 * display.cpp
 *
 * Created: 2017-02-11 21:52:09
 *  Author: Ilya.Anakhov
 */ 

#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
//#include <util/delay.h>
#include "display.h"
#include "procvar.h"
#include "modbus.h"	// For the display of parity

#define DEBUG_DISPALY_STATE	0

// String constants
const char txt_none[] PROGMEM = "";
const char txt_test[] PROGMEM = "tESt";
const char txt_done[] PROGMEM = "donE";
const char txt_fail[] PROGMEM = "FAIL";
const char txt_p00[] PROGMEM = "P00";
const char txt_on[] PROGMEM = " ON ";
const char txt_stby[] PROGMEM = "Stby";
const char txt_ir[] PROGMEM = "Ir";
const char txt_cool[] PROGMEM = "COOL";
const char txt_halt[] PROGMEM = "HALt";
const char txt_high[] PROGMEM = "HIGH";
const char txt_low[] PROGMEM = " LO";
const char txt_dry[] PROGMEM = "dry ";
const char txt_intr[] PROGMEM = "Intr";
const char txt_err[] PROGMEM = "Err";
const char txt_good[] PROGMEM = "Good";
const char txt_even[] PROGMEM = "EUEN";
const char txt_odd[]  PROGMEM = " Odd";
const char txt_nopr[] PROGMEM = "nonE";
const char txt_auto[] PROGMEM = "Auto";
const char txt_man[] PROGMEM = "HAnd";
const char txt_off[] PROGMEM = "OFF";
const char txt_hand[] PROGMEM = "HAnd";
const char txt_reset[] PROGMEM = "rStF";
const char txt_heat[] PROGMEM = "HEAt";
const char txt_paus[] PROGMEM = "PAUS";
const char txt_req[] PROGMEM = "rEq";
const char txt_for[] PROGMEM = "For";
#if 1
const char txt_14k4[] PROGMEM = "14_4";
const char txt_19k2[] PROGMEM = "19_2";
const char txt_28k8[] PROGMEM = "28_8";
const char txt_38k4[] PROGMEM = "38_4";
#endif

PGM_P const disp_table[] PROGMEM =
{
	txt_none,
	txt_test,
	txt_done,
	txt_fail,
	txt_p00,
	txt_on,
	txt_stby,
	txt_ir,
	txt_cool,
	txt_halt,
	txt_high,
	txt_low,
	txt_dry,
	txt_intr,
	txt_err,
	txt_good,
	txt_even,
	txt_odd,
	txt_nopr,
	txt_auto,
	txt_man,
	txt_off,
	txt_hand,
	txt_reset,
	txt_heat,
	txt_paus,
	txt_req,
	txt_for,
#if 1
	txt_14k4,
	txt_19k2,
	txt_28k8,
	txt_38k4
#endif
};

/* Display Segments:
 -a- 
f   b
 -g-
e   c
 -d-
*/

// Glyphs
const uint8_t PROGMEM a = 0b01111011, b = 0b11011011, c = 0b11110011, d = 0b11111001, e = 0b11111010, f = 0b10111011, g = 0b11101011, dp = 0b11111011; //definitions for seven-segment display
const uint8_t PROGMEM zero = 0b00010000, one = b & c, two = a & b & g & e & d, three = a & b & c & d & g, four = f & g & b & c, five = a & f & g & c & d, six = a & f & g & c & d & e, seven = a & b & c, eight = a & b & c & d & e & f & g, nine = a & b & f & g & c & d, dot = dp;
const uint8_t PROGMEM dig1 = 0b11100000, dig2 = 0b11010000, dig3 = 0b11001000, dig4 = 0b00000100; //definitions for seven-segment display
const uint8_t PROGMEM display_A = f & a & b & g & e & c;
const uint8_t PROGMEM display_b = c & d & e & f & g, display_c = d & e & g, display_C = a & d & e & f, display_d = b & c & d & e & g, display_e = g & b & a & f & e & d, display_E = a & f & g & e & d, display_F = a & f & g & e;
const uint8_t PROGMEM display_G = a & f & e & d & c, display_H = f & e & b & c & g, display_h = f & e & g & c, display_I = f & e, display_J = b & c & d & e, display_L = f & e & d, display_N = a & b & c & e & f, display_n = c & e & g;
const uint8_t PROGMEM display_O = zero, display_o = c & d & e & g, display_P = a & f & g & b & e, display_q = a & f & g & b & c, display_r = e & g, display_S = a & f & g & c & d, display_t = f & g & e & d, display_u = c & d & e, display_U = b & c & d & e & f;
const uint8_t PROGMEM display_y = b & c & d & f & g, display_Z = two, display_minus = g, display_underscr = d, display_leftbrk = a & f & e & d, display_rightbrk = a & b & c & d, display_caret = a, display_space = 0xFF, display_all = 0;      
const uint8_t PROGMEM display_M = b & c & e & f; // "| |"
const uint8_t PROGMEM display_QUOT = f; // "' "
const uint8_t PROGMEM display_DQUOT = f & b; // "''"
const char PROGMEM charset[]  = "0123456789AbcCdeEFGHhIJLMNnOoPqrStuUyZ-_[]^! '\"\0";
const char PROGMEM glyphs[]  = {zero, one, two, three, four, five, six, seven, eight, nine, display_A, display_b, display_c, display_C, display_d, display_e, display_E, display_F, display_G, display_H, display_h, display_I, display_J,
							   display_L, display_M, display_N, display_n, display_O, display_o, display_P, display_q, display_r, display_S, display_t, display_u, display_U, display_y, display_Z, display_minus, display_underscr, display_leftbrk, display_rightbrk, display_caret, display_all, display_space, display_QUOT, display_DQUOT};

extern volatile uint32_t systime;
uint8_t  dg[4];
Sequence queue[6];

#if DEBUG_DISPALY_STATE!=0
uint8_t  dg_st[4];
uint8_t dg_st_cnt;
extern uint8_t pr_state;
extern uint16_t status;
#endif


void display(uint32_t numeric, uint8_t dot, uint8_t text, bool override)
{
	char buf[10];
	char* s;
	uint8_t len, i; //, num_d[4];
	uint8_t j = 0;
	uint8_t c;
	if ((!override) && ((pv_is_ui_active()) || (is_status(ST_FAIL))))	// Don't proceed if the UI is reserved for the menu and there's no override permission
		return;
	if (text == DISP_RESISTANCE)
	{
		if (numeric < DISP_MIN_KOHMS)
			text = DISP_LOW;
		else
		if (numeric >= DISP_MAX_MOHMS)
			text = DISP_HIGH;
		else
		if (numeric > DISP_MAX_KOHMS)
			numeric = (numeric + 500);	// Add 500 for proper rounding before the integer division
	}
	if ( text == DISP_SEC )
	{
		bool blink = (numeric & 1) == 1;
		
		if ( numeric < 1000 )
		{
			buf[3] = '"';		
		} else if ( numeric <= (999UL * 60UL) )
		{
			buf[3] = '\'';
			numeric /= 60;
		} else if ( numeric <= (999UL * 60UL * 60UL) )
		{
			buf[3] = 'h';
			numeric /= (60 * 60);
		} else if ( numeric <= (999UL * 60UL * 60UL * 24UL) )
		{
			buf[3] = 'd';
			numeric /= (60UL * 60UL * 24UL);
		} else 
		{
			buf[3] = 'y';
			numeric /= (60UL * 60UL * 24UL * 365UL);
		}
		if (blink)
			buf[3] = ' ';
			
		buf[2] = '0'+(numeric % 10);
		if(numeric > 9)
		{
			numeric /= 10;
			buf[1] = '0'+(numeric % 10);
			if(numeric > 9)
			{
				numeric /= 10;
				buf[0] = '0'+(numeric % 10);
			}
			else
			{
				buf[0] = ' ';
			}
		}
		else
		{
			buf[0] = buf[1] = ' ';
		}
	}
	else
	{
		// Convert a numeric value to a string
		s = ultoa(numeric, buf, 10);
	
		if ((text != DISP_NONE) && (text != DISP_P00) && (text != DISP_RESISTANCE) && (text != DISP_RESISTANCE_HIGHVALUE))
		{
			strcpy_P(buf, (PGM_P)pgm_read_word(&(disp_table[text])));
		}

		if ((text == DISP_RESISTANCE) && (numeric > DISP_MAX_KOHMS))
		{
			buf[2] = '^';
			buf[3] = '3';
		}
		else if ((text == DISP_RESISTANCE_HIGHVALUE) && (numeric > DISP_MAX_KOHMS))
		{
			buf[2] = '^';
			if (numeric > DISP_MAX_TENKOHMS)
				buf[3] = '4';
			else
				buf[3] = '3';
		}
		s = buf;
		len = strlen(s);
		if (len < 4)
		{
			for (i = 3; i > 3 - len; i--)
				buf[i] = buf[i - (4 - len)];
			while (i > 0)
				buf[i--] = ' ';
			buf[0] = ' ';
		}
		// Special case 1: "P" + parameter number
		if (text == DISP_P00)
			buf[0] = 'P';
		// Special case 2: "Dry" + sequence number
		if ((text == DISP_DRY) && (numeric != 0))
		{
			buf[3] = numeric + '0';	// Assumed that the numeric value is between 0 and 9
		}
	}

	cli();
	dg[0] = 0;
	dg[1] = 0;
	dg[2] = 0;
	dg[3] = 0;

	for (i = 0; i < 4; i++)
	{
		j = 0;
		c = '%';
		while ((c != buf[i]) && (c != 0)) {
			c = pgm_read_byte(&charset[j]);
			j++;
		} ;
		--j;
		dg[i] = pgm_read_byte(&glyphs[j]);
	}
	sei();
 }

 void display_update()
 {
	static uint8_t digit = 0;
	PORTB = 255;

	// Turn off the previous digit
	if (digit == 0)
	{
		PORTB |= (1 << 2);
	}
	else
	{
		PORTD |= 1 << (digit + 2);
	}
	// Move to the next digit
	digit = (digit + 1) & 3;

	// Turn on the new digit
#if DEBUG_DISPALY_STATE!=0
	dg_st_cnt++;
	if(dg_st_cnt & 128)
	{
#endif

		PORTB = ~dg[digit];

#if DEBUG_DISPALY_STATE!=0
	}
	else
	{
		dg_st[0]= a & d & g;
		dg_st[1]= pgm_read_byte(&glyphs[status & 0xF]);  //  display_space;
		dg_st[2]= pgm_read_byte(&glyphs[pr_state / 10]);
		dg_st[3]= pgm_read_byte(&glyphs[pr_state % 10]);;
	
		PORTB = ~dg_st[digit];
	}
#endif
	if (digit == 0)
	{
		PORTB &= ~(1 << 2);
	}
	else
	{
		PORTD &= ~(1 << (digit + 2));
	}
}
/* Routine to display non-numeric parameters and numeric parameters with ranges beyond [0...9999]	*/
/* Input:																							*/
/*		id: The index (identity) of the parameter													*/
/*		level: The numeric level or an enumeration of the displayed information						*/

 void display_level(uint8_t id, uint16_t level)
 {
	uint8_t text;
	uint32_t level32;

	/* PV_PARALLEL is processed as if...then structure because including it in the case structure makes the code crash */
	text = DISP_NONE;
	if (id == PV_PARALLEL)
	{
		level32 = 100UL * (uint32_t)level;
		text = DISP_RESISTANCE_HIGHVALUE;
	}
	else
		level32 = level;
	switch (id)
	{
#if 1
		case PV_UART_RATE:
			switch (level)
			{
				case 14400:
					text = DISP_14k4;
					break;
				case 19200:
					text = DISP_19k2;
					break;
				case 28800:
					text = DISP_28k8;
					break;
				case 38400UL:
					text = DISP_38k4;
					break;
				default:
					text = DISP_NONE;	
			}
			break;	
#endif
		case PV_PARITY:
			switch (level)
			{
				case PAR_EVEN:
					text = DISP_EVEN;
					break;
				case PAR_ODD:
					text = DISP_ODD;
					break;
				default:
					text = DISP_NOPR;
			}
			break;
		case PV_AUTOMATIC:
			switch (level)
			{
				case AUTOMATIC:
					text = DISP_AUTO;
					break;
				case MANUAL:
					text = DISP_MAN;
					break;
				default:
					text = DISP_AUTO;
			}
			break;
		case PV_ON_OFF:
			/* no break; */
		case PV_STANDBY_ENABLE:
			/* no break; */
		case PV_PAUSE_OPERATION:
			switch (level)
			{
				case ON:
					text = DISP_ON;
					break;
				case OFF:
					text = DISP_OFF;
					break;
				default:
					text = DISP_OFF;
			}
			break;
		default:
			;
		
	}
	display(level32, 0, text, true);
 }

#if 0
#define TESTMODE_PHASE1 120 // 8 ms units
#define TESTMODE_PHASE3 120	// 8 ms units
#define TESTMODE_PHASE2 240	// 8 ms units

void display_test_mode(uint16_t value)
{
	static uint16_t test_mode_counter = 0;

	test_mode_counter++;
	if (test_mode_counter == 1)
		display(0, 0, DISP_TEST, true);
	else
	if (test_mode_counter == TESTMODE_PHASE1)
		display(8888, 0, DISP_NONE, true);
	else
	if (test_mode_counter == TESTMODE_PHASE1 + TESTMODE_PHASE2)
		display(value, 0, DISP_RESISTANCE, true);
	else
	if (test_mode_counter == TESTMODE_PHASE1 + TESTMODE_PHASE2 + TESTMODE_PHASE3)
		test_mode_counter = 0;
}
#endif


/* Variables for display */
uint8_t disp_seq_ix;			/* Sequence index of message to be displayed */
uint32_t disp_prev_systime;		
uint8_t disp_char_ix;

void display_sequence_reset()
{
	disp_seq_ix = 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		disp_prev_systime = systime;
	}
}


void display_sequence()
{
	if (disp_prev_systime == 0UL)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			disp_prev_systime = systime;	// Ensure startup when systime has advanced before the first call of the function
		}
	}
	if ((disp_seq_ix > 5) || (queue[disp_seq_ix].time == 0))
		disp_seq_ix = 0;
	display(queue[disp_seq_ix].numeric, queue[disp_seq_ix].point, queue[disp_seq_ix].text, queue[disp_seq_ix].override);

	/* Check if it is time to show next text in sequence */
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		if ((systime - disp_prev_systime) > queue[disp_seq_ix].time)
		{
			disp_seq_ix++;
			disp_prev_systime = systime;
			disp_char_ix = 0;
		}
	}
}

void display_sequence_set(uint8_t index, uint16_t numeric, uint8_t point, uint8_t text, bool override, uint16_t time)
{
	queue[index].numeric = numeric;
	queue[index].point = point;
	queue[index].text = text;
	queue[index].override = override;
	queue[index].time = time;
}