/*
 * display.h
 *
 * Created: 2017-02-11 21:51:55
 *  Author: Ilya.Anakhov
 */ 


#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "hardware.h"

#define DISP_MIN_KOHMS	100
#define DISP_MAX_KOHMS	9999		/* Switch over to megaohm display if the reading is above the limit (in kilo-ohms) */
#define DISP_MAX_TENKOHMS	99999UL
#define DISP_MAX_MOHMS	((uint32_t)(R_CEILING))		/* Display "High" if the resistance is above the limit (in kilo-ohms) */
		
typedef struct {
	uint16_t numeric;
	uint8_t point;
	uint8_t text;
	bool override;
	uint16_t time;
} Sequence;

// Enumerated constants for referring to the string constants. An exception is the DISP_RESISTANCE that is an indicator of displaying a resistance reading that requires special handling */
enum {DISP_NONE, DISP_TEST, DISP_DONE, DISP_FAIL, DISP_P00, DISP_ON, DISP_STBY, DISP_IR, DISP_COOL, DISP_HALT, DISP_HIGH, DISP_LOW, DISP_DRY, DISP_INTR, DISP_ERR, DISP_GOOD, DISP_EVEN, DISP_ODD, DISP_NOPR, DISP_AUTO, DISP_MAN, DISP_OFF, DISP_HAND, DISP_RESET, DISP_HEAT, DISP_PAUS, DISP_REQ,  DISP_FOR, DISP_14k4, DISP_19k2, DISP_28k8, DISP_38k4, DISP_RESISTANCE, DISP_RESISTANCE_HIGHVALUE, DISP_SEC };

void display(uint32_t numeric, uint8_t dot, uint8_t text, bool override);
void display_level(uint8_t id, uint16_t level);
//void display_test_mode(uint16_t value);
//void display_hold_mode(uint16_t r, uint16_t k);
void display_dry_mode(uint16_t r);
void display_sequence();
void display_sequence_set(uint8_t index, uint16_t numeric, uint8_t point, uint8_t text, bool override, uint16_t time);
void display_sequence_reset();
void display_update();

#endif /* DISPLAY_H_ */