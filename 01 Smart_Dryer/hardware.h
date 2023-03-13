/*
 * hardware.h
 *
 * Created: 2017-03-02 10:20:18
 * edited: 2017-09-04 
 *  Author: hannu.saari
 *  Author: Ilya Anakhov
 
 */ 
#include "ver.h"

#ifndef HARDWARE_H_
/* Constants used for calculating the resistance from the ADC value */
/* These depend on the resistor dividers in the hardware			*/
/* See separate documentation for determining the values			*/

#ifndef VER107V 
#error VER107 must be defined to either 0 or 1
#endif
#if (VER107V != 0)
	/* Range 1: Rx = 81 kohm											*/
	const uint32_t a1 = 1704255L;
	const int b1 = 0;
	const int c1 = -278;

	/* Range 2: Rx = 81 kohm || 16.2 kohm = 13.5 kohm					*/
	const uint32_t a2 = 282904L;
	const int b2 = 0;
	const int c2 = -49;

#else
	/* Range 1: Rx = 81 kohm											*/
	const uint32_t a1 = 820000L;
	const int b1 = 7;
	const int c1 = -100;

	/* Range 2: Rx = 81 kohm || 16.2 kohm = 13.5 kohm					*/
	const uint32_t a2 = 128000L;
	const int b2 = 0;
	const int c2 = -20;
#endif

/* 2017-08-21 ir - Test measurement has different range.			*/
/*		In test mode Rx = 20 kohm									*/
/*		No modifications to code now. It is enough to have a		*/
/*		constant reading in test mode								*/
/* Range test: Rx = 20 kohm (test measurement)						*/

// Switchover point from range 1 to range 2 (raw ADC value, not resistance)
#define RANGE_SWITCHOVER_LOW	590

// Switchover point from range 2 to range 1 (raw ADC value, not resistance)
#if (VER107V != 0)
	#define RANGE_SWITCHOVER_HIGH	100
#else
	#define RANGE_SWITCHOVER_HIGH	73
#endif
// Internal "test resistor", resistance in kilo-ohms
const int r_internal = 300;

// Allowed tolerance in kilo-ohms for accepting the self-test
const int r_internal_tolerance_low = 50;
const int r_internal_tolerance_high = 250;

#if (VER107V != 0)
	#define R_CEILING		60000
#else
	#define R_CEILING		20000
#endif

// Minimum difference between the start and end values of the motor resistance
const uint16_t R_DIFF_MIN = 400;

// A long drying cycle must increase the motor resistance by at least R_MINIMUM_RISE kohms. If not, an alert is given.
#define R_MINIMUM_RISE	100

#define HARDWARE_H_
#endif /* HARDWARE_H_ */