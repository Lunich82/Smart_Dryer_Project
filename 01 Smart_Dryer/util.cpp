/*
 * util.cpp
 *
 * Created: 2017-02-10 18:37:07
 *  Author: Ilya.Anakhov
 */ 

#include <avr/io.h>


 uint8_t Hi(uint16_t Int)
{
	return (uint8_t)(Int>>8);
}

// #define Low(Int) (uint8_t) (Int)
 uint8_t Low(uint16_t Int)
{
	return (uint8_t) Int;
}

