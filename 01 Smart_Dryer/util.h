/*
 * util.h
 *
 * Created: 2017-02-10 18:36:53
 *  Author: Ilya.Anakhov
 */ 


#ifndef UTIL_H_
#define UTIL_H_
#define SetBit(Port,bit) Port|=(1<<bit)
#define ClrBit(Port,bit) Port&=~(1<<bit)
#define InvBit(Port,bit) Port^=(1<<bit)

uint8_t Hi(uint16_t Int);
uint8_t Low(uint16_t Int);


#endif /* UTIL_H_ */