/*
 * ver.h
 *
 * Created: 2017-02-10 22:46:10
 *  Author: Ilya.Anakhov
 */ 

#ifndef VER_H_
#define VER_H_

#define VER107V 0

#define VERSION			0x1106;
#define F_CPU			8000000UL

/* Device code for modbus */
#if (VER107V != 0)
	#define DEVICE_CODE		2
#else
	#define DEVICE_CODE		1
#endif


#endif /* VER_H_ */