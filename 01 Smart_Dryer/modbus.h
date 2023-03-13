/*
 * modbus.h
 *
 * Created: 2017-02-10 18:22:49
 *  Author: Ilya.Anakhov
 */ 


#ifndef MODBUS_H_
#define MODBUS_H_

#include "procvar.h"

#define UART_RATE		9600UL
#define MODBUS_ADDRESS	32
#define CRC_POLYNOMIAL	0x8005

#define DG_MSGCOUNT		0x000B
#define DG_CRCCOUNT		0x000C
#define DG_EXCOUNT		0x000D

#define IR_R_STARTER	0x0004 //2017-08-23 IR - added
#define IR_HOLDTIME		0x0005 //PL 17.8.2017 Changed from 0x22
#define IR_COOLTIME		0x0006 //PL 17.8.2017 Changed from 0x23
#define IR_RESISTANCE	0x0007
#define IR_STATUS		0x0008
#define IR_R			0x0009
#define IR_R30			0x000A
#define IR_R60			0x000B
#define IR_MOTOR_RUN	0x000C
#define IR_R5			0x000D
#define IR_OUTPUTSTAT	0x000E
#define IR_MODE			0x000F
#define IR_R_ALIAS		0x0018
#define IR_R60_ALIAS	0x0019
#define IR_R30_ALIAS	0x0020
#define IR_DRY_FAIL		0x0010
#define IR_ADC			0x0080
#define IR_SWVERSION	0x0088
#define IR_DEVICE_CODE	0x0089

enum HOLDINGVAR 
{	
	HR_R_CRITICAL		= PV_R_CRITICAL,
	HR_R_BEGIN			= PV_R_START,
	HR_R_END			= PV_R_END,
	HR_R_PARALLEL		= PV_PARALLEL,
	HR_AUTOMATIC		= PV_AUTOMATIC,
	HR_ON_OFF			= PV_ON_OFF,
	HR_START_DELAY		= PV_START_DELAY,
	HR_STANDBY_ENABLE	= PV_STANDBY_ENABLE,
	HR_STANDBY_ONTIME	= PV_STANDBY_ONTIME,
	HR_STANDBY_OFFTIME	= PV_STANDBY_OFFTIME,
	HR_MODBUS_ADDR		= PV_MODBUS_ADDR,
	HR_UART_RATE		= PV_UART_RATE,
	HR_PARITY			= PV_PARITY,
	HR_HALT_TIME		= PV_HALT_TIME,
	HR_COOLING_TIME		= PV_COOLING_TIME,
	HR_FACTORY_RESET	= PV_FACTORY_RESET,
	HR_DRYING_MEASUREMENT_INTERVAL	= PV_DRYING_MEASUREMENT_INTERVAL,
	HR_PAUSE_OPERATION	= PV_PAUSE_OPERATION,
	HR_RPAR_OFFSET		= PV_RPAR_OFFSET,
#if 0
	,
	HR_STATUS			= 0x0008,
	HR_DOP				= 0x0010
#endif
};
#define PR_DOP			0x0010


enum {EX_NONE = 0, EX_ILLEGAL_FUNCTION , EX_ILLEGAL_DATA_ADDRESS, EX_ILLEGAL_DATA_VALUE, EX_SLAVE_DEVICE_FAILURE, EX_ACK, EX_SLAVE_DEVICE_BUSY, EX_NACK, EX_MEMORY_PARITY_ERROR, EX_GATEWAY_PATH_UNAVAILABLE, EX_GATEWAY_NO_RESPONSE};

const uint16_t MODBUS_RATES[5] = {9600, 14400, 19200, 28800, 38400U};

enum MODBUS_PARITY {PAR_NONE, PAR_ODD, PAR_EVEN, PAR_ALTERNATIVES};

uint8_t modbus_read_register(uint8_t functioncode, uint8_t *inbuf, uint8_t *outbuf, uint8_t *len);
uint8_t modbus_preset_register(uint8_t *inbuf, uint8_t *outbuf, uint8_t *len);
uint8_t modbus_diag(uint8_t *inbuf, uint8_t *outbuf, uint8_t *len);
uint16_t modbus_rate_change(uint16_t rate, int dir);
void modbus_diag_inc_buscount();
void modbus_diag_inc_msgcount();
void modbus_diag_inc_counters(uint8_t *outbuf, uint8_t len);
void modbus_init(bool clear_counters);
uint8_t modbus_handler(uint8_t cNumRcByte0, uint8_t *rx_buff, uint8_t *answer);
void modbus_check();


#endif /* MODBUS_H_ */