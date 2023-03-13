/*
 * modbus.cpp
 *
 * Created: 2017-02-10 18:22:22
 *  Author: Ilya.Anakhov
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include "modbus.h"
#include "procvar.h"
#include "util.h"
#include "ver.h"
#include "main.h"

#define USART_RXC_vect         _VECTOR(11)
const uint8_t MaxLenghtRecBuf = 25; // UART receive data buffer size
const uint8_t MaxLenghtTrBuf  = 30;  //UART transmit data buffer size
bool StartRec = false; // false/true Start / receiving packages
bool bModBus;  // status flags



// Pause between modbus package - 1.5*(8+1+2)/bod = 1.7?? -> TCNT0~40 ??? 9600 bound rate and 8MHz CPU
// Must be 1.5 char (Modbus spec ch. 2.5.1.1)
uint8_t dTCNT0 = 40;

uint16_t diag_buscount;		// Number of all detected messages (also addressed to other slaves)
uint16_t diag_msgcount;		// Number of detected messages addressed to this device
uint16_t diag_crccount;		// Number of CRC errors
uint16_t diag_excount;		// Number of returned exceptions
uint16_t diag_retcount;		// Number of processed messages

uint8_t rx_buff[MaxLenghtRecBuf]; // UART receive data buffer
uint8_t answer[MaxLenghtTrBuf]; //Modbus UART transmit data buffer
uint8_t RcCount;				 // counter received

uint8_t cNumRcByte0;			//Aux variable for UART receive / transmit
uint8_t cNumTrByte0;			//aux variable for UART receive / transmit

uint8_t TrCount; /// transmitted data data


/* Private function declarations */
uint16_t modbus_startaddr(uint8_t *inbuf);
uint16_t modbus_querylen(uint8_t *inbuf);
uint16_t modbus_presetvalue(uint8_t *inbuf);
uint16_t modbus_get_subfunction(uint8_t *inbuf);
//uint8_t modbus_get_address();

//UART interrupt
ISR(USART_RXC_vect)
{
	StartRec = true;
	StartTimer0();

	if(RcCount<MaxLenghtRecBuf)
	rx_buff[RcCount++] = UDR; // TBD: overflow

	if (UCSRA & (1 << FE))
	return; //FE-frame error, OVR - data overload

	TCNT0 = dTCNT0; //timer0 restart
}

//	Subroutine processing pauses between modbus packages
ISR(TIMER0_OVF_vect)
{
	if (StartRec)
	{
		StartRec = false;
		cNumRcByte0 = RcCount;
		bModBus = true;
		RcCount = 0;
		TCCR0 = 0;
	}
}

// Timer0 start for Modbus RTU
inline void StartTimer0(void)
{
#if 0
	TCNT0 = dTCNT0;
	TCCR0 = 0x03;
#else
	TCCR0 = 5;	// Divide by 1024
	TCNT0 = dTCNT0;
#endif
}

// Timer0 initialization
inline void InitTimer0(void)
{
	TIMSK |= (1 << TOIE0);
}


// CRC16 calculation for Modbus packet
uint16_t GetCRC16(uint8_t* buf, uint8_t len)
{
	uint8_t i;
	volatile uint16_t CRC = 0xFFFF;
	volatile uint16_t CRC1;

	for (i = 0; i < len; i++)
	{
		CRC = _crc16_update(CRC, buf[i]);
	}

	CRC1 = CRC;

	return CRC1;
}

uint16_t modbus_rate_change(uint16_t rate, int dir)
{
	uint8_t ix = 0;

//	while ((ix < (sizeof(MODBUS_RATES) / sizeof(uint16_t)) - 1) && (rate < MODBUS_RATES[ix]))
	while ((ix < 4) && (MODBUS_RATES[ix] > rate))
		ix++;

	// Check in case of an invalid value, i.e. the highest bit rate did not match
	/*
	if (rate != MODBUS_RATES[ix])
	{
		rate = UART_RATE;
		ix = 0;
		while ((ix < sizeof(MODBUS_RATES) / sizeof(uint16_t) - 1) && (rate > MODBUS_RATES[ix]))
		ix++;
	}*/
	if ((dir >  0) && (ix < 4 /*(sizeof(MODBUS_RATES) / sizeof(uint16_t)) - 1 */))
		ix++;
	if ((dir < 0) && (ix > 0))
		ix--;
	return MODBUS_RATES[ix];
}

uint8_t modbus_get_reg(uint8_t functioncode, uint16_t address, uint16_t *outval)
{
	uint16_t value = 0;	// TODO: Redundant, place the value right away in *outval
	uint8_t exception = EX_NONE;
	

	if (functioncode == 4) // Input reg
	{
		switch (address)
		{
			case IR_RESISTANCE:
				value = pv_get_resistance();
				break;
			case IR_STATUS:
				value = pv_get_status();
				break;
			case IR_R:	
			case IR_R_ALIAS:
				value = pv_get_r();
				break;
			case IR_R5:
				value = pv_get_r5();
				break;
			case IR_OUTPUTSTAT:
				value = pv_get_output() & 0xFF;
				break;
			case IR_MODE:
				value = pv_get_state_and_mode();
				break;
			case IR_R30:
			case IR_R30_ALIAS:
				value = pv_get_r30();
				break;
			case IR_R60:
			case IR_R60_ALIAS:
				value = pv_get_r60();
				break;
			case IR_MOTOR_RUN:
				value = pv_get_motor_run_time();
				pv_reset_motor_run_time();
				break;
			case IR_DRY_FAIL:
				value = (pv_get_drying_fail() == true);
				break;
			case IR_R_STARTER:
				value = get_starter_resistance();
				break;
			case IR_HOLDTIME:
				value = pv_get_hold_time();
				break;
			case IR_COOLTIME:
				value = pv_get_cool_time();
				break;
			case IR_ADC:	// Debug register for reading out the raw ADC value
				value = getVolt();
				break;

			case IR_DEVICE_CODE:
				value = DEVICE_CODE;
				break;

			case IR_SWVERSION:
				value = VERSION;
				break;
			default: // Invalid address
				value = EX_ILLEGAL_DATA_ADDRESS;
				exception = EX_ILLEGAL_DATA_ADDRESS;
		}
	}
	else
	if (functioncode == 3) // Holding reg
	{
		if ((address >= 0x0001) && (address < 0x0000 + PV_SIZE))
		{
				value = pv_get_level(address);
		}
		else
		{
				value = EX_ILLEGAL_DATA_ADDRESS;
				exception = EX_ILLEGAL_DATA_ADDRESS;
		}

#if 0
	2017-08-26 ir - Is there any need to limit read operations to specific addresses only?
		switch (address)
		{
			//case IR_STATUS:
			//	value = pv_get_status();
			//	break;
			case HR_R_CRITICAL:
			case HR_R_BEGIN:
			case HR_R_END:
			case HR_R_PARALLEL:
			case HR_AUTOMATIC:
			case HR_ON_OFF:
			case HR_START_DELAY:
			case HR_MODBUS_ADDR:
			case HR_HALT_TIME:
			case HR_COOLING_TIME:
			case HR_DRYING_MEASUREMENT_INTERVAL:
			case HR_PAUSE_OPERATION:
				value = pv_get_level(address);
				break;
#if 0
			case HR_DOP:
				value = pv_get_dop();
				break;
#endif

			default: // Invalid address
				value = EX_ILLEGAL_DATA_ADDRESS;
				exception = EX_ILLEGAL_DATA_ADDRESS;
		}
#endif
	}
	else
	if (functioncode == 8) // Diag
	{
		switch (address)
		{
			case DG_MSGCOUNT:
				value = diag_msgcount;
				break;
			case DG_CRCCOUNT:
				value = diag_crccount;
				break;
			case DG_EXCOUNT:
				value = diag_excount;
				break;
			default: // Invalid address
				value = EX_ILLEGAL_DATA_ADDRESS;
				exception = EX_ILLEGAL_DATA_ADDRESS;
		}
	}

	*outval = value;
	return exception;
}

int8_t modbus_set_reg(uint8_t functioncode, uint16_t address, uint16_t value)
{
	int8_t exception = EX_NONE;
	/* uint32_t fc_addr; */

	/* Only register write co  functions are accepted */
	if(functioncode != 0x06)
	{
		exception = EX_ILLEGAL_FUNCTION;
	}
	else
	{
		/* 1. Handle special cases */
		/* FACTORY_RESET is not written to memory. Do immediate action */
		if (address == HR_FACTORY_RESET)
		{
			if(value == DEVICE_RESET_MAGIC_NUMBER)
			{
				while(1);		//Let watchdog bite
			}
			
			
			if(value == FACTORY_RESET_MAGIC_NUMBER)
			{
				revert_eeprom_to_defaults();
				while(1);		//Let watchdog bite
			}
		}
		


		
		
		
		// Case 1: "Level" variables
		else if ((address >= 0x0000) && (address < 0x0000 + PV_SIZE))
		{
			pv_set_level(address, value);
		}
 
		/* 2017-08-21 ir - cases below are handled by 'default' handling 
		else // Case 2:Set modbus addr. PL 17.8.2017 Added
		if (fc_addr == HR_MODBUS_ADDR)
		{
			pv_set_level(PV_MODBUS_ADDR, value);		
		}else if (fc_addr == HR_HALT_TIME)
		{
			pv_set_level(PV_HALT_TIME, value);
		}else if (fc_addr == HR_COOLING_TIME)
		{
			pv_set_level(PV_COOLING_TIME, value);
		}
		*/ 
#if 0
		else // Case 2: Register d_op
		if (fc_addr == PR_DOP)
		{
			pv_set_dop(value);	
		}
#endif
		else // Error, unknown address
		{
			exception = EX_ILLEGAL_DATA_ADDRESS;
		}
	}
	return exception;
}


uint8_t modbus_read_register(uint8_t *inbuf, uint8_t *outbuf, uint8_t *len)
{
	uint16_t ix = 0;
	uint8_t reg, first, count, exception;
	uint16_t outval;
	uint8_t functioncode;
	
	functioncode = inbuf[1];
	first = modbus_startaddr(inbuf);
	count = modbus_querylen(inbuf);
	outbuf[ix++] = inbuf[0];	// Slave address
	outbuf[ix++] = inbuf[1];	// Function code
	outbuf[ix++] = Low(2 * count);					// Skip the response length field
	exception = EX_NONE;
	reg = first;
	do {
		exception = modbus_get_reg(functioncode, reg, &outval);
		if (exception == EX_NONE)
		{
			outbuf[ix++] = Hi(outval);
			outbuf[ix++] = Low(outval);
		} 	
	} while ((exception == EX_NONE) && (++reg < first + count));
	//ix += 2 * count;
	if (exception != EX_NONE)  // Error, build an exception response
	{
		outbuf[1] |= 0x80;	// Exception flag
		outbuf[2] = exception;
		ix = 3;	// Slave address + function code + exception
	}
	*len = ix;	// The number of bytes appended to the output buffer
	return exception;
}

uint8_t modbus_preset_register(uint8_t *inbuf, uint8_t *outbuf, uint8_t *len)
{
	uint8_t ix = 0;
	uint8_t exception;
	
	exception = modbus_set_reg(inbuf[1], modbus_startaddr(inbuf), modbus_presetvalue(inbuf));
	if (exception == EX_NONE) // Ok, build a normal response
	{
		for (ix = 0; ix < 6; ix++)
			outbuf[ix] = inbuf[ix];	// The response is a copy of the request
	}
	else // Error, build an exception response
	{
		outbuf[0] = inbuf[0];
		outbuf[1] = inbuf[1] | 0x80;	// Exception flag
		outbuf[2] = exception;
		ix = 3;	// Slave address + function code + exception
	}
	*len = ix;	// The number of bytes appended to the output buffer
	return exception;
}

uint8_t modbus_echo_request(uint8_t *inbuf, uint8_t *outbuf)
{
	uint8_t ix;
	for (ix = 0; ix < cNumRcByte0 - 2; ix++) // -2 as the CRC is not echoed but recalculated
	outbuf[ix] = inbuf[ix];
	return ix;
}

uint8_t modbus_insert(uint8_t *outbuf, uint16_t value, uint8_t ix)
{
	outbuf[ix++] = Hi(value);
	outbuf[ix++] = Low(value);
	return ix;
};

uint8_t modbus_diag(uint8_t *inbuf, uint8_t *outbuf, uint8_t *len)
{
	uint8_t ix = 0;
	uint8_t exception;
	uint16_t subf;
	
	exception = EX_NONE;
	outbuf[ix++] = inbuf[0];	// Slave address
	outbuf[ix++] = inbuf[1];	// Function code
	outbuf[ix++] = inbuf[2];	// Subfunction code H
	outbuf[ix++] = inbuf[3];	// Subfunction code L
	subf = modbus_get_subfunction(inbuf);
	switch (subf)
	{
		case 0x00:	// Return query data
			ix = modbus_echo_request(inbuf, outbuf);
			break;
		case 0x01:	// Restart communications Option
			modbus_init(inbuf[4] == 0xFF);	// Clear all counters if the data field is FF 00
			ix = modbus_echo_request(inbuf, outbuf);
			break;
		//case 0x02:	// Return diagnostic register
		case 0x0B:	// Return bus message count
			ix =  modbus_insert(outbuf, diag_buscount, ix);
			break;
		default:
			exception = EX_ILLEGAL_FUNCTION;
	}
	*len = ix;
	return exception;
}

uint8_t modbus_exception(uint8_t exceptioncode, uint8_t functioncode, uint8_t *outbuf)
{
	diag_excount++;	// Increment the diagnostic counter
	outbuf[0] = pv_get_level(PV_MODBUS_ADDR);
	outbuf[1] = functioncode | 0x80;
	outbuf[2] = exceptioncode;
	return 3;	// Length of the exception response
}

void modbus_diag_inc_buscount()
{
	diag_buscount++;
}

void modbus_diag_inc_msgcount()
{
	diag_msgcount++;
}

void modbus_diag_inc_retcount()
{
	diag_retcount++;
}



void modbus_diag_clear_counters()
{
	diag_buscount = 0;
	diag_msgcount = 0;
	diag_crccount = 0;
	diag_excount = 0;
}

// USART initialization
void USART_Init(uint16_t bitrate, uint16_t parity)
{
	uint16_t ubrr;
	//if (parity != PAR_NONE)
	parity = (4 - parity) & 0x03; // 0 (none) -> 0b00, 1 (odd) -> 0b11, 2 (even) -> 0b10
	if (bitrate == 0)	// Calculate from the saved value
	{
		bitrate = pv_get_level(PV_UART_RATE);
	}
	ubrr = F_CPU / (uint32_t)(16 * (uint32_t)bitrate) - 1;
	UBRRH = Hi(ubrr);
	UBRRL = Low(ubrr);
	UCSRA = 0x00;		// Double speed off, multi-process communication mode off
	UCSRB = 0x98;		// Receive complete interrupt enable, receiver enable, transmitter enable, 8 data bits
	UCSRC = 0b10000110 | (parity << 4);	// UCSRC select, UART mode, 8 data bits, 1 stop bit, parity as selected  
	
	// Select the maximum packet transfer time based on bit rate
	switch (bitrate)
	{
		case 9600:
			dTCNT0 = 255 - 13;
			break;
		case 14400:
			dTCNT0 = 255 - 9;
			break;
		case 19200:
			dTCNT0 = 255 - 7;
			break;
		case 28800:
			dTCNT0 = 255 - 4;
			break;
		case 38400:
			dTCNT0 = 255 - 3;
			break;
		default:
			dTCNT0 = 108;
	}
	 
	InitTimer0();
	StartTimer0();
}

void modbus_init(bool clear_counters)
{
	if (clear_counters)
		modbus_diag_clear_counters();
	USART_Init(pv_get_level(PV_UART_RATE), pv_get_level(PV_PARITY));
}

uint16_t modbus_startaddr(uint8_t *inbuf)
{
	return ((inbuf[2] << 8) + inbuf[3]);
}

uint16_t modbus_querylen(uint8_t *inbuf)
{
	return ((inbuf[4] << 8) + inbuf[5]);
}

uint16_t modbus_presetvalue(uint8_t *inbuf)
{
	return ((inbuf[4] << 8) + inbuf[5]);
}

uint16_t modbus_get_subfunction(uint8_t *inbuf)
{
	return ((inbuf[2] << 8) + inbuf[3]);
}

uint8_t modbus_handler(uint8_t cNumRcByte0, uint8_t *rx_buff, uint8_t *answer) //modbus answer data
{
	uint16_t tempi;
	uint8_t exception;
	uint8_t len = 0;
  
	modbus_diag_inc_buscount(); // Count all the messages that traverse on the bus, regardless of the slave address
  
	// Check if the message is addressed to us
	if (rx_buff[0] != pv_get_level(PV_MODBUS_ADDR))
	{
		return 0;	// Slave address mismatch, don't reply
	}
	modbus_diag_inc_msgcount();	// Count the messages addressed to us
	if (cNumRcByte0 < 8)			// Check the packet length (TDB: Should the condition be "== 8" instead of "< 8"?
	{
		return modbus_exception(EX_ILLEGAL_DATA_VALUE, rx_buff[1], answer);
    //return ModbusError(ModbusIllegalDataValue); // All message have at least eight bytes
	}
  
	tempi = GetCRC16(rx_buff, cNumRcByte0 - 2);
	if (rx_buff[cNumRcByte0-2] != Low(tempi) || rx_buff[cNumRcByte0-1] != Hi(tempi))
		return 0; // CRC error, ignore
	len = 0;
	// Function codes

  switch (rx_buff[1])
  {
    case 0x03:	// Read Holding Registers
		exception = modbus_read_register(rx_buff, answer, &len);
		break;
	case 0x04:	// Read Input Registers
		exception = modbus_read_register(rx_buff, answer, &len);
		break;
    case 0x06:	// Preset Single Register
		exception = modbus_preset_register(rx_buff, answer, &len);
/*		if (exception == EX_NONE)
		{
			drying_disable();
		}
		*/
		break;
	case 0x08:	// Diagnostics
		exception = modbus_diag(rx_buff, answer, &len);
		break;
	default:
		len = modbus_exception(EX_ILLEGAL_FUNCTION, rx_buff[1], answer);
  }
  tempi = GetCRC16(answer, len);
  
  answer[len++] = Low(tempi);
  answer[len++] = Hi(tempi);		
  modbus_diag_inc_retcount();	// Increment the returned messages counter
  return len;					// Transmission starts in main() if the return value is greater than zero
}

void StartTrans0(void)
{
	TrCount = 0;
#if 1
	cli();
	while (TrCount < cNumTrByte0)
	{
		while(!(UCSRA & (1 << UDRE)));
		UDR = answer[TrCount];
		TrCount++;
	}
	sei();
#else
	GoTrans0();
#endif
}

void modbus_check()
{
	if (bModBus)
	{
		cNumTrByte0 = modbus_handler(cNumRcByte0, rx_buff, answer);
		if (cNumTrByte0 != 0)
			StartTrans0();
	}
	bModBus = false;
}