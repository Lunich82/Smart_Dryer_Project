/*
 * procvar.h
 *
 * Created: 2017-02-10 19:07:15
 *  Author: Ilya.Anakhov
 */ 


#ifndef PROCVAR_H_
#define PROCVAR_H_

#define ST_DRYING				0
#define ST_MOTOR_RUNNING		1
#define ST_FAIL					2
#define ST_CRITICAL_RESISTANCE	3
#define ST_MAIN_CONTACTOR_ON	4
#define ST_RANGE2				5
#define ST_TEST_MODE			6
#define ST_R3060				8
#define ST_MOTOR_READY			9
#define ST_DECREASED_R			10
#define ST_STANDBY_ACTIVE		11
#define ST_MANUAL_MODE			12
#define ST_PARALLEL_R			13
#define ST_DRYING_FAIL			14
#define ST_DRYING_REQUIRED		15

#define AUTOMATIC				0
#define MANUAL					1
#define OFF						0
#define ON						1

#define FACTORY_RESET_MAGIC_NUMBER		121
#define DEVICE_RESET_MAGIC_NUMBER		0x6bc8

//#define DEVICE_CODE				1 //Device code for modbus

uint16_t pv_get_resistance();
uint16_t pv_get_r();
uint16_t pv_get_r5();
uint16_t pv_get_r30();
uint16_t pv_get_r60();
uint16_t pv_get_status();
uint16_t pv_get_output();
uint16_t pv_get_state_and_mode(void);
uint16_t pv_get_rparallel();
uint16_t get_starter_resistance(void);

#if 0
bool pv_get_dop();
#endif
bool pv_set_level(uint16_t address, uint16_t value);
uint16_t pv_get_level(uint16_t address);
void pv_set_dop(uint16_t value);

void InitTimer0();
void StartTimer0();
void drying_disable(void);
uint16_t getVolt(/*int num_enter*/);
bool pv_is_ui_active();
uint16_t pv_get_motor_run_time();
bool pv_get_drying_fail();
void pv_reset_motor_run_time();
bool is_motor_energized();
bool is_status(uint8_t bit);
bool pv_get_drying_fail();
uint16_t pv_get_hold_time();
uint16_t pv_get_cool_time();

enum PROCVAR {
	PV_STARTER,			/*  0 */
	PV_R_CRITICAL,		/*  1 */
	PV_R_START,			/*  2 */
	PV_R_END,			/*  3 */
	PV_PARALLEL,		/*  4 */
	PV_AUTOMATIC,		/*  5 */
	PV_ON_OFF,			/*  6 */
	PV_START_DELAY,		/*  7 */
	PV_STANDBY_ENABLE,	/*  8 */
	PV_STANDBY_ONTIME,	/*  9 */
	PV_STANDBY_OFFTIME,	/* 10 */
	PV_MODBUS_ADDR,		/* 11 */
	PV_UART_RATE,		/* 12 */
	PV_PARITY,			/* 13 */
	PV_HALT_TIME,		/* 14 */
	PV_COOLING_TIME,	/* 15 */
	PV_FACTORY_RESET,	/* 16 */
	PV_DRYING_MEASUREMENT_INTERVAL,	/* 17 */
	PV_PAUSE_OPERATION,	/* 18 */
	PV_RPAR_OFFSET,		/* 19 */
	PV_LAST_MEASURED,	/* 20 */
	PV_REGISTER_RPAR,	/* 21 */
	
	PV_SIZE,			/* 22 */
};
/* 2017-08-24 ir - Manual pause feature not required. Therefore menu selection is hidden. */
#define PV_LAST_IN_UI	PV_PAUSE_OPERATION
enum {UI_INITIAL, UI_TEST, UI_PRM_INDEX, UI_PRM_VALUE, UI_DONE};

#endif /* PROCVAR_H_ */