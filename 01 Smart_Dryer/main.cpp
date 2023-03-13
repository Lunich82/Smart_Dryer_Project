/* NOTE! General notes about the code.
2017-08-24 
	Code required some re-write.
	ISR handlers can be simplified and functions moved to main loop.
	Functions called from main loop must be checked against time consuming operations.
	If there is time consuming operations consider splitting to several steps.
	For example getResistance() is quite time consuming - lot of samples and even 300 ms delay. At least the delay is better to handle using separated steps in state machine.
	In one place getResistance() is called from ISR!
	If main loop runs smooth enough it is possible to move some UI operations from ISR to main loop.
	Variables longer than 1 byte that are updated in ISR need to be interrupt protected in other parts of code.
	Some variables are now fixed - like 'time' that is now 'time1s' and 'systime'.


2017-08-26 
	Idea to 'level' editor.
	Store and edit current 'level'/'P' value in a editing variable.
	This makes possible to cancel editing using 'Test' button.
	Also process is not affected before 'Enter' is pressed. Now all edit go direct to process use. 
	
2017-08-27 
	Basic structure of 'process' state machine mixes state machine features and measurement.
	Measurement makes many decisions about next state.
	More clear is if measurement just measures and then returns to state that initiated the measurement.
	After return original step can make decision based to measurements and other relevant data.
	Another issue is two different measurement methods. Is there really a need for them both.  
*/

/* # define FAST_TEST */

#ifndef F_CPU
#define F_CPU			8000000UL
#endif
#include "ver.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "modbus.h"
#include "procvar.h"
#include "display.h"
#include "util.h"
#include "hardware.h"

#ifdef FAST_TEST
#define SECINMIN            6
#define MININH              6
#define TICKS_PER_SECOND	24
#else
#define SECINMIN            60
#define MININH              60
#define TICKS_PER_SECOND	244
#endif

#define RANGE_SWITCH_TIME ((TICKS_PER_SECOND * 3) / 10)

#define SetBit(Port,bit) Port|=(1<<bit)
#define ClrBit(Port,bit) Port&=~(1<<bit)
#define InvBit(Port,bit) Port^=(1<<bit)

void read_from_eeprom(void);
void save_to_eeprom(void);
void revert_eeprom_to_defaults();
void init(void);
void StartTrans0(void);
void relays_set(uint8_t state);
void test(bool init_test_state);
bool getResistance(uint8_t show);
#if 0
void getR3060(void);
#endif
uint16_t convertResult(int16_t result);
void drying_start(uint8_t next_state);
void set_state(uint8_t state);
uint8_t is_drying_relay_allowed(void);
bool is_standby_heat(void);

volatile uint16_t time1s_system;		/* Continuously counting time 1s/bit */

volatile uint16_t process_state_time1s;	/* Process state running time. Not incremented when PAUSE mode activated. */
volatile uint16_t drying_time1s;		/* Cleared when drying is started.  Not incremented when PAUSE mode activated. */
volatile uint16_t standby_time1s;		/* Cleared when standby is started.  Not incremented when PAUSE mode activated. */
volatile uint16_t measure_time1s;		/* Cleared when measure is completed.  Not incremented when PAUSE mode activated. */

volatile uint8_t  process_in_pause;		/* Indicates when process is paused by MODbus command */

volatile uint16_t motor_run_time;	// Cumulative run time of the motor. Resets only upon read via the Modbus
volatile uint8_t fail_time;			// Counter to determine how long the "FAIL" message is displayed
volatile uint16_t hold_time=1;
volatile uint16_t cool_time=1;
//volatile uint8_t G_key_pressed;		// A status variable for reading the keys
uint32_t mbuff, mj; // buffers

uint16_t real_voltage;
uint16_t status; // Contains all main status flags for modbus transfer
uint16_t outputon; // Contains all output status flags for modbus transfer
uint16_t error_bits;
enum ERROR_MASKS { ERR_PARALLEL_RESISTANCE = 0x0001 };
	
bool d_op; // device status flags
bool critical_resistance;
bool d_phase;
bool drying_fail;
volatile uint8_t ui_state = UI_INITIAL;
volatile uint16_t G_enter_pressed;
volatile uint16_t G_test_pressed;
volatile uint32_t systime;
uint16_t noise;
uint8_t range;

uint8_t pending_reset_request;

#define RES_PARALLEL_MAX_ERROR 500

#define KEY_ENTER_PIN	PINC
#define KEY_UP_PIN		PIND
#define KEY_DOWN_PIN	PIND
#define KEY_TEST_PIN	PINA
#define KEY_ENTER_BIT	0
#define KEY_UP_BIT		6
#define KEY_DOWN_BIT	7
#define KEY_TEST_BIT	7

enum {KEY_NONE, KEY_UP, KEY_DOWN, KEY_UPDOWN, KEY_ENTER, KEY_TEST = 8};
enum {OUT_NONE, OUT_MAIN, OUT_RUNNING, OUT_READY, OUT_RANGE2, OUT_TEST, OUT_DRYING, OUT_AUX, OUT_COUNT};
enum PR_STATES {PR_START, PR_HALT_AFTER_DRY, PR_HALT_AFTER_RUN, PR_MEASURE, PR_HOLD, PR_NEXT_CYCLE, PR_DRYING, PR_MOTOR_READY, PR_MOTOR_START, PR_MOTOR_RUN, PR_MOTOR_WAIT, PR_FAIL, PR_TEST, PR_STANDBY_HEAT, PR_STANDBY_PAUSE, PR_R_PARALLEL_SET, PR_R_PARALLEL_ERROR, PR_MEASURE_20S_AVG, PR_MEASURE_20S_AVG_RUN, PR_PAUSE };

enum PR_MODES 
{
	MODE_NONE = 0, 
	MODE_IDLE = 1,		/* No heating or cooling active */
	MODE_DRY = 2,		/* Drying active */
	MODE_HEAT = 4,		/* Heating active */
	MODE_COOL = 8,		/* Cooling active */
	MODE_PAUSE = 64,	/* Pause operation active */
	MODE_MOTOR = 128,	/* Motor cycle active */
};
#define MODE_OVERRIDES (MODE_MOTOR|MODE_PAUSE)
#define MODE_MODAL (MODE_IDLE|MODE_DRY|MODE_HEAT|MODE_COOL)

uint8_t pr_state = PR_START;
uint8_t pr_sub_state;

uint8_t pr_mode_modal;
uint8_t pr_mode_override;

/* Only one modal mode can be active.
   Override modes can be active with otehr modes. 
*/
uint8_t pr_mode_set(uint8_t new_mode, bool mode_state)
{
	
	uint8_t mode_z = pr_mode_modal | pr_mode_override;
	if( (new_mode & MODE_OVERRIDES) != 0 )
	{
		if(mode_state != false)
		{
			pr_mode_override |= new_mode;
		}
		else
		{
			pr_mode_override &= ~new_mode;
		}
	}
	else
	{
		if(mode_state == false)
		{
			if(pr_mode_modal == new_mode)
				pr_mode_modal = MODE_NONE;
		}
		else
		{
			pr_mode_modal = new_mode;		
		}
	}
	
	return ((pr_mode_modal | pr_mode_override) != mode_z);
}


uint8_t pr_mode_get( void )
{
	if(pr_mode_override & MODE_MOTOR)
	{
		return MODE_MOTOR;
	}
	if(pr_mode_override & MODE_PAUSE)
	{
		return MODE_PAUSE;
	}
	return pr_mode_modal;
}

void set_status(uint8_t bit, bool state)
{
	if (state)
		status |= (1 << bit);
	else
		status &= ~(1 << bit);

}

void output_set(uint8_t output, bool state)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if ((output > OUT_NONE) && (output < OUT_COUNT))
		{
			if (state)
			{
				outputon |= (1 << output);
				SetBit(PORTC, output);
			}
			else
			{
				/* 2017-08-21 ir - target variable changed to 'outputon'. The line was: status &= ~(1 << output); */
				outputon &= ~(1 << output);
				ClrBit(PORTC, output);
			}
		}
		if (output == OUT_MAIN)
		{
			set_status(ST_MAIN_CONTACTOR_ON, state);
		} 
		else if( output == OUT_READY )
		{
			set_status(ST_MOTOR_READY, state);
		}
	}
}


bool is_status(uint8_t bit)
{
	return (status & (1 << bit));
}

bool pv_is_ui_active()
{
	return (ui_state != UI_INITIAL);
}

volatile uint16_t  level[PV_SIZE]; // contains basic settings of device. Initialize from EE_level on startup (Should be in EEPROM in next version of Firmware.)

const uint16_t level_defaults[PV_SIZE] {
	0,				//  0 Measured parallel resistance
	1000,			//  1 Setup value of Critical resistance
	4000,			//  2 Setup value of beginning Drying process
	8000,			//  3 Setup value of Ending Drying process
	0,				//  4 Parallel resistance (use 0 for infinite)
	AUTOMATIC,		//  5 Automatic drying
	ON,				//  6 Use "On" as default with automatic
	0,			//  7 Motor start-up delay in milliseconds
	OFF,			//  8 Stand-by warming
	15,			//  9 Default stand-by on time in minutes
	5,				// 10 Default stand-by off time in minutes
	MODBUS_ADDRESS,	// 11 ModBus address setup
	UART_RATE,
	PAR_NONE,
	29,				// 14 Default pause between measurements in minutes
	15,				// 15 Default cooling time after motor run in minutes
	100,			// 16 Default value in the factory reset menu
	5,				// 17 Default drying measurement interval
	0,				// 18 Default pause operation
	100,			// 19 Default RPAR offset
	1000,			// 20 "Last measured" value - not really a default value
	0,				// 21 "Parallel resistance measurement flag" - not a real 'level' parameter, but something required on EE
	};

const int EEPROM_FILL = 4;	// Number of dummy words at the start of the EEPROM space

uint16_t EEMEM EE_level[PV_SIZE+EEPROM_FILL] {
	0x0100,0x0302,0x0504,0x0706, // Insert some dummy bytes to prevent corruption of EEPROM data
	0,			// Measured parallel resistance
	1000,		// Setup value of Critical resistance
	4000,		// Setup value of beginning Drying process
	8000,		// Setup value of Ending Drying process
	0,			// Parallel resistance (use 0 for infinite)
	AUTOMATIC,	// Automatic drying
	ON,			// Use "On" as default with automatic
	0,		// Motor start-up delay in milliseconds
	OFF,		// Stand-by warming
	15,		// Default stand-by on time in minutes
	5,			// Default stand-by off time in minutes
	MODBUS_ADDRESS,	// ModBus address setup
	UART_RATE,
	PAR_NONE,
	29,			// Default pause between measurements in minutes
	15,			// Default cooling time after motor run in minutes
	100,		// Default value in the factory reset menu
	5,			// Default drying measurement interval (minutes)
	0,			// Default pause operation
	100,        // Default Rpar offset
	1000,		// "Last measured" value - not really a default value	case
	0,			// "Parallel resistance measurement flag - not a real 'level' parameter, but something required on EE
	};

const int EEPROM_LEN  = sizeof(EE_level) / sizeof(uint16_t);
 
bool level_changed;
volatile uint16_t resistance; // instantaneous value of winding resistance.
volatile uint16_t resistance_display; // resistance to display and modbus.
uint16_t measure_while_drying_cnt;	// Measurement count for average resistance
uint32_t measure_while_drying_sum;	// Measurement sum for average resistance

#define MEASURE_SAMPLE_COUNT 16
#if ( (MEASURE_SAMPLE_COUNT == 0) || ((MEASURE_SAMPLE_COUNT & (MEASURE_SAMPLE_COUNT - 1)) != 0) )
#error MEASURE_SAMPLE_COUNT must be power of 2
#endif
uint16_t measure_sample_index;
uint16_t measure_sample_buffer[MEASURE_SAMPLE_COUNT];
uint16_t pr_measure_display_z;
#define MEASURE_DIPLAY_TIME 3

uint16_t buff;
uint16_t  m; // main time couter,increase per unit of about 5 seconds;
// uint16_t  mt;
int  k; // display value of m
uint16_t   mi;
uint16_t  r5; // the  value of winding resistance after 5 seconds
uint16_t  r30; // the  value of winding resistance after 30 seconds
uint16_t  r60; // the value of winding resistance after 60 seconds
uint16_t  r_initial; // Resistance at the beginning of a drying cycle
#if 0
uint16_t  rtime[80]; // Resistance measurements. TBD: use a const to 120
#endif
uint16_t  r; //  the average value of winding resistance for 60 seconds

 // get data from ADC
 uint16_t getVolt(/*int num_enter*/)
 {
	 int16_t value;
	 ADCSRA |= 0x10;
	 ADMUX = 0;//num_enter & 0x1F; // TODO: Bits 7:6 set the reference, bit 5 sets the word alignment
	 ADCSRA |= 0x40;
	 while((ADCSRA & 0x10) == 0);
	 value = ADC;
	 return value;

 }

uint16_t pv_get_resistance()
{
	return resistance_display;
}

uint16_t pv_get_r()
{
	return r;
}

uint16_t pv_get_r5()
{
	return r5;
}

uint16_t pv_get_r30()
{
	return r30;
}

uint16_t pv_get_r60()
{
	return r60;
}

uint16_t pv_get_output()
{
	return outputon;
}

uint16_t pv_get_state_and_mode(void)
{
	return (((uint16_t)pr_state)<<8)|pr_mode_modal;
}

uint16_t pv_get_status()
{
	uint16_t st;

	st = status;
	if (level[PV_AUTOMATIC] == MANUAL)
		st |= (1 << ST_MANUAL_MODE);
	if (level[PV_PARALLEL] != 0)
		st |= (1 << ST_PARALLEL_R);
	if (drying_fail != 0)
		st |= (1 << ST_DRYING_FAIL);

	// Reset those bits that are to be only read once
	set_status(ST_DECREASED_R, false);
	return st;
}

uint16_t pv_get_rparallel()
{
	return level[PV_PARALLEL];
}

uint16_t get_starter_resistance(void)
{
	return level[PV_STARTER];
}

uint16_t pv_get_hold_time()
{
	return hold_time;
}

uint16_t pv_get_cool_time()
{
	return cool_time;
}

#if 0
bool pv_get_dop()
{
	return d_op;
}
void pv_set_dop(uint16_t value)
{
	d_op = (value != 0);
}
#endif

void pv_set_drying_fail(bool state)
{
	drying_fail = state;
}

bool pv_get_drying_fail()
{
	return drying_fail;
}

uint16_t pv_get_motor_run_time()
{
	uint16_t t;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		t = motor_run_time;
	}
	return t;
}

void pv_reset_motor_run_time()
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		motor_run_time = 0;	
	}
}

void pv_set_r_initial(uint16_t r)
{
	r_initial = r;
}

bool pv_set_level(uint16_t address, uint16_t value)
{
	if ((address >= 0) && (address < PV_SIZE))
	{
		if(level[address] != value)
		{
			level[address] = value;
			level_changed = true;
			if ((address == PV_UART_RATE) || (address == PV_PARITY))
				modbus_init(false);	// Re-initialize the USART if the settings were altered
		
		}
		return true;	 
	}
	
	else
		return false;
}

uint16_t pv_get_level(uint16_t address)
{
	if ((address >= 0) && (address < PV_SIZE))
	{
		uint16_t v;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			v = level[address];
		}
		return v;
	}
	else
		return 0xFFFF;	// TBD: Pass a pointer for return value and return a success value?
}

//Timer1 start
inline void StartTimer1(void)
{
  TCNT1 = 30000;
  TCCR1B = (1 << CS12) | (0 << CS11) | (1 << CS10);
}

//Timer1 stop
inline void StopTimer1(void)
{
  TCCR1B = 0x0;
}

//Timer1 initialization
inline void InitTimer1(void)
{
  TIMSK |= (1 << TOIE1);
}


void range_set(uint8_t new_range)
{
	range = new_range;
	if (range == 2)
	{
		output_set(OUT_RANGE2, true);
		set_status(ST_RANGE2, true);
	}
	else
	{
		output_set(OUT_RANGE2, false);
		set_status(ST_RANGE2, false);
	}
	
}

// Base initialization

void init(void)
{

  // PORT A - seven-segment display

  //          TEST_460K
  //          |RS486_DIR (unused in code, keep as input)
  //          ||nc
  //          |||nc
  //          ||||nc
  //          |||||nc
  //          ||||||nc
  //          |||||||ADC0 measurement
  //          76543210
  DDRA = 0b00000000;	/* 2017-08-21 ir - Pull-up enabled for Test button input and unused pins. */
  PORTA = 0b11111110;

	
  // PORT B - seven-segment display

  //          a
  //          |f
  //          ||b
  //          |||g
  //          ||||c
  //          |||||dig 1
  //          ||||||d
  //          |||||||e
  //          76543210
  DDRB = 0b11111111;
  PORTB = 0b00000010;// "____" //0b00010000;//PORTB = 0b11111111; // Display '-' on each digit

  // PORT C - Button and digit in/out

  //          Digital out - Break measuring circuit
  //          |Digit out - Drying process on/off
  //          ||Digit out - Test mode on /off
  //          |||Digit out - measurement range 2 (not in use now,but we need it in future)
  //          ||||Digit out- Alarm <500kOhm
  //          |||||Digit out - Command for electric motor start
  //          ||||||Digit out - main �ontactor (on/off)
  //          |||||||Digital in - button Up (1 if pushed)
  //          76543210
  DDRC = 0b11111110;
  PORTC = 0b00000001;	/* 2017-08-21 ir - Pull-up enabled for Up button input */

  // PORT D -

  //          IN button Down
  //          |IN - button Enter
  //          ||OUT - dig 4
  //          ||OUT - dig 3
  //          ||||OUT - DIG2
  //          |||||Digit in PB2- control signal from the process control system for a electric motor launch
  //          ||||||UArt tx;
  //          |||||||UArt rx;
  //          76543210
  DDRD = 0b00111010;
  PORTD = 0b11000011; /* 2017-08-21 ir - Pull-up enabled for Down and Enter button inputs */

  ADCSRA = 135;

  read_from_eeprom();
  range_set(1);
  d_op = false;
  status = 0;
  _delay_ms(100);
  //critical_resistance=false;
  if (bit_is_clear(PIND, 2))
  {
    output_set(OUT_DRYING, false); //ClrBit(PORTC, 6);
    output_set(OUT_RUNNING, false); // ClrBit(PORTC, 2);
   // _delay_ms(500);
    output_set(OUT_MAIN, true); //SetBit(PORTC, 1);
    status = status | (1 << 4);
  }
  else
  {
    output_set(OUT_DRYING, false); //ClrBit(PORTC, 6);
    output_set(OUT_MAIN, false); //ClrBit(PORTC, 1);
    //_delay_ms(500);
    output_set(OUT_RUNNING, true); // SetBit(PORTC, 2);
    //status = status | (1 << 1);
    d_op = true;
  }
  output_set(OUT_AUX, true);
   d_phase = false;
  mi = 0;
  set_status(ST_R3060, false);
  
  pending_reset_request = 0;
}

void revert_eeprom_to_defaults()
{
	for(int i = 0; i < EEPROM_LEN - EEPROM_FILL; i++)
	{
		level[i] = level_defaults[i];
	}
	level_changed = true;
	save_to_eeprom();
}

// Read initial values from eeprom
void read_from_eeprom(void)
{
	
	uint16_t dummy[EEPROM_LEN];
	eeprom_read_block((void*)dummy, (const void*)EE_level, EEPROM_LEN * sizeof(uint16_t));
	for(int i = 0; i < EEPROM_LEN - EEPROM_FILL; i++)
	{
		if (dummy[EEPROM_FILL + i] != 0xFFFF)
			level[i] = dummy[EEPROM_FILL + i]; // Ignore the filler at start
		else
			level[i] = level_defaults[i];
	}
	// Retrieve the last measured value. This was demanded to prevent the device indicating zero resistance on startup.
	resistance = level[PV_LAST_MEASURED];
	resistance_display = resistance;
	level_changed = false; // EEPROM has current values of MB_address and level[], no need to save them
}

// If key_check() has changed MB_address or level[], save the values to corresponding EEPROM location
void save_to_eeprom(void)
{
	if (level_changed) // either MB_address or level[] has changed, save to eeprom
	{
		uint16_t dummy[EEPROM_LEN];
		for (int i = 0; i < EEPROM_LEN - EEPROM_FILL; i++)
		  dummy[i + EEPROM_FILL] = level[i];
		eeprom_update_block((const void*)dummy, (void*)EE_level, EEPROM_LEN * sizeof(uint16_t));
		level_changed = false;
	}
}




// D7: (was: down)
// D6: up (was: enter)
// C0: enter (was: up)
// 

uint8_t key_pressed(uint8_t mask)
{
	uint8_t key = KEY_NONE;

	if (bit_is_clear(KEY_UP_PIN, KEY_UP_BIT))
		key |= KEY_UP;
	if (bit_is_clear(KEY_DOWN_PIN, KEY_DOWN_BIT))
		key |= KEY_DOWN;
	if (bit_is_clear(KEY_ENTER_PIN, KEY_ENTER_BIT))
		key = KEY_ENTER;
	if (bit_is_clear(KEY_TEST_PIN, KEY_TEST_BIT))
		key = KEY_TEST;
	return mask & key;
}

uint8_t key_get_state(uint8_t key)
{
	uint8_t result = 1;
	switch (key)
	{
		case KEY_UP:
			if (bit_is_clear(KEY_UP_PIN, KEY_UP_BIT))
				result = 0;
			break;
		case KEY_DOWN:
			if (bit_is_clear(KEY_DOWN_PIN, KEY_DOWN_BIT))
				result = 0;
			break;
		case KEY_ENTER:
			if (bit_is_clear(KEY_ENTER_PIN, KEY_ENTER_BIT))
				result = 0;
			break;
		case KEY_TEST:
			if (bit_is_clear(KEY_TEST_PIN, KEY_TEST_BIT))
				result = 0;
		break;
	}
	return result;
}

void pv_inc(uint8_t param)
{

	switch (param)
	{
		case PV_R_CRITICAL:				// P1
		case PV_R_START:				// P2
		case PV_R_END:					// P3
			if (level[param] < 9950)
				level[param] += 50;
			break;
		case PV_PARALLEL:				// P4
			if (level[param] < 100)			// < 10 Mohm
				level[param] += 2;
			else if (level[param] < 1000)	// < 100 Mohm
				level[param] += 10;
			else if (level[param] < (10000 - 100))	// < 1000 Mohm
				level[param] += 100;		
			break;
		case PV_AUTOMATIC:				// P5
		case PV_ON_OFF:					// P6
		case PV_STANDBY_ENABLE:			// P8
		case PV_PAUSE_OPERATION:		// P18
			level[param] = (level[param] == 0 ? 1 : 0);
			break;
		case PV_START_DELAY:			// P7
			level[param] = (level[param] >= 1500 ? 1500 : level[param] + 50);
			break;
		case PV_STANDBY_ONTIME:			// P9
			if(level[param] < 5)
				level[param] += 1;
			else if (level[param] < 15)
				level[param] += 5;
			else if (level[param] <= (480 - 15))
				level[param] += 15;
			else
				level[param] = 480;
			break;
		case PV_STANDBY_OFFTIME:		// P10
			if(level[param] < 5)
				level[param] += 1;
			else if (level[param] < 15)
				level[param] += 5;
			else if (level[param] <= (180 - 15))
				level[param] += 15;
			else
				level[param] = 180;
			break;
		case PV_HALT_TIME:
			switch (level[param])
			{
				case 1:
					level[param] = 14;
					break;
				case 14:
					level[param] = 29;
					break;
				case 29:
					level[param] = 59;
					break;
				case 59:
					level[param] = 119;
					break;
				case 119:
					level[param] = 179;
					break;
				case 179:
					level[param] = 239;
					break;
				default:
					level[param] = 1;
			}
			break;
		case PV_COOLING_TIME:
			switch (level[param])
			{
				case 1:					/* 2017-08-21 ir - 1 min cooling time added, request from Ville Halonen 2017-08-20 */
					level[param] = 15;
					break;
				case 15:
					level[param] = 30;
					break;
				case 30:
					level[param] = 60;
					break;
				case 60:
					level[param] = 120;
					break;
				default:
					level[param] = 1;
			}
			break;
		case PV_FACTORY_RESET:
			if (level[param] < 130)
				level[param]++;
			break;
		/* 2017-08-22 ir - Drying measurement interval added, request from Ville Halonen 2017-08-20 */
		case PV_DRYING_MEASUREMENT_INTERVAL:	// 17
			switch (level[param])
			{
				case 1:
					level[param] = 5;
					break;
				case 5:
					level[param] = 15;
					break;
				case 15:
					level[param] = 60;
					break;
				case 60:
					level[param] = 120;
					break;
				case 120:
					level[param] = 180;
					break;
				case 180:
					level[param] = 240;
					break;
				case 240:
					level[param] = 300;
					break;
				case 300:
					level[param] = 360;
					break;
				case 360:
					level[param] = 420;
					break;
				case 420:
					level[param] = 480;
					break;
				case 480:
					level[param] = 1;
					break;
				default:
					level[param] = 1;
			}
			break;
		case PV_MODBUS_ADDR:			// P11
			level[param] = (level[param] >= 247 ? 247 : level[param] + 1);
			break;
		case PV_UART_RATE:				// P12
			level[param] = modbus_rate_change(level[param], 1);
			break;
		case PV_PARITY:					// P13
			level[param] = level[param] == 2 ? 0 : level[param] + 1;
			break;	
	}
#if 0
	if (param == PV_UART_RATE)
		level[param] = modbus_rate_change(level[param], 1);
	else
	if (param == PV_PARITY)
		level[param] = level[param] == 2 ? 0 : level[param] + 1;
	else
	if (param == PV_START_DELAY)
		level[param] = level[param] >= 1500 ? 1500 : level[param] + 50;
	else
	if ((param == PV_ON_OFF) || (param == PV_AUTOMATIC))
		level[param] = level[param] == 0 ? 1 : 0;
	else
	if (param == PV_STANDBY_ONTIME)
		level[param] = level[param] >= (120 - 15) ? 120 : level[param] + 15; 
	else
	if (param == PV_STANDBY_OFFTIME)
		level[param] = level[param] >= (180 - 15) ? 180 : level[param] + 15;
	else
	if (param <= PV_R_END)
	{
		if (level[param] < 9950)
			level[param] += 50;
	}
	else
		level[param]++;
#endif
}

void pv_dec(uint8_t param)
{
	switch (param)
	{
		case PV_R_CRITICAL:				// P1
		case PV_R_START:				// P2
		case PV_R_END:					// P3
			if (level[param] < 50)
				level[param] = 0;
			else
				level[param] -= 50;
			break;
		case PV_PARALLEL:				// P4
			if (level[param] > 1000)		// > 100 Mohm
				level[param] -= 100;
			else if (level[param] > 100)	// > 10 Mohm
				level[param] -= 10;
			else if (level[param] > 0)		// < 10 Mohm
				level[param] -= 2;
			break;
		case PV_AUTOMATIC:				// P5
		case PV_ON_OFF:					// P6
		case PV_STANDBY_ENABLE:			// P8
		case PV_PAUSE_OPERATION:		// P18
			level[param] = (level[param] == 0 ? 1 : 0);
			break;
		case PV_START_DELAY:			// P7
			level[param] = (level[param] > 50 ? level[param] - 50 : 0);
			break;
		case PV_STANDBY_ONTIME:			// P9
			// level[param] = (level[param] <= 30 ? 15 : level[param] - 15);
			if(level[param] > 15)
				level[param] -= 15;
			else if(level[param] > 5)
				level[param] -= 5;
			else if(level[param] > 1)
				level[param] -= 1;
			else
				level[param] = 1;
			break;
		case PV_STANDBY_OFFTIME:		// P10
			// level[param] = (level[param] <= 30 ? 15 : level[param] - 15);
			if(level[param] > 15)
				level[param] -= 15;
			else if(level[param] > 5)
				level[param] -= 5;
			else if(level[param] > 1)
				level[param] -= 1;
			else
				level[param] = 1;
			break;
		case PV_HALT_TIME:
			switch (level[param])
			{
				case 1:					/* 2017-08-21 ir - Looping added */
					level[param] = 239;
					break;
				case 14:
					level[param] = 1;
					break;
				case 29:
					level[param] = 14;
					break;
				case 59:
					level[param] = 29;
					break;
				case 119:
					level[param] = 59;
					break;
				case 179:
					level[param] = 119;
					break;
				case 239:
					level[param] = 179;
					break;
				default:
					level[param] = 14;
			}
			break;
		case PV_COOLING_TIME:
			switch (level[param])
			{
				case 1:					/* 2017-08-21 ir - Looping added */
					level[param] = 120;
					break;
				case 15:				/* 2017-08-21 ir - 1 min cooling time added, request from Ville Halonen 2017-08-20 */
					level[param] = 1;
					break;
				case 30:
					level[param] = 15;
					break;
				case 60:
					level[param] = 30;
					break;
				case 120:
					level[param] = 60;
					break;
				default:
					level[param] = 1;
			}
			break;
		case PV_FACTORY_RESET:
			if (level[param] > 100)
				level[param]--;
			break;
		/* 2017-08-22 ir - Drying measurement interval added, request from Ville Halonen 2017-08-20 */
		case PV_DRYING_MEASUREMENT_INTERVAL:	// 17
			switch (level[param])
			{
				case 1:
					level[param] = 480;
					break;
				case 5:
					level[param] = 1;
					break;
				case 15:
					level[param] = 5;
					break;
				case 60:
					level[param] = 15;
					break;
				case 120:
					level[param] = 60;
					break;
				case 180:
					level[param] = 120;
					break;
				case 240:
					level[param] = 180;
					break;
				case 300:
					level[param] = 240;
					break;
				case 360:
					level[param] = 300;
					break;
				case 420:
					level[param] = 360;
					break;
				case 480:
					level[param] = 420;
					break;
				default:
					level[param] = 5;
			}
			break;
		case PV_MODBUS_ADDR:			// P11
			level[param] = (level[param] <= 1 ? 1 : level[param] - 1);
			break;
		case PV_UART_RATE:				// P12
			level[param] = modbus_rate_change(level[param], -1);
			break;
		case PV_PARITY:					// P13
			level[param] = level[param] == 0 ? 2 : level[param] - 1;
			break;	
	}
#if 0
	if (param == PV_UART_RATE)
		level[param] = modbus_rate_change(level[param], -1);
	else
	if (param == PV_PARITY)
		level[param] = level[param] == 0 ? 2 : level[param] - 1;
	else
	if (param == PV_START_DELAY)
		level[param] = level[param] <= 150 ? 100 : level[param] - 50;
	else
	if ((param == PV_ON_OFF) || (param == PV_AUTOMATIC))
		level[param] = level[param] == 0 ? 1 : 0;
	else
	if ((param <= PV_R_END)	&& (level[param] >= 50))
		level[param] -= 50;
	else
	if ((param == PV_STANDBY_ONTIME) || (param == PV_STANDBY_OFFTIME))
		level[param] = level[param] <= 30 ? 15 : level[param] - 15;
	else
	if (level[param] > 0)
		level[param]--;
#endif
}

uint8_t key_debounce(uint8_t key)
{
	static uint16_t state[4] = {0, 0, 0, 0};
	
	state[key] = (state[key] << 1) | key_get_state(1 << key) | 0xE000;
	if (state[key] == 0xF000)
		return 1;
	else
		return 0;
}

#define KEY_LONGPRESS	100	// 8 ms units

#define TIMER2_DIVIDER	1 // Divide the Timer 2 overflow frequency 2 (1+1) to get ca. 8 ms intervals
// Timer 2 overflow vector executed 8000000 / 128 / 256 = 244 times per second = at 4 ms intervals
ISR(TIMER2_OVF_vect)
{
	static uint16_t counter = 0;
	static uint16_t done_counter = 0;
	static uint8_t param_ix = 1;
	static uint16_t original;
	static uint8_t G_key_pressed = 0;		// A status variable for reading the keys
	
	display_update();
	systime++;
		
	// Perform the UI checks at roughly 100 ms intervals
	if (counter++ >= TIMER2_DIVIDER)
	{
		G_key_pressed = G_key_pressed << 4;	// Move the current state to the "previous" state
		for (uint8_t i = 0; i < 4 ; i++)
		{
			G_key_pressed |= (key_debounce(i) << i);	// Update the new debounced key states
		}
		if (!key_get_state (KEY_ENTER))
		{
			if (G_enter_pressed <= KEY_LONGPRESS)
				G_enter_pressed++;
		}
		else
			G_enter_pressed = 0;

		if (!key_get_state (KEY_TEST))
		{
			if (G_test_pressed <= KEY_LONGPRESS)
			G_test_pressed++;	
		}
		else
			G_test_pressed = 0;

		switch (ui_state)
		{
			case UI_INITIAL:		
				if (G_test_pressed == KEY_LONGPRESS)
				{
					ui_state = UI_TEST;
					G_key_pressed &= ~KEY_TEST;
					set_status(ST_TEST_MODE, true);
					/* TODO: 2017-08-24 ir - Fix this! getResistance() contains a 300 ms delay! */
					/* 2017-08-26 ir - getResistance() removed */
					/* getResistance(true); */
				}
				if (G_enter_pressed == KEY_LONGPRESS)
				{
					ui_state = UI_PRM_INDEX;
					G_key_pressed &= ~KEY_ENTER;
					param_ix = 1;
				}
				break;
			case UI_TEST:
				if (G_test_pressed == KEY_LONGPRESS)	// Deactivate the test mode
				{
					ui_state = UI_INITIAL;
					G_test_pressed = 0;
					set_status(ST_TEST_MODE, false);
				}				
				break;
			case UI_PRM_INDEX:
				if (G_key_pressed & KEY_UP)
					param_ix = param_ix >= PV_LAST_IN_UI ? 1 : param_ix + 1;
				if (G_key_pressed & KEY_DOWN)
					param_ix = param_ix > 1 ? param_ix - 1 : PV_LAST_IN_UI;
				display(param_ix, 0, DISP_P00, true);
				
				if (G_key_pressed & KEY_ENTER & ~(KEY_ENTER << 4))
				{
					original = level[param_ix];
					ui_state = UI_PRM_VALUE;
					G_enter_pressed = 0;
					G_key_pressed &= ~KEY_ENTER | (KEY_ENTER << 4);
					
				}
				break;
			case UI_PRM_VALUE:
				if (G_key_pressed & KEY_UP)
					pv_inc(param_ix);
				if (G_key_pressed & KEY_DOWN)
					pv_dec(param_ix);
				display_level(param_ix, level[param_ix]);
				if (G_key_pressed & KEY_ENTER & ~(KEY_ENTER << 4))
				{
					if (level[param_ix] != original)
					{
						level_changed = true;
						if ((param_ix == PV_PARITY) || (param_ix == PV_UART_RATE))
							modbus_init(true);
						if (param_ix == PV_R_START)
						{
							if (level[PV_R_END] < (level[PV_R_START] + R_DIFF_MIN))
								level[PV_R_END] = level[PV_R_START] + R_DIFF_MIN;
						}
						if (param_ix == PV_AUTOMATIC)
						{
							if (level[PV_AUTOMATIC] == AUTOMATIC)
								level[PV_ON_OFF] = ON;
							else
								level[PV_ON_OFF] = OFF;
						}
						if (param_ix == PV_ON_OFF)
						{
							/* 2017-08-25 IR - No special handling. Manual drying is started after next measurement cycle if required.
							if ((level[PV_ON_OFF] != OFF) && (level[PV_AUTOMATIC] == MANUAL))
								if (resistance <= level[PV_R_START])
									drying_start(PR_DRYING);
							*/
						}
#if 1
						if (param_ix == PV_PARALLEL)
						{
							if (level[PV_PARALLEL] == 0)
							{
								/* Just remove starter resistance - no other actions required */
								pv_set_level(PV_STARTER, 0);
								pv_set_level(PV_REGISTER_RPAR, 0);
							}
							else
							{
								pv_set_level(PV_STARTER, 0);		/* Set starter to 0 to get proper calculation on next start */
								pv_set_level(PV_REGISTER_RPAR, 1);	/* Request starter resistance calculation */
								pending_reset_request = 1;
							}
							//if (level[PV_PARALLEL] != 0)
							//	set_state(PR_R_PARALLEL_SET);
						}
#endif 
						if ((param_ix == PV_FACTORY_RESET) && (level[param_ix] == FACTORY_RESET_MAGIC_NUMBER))
						{
							display(0, 0, DISP_RESET, true); // Doesn't work because the interrupt is called over and over again
							revert_eeprom_to_defaults();
							// The preferred way of doing a soft reset is to let the watchdog bite.
							// Jumping to the reset vector would not reset all the registers!
							while(1); 		
						}
						ui_state = UI_DONE;
						done_counter = 120;
					}
					else
						ui_state = UI_INITIAL;
				}
				break;
			case UI_DONE:
				display(0, 0, DISP_DONE, true);
				if (done_counter > 0)
					done_counter--;
				else
					ui_state = UI_INITIAL;
				break;
			default:
				ui_state = UI_INITIAL;	// Backup, should never end up here
		}
		// Clear the counter for the next round
		counter = 0;
	}
}

uint32_t calculate_r_from_parallel(uint32_t combined, uint32_t known)
{
	uint32_t result;
	// TODO: This probably fails due to integer casts if 100 * level[PV_PARALLEL] is too high
	result = combined;
	if (known != 0)
	{
		// Protect against division by zero
		if (combined >= known)
			result = R_CEILING;
		else
			result = (combined * known) / (known - combined); //PL 17.8.2017: Non floating calculation
	}
	return result;
}

// If there is a known resistance in parallel with the motor, deduct its effect
// Measured Rx = Rmotor || Rparallel = Rmotor * Rparallel / (Rmotor * Rparallel)
// => Rmotor = Rx / (1 - Rx / Rparallel)
uint16_t eliminate_parallel_r(uint16_t r)
{
	uint16_t result;
	// TODO: This probably fails due to integer casts if 100 * level[PV_STARTER] is too high
	result = r;
	if (level[PV_STARTER] != 0)
	{
		#if 1
			if( r > level[PV_STARTER] + RES_PARALLEL_MAX_ERROR)
			{
				//TODO: parallel resistance error
				result = R_CEILING;
				error_bits |= ERR_PARALLEL_RESISTANCE;
			}
			else
			{
				uint32_t res =  calculate_r_from_parallel(r, level[PV_STARTER]);
				result = res > R_CEILING ? R_CEILING : res;
				error_bits &= ~ERR_PARALLEL_RESISTANCE;
			}

		#else
			// Protect against division by zero
			if (r == level[PV_STARTER])
				result = R_CEILING;
			else
				result = r / (1 - r/(level[PV_STARTER]));
		#endif
	}
	return result;
}

// ADC data converting into Resistance
uint16_t convertResult(int16_t result)
{
	int32_t a;
	int	b, c;
	uint16_t rb;
	int32_t v; 
	
	int32_t previous = resistance;

	if (range == 2)	// Lower measurement range
	{
		a = a2;
		b = b2;
		c = c2;
	}
	else			// Higher measurement range. "Else" used to make any erraneous states to revert to the range 1.
	{
		a = a1;
		b = b1;
		c = c1;
	}
	rb = result + b;	// An intermediate result is needed. Otherwise v is promoted to signed integer and the result is incorrect.
	// Limit the minimum value to prevent overflows
	if ((range == 1) && (rb < 23))
		rb = 13;
	v = a / rb + c;
	// Limit the measured value to a fixed ceiling value
	v = eliminate_parallel_r(v);
	if (v > R_CEILING)
		v = R_CEILING;
	else
	{
		if (previous >= R_CEILING)	// The previous value was above the ceiling, the new one is below the ceiling. Give an alert.
		{
			set_status(ST_DECREASED_R, true);
		}
	}
	return v;
}


// subroutine for getting resistance.
bool getResistance(uint8_t show)
{
	uint16_t adc_temp;
	static uint16_t range_switch_systime;
	static bool range_switch_pending = false;
  
  	adc_temp = (getVolt() + getVolt() + getVolt() + getVolt()) / 4;
	
	/* 2017-09-05 ir - range_switch_pending==false condition added to range switch to make at least one completed 
	                   measurement between range switches.                                                        */
	if ( (range_switch_pending == false) && (range == 2) && (adc_temp < RANGE_SWITCHOVER_HIGH))	// Above 1.9 Mohm
	{
		range_set(1);
		range_switch_pending = true;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			range_switch_systime = (uint16_t)systime;
		}
	}
	else
	if ( (range_switch_pending == false) && (range == 1) && (adc_temp > RANGE_SWITCHOVER_LOW))	// Below 1.3 Mohm
	{
		range_set(2);
		range_switch_pending = true;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			range_switch_systime = (uint16_t)systime;
		}
	}
	
	if( range_switch_pending )
	{
		uint16_t st;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			st = (uint16_t)systime;
		}
		if( (st - range_switch_systime) >= RANGE_SWITCH_TIME )
		{
			range_switch_pending = false;
		}
		else
		{
			/* No measurement as long as range switch time is active */
			return false;
		}
	}
	
	/* Measure average of 1000 samples */
	mbuff = 0;
	for (mj = 0; mj < 1000; mj++)
	{ 
		adc_temp = getVolt();
		if (adc_temp > noise)
			adc_temp -= noise;
		mbuff = mbuff + adc_temp;
	}

	uint16_t  tm = (uint16_t)(mbuff / (mj - 1));
	real_voltage = tm;
   
    resistance = convertResult(real_voltage);
	if(show)
	{
		display(resistance, 0, DISP_RESISTANCE, false);
	}
	
	// TODO: Is this correct check??
	//       This disables rtime table update during measurement while drying.
	//       ??? resistance not stored on wrap around.
	#if 0
    if ((!is_status(ST_DRYING)) && (mi < 79))
    {
		mi++;
		rtime[mi] = resistance;
    }
	else
		mi = 0;
	#endif
	
	return true;
}

void getNoise()
{
	uint16_t noise_meas = 0;

	for (int i = 0; i < 16; i++)
	{
		noise_meas += getVolt();
	}
	noise = noise_meas >> 4;
	noise = 0;	// DEBUG
}




//subroutine for Timer1- Drying process time control and main time program control
// Timer 1 runs at 7812.5 Hz
ISR(TIMER1_OVF_vect)
{
	static uint8_t drying_phase_time = 0;
	//uint16_t noise_meas = 0;
	
	time1s_system++;

	if (fail_time > 0)
		fail_time--;
	// Increase motor run time counter
	if (is_motor_energized())
		motor_run_time++;

	/* 2017-08-22 ir - is_status(ST_DRYING) allows drying relay activation when measurement is running. */
	/* was: if (is_status(ST_DRYING)) */
	if (is_drying_relay_allowed())
	{
		drying_phase_time++;
		if (drying_phase_time >= 10)
			drying_phase_time = 0;
		if (drying_phase_time == 0)
			d_phase = true;
		if (drying_phase_time == 7)
			d_phase = false;

		if (d_phase == false)
		{
			// Move to the "pause" phase
			output_set(OUT_DRYING, false); // ClrBit(PORTC, 6); // Drying off
			//d_phase = false;
			//TCNT1 = 42098;//65535UL - 23437UL;	// 3.0 s
		}
		else
		{
		
			if ((bit_is_clear(PIND, 2))  && !d_op)	// K201 on and not "dp" (not drying?)
			{
				// Move to the "dry" phase
				output_set(OUT_DRYING, true); // SetBit(PORTC, 6);
				//d_phase = true;
				//TCNT1 = 10848; //65535UL - 54687UL;	// 7.0 s
			}
		}
	}
	else
	{
		if (d_phase == true)
		{
			output_set(OUT_DRYING, false);
			d_phase = false;
		}
		drying_phase_time = 0;
	}
	// Reload the counter for the next interrupt
	TCNT1 =	57722; //  1.0 s: 65535 - 1.0 s * 80000000 Hz / 1024 = 57722.5 (1024 is the timer prescaler)
		//26473; // 5.0 s: 65535 - 5.0 s * 80000000 Hz / 1024 = 26472.5 (1024 is the timer prescaler)
}

void process_state_time1s_reset( void )
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		process_state_time1s = 0;
	}
}

#if 0
// subroutine for r30 and r60 calculation
void getR3060(void)
{
  uint16_t  ii;
  mbuff = 0;
  uint16_t  mii = mi / 2;
  for (ii = 1; ii < mi; ii++)
  {
    mbuff = mbuff + rtime[mi];
  }
  uint16_t  ff = mbuff / (mi - 1);
  r = ff;
  r60 = rtime[mi];
  mbuff = 0;
  set_status(ST_R3060, true);
  r30 = rtime[mii];
  r5 = rtime[5];
}
#endif

typedef enum { MEAS_R3060_R = 0, MEAS_R3060_R5, MEAS_R3060_R30, MEAS_R3060_R60, MEAS_R3060_READY } MEAS_R3060_t;
MEAS_R3060_t meas_r3060;

void r3060_init( void )
{
	meas_r3060 = MEAS_R3060_R;
}

void r3060_update( uint16_t state_time1s, uint16_t res )
{
	switch( meas_r3060 )
	{
		case MEAS_R3060_R:
			r = res;
			meas_r3060 = MEAS_R3060_R5;
			break;

		case MEAS_R3060_R5:
			if( state_time1s >= 5)
			{
				r5 = res;
				meas_r3060 = MEAS_R3060_R30;
			}
			break;
		case MEAS_R3060_R30:
			if( state_time1s >= 30)
			{
				r30 = res;
				meas_r3060 = MEAS_R3060_R60;
			}
			break;
		case MEAS_R3060_R60:
			if( state_time1s >= 60)
			{
				r60 = res;
				meas_r3060 = MEAS_R3060_READY;
				set_status( ST_R3060, true);	/* Indicate R5, R30, and R60 values are valid */
			}
			break;
		default:
			meas_r3060 = MEAS_R3060_READY;
			break;
	}
}

#ifdef FAST_TEST
	#define TM_START_END					2
	#define TM_HALT_END						(3 * SECINMIN)
	#define TM_MOTOR_WAIT_END				6
	#define TM_MEASURE_END					60
	#define TM_HOLD_END						(4 * SECINMIN)
	#define TM_NEXT_CYCLE_END				2
	//#define TM_DRYING_MEASUREMENT_INTERVAL	(5 * SECINMIN)
	#define TM_MEASUREMENT_DELAY			2
	#define TM_HALT_AFTER_DRY_END			(3 * SECINMIN)
	#define TM_HALT_AFTER_RUN_END			(6 * SECINMIN)
	#define TM_LONGDRY						(500 * SECINMIN)
	#define TM_FAIL_END						3
#else
	//#define TIMESTEP	 1		// Step size of variable "m" in seconds
	#define TM_START_END					10
	#define TM_HALT_END						(30 * SECINMIN)
	#define TM_MOTOR_WAIT_END				60
	#define TM_MEASURE_END					60
	#define TM_HOLD_END						(14 * SECINMIN)
	#define TM_NEXT_CYCLE_END				4
	//#define TM_DRYING_MEASUREMENT_INTERVAL	(5 * SECINMIN)
	#define TM_MEASUREMENT_DELAY			5
	#define TM_HALT_AFTER_DRY_END			(15 * SECINMIN)
	#define TM_HALT_AFTER_RUN_END			(30 * SECINMIN)
	#define TM_LONGDRY						(8 * MININH * SECINMIN)
	#define TM_FAIL_END						30
#endif
#define TM_TEST1_END					3
#define TM_TEST2_END					3
#define TM_TEST3_END					5
#define TM_TEST4_END					2
#define TM_TEST5_END					2
#define TM_TEST6_END					7
#define TM_TEST7_END					3
#define TM_TEST8_END					7
#define TM_TEST9_END					3
#define TM_TEST10_END					7
#define TM_TEST11_END					3
#define TM_TEST12_END					2

bool is_motor_energized()
{
	return bit_is_set(PIND, 2);
}

bool is_resistance_critical(void)
{
	return is_status(ST_CRITICAL_RESISTANCE);
}

void determine_motor_dryness()
{
#if 0
	if (!is_status(ST_R3060))
	{
		output_set(OUT_AUX, false); // SetBit(PORTC, 7);
		getR3060();
	}
#endif
	if (resistance < level[PV_R_CRITICAL])
	{
		set_status(ST_CRITICAL_RESISTANCE, true);
	}
	else
	{
		set_status(ST_CRITICAL_RESISTANCE, false);
	}
	output_set(OUT_READY, is_resistance_critical() != true);
	
	if (resistance < level[PV_R_START])
	{
		set_status(ST_DRYING_REQUIRED, true);
		//set_status(ST_DRYING, true);
	}
	if (resistance > level[PV_R_END])
	{
		set_status(ST_DRYING_REQUIRED, false);
		//set_status(ST_DRYING, false);
	}
}

void set_state(uint8_t state)
{
	pr_state = state;
	pr_sub_state = 0;
	relays_set(state);
	process_state_time1s_reset();
}

uint8_t get_state( void )
{
	return pr_state;	
}

void drying_start(uint8_t next_state)
{
	pv_set_r_initial(resistance);
	set_status(ST_DRYING, true);
	set_state(next_state);	// Either PR_DRYING or PR_STANDBY_DRY
	if(next_state == PR_DRYING)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if(drying_time1s == 0)
				drying_time1s = 1;
		}
	}
}

void drying_stop()
{
	set_status(ST_DRYING, false);
	pr_mode_set(MODE_DRY, false);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		drying_time1s = 0;
	}
		
}

uint8_t is_drying_relay_allowed(void)
{
	uint8_t st = get_state();
	return ((st == PR_DRYING) || (st == PR_STANDBY_HEAT));
}

uint8_t pending_standby_state;	/* Signal used to restore correct standby state after measurement/pause */

bool is_time_to_measure()
{
	/* Check if it is time to measure */
	return (measure_time1s >= (SECINMIN * level[PV_DRYING_MEASUREMENT_INTERVAL]));
}
void measure_done()
{
	/* Count for next measure */
	measure_time1s = 0;
}

void standby_time_count_start(void)
{
	if(standby_time1s == 0)
		standby_time1s = 1;
}

void standby_prepare_state( uint8_t next_standby_state )
{
	standby_time1s = 0;
	pending_standby_state = next_standby_state;
}

bool is_standby_heat(void)
{
	return ( (pending_standby_state == PR_STANDBY_HEAT) && (standby_time1s != 0) );
}

/* Return states and state times for overriding modes */
uint8_t pr_pause_return_state;	
uint8_t pr_motor_return_state;
uint16_t pr_pause_return_state_time1s;
uint16_t pr_motor_return_state_time1s;
/* Return state for measurements */
//uint8_t pr_meas_return_state;

//void pr_enter_to_meas(uint8_t return_state, uint8_t meas_state)
void pr_enter_to_meas(uint8_t meas_state)
{
	//pr_meas_return_state = return_state;
	set_state(meas_state);
}


void set_process_state_after_measure(void)
{
	uint8_t mode = pr_mode_get();


	switch( mode )
	{
	//	case MODE_NONE:		break;

		case MODE_DRY:
			/* drying can end after measurement */
			/* If drying has been on for 8 hours, check that the resistance has risen at least by a minimum step value */
			if( (drying_time1s >= TM_LONGDRY) && (resistance <= (r_initial + R_MINIMUM_RISE)) )
			{
				drying_stop();
				pv_set_drying_fail(true);
				set_state(PR_HOLD);
				/* TODO: No cooling after long heat! Is this correct? */
			}
			else if( ( (pv_get_level(PV_AUTOMATIC) == MANUAL) && (pv_get_level(PV_ON_OFF) != ON) ) || !is_status(ST_DRYING_REQUIRED) )
			{
				if(pv_get_level(PV_ON_OFF) != OFF)
					pv_set_level(PV_ON_OFF, OFF);
				drying_stop();
				set_state(PR_HALT_AFTER_DRY);
			}
			else
			{
				set_state(PR_DRYING);  
			}
			break;

		case MODE_HEAT:
			if ( (pv_get_level(PV_AUTOMATIC) == MANUAL) )
			{
				standby_prepare_state(PR_STANDBY_HEAT);
				pr_mode_set(MODE_NONE, true);
				set_state(PR_HOLD);
			} 
			else if( is_status(ST_DRYING_REQUIRED) )
			{
				standby_prepare_state(PR_STANDBY_HEAT);
				pr_mode_set(MODE_DRY, true);
				drying_start(PR_DRYING);
			}
			else
			{
				if( pv_get_level(PV_STANDBY_ENABLE) == ON)
				{
					set_state(pending_standby_state);
				}
				else 
				{
					/* TODO: Is a cooling state required after PR_STANDBY_HEAT when PV_STANDBY_ENABLE is turned OFF */
					/* TODO: What is the correct state after HEAT */
					standby_prepare_state(PR_STANDBY_HEAT);
					pr_mode_set(MODE_NONE, true);
					set_state(PR_HOLD);
				}
			}
			break;

		case MODE_COOL:
			set_state(PR_HALT_AFTER_DRY);
			break;
			
		case MODE_PAUSE:
			/* Measurement never started within PAUSE mode ??? How about TEST?? */
			set_state(PR_PAUSE);
			break;
			
		case MODE_MOTOR:
			/* Measurement never started within MOTOR mode */
			set_state(PR_NEXT_CYCLE);
			break;

		default:
			pr_mode_set(mode, false);
			if( is_status(ST_DRYING_REQUIRED) )
			{
				standby_prepare_state(PR_STANDBY_HEAT);
				if ( (pv_get_level(PV_AUTOMATIC) != MANUAL) || (pv_get_level(PV_ON_OFF) == ON) )
				{
					drying_start(PR_DRYING);
				}
				else
				{
					set_state(PR_HOLD);
					pr_mode_set(MODE_NONE, true);
				}
			}
			else
			{
				if ( (pv_get_level(PV_AUTOMATIC) == MANUAL) )
				{
					set_state(PR_HOLD);
					pr_mode_set(MODE_NONE, true);
				}
				else
				{
					if(pv_get_level(PV_STANDBY_ENABLE) == ON)
					{
						/* Standby can be HEATing or PAUSE. */
						set_state( pending_standby_state ? pending_standby_state : PR_STANDBY_HEAT);
						pr_mode_set(MODE_HEAT, true);
					}
					else
					{
						set_state(PR_HOLD);
					}
				}
			}
			break;
	}
	measure_done();
}



bool pr_activate_pause_mode()
{
	/* Pause mode is active if PV_PAUSE_OPERATION (P18) is true AND there is no mode overriding PAUSE (MOTOR mode) */
	if( (level[PV_PAUSE_OPERATION] == ON) && (pr_mode_get() <= MODE_PAUSE)  && (get_state() != PR_START) )
	{
		if( pr_mode_set(MODE_PAUSE, true) )
		{
			pr_pause_return_state = get_state();
			pr_pause_return_state_time1s = process_state_time1s;
		}
		set_state(PR_PAUSE);
		return true;
	}
	return false;
}

bool pr_activate_test_mode()
{
	return false;
}

void pr_check_pause_mode_end(void)
{
	if (pv_get_level(PV_PAUSE_OPERATION) != ON)
	{
		pr_mode_set(MODE_PAUSE, false);
		set_state(pr_pause_return_state);
		process_state_time1s = pr_pause_return_state_time1s;
		pr_pause_return_state = 0;
	}
}

void average_init( void )
{
	measure_sample_index = 0;
}

/* 
	Adds global variable resistance to average buffer.
	Sets average of samples to global variable resistance if enough samples registered.
	There must be less than 0xffff samples during average calculation sequence.
*/
bool average_add_resistance( void )
{
	uint8_t ix;

	/* Fill buffer with the firs sample */
	if( measure_sample_index == 0 )
	{
		for(ix = 0; ix < MEASURE_SAMPLE_COUNT; ix++)
			measure_sample_buffer[ix] = resistance;
	}

	/* store resistance to average filter */
	measure_sample_buffer[measure_sample_index & (MEASURE_SAMPLE_COUNT - 1)] = resistance;
	measure_sample_index ++;
	
	/* calculate average */
	measure_while_drying_sum = 0;
	for(ix = 0; ix < MEASURE_SAMPLE_COUNT; ix ++)
	{
		measure_while_drying_sum += (uint32_t)(measure_sample_buffer[ix]);
	}
	resistance = measure_while_drying_sum / MEASURE_SAMPLE_COUNT;
	return true;
}

void process()
{
	static uint32_t start_systime;
	uint16_t temp_u16;	/* temporary variable for different calculations */
	uint16_t tdiff;
	static uint16_t time1s_z = 0;

	/* Keep track of time */
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		temp_u16 = time1s_system;
	}
	tdiff = temp_u16 - time1s_z;
	if( tdiff )
	{
		process_state_time1s += tdiff;
		/* No mode time counting in PAUSE mode */
		if ( pr_mode_get() != MODE_PAUSE )
		{
			if( drying_time1s && (drying_time1s < (0xfffeU - tdiff)) )
				drying_time1s += tdiff;
			if( standby_time1s && (standby_time1s < (0xfffeU - tdiff)) )
				standby_time1s += tdiff;
			if( measure_time1s < (0xfffeU - tdiff))
				measure_time1s += tdiff;
		}
	}
	time1s_z = temp_u16;

	/* 2017-08-23 ir - make a reset after a few main cycles to make sure EEPROM write is completed */
 	if(pending_reset_request > 0)
	{
		pending_reset_request ++;
		
		if(pending_reset_request > 3)
		while(1)
		;
	}
	
	
	/* MOTOR ENERGIZED has the highest priority in process.
	   motor start can interrupt any other state.
	*/
	if (is_motor_energized())
	{
		if (is_status(ST_CRITICAL_RESISTANCE))
		{
			fail_time = TM_FAIL_END;
			set_status(ST_FAIL, true);
			set_state(PR_MOTOR_WAIT);
		}
		else
		{
			set_status(ST_MOTOR_RUNNING, true);
			/* Motor restart allowed whil cooling after motor run */
			if ((get_state() != PR_MOTOR_RUN) && (get_state() != PR_MOTOR_START))
			{
				/* pr_motor_return_state and pr_motor_return_state_time1s can be used in PR_MOTOR_START state only. */
				/* It must be possible to return previous state if only a short motor start command is given. */
				pr_motor_return_state = get_state();
				pr_motor_return_state_time1s = process_state_time1s;
				set_state(PR_MOTOR_START);
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					start_systime = systime;
				}
			}
		}
	} 
	else	
	{
		if( ! pr_activate_pause_mode() )
		{
			// Test mode is only allowed if the motor is not running and pause is not active
			if (is_status(ST_TEST_MODE))
			{
				if (pr_state != PR_TEST)
				{
					test(true);
					set_state(PR_TEST);
				}
			}
		}
		else
		{
			set_status(ST_TEST_MODE, false);
		}
	}

	
	if (fail_time > 0)
		display(0, 0, DISP_FAIL, true);
	else
		set_status(ST_FAIL, false);
	relays_set(pr_state);
	switch (pr_state)
	{
		case PR_START:
			range_set(1);
			pr_mode_set(MODE_NONE, true);
			
			display(0, 0, DISP_HALT, false);
			mi = 0;
			if (process_state_time1s >= TM_START_END)
			{
				set_state(PR_NEXT_CYCLE);
			}
			break;
			
		/* PAUSE mode state */
		case PR_PAUSE:
			if (pr_sub_state == 0)
			{
				pr_mode_set(MODE_PAUSE, true);
				pr_sub_state ++;

				range_set(1);
				display_sequence_set(0, 0, 0, DISP_PAUS, false, 60 * TICKS_PER_SECOND);
				display_sequence_reset();
				display_sequence_set(1, 0, 0, DISP_NONE, false, 0);
			}
			display_sequence();

			pr_check_pause_mode_end();
			
			break;
		
		/* DRYING mode states */	
		case PR_DRYING:
			if(pr_sub_state == 0)
			{
				pr_sub_state++;

				pr_mode_set(MODE_DRY, true);

				range_set(1);
				display_sequence_set(0, 0, 0, DISP_DRY, false, 2*TICKS_PER_SECOND);
				display_sequence_set(1, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
				display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(4, 0, 0, DISP_NONE, false, 0);
				display_sequence_reset();
			}
			display_sequence();

			if ( is_time_to_measure() && (d_phase == false) )
			{
				// pr_enter_to_meas(pr_state, PR_MEASURE_20S_AVG);
				pr_enter_to_meas(PR_MEASURE_20S_AVG);
			}
			break;
			
		case PR_HALT_AFTER_DRY:
			if (pr_sub_state == 0)
			{
				pr_sub_state ++;

				pr_mode_set(MODE_COOL, true);
				
				range_set(1);
				display_sequence_set(0, 0, 0, DISP_COOL, false, 15 * TICKS_PER_SECOND);
				display_sequence_reset();
				display_sequence_set(2, 0, 0, DISP_COOL, false, 15 * TICKS_PER_SECOND);
				display_sequence_set(5, 0, 0, DISP_NONE, false, 0);		
			}
			cool_time = TM_HALT_AFTER_DRY_END - process_state_time1s;
			display_sequence_set(1, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
			display_sequence_set(3, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
			display_sequence_set(4, (TM_HALT_AFTER_DRY_END - process_state_time1s), 0, DISP_SEC, false, 10 * TICKS_PER_SECOND);
			//display_sequence_set(4, (TM_HALT_AFTER_DRY_END - process_state_time1s) / SECINMIN, 0, 0, false, 10 * TICKS_PER_SECOND);
			display_sequence();

			// TODO: NO Measure. Is this OK?
			if (process_state_time1s >= TM_HALT_AFTER_DRY_END)
			{	
				pr_mode_set(MODE_NONE, true);
				set_state(PR_NEXT_CYCLE);
				cool_time = 1;
			}
			break;

			

		case PR_HOLD:
			if(pr_sub_state == 0)
			{
				pr_sub_state++;
				
				range_set(1);
				display_sequence_set(0, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
				display_sequence_reset();
				display_sequence_set(1, 0, 0, DISP_NONE, false, 0);
				display_sequence_reset();
			}

			// Calculate the remaining time for Modbus queries
			temp_u16 = SECINMIN * level[PV_HALT_TIME];
			hold_time = temp_u16 - process_state_time1s;

			/* Update display information */
			/* 1. Resistance and remaining time are shown in all cases */
			display_sequence_set(0, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
			display_sequence_set(1, hold_time, 0, DISP_SEC, false, 10*TICKS_PER_SECOND);

			/* 2. Show "HAnd" in manual mode is active */
			if((pv_get_level(PV_AUTOMATIC) == MANUAL))
			{
				display_sequence_set(2, 0, 0, DISP_MAN, false, 10*TICKS_PER_SECOND);
				/* 3. Show "rEq"+"for"+"dry" if manual mode and drying is required */
				if( is_status(ST_DRYING_REQUIRED) )
				{
					display_sequence_set(3, 0, 0, DISP_REQ, false, 5*TICKS_PER_SECOND);
					display_sequence_set(4, 0, 0, DISP_FOR, false, 5*TICKS_PER_SECOND);
					display_sequence_set(5, 0, 0, DISP_DRY, false, 5*TICKS_PER_SECOND);
				}
				else
				{
					display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
					display_sequence_set(4, 0, 0, DISP_NONE, false, 0);
					display_sequence_set(5, 0, 0, DISP_NONE, false, 0);
				}
			}
			else
			{
				display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(4, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(5, 0, 0, DISP_NONE, false, 0);
			}
			display_sequence();
			

			getNoise();
			if (process_state_time1s >= temp_u16 /* TM_HOLD_END */)	// Restart the cycle
			{
				mi = 0;
				hold_time=1; //PL 17.8.2017. hold_time=1 when timer not running
				set_state(PR_NEXT_CYCLE);
			}
			break;

		case PR_NEXT_CYCLE:
		case PR_R_PARALLEL_ERROR:
			// pr_mode_set(MODE_NONE, true);
			if (pr_state == PR_NEXT_CYCLE)
			{
				set_status(ST_R3060, false);	/* Invalidate R3060 until next PR_MEASURE is completed */

				display_sequence_set(0, 0, 0, DISP_IR, false, TICKS_PER_SECOND * 2);
				display_sequence_set(1, 0, 0, DISP_TEST, false, TICKS_PER_SECOND * 2);
				display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
			}
			else
			{
				display_sequence_set(0, 0, 0, DISP_ERR, false, TICKS_PER_SECOND);	// Displayed twice because the cycle time is 4 seconds
				display_sequence_set(1, r, 0, DISP_TEST, false, TICKS_PER_SECOND);
				display_sequence_set(2, level[PV_PARALLEL], 0, DISP_RESISTANCE, false, TICKS_PER_SECOND);
				display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
			}
			display_sequence();
			if (process_state_time1s >= TM_NEXT_CYCLE_END)	// Restart the cycle
			{
				//pr_enter_to_meas(pr_state, PR_MEASURE);
				pr_enter_to_meas(PR_MEASURE);
			}
			break;
			
		/* MOTOR mode states */
		case PR_MOTOR_START:
			{
				uint32_t systimeFromStart;

				pr_mode_set(MODE_MOTOR, true);	/* Motor mode is active */

				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					systimeFromStart = systime - start_systime;
				}
				
				set_status(ST_DRYING, false);
			
				if ( systimeFromStart >= ((uint32_t)level[PV_START_DELAY] * (uint32_t)TICKS_PER_SECOND) / 1000UL)
				{
					range_set(1);
					set_state(PR_MOTOR_RUN);
				}
				else
				if (is_motor_energized() == false)
				{
					/* Only a short pulse, return to previous state */
					set_state(pr_motor_return_state);
					process_state_time1s = pr_motor_return_state_time1s;
					pr_mode_set(MODE_MOTOR, false);	/* Motor mode ends */
				}
			}
			break;
		case PR_MOTOR_RUN:
			display(0, 0, DISP_STBY, false);
			if (is_motor_energized() == false)
			{
				set_state(PR_MOTOR_WAIT);
			}
			break;
		case PR_MOTOR_WAIT:
			pr_mode_set(MODE_MOTOR, true);	/* Motor mode is active */
			set_status(ST_MOTOR_RUNNING, false);
			display(0, 0, DISP_COOL, false);
			if (process_state_time1s >= TM_MOTOR_WAIT_END)
			{
				set_state(PR_HALT_AFTER_RUN);
			}
			break;
		case PR_HALT_AFTER_RUN:
			if ( pr_sub_state == 0 )
			{
				pr_sub_state ++;

				pr_mode_set(MODE_MOTOR, true);	/* Motor mode is active */

				display_sequence_set(0, 0, 0, DISP_COOL, false, 50 * TICKS_PER_SECOND);
				display_sequence_set(1, 0, 0, DISP_NONE, false, 0);
				display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
				display_sequence_reset();
				range_set(1);
			}
			// Calculate the remaining cooling time for Modbus queries
			temp_u16 = SECINMIN * level[PV_COOLING_TIME];
			cool_time = temp_u16 - process_state_time1s;
			//display_sequence_set(1, (SECINMIN * level[PV_COOLING_TIME] - process_state_time1s) / SECINMIN, 0, 0, false, 10 * TICKS_PER_SECOND);
			display_sequence_set(1, cool_time, 0, DISP_SEC, false, 10 * TICKS_PER_SECOND);
			display_sequence();
			if (process_state_time1s >= temp_u16)
			{
				cool_time = 1;
				pr_mode_set(MODE_MOTOR, false);	/* Motor mode ends */
				pr_mode_set(MODE_COOL, false);	/* If COOL mode was active before motor run it can be terminated now */
												/* There is no need to continue another cooling. */
				set_state(PR_NEXT_CYCLE);
			}
			break;

		/* Measurement states */
		case PR_MEASURE:
		case PR_R_PARALLEL_SET:
			wdt_reset();
			if ( pr_sub_state == 0)
			{
				pr_sub_state++;
				pr_measure_display_z = process_state_time1s - 2;
				average_init();
				r3060_init();
			}
			
			pv_set_drying_fail(false);
			
			if ( !getResistance(false) )
				break;	/* Do not continue if measurement failed */

			if ( !average_add_resistance() )
				break;

			/* Update R5, R30 and R60 registration */
			if( !is_status(ST_R3060) )
			{
				r3060_update(process_state_time1s, resistance);
			}

			wdt_reset();
			
			if( (process_state_time1s - pr_measure_display_z) >= MEASURE_DIPLAY_TIME)
			{
				resistance_display = resistance;
				pr_measure_display_z = process_state_time1s;
				display(resistance_display, 0, DISP_RESISTANCE, false);
			}
			
			if (process_state_time1s >= TM_MEASURE_END)
			{
				/* Trigger save or 'level' variables */
				/* EEPROM life time is 100000 writes */
				/* With 5 min measurement period this makes about 1 year */
				uint16_t r_margin = (resistance / 4) + (resistance / 8) - (resistance / 32); /* R/4 + R/8 - R/32 = 11R/32 = 0,343 R */
				if ( ((level[PV_LAST_MEASURED] >= r_margin) && (resistance < (level[PV_LAST_MEASURED] - r_margin)) ) || ( (r_margin < (UINT16_MAX - R_CEILING)) && (resistance > (level[PV_LAST_MEASURED] + r_margin)) ) )
				{
					level[PV_LAST_MEASURED] = resistance;
					level_changed = true;
				}

				resistance_display = resistance;
				
				if( pv_get_level(PV_REGISTER_RPAR) == 1)
				{
					if( pv_get_level(PV_PARALLEL) == 0 )
					{
						pv_set_level(PV_STARTER, 0);
						pv_set_level(PV_REGISTER_RPAR, 0);
						set_state(PR_NEXT_CYCLE);
					}
					else
					{
						uint32_t rpar = ((uint32_t)level[PV_PARALLEL]) * 100UL;
						if (resistance <  rpar)
						{
							uint32_t r_starter = calculate_r_from_parallel(resistance, rpar);
							// 2018-08-26 ir - Add offset value from PV register to calculated value (request from Ville )
							r_starter += pv_get_level(PV_RPAR_OFFSET);
							pv_set_level(PV_STARTER, (r_starter > R_CEILING) ? R_CEILING : r_starter);
							pv_set_level(PV_REGISTER_RPAR, 0);
							pending_reset_request = 1; /* set a pending reset */
							set_state(PR_HOLD);	/* wait reset in HOLD state */
						}
						else
						{
							set_state(PR_R_PARALLEL_ERROR);						
						}
					}
					/* New state set in all branches above. 
					   No other actions allowed because resistance reading might be invalid. 
					*/
				}
				else
				{
					// Normal measurement operation
					determine_motor_dryness();		

					set_process_state_after_measure();
				}
			}
			break;

		case PR_MEASURE_20S_AVG:
			if (pr_sub_state == 0)
			{
				pr_sub_state ++;
				if (error_bits & ERR_PARALLEL_RESISTANCE)
				{
					display_sequence_set(0, 0, 0, DISP_IR, false, TICKS_PER_SECOND * 2);
					display_sequence_set(1, 0, 0, DISP_TEST, false, TICKS_PER_SECOND * 2);
					display_sequence_set(2, 0, 0, DISP_ERR, false,  TICKS_PER_SECOND * 6);
					display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
				}
				else
				{
					display_sequence_set(0, 0, 0, DISP_IR, false, TICKS_PER_SECOND * 2 * TM_MEASUREMENT_DELAY);
					display_sequence_set(1, 0, 0, DISP_TEST, false, TICKS_PER_SECOND * 2 * TM_MEASUREMENT_DELAY);
					display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
					display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
				}
				display_sequence_reset();
			}
			display_sequence();
			if (process_state_time1s >= TM_MEASUREMENT_DELAY)
			{
				/* stabilize resistance reading */
				getResistance(false);
			}
			if (process_state_time1s >= 2 * TM_MEASUREMENT_DELAY)
			{
				/* start measurement */
				measure_while_drying_cnt = 0;
				measure_while_drying_sum = 0;
				set_state(PR_MEASURE_20S_AVG_RUN);
			}
			break;
		case PR_MEASURE_20S_AVG_RUN:
			getResistance(false);
			display_sequence();
			/* handle sum overflow */
			if( ((measure_while_drying_cnt & 1) == 0) && (measure_while_drying_sum >= (UINT32_MAX - (4 * (uint32_t)R_CEILING))) )
			{
				measure_while_drying_sum /= 2;
				measure_while_drying_cnt /= 2;
			}
			measure_while_drying_sum += resistance;
			measure_while_drying_cnt ++;
			
			/* 2017-08-21 ir: when measurement time ends, calculate average */
			if (process_state_time1s >= 4 * TM_MEASUREMENT_DELAY)
			{
				if( measure_while_drying_cnt > 0 )
				{
					measure_while_drying_sum += (measure_while_drying_cnt / 2);
					resistance = measure_while_drying_sum / measure_while_drying_cnt;
				}
				else
				{
					resistance = R_CEILING;
				}
				if(resistance > R_CEILING)
				{
					resistance = R_CEILING;
				}

				resistance_display = resistance;

				determine_motor_dryness();

				set_process_state_after_measure();
			}
			break;

		/* Standby heating mode */
		case PR_STANDBY_HEAT:
			if(pr_sub_state == 0)
			{
				pr_sub_state++;
				
				pr_mode_set(MODE_HEAT, true);

				pending_standby_state = PR_STANDBY_HEAT;
				set_status(ST_DRYING, true);
				standby_time_count_start();
				range_set(1);
				display_sequence_set(0, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);
				display_sequence_set(1, 0, 0, DISP_HEAT, false, 5*TICKS_PER_SECOND);

				display_sequence_set(3, 0, 0, DISP_NONE, false, 0);
				display_sequence_reset();
			}
			temp_u16 = SECINMIN * level[PV_STANDBY_ONTIME];	// * 60 because the time is given in minutes
			display_sequence_set(2, temp_u16 - standby_time1s, 0, DISP_SEC, false, 5*TICKS_PER_SECOND);
			display_sequence();			

			if ( is_time_to_measure() )
			{
				pr_enter_to_meas(PR_NEXT_CYCLE);
			}
			else
			// See if the manual standby drying state time has run out
			if ( standby_time1s >= temp_u16 )
			{
				standby_prepare_state(PR_STANDBY_PAUSE);
				set_state(PR_STANDBY_PAUSE);
				display_sequence_reset();
				set_status(ST_DRYING, false);
				//output_set(OUT_READY, true);
			}
			break;
		case PR_STANDBY_PAUSE:
			if(pr_sub_state == 0)
			{
				pr_sub_state++;

				pr_mode_set(MODE_HEAT, true);

				pending_standby_state = PR_STANDBY_PAUSE;
				standby_time_count_start();
				range_set(1);
				display_sequence_reset();
				display_sequence_set(0, resistance, 0, DISP_RESISTANCE, false, 10 * TICKS_PER_SECOND);

				display_sequence_set(2, 0, 0, DISP_NONE, false, 0);
			}
			temp_u16 = SECINMIN * level[PV_STANDBY_OFFTIME];	// * 60 because the time is given in minutes
			display_sequence_set(1, temp_u16 - standby_time1s, 0, DISP_SEC, 0, 5*TICKS_PER_SECOND);
			display_sequence();

			if ( measure_time1s >= (SECINMIN * level[PV_HALT_TIME]) )
			{
				pr_enter_to_meas(PR_NEXT_CYCLE);
			}
			else
			if (standby_time1s >= temp_u16)
			{
				standby_prepare_state(PR_STANDBY_HEAT);
				/* Make sure at least one measurement is made during STADBY PAUSE */
				if( level[PV_HALT_TIME] > level[PV_STANDBY_OFFTIME] )
				{
					set_state(PR_NEXT_CYCLE);
				}
				else
				{
					set_state(PR_STANDBY_HEAT);
				}
			}
			break;



		// case 
#if 0
		case PR_FAIL:
			display(0, 0, DISP_FAIL, false);
			if (time >= TM_FAIL_END)
				set_state(PR_DRYING);
			break;
#endif
		case PR_TEST:
			test(false);
			if (is_status(ST_TEST_MODE) == false)
			{
				// pr_enter_to_meas(PR_NEXT_CYCLE, PR_MEASURE);
				pr_enter_to_meas(PR_MEASURE);
			}
			break;
	}
}

// Lengths of the test mode timesteps in seconds
const uint8_t PROGMEM test_step[]  = {0, 3, 3, 5, 2, 2, 7, 3, 7, 3, 7, 3, 7, 3, 2};

uint8_t test_advance(uint8_t step)
{
	uint16_t time;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		time = process_state_time1s;
	}
	if (time >= pgm_read_byte(&test_step[step]))
	{
		process_state_time1s_reset();
		return (step + 1);
	}
	return step;
}


void relays_set(uint8_t state)
{
	switch (state)
	{
		case PR_START:
			output_set(OUT_MAIN, true);
			output_set(OUT_AUX, true);
			output_set(OUT_RUNNING, false);
			break;
		case PR_MEASURE:
			/* no break */
		case PR_MEASURE_20S_AVG:
			/* no break */
		case PR_MEASURE_20S_AVG_RUN:
			/* no break */
		case PR_R_PARALLEL_SET:
			/* no break */
		case PR_NEXT_CYCLE:
			output_set(OUT_MAIN, true);
			output_set(OUT_AUX, false);
			output_set(OUT_DRYING, false);
			output_set(OUT_RUNNING, false);
			break;
		case PR_MOTOR_START:
			output_set(OUT_DRYING, false);
			break;
		case PR_MOTOR_RUN:
			output_set(OUT_MAIN, false);
			output_set(OUT_RUNNING, true);
			break;
		case PR_MOTOR_WAIT:
			output_set(OUT_MAIN, false);
			output_set(OUT_RUNNING, false);
			break;
		case PR_HALT_AFTER_DRY:
			/* no break */
		case PR_HALT_AFTER_RUN:
			/* no break */
		case PR_HOLD:
			output_set(OUT_RUNNING, false);
			output_set(OUT_AUX, true);
			break;
		case PR_TEST:
			//output_set(OUT_MAIN, false);
			output_set(OUT_AUX, true);
			//output_set(OUT_DRYING, false);	// Drying initially off
			output_set(OUT_READY, false);
			output_set(OUT_RUNNING, false);
			break;
		case PR_DRYING:
			output_set(OUT_MAIN, true);
			output_set(OUT_AUX, true);
			//output_set(OUT_READY, false);
			output_set(OUT_RUNNING, false);
			break;	
		case PR_PAUSE:
			output_set(OUT_MAIN, false);
			output_set(OUT_AUX, true);
			output_set(OUT_RUNNING, false);
			output_set(OUT_DRYING, false);
			break;
	}
}

void test(bool init_test_state)
{
	static uint8_t pr_test_step = 0;
	if (init_test_state)
		pr_test_step = 0;
	if (is_status(ST_TEST_MODE) == false)	// Check if the test mode is still activate
		pr_test_step = 0xFF;				// If not, jump to the "default" branch for a clean exit
	switch (pr_test_step)
	{
		case 0:
			output_set(OUT_MAIN, false);
			output_set(OUT_DRYING, true);
			range_set(2);
			process_state_time1s_reset();
			pr_test_step = 1;
		case 1:	// Steps 1 and 4 are the same
		case 4:
			display(0, 0, DISP_TEST, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		case 2:
			display(0, 0, DISP_INTR, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		case 3:
			getResistance(true);
			display(resistance, 0, DISP_RESISTANCE, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		/* case 4 is combined with case 1 */
		case 5:
			display(0, 0, DISP_DRY, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		case 6: // Steps 6, 8, 10 and 12 are the same
		case 8:
		case 10:
		case 12:
			output_set(OUT_MAIN, true);
			output_set(OUT_DRYING, true);
			display((pr_test_step >> 1) - 2, 0, DISP_DRY, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		case 7: // Steps 7, 9, 11 and 13 are the same
		case 9:
		case 11:
		case 13:
			output_set(OUT_DRYING, false);
			display((pr_test_step >> 1) - 2, 0, DISP_DRY, true);
			pr_test_step = test_advance(pr_test_step);
			break;
		case 14:
			if ((resistance >= (r_internal - r_internal_tolerance_low)) && (resistance <= (r_internal + r_internal_tolerance_high)))
			{
				display(0, 0, DISP_GOOD, true);
			}
			else
			{
				display(0, 0, DISP_ERR, true);
			}
			pr_test_step = test_advance(pr_test_step);
			break;
		default:
			set_status(ST_TEST_MODE, false);
			ui_state = UI_INITIAL;
			pr_test_step = 0;		
	}
}

void InitTimer2()
{
	TCCR2 = 0b00000101;	 /// div 128 => 62.5 kHz, div 256 => 244 Hz  // Normal mode, source = F_OSC / 1024 = 7812,5 Hz
	TIMSK |= (1 << 6);	// Enable Timer 2 overflow interrupt
}

void InitWatchdog()
{
	WDTCR = (1 << WDTOE) | (1 << WDE);
	WDTCR |= (1 << WDTOE) | (WDTO_2S);
}

//main program
int main(void)
{
	init();
	InitTimer1();
	InitTimer2();
	StartTimer1();
	InitWatchdog();
	modbus_init(true);	// Also clear the counters
	sei(); // Enable interrupts
	
	while (1)//main loop
	{
		process();
		modbus_check();
		save_to_eeprom();	// Save any changed settings, if altered (checked in the function)
		wdt_reset();		// Reset the watchdog timer 
	}
}