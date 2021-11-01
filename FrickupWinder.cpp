/* Copyright (c) 2014-2021 Matt Frick All Rights Reserved. */

/* This file is best viewed with Tabs = 4 spaces */

#ifdef OUTSIDE_OF_ARDUINO_IDE
#include "Arduino.h"
#include "core_pins.h"
#include "mk20dx128.h"
#endif
#include <LiquidCrystal.h>

#include "EEPROM.h"

#define SOFTWARE_VERSION 1

#define USE_SERIAL
#define USE_ENCODER 1
#if USE_ENCODER
#include <Encoder.h>
#endif

/* TODO: macro definitions for pins: */
LiquidCrystal lcd(7, 8,  9, 10, 11, 12);
#ifdef USE_ENCODER
Encoder encoder_knob(18, 19);
#endif
volatile int strobe_rotation;

#define DIG_FILTER_MASK 0x1FF
#define FILT_LEN 10
typedef struct filter {
  int buf[FILT_LEN];
  int idx;
  int last;
} filter_t;

typedef enum {
  PT_UNUSED,
  PT_DIGITAL,
  PT_ANALOG
} pintype_t;

// Most all digital inputs conduct to ground when asserted:
#define DIG_ASSERTED 0
// There are some exceptions: STOP
#define STOP_ASSERTED 1

typedef enum {
  PD_UNUSED,
  PD_OUTPUT = OUTPUT,
  PD_INPUT = INPUT_PULLUP,
} pindir_t;

typedef enum {
  SIG_INVALID,    // 0
  SIG_TRAV_FAULT,
  SIG_TRAV_ENABLE,
  SIG_TRAV_DIR,
  SIG_TRAV_STEP,
  SIG_SPIN_STEP,  // 5
  SIG_SPIN_ENABLE,
  SIG_SPIN_DIR,
  SIG_LCD1,
  SIG_LCD2,
  SIG_LCD3,      // 10
  SIG_LCD4,
  SIG_LCD5,
  SIG_LCD6,
  SIG_TEENSY_LED,
  SIG_LIMIT_SW2,  // 15
  SIG_LIMIT_SW1,
  SIG_SPIN_SPEED,
  SIG_TRAV_RATE,
  SIG_ENCODER_B,
  SIG_ENCODER_A,  //20
  SIG_ENCODER_BUTTON,
  SIG_COUNT_RESET,
  SIG_MAX_DOWN,
  SIG_MAX_UP,
  SIG_LLIM_LED,  // 25
  SIG_SET_LIM_R,
  SIG_STOP,
  SIG_WIND_DIR,  // input, not stepper dir output!
  SIG_STROBE,
  SIG_RLIM_LED,  // 30
  SIG_START,
  SIG_SET_LIM_L,
  SIG_RLIM,
  SIG_LLIM,    // 34
  
  SIG_LAST, // INVALID,
} signal_t;

typedef struct pinmap {
    int        num;
    pindir_t   dir;
    pintype_t  type;
    signal_t   signal;
    char       name[20];
    filter_t   *filter;
    int        value;
    int        prev_value;
} pinmap_t;

filter_t analog_filter1;
filter_t analog_filter2;
filter_t analog_filter3;
filter_t analog_filter4;

#define NUM_PINS (int)(sizeof(pin_map)/sizeof(pin_map[0]))
pinmap_t pin_map[] = {
 { 0, PD_UNUSED, PT_UNUSED, SIG_INVALID, "Invalid" },
 { 0, PD_INPUT,  PT_DIGITAL, SIG_TRAV_FAULT, "TravFault" },
 { 1, PD_OUTPUT, PT_DIGITAL, SIG_TRAV_ENABLE, "TravEnable" },
 { 2, PD_OUTPUT, PT_DIGITAL, SIG_TRAV_DIR, "TravDir" },
 { 3, PD_OUTPUT, PT_DIGITAL, SIG_TRAV_STEP, "TravStep" },
 { 4, PD_OUTPUT, PT_DIGITAL, SIG_SPIN_STEP, "SpinStep" },
 { 5, PD_OUTPUT, PT_DIGITAL, SIG_SPIN_ENABLE, "SpinEnable" },
 { 6, PD_OUTPUT, PT_DIGITAL, SIG_SPIN_DIR, "SpinDir" },
 { 7, PD_OUTPUT, PT_DIGITAL, SIG_LCD1, "LCD1" },
 { 8, PD_OUTPUT, PT_DIGITAL, SIG_LCD2, "LCD2" },
 { 9, PD_OUTPUT, PT_DIGITAL, SIG_LCD3, "LCD3" },
 { 10, PD_OUTPUT, PT_DIGITAL, SIG_LCD4, "LCD4" },
 { 11, PD_OUTPUT, PT_DIGITAL, SIG_LCD5, "LCD_5" },
 { 12, PD_OUTPUT, PT_DIGITAL, SIG_LCD6, "LCD_6" },
 { 13, PD_OUTPUT, PT_DIGITAL, SIG_TEENSY_LED, "TeensyLED" },
 { 14, PD_INPUT, PT_DIGITAL, SIG_LIMIT_SW2, "LimitSw2" },
 { 15, PD_INPUT, PT_DIGITAL, SIG_LIMIT_SW1, "LimitSw1" },
 { 16, PD_INPUT, PT_ANALOG, SIG_SPIN_SPEED, "SpinSpeed", &analog_filter1 },
 { 17, PD_INPUT, PT_ANALOG, SIG_TRAV_RATE, "TravRate", &analog_filter2 },
 { 18, PD_INPUT, PT_DIGITAL, SIG_ENCODER_B, "EncoderB" },
 { 19, PD_INPUT, PT_DIGITAL, SIG_ENCODER_A, "EncoderA" },
 { 20, PD_INPUT, PT_DIGITAL, SIG_ENCODER_BUTTON, "EncButton" },
 { 21, PD_INPUT, PT_DIGITAL,  SIG_COUNT_RESET, "CntReset" },
 { 22, PD_INPUT, PT_DIGITAL, SIG_MAX_DOWN, "Max Down" },
 { 23, PD_INPUT, PT_DIGITAL, SIG_MAX_UP, "Max Up" },
 { 24, PD_OUTPUT, PT_DIGITAL, SIG_LLIM_LED, "LLim LED" },
 { 25, PD_INPUT, PT_DIGITAL, SIG_SET_LIM_R, "SetMaxR" },
 { 26, PD_INPUT, PT_DIGITAL, SIG_STOP, "Stop" },
 { 27, PD_INPUT, PT_DIGITAL, SIG_WIND_DIR, "WindDir" },
 { 28, PD_OUTPUT, PT_DIGITAL, SIG_STROBE, "Strobe" },
// { 29, PD_UNUSED, PT_DIGITAL, "_unused2_" },
 { 30, PD_OUTPUT, PT_DIGITAL, SIG_RLIM_LED, "RLim LED" },
 { 31, PD_INPUT, PT_DIGITAL, SIG_START, "Start" },
 { 32, PD_INPUT, PT_DIGITAL, SIG_SET_LIM_L, "SetLimL" },
#if 0 // DONT ENABLE: This made it not program when socketed!
 { 33, PD_OUTPUT, PT_DIGITAL, SIG_RLIM_LED, "RLim LED" },
#endif
 { A12, PD_INPUT, PT_ANALOG, SIG_RLIM, "RLim", &analog_filter3 },
 { A13, PD_INPUT, PT_ANALOG, SIG_LLIM, "LLim", &analog_filter4 },
};

#define SPINDLE_STEPS_PER_REV 2000
#define SPINDLE_INITIAL_PERIOD 65000
#define TRAV_STEPS_PER_REV 200
#define TRAV_MAX_ACCEL_STEPS 125
#define TRAV_INITIAL_PERIOD 2000
#define TRAV_MAX_ACCEL_STEPS_HFAST 10
#define TRAV_LIMSW_OPEN_TO_AXIS_ZERO 5
#define TRAV_LIM_TO_LIM (TRAV_STEPS_PER_REV * 40)
#define H_CLOSE2_MARGIN TRAV_LIMSW_OPEN_TO_AXIS_ZERO

#define TRAV_DISTANCE_THOU 1882

typedef enum {
  MOT_STOPPED,
  MOT_ACCEL,
  MOT_AT_MAX,
  MOT_DECEL
} motor_state_t;

typedef void (EnableStepDriver)(int val);
typedef int (SendStepPulse)(void);
typedef void (SetDirection)(int dir);
typedef void (StartTimer)(int init, unsigned long period);
typedef void (StopTimer)(void);
typedef int (GetTimerHz)(void);
typedef int (CheckLimit)(void);

typedef struct MotFunctions {
	EnableStepDriver	*enable_driver;
	SendStepPulse		*send_step;
	SetDirection		*set_direction;
	StartTimer			*start_timer;
	StopTimer			*stop_timer;
	GetTimerHz			*get_timer_hz;
	CheckLimit			*check_limit1;
	CheckLimit			*check_limit2;
} MotFunctions_t;

/***************************  FTM Spindle Specifics: ********************/

#define FTM_CLOCK_PRESCALE 0
#define FTM_STEPS_PER_IRQ 2
#define FTM_STEP_PIN 4
void SetSpindleEnable(int enable)
{
	digitalWrite(pin_map[SIG_SPIN_ENABLE].num, enable);
}

void SetSpindleDir(int dir)
{
	digitalWrite(pin_map[SIG_SPIN_DIR].num, dir > 0 ? 1 : 0);
}

void SpindleStartFtmTimer(int init, unsigned long period)
{
	if (init) {
	  // Enable clock to FTM1
	  SIM_SCGC6 |= SIM_SCGC6_FTM1;

	  // Init FTM 1 count/mod/mode
	  FTM1_CNTIN = 0;
	  FTM1_CNT = 0;
	  FTM1_MOD = 65535;
	  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(FTM_CLOCK_PRESCALE) | FTM_SC_TOIE;
	  FTM1_CONF = FTM_STEPS_PER_IRQ - 1;  // set NUMTOF

	  // Configure channel 1:
	  FTM1_C1V = 0;
	  FTM1_C1SC = 0x14;
	  *portConfigRegister(FTM_STEP_PIN) = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
	  NVIC_ENABLE_IRQ(IRQ_FTM1);
	}
	FTM1_SC = FTM1_SC & ~(FTM_SC_TOF);
	FTM1_MOD = period;
}

void SpindleSetFtmPeriod(unsigned long period)
{
	FTM1_MOD = period;
}

void SpindleStopFtmTimer(void)
{
	FTM1_SC = FTM_SC_PS(FTM_CLOCK_PRESCALE) | FTM_SC_TOIE;
}

int SpindleGetTimerHz(void)
{
	int hz = F_CPU / 2;
	hz /= (1 << FTM_CLOCK_PRESCALE) * FTM1_MOD * 2;
	return hz;
}

MotFunctions SpindleMotFunctions = {
	SetSpindleEnable,
	NULL,
	SetSpindleDir,
	SpindleStartFtmTimer,
	SpindleStopFtmTimer,
	SpindleGetTimerHz,
};

/***************************  Trav Specifics: ********************/
#if 1
/* This is the interval timer version. */
IntervalTimer g_travTimer;
unsigned long last_period = 0;
int travIsrs;
void TravIsr(void);

void SetTravEnable(int enable)
{
	if (enable >= 0) {
		digitalWrite(pin_map[SIG_TRAV_ENABLE].num, 1); // just keep traversal enabled
	} else {
		digitalWrite(pin_map[SIG_TRAV_ENABLE].num, 0); // just keep traversal enabled
	}
}

int TravSendStep(void)
{
	volatile static int last_val = 0;
	if (last_val == 0) {
		last_val = 1;
	} else {
		last_val = 0;
	}
	digitalWrite(pin_map[SIG_TRAV_STEP].num, last_val);
	return last_val;
}

void SetTravDir(int dir)
{
	digitalWrite(pin_map[SIG_TRAV_DIR].num, dir > 0 ? 1 : 0);
}

void TravStartTimer(int init, unsigned long period)
{
	last_period = period;
	g_travTimer.begin(TravIsr, period);
}

void TravStopTimer(void)
{
	g_travTimer.end();
}

int TravGetTimerHz(void)
{
	return F_CPU / last_period;
}

int TravCheckLimit1(void)
{
	return digitalRead(pin_map[SIG_LIMIT_SW1].num);
}

int TravCheckLimit2(void)
{
	return digitalRead(pin_map[SIG_LIMIT_SW2].num);
}

MotFunctions TravMotFunctions = {
	SetTravEnable,
	TravSendStep,
	SetTravDir,
	TravStartTimer,
	TravStopTimer,
	TravGetTimerHz,
	TravCheckLimit1,
	TravCheckLimit2
};

#else
/* This is the PIT version of traversal: */
void TravIsr();
int travIsrs;
void pit0_isr();
void pit0_isr(void)
{
	travIsrs++;
	TravIsr();
}

void SetTravEnable(int enable)
{
	digitalWrite(pin_map[SIG_TRAV_ENABLE].num, enable);
}

int TravSendStep(void)
{
	volatile static int last_val = 0;
	if (last_val == 0) {
		last_val = 1;
	} else {
		last_val = 0;
	}
	digitalWrite(pin_map[SIG_TRAV_STEP].num, last_val);
	return last_val;
}

void SetTravDir(int dir)
{
	digitalWrite(pin_map[SIG_TRAV_DIR].num, dir);
}

void TravStartTimer(int init, unsigned long period)
{
	if (init) {
		PIT_TCTRL0 = 0;
		// enable
		SIM_SCGC6 |= SIM_SCGC6_PIT;
		PIT_MCR = 0;
		NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
	}
	PIT_LDVAL0 = period;
	PIT_TCTRL0 = 3;
}

void TravStopTimer(void)
{
	PIT_MCR = 1;
	SIM_SCGC6 &= ~SIM_SCGC6_PIT;
	// disable interrupt and PIT
	NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
	PIT_TCTRL0 = 0;
}

int TravGetTimerHz(void)
{
	return F_CPU / PIT_LDVAL0;
}

MotFunctions TravMotFunctions = {
	SetTravEnable,
	TravSendStep,
	SetTravDir,
	TravStartTimer,
	TravStopTimer,
	TravGetTimerHz,
};

#endif
/***************************  Motor Specifics: ********************/
typedef void (DoneCallback)(void *);

class Motor {
//private:
public:
	MotFunctions_t func;
	volatile int estopped;
	volatile motor_state_t	mot_state;
	volatile unsigned long timer_period;
	volatile unsigned long min_period;	// Set when threshold is crossed.
	volatile unsigned long accel_steps;
	volatile unsigned long accel_remainder;
	volatile unsigned long max_accel_steps;
	volatile unsigned long min_accel_steps = 0;
	volatile int direction = 1;
	volatile int pos = 0;
	volatile int cmd_pos = 0;
	volatile int next_cmd_pos = 0;	// Used for when changing directions
	volatile unsigned long initial_period = 2000;
	int accel;
	unsigned long GetStepsToGo(void);
	void SpeedUpdateInISR(void);
	void AccountForStep(void);
	DoneCallback	*done_callback;
	void			*done_cookie;
//public:
	void SetMotorFunctions(MotFunctions_t *mf) { func = *mf; }
 	void OverwriteCmdCurPos(int p) { pos = cmd_pos = next_cmd_pos = p; }
	void ClearCounter(void) { cmd_pos -= pos; next_cmd_pos -= pos; pos = 0; }
	int GetCurPos(void) { return pos; }
	int GetCmdPos(void) { return next_cmd_pos; } // queued pos for reversal case
	int SetCmdPos(int);
	void SetMaxSpeed(int s) { max_accel_steps = (unsigned long)s; }
	void SetInitialPeriod(unsigned long p) { initial_period = p; }
	void SetAccel(int a) { accel = a; }
	void TimerCallback(void);
	int GetTimerHz(void);
	int IsStopped(void) { return (mot_state == MOT_STOPPED); }
	void Stop(void);
	void EStop(int);
	void RegisterDoneCallback(DoneCallback *cb, void *c) { done_callback = cb; done_cookie = c; }
};

unsigned long Motor::GetStepsToGo(void)
{
	int to_go = direction * (cmd_pos - pos);
	//assert(to_go >= 0);
	if (to_go < 0) {
		return 0;
	}
	return (unsigned long)to_go;
}

void Motor::AccountForStep(void)
{
	pos += direction * 1;
}

void Motor::SpeedUpdateInISR(void)
{
	unsigned long num, den;

	switch (mot_state) {
	case MOT_STOPPED:
		func.stop_timer();
		// Leaving driver enabled, to see if this gets hit: func.enable_driver(0); 
		break;
	case MOT_ACCEL:
		accel_steps++;
		num = (2 * timer_period + accel_remainder);
		den = (4 * (accel_steps + min_accel_steps) + 1);
		timer_period -= (num / den);
		accel_remainder = num % den;
		if (accel_steps >= max_accel_steps) {
			mot_state = MOT_AT_MAX;
		}
		/* Fallthrough to evaluate if it's time to stop. */
	case MOT_AT_MAX:
		if (accel_steps < GetStepsToGo()) {
			/* Not time to start braking */
			if (accel_steps < max_accel_steps) {
				/* Speed was increased.*/
				mot_state = MOT_ACCEL;
				break;
			}
			if (accel_steps == max_accel_steps) {
				/* Speed was not reduced since we hit peak speed. */
				break;
			}
		}
		mot_state = MOT_DECEL;
		/* Fallthrough to decelerate: */
	case MOT_DECEL:
		if (GetStepsToGo() > accel_steps) {
			/* Don't need to brake yet, see if we should accelerate. */
			if (accel_steps < max_accel_steps) {
				/* Not at max speed, which must have just been raised */
				mot_state = MOT_ACCEL;
				break;
			}
		}
		if (accel_steps > 0) {
			accel_steps--;
		}
		num = (2 * timer_period + accel_remainder);
		den = (4 * (accel_steps + min_accel_steps) + 1);
		timer_period += (num / den);
		accel_remainder = num % den;
		if (timer_period >= initial_period) {
			timer_period = initial_period;
			accel_remainder = 0;
			//assert(accel_steps == 0);
			accel_steps = 0;
		}
		if (GetStepsToGo() == 0) {
			// assert(timer_period == initial_period);
			mot_state = MOT_STOPPED;
			//assert(pos == cmd_pos);
			if (next_cmd_pos != cmd_pos) {
				/* Leave timer/driver enabled, this will reverse direction and start. */
				SetCmdPos(next_cmd_pos);
				return;
			}
			func.stop_timer();
			func.enable_driver(0);
			if (done_callback != NULL) {
				done_callback(done_cookie);
			}
			return;
		}
		break;
	default:
		break;
	}

	/* Restart timer, return early if not wanted. */
	func.start_timer(0, timer_period);
}

int Motor::SetCmdPos(int new_pos)
{
	int to_go = new_pos - pos;
	if (estopped || cmd_pos == new_pos) {
		return 0;
	}
	if (mot_state == MOT_STOPPED) {
		cmd_pos = next_cmd_pos = new_pos;
		if (to_go == 0) {
			return 0;
		}
		if (to_go >= 0) {
			direction = 1;
		} else {
			direction = -1;
		}
		func.set_direction(direction);
		func.enable_driver(1);
		mot_state = MOT_ACCEL;
		accel_steps = 0;
		accel_remainder = 0;
		timer_period = initial_period;
		func.start_timer(1, timer_period);
	}

	/* Moving, see soonest we could stop in case we need
	 * to change directions. */
	int stop_at = pos + (direction * accel_steps);
	if ((direction * stop_at) < (direction * new_pos)) {
		/* Don't need to change directions. */
		if (abs(to_go) > accel_steps) {
			/* Keep going: accelerate to minimum braking distance*/
			mot_state = MOT_ACCEL;
		} else {
			/* Should only be exact case already at minimum braking distance */
			//assert(to_go == accel_steps);
			mot_state = MOT_DECEL;
		}
		cmd_pos = next_cmd_pos = new_pos;
	} else {
		/* Stop to reverse direction, queue up next cmd pos */
		mot_state = MOT_DECEL;
		cmd_pos = stop_at;
		next_cmd_pos = new_pos;
	}
	return to_go;
}

void Motor::TimerCallback(void)
{
	if (estopped) {
		/* Just in case we somehow started again after EStop(): */
		func.stop_timer();
		func.enable_driver(-1);
	}
	if (func.send_step != NULL) {
		if (func.send_step()) {
			/* De-asserting step output. */
			return;
		}
	}
	AccountForStep();
	SpeedUpdateInISR();
}

int Motor::GetTimerHz(void)
{
	if (mot_state == MOT_STOPPED) {
		return 0;
	}
	return func.get_timer_hz();
}

void Motor::Stop(void)
{
	if (mot_state > MOT_STOPPED) {
		mot_state = MOT_DECEL;
		cmd_pos = next_cmd_pos = pos + (direction * accel_steps);
	}
}

void Motor::EStop(int val)
{
	estopped = val;
	if (estopped) {
		mot_state = MOT_STOPPED;
		func.stop_timer();
		func.enable_driver(-1);
		if (done_callback != NULL) {
			done_callback(done_cookie);
		}
	}
}

/***************************  Axis Specifics: ********************/
typedef enum {
	HOMING_NOT_STARTED,
	HOMING_1_INITIAL_CLOSE,
	HOMING_2_INITIAL_BACKOFF,
	HOMING_3_SLOW_CLOSE,
	HOMING_4_SLOW_BACKOFF,
	HOMING_5_CLOSE_OTHER_SIDE,
	HOMING_6_OPEN_OTHER_SIDE,
	HOMING_7_TO_HOME,
	HOMING_ERROR,
	HOMED
} homing_state_t;

class Axis {
public:
	Motor			motor;
	homing_state_t	homing_state;
	int				estopped;
	int				h_close1;	// Homing positions on switch changes
	int				h_open1;
	int				h_close2;
	int				h_open2;
	int				hard_limit[2]; // Min is [0] max is [1]
	int				steps_per_rev;
	MotFunctions	mot_functions;
	Axis(MotFunctions_t *mf, int s) {	mot_functions = *mf;	// Save a copy
										motor.SetMotorFunctions(mf);
										steps_per_rev = s; }
	void StartHomingCycle(void);
	void HomingStateHandler(void);
	homing_state_t GetHomedStatus(void) { return homing_state; }
	void ZeroAxis(void) { motor.OverwriteCmdCurPos(0); }
	void ClearCounter(void) { motor.ClearCounter(); }
	int GetMaxPos(void) { return (h_close2 - H_CLOSE2_MARGIN); }
	int GetCurPos(void) { return motor.GetCurPos(); }
	int GetCmdPos(void) { return motor.GetCmdPos(); }
	int GetTurns(void)  { return (motor.GetCurPos() / steps_per_rev); }
	void SetInitialPeriod(int m) { motor.SetInitialPeriod(m); }
	void SetMaxSpeed(int s) { motor.SetMaxSpeed(s); }
	int GetRpm(void);
	int MoveTo(int pos) { return motor.SetCmdPos(pos); }
	int MoveTurns(int t) { return MoveTo(GetCmdPos() + (t * steps_per_rev)); }
	int IsStopped(void) { return motor.IsStopped(); }
	void Stop(void) { motor.Stop(); }
	void EStop(int v) { estopped = v; motor.EStop(v); }
	void TimerCallback(void);
	void RegisterDoneCallback(DoneCallback *cb, void *c) { motor.RegisterDoneCallback(cb, c); }
};

void Axis::StartHomingCycle(void)
{
	if (homing_state != HOMING_NOT_STARTED) {
		return;
	}
	motor.SetInitialPeriod(TRAV_INITIAL_PERIOD);
	SetMaxSpeed(1);
	homing_state = HOMING_1_INITIAL_CLOSE;
	MoveTurns(-10);		// XXX Constant for max homing distance
	/* From here it is event driven */
}

void Axis::TimerCallback(void)
{
	motor.TimerCallback();
	if (!mot_functions.check_limit1 || !mot_functions.check_limit2) {
		return;
	}
	if (homing_state == HOMED) {
		if (DIG_ASSERTED == mot_functions.check_limit1() ||
			DIG_ASSERTED == mot_functions.check_limit2())
		{
			/* Emergency shutdown! */
			motor.EStop(1);
		}
		return;
	}

	switch(homing_state) {
		case HOMING_1_INITIAL_CLOSE:
			if (DIG_ASSERTED == mot_functions.check_limit1()) {
				h_close1 = GetCurPos();
				homing_state = HOMING_2_INITIAL_BACKOFF;
#define HOMING_BACKOFF_DISTANCE TRAV_STEPS_PER_REV
				MoveTo(h_close1 + HOMING_BACKOFF_DISTANCE);
			}
			break;
		case HOMING_2_INITIAL_BACKOFF:
			if (DIG_ASSERTED != mot_functions.check_limit1()) {
				h_open1 = GetCurPos();
				homing_state = HOMING_3_SLOW_CLOSE;
				MoveTo(h_close1 - HOMING_BACKOFF_DISTANCE);
			}
			break;
		case HOMING_3_SLOW_CLOSE:
			if (DIG_ASSERTED == mot_functions.check_limit1()) {
				h_close1 = GetCurPos();
				homing_state = HOMING_4_SLOW_BACKOFF;
				MoveTo(h_open1 + HOMING_BACKOFF_DISTANCE);
			}
			break;
		case HOMING_4_SLOW_BACKOFF:
			if (DIG_ASSERTED != mot_functions.check_limit1()) {
				h_open1 = GetCurPos();
				homing_state = HOMING_5_CLOSE_OTHER_SIDE;
				/* This is it, zero axis by resetting to slightly negative,
				 * so that we're not always right on the limit switch. */
				//motor.Stop();
				motor.OverwriteCmdCurPos(-TRAV_LIMSW_OPEN_TO_AXIS_ZERO);
				SetMaxSpeed(TRAV_MAX_ACCEL_STEPS_HFAST);
				MoveTo(h_open1 + TRAV_LIM_TO_LIM);
			}
			break;
		case HOMING_5_CLOSE_OTHER_SIDE:
			if (DIG_ASSERTED == mot_functions.check_limit2()) {
				h_close2 = GetCurPos();
				motor.SetMaxSpeed(TRAV_MAX_ACCEL_STEPS);
				MoveTo(0);
				homing_state = HOMING_6_OPEN_OTHER_SIDE;
			}
			break;
		case HOMING_6_OPEN_OTHER_SIDE:
			if (DIG_ASSERTED != mot_functions.check_limit2()) {
				h_open2 = GetCurPos();
				homing_state = HOMING_7_TO_HOME;
			}
			break;
		case HOMING_7_TO_HOME:
			if (IsStopped()) {
				homing_state = HOMED;
			}
			break;
		default:
			break;
	}
}

int Axis::GetRpm(void)
{
	return (motor.GetTimerHz() * 60 / steps_per_rev);
}

/***************************  MENU specifics: ********************/
typedef struct {
	int num;
	int winds;
	int trav;
} ProgOp;

typedef enum {
	DATA_NONE,
	DATA_YESNO,
	DATA_STRING,
	DATA_UINT,
	DATA_INT,
} ItemData;

typedef enum {
	MENU_OP_SELECT,
	MENU_OP_ADJUST,
	MENU_OP_GETVAL,
	MENU_OP_DRAW,
} MenuOp;

typedef int (MenuFunc)(class Menu *m, struct MenuItem *mi, MenuOp op);

/* All the handlers are located far below so that they can
 * reference g_system, etc. */
int ExitSelect(Menu *m, MenuItem *mi, MenuOp op);
int ProgramModeSelect(Menu *m, MenuItem *mi, MenuOp op);
int WindingProgMenu(Menu *m, MenuItem *mi, MenuOp op);
int TravProgMenu(Menu *m, MenuItem *mi, MenuOp op);
int SaveConfigProgMenu(Menu *m, MenuItem *mi, MenuOp op);
int SaveCountProgMenu(Menu *m, MenuItem *mi, MenuOp op);

#define MENU_WIDTH		16
#define MENU_HEIGHT		2
typedef struct MenuItem {
	char		name[MENU_WIDTH + 1];
	union {
		char 			*dstr;
		int				dyesno;
		unsigned int 	duint;
		int 			dint;
	} data;
	int			external_id;	// app specific int
	ItemData	data_type;
	MenuFunc	*func;	// Enter is hit on this item
} MenuItem;


#define MENU_PROG_ITEMS 10
#define MAX_ITEMS 2 + (MENU_PROG_ITEMS * 2) + 2

class Menu {
private:
	MenuItem	items[MAX_ITEMS];	// Storage
	MenuItem	*display_top;	// top/left item in menu
	int			editing = 0;
	int			items_used = 0;
	int			prev_encoder_val = 0;
	int			exit_requested = 0;
	unsigned int screen_redraws = 0;
public:
	int			ui_diff;
	MenuItem	*selected;		// may equal display_top
	Menu(void);
	MenuItem *AddMenuItem(void);
	void DelMenuItem(MenuItem *m);
	MenuItem *ItemNext(MenuItem *mi);
	MenuItem *ItemPrev(MenuItem *mi);
	void ConvertData(MenuItem *item, char *buf);
	void DrawScreen(LiquidCrystal *lcd);
	void RequestExit() { exit_requested = 1; }
	void ResetMenu(void);
	int ItemOnScreen(MenuItem *item);
	int RunMenu(int encoder_val, int encoder_button);
};

Menu::Menu(void)
{
	/* Create basic menu items. */
	MenuItem *exit;
	MenuItem *prog;
	items_used = 0;
	if (NULL == (exit = AddMenuItem()) ||
		NULL == (prog = AddMenuItem()))
	{
		/* Don't care about freeing partial allocation,
		 * this will "never" happen. :) */
		return;
	}
	strncpy(exit->name, "Exit menu mode  ", MENU_WIDTH);
	exit->data_type = DATA_NONE;
	exit->func = ExitSelect;
	selected = exit;

	strncpy(prog->name, "Program Mode: ", MENU_WIDTH);
	prog->data_type = DATA_YESNO;
	prog->func = ProgramModeSelect;

	MenuItem *wind;
	MenuItem *trav;
	int i;
	for (i = 0; i < MENU_PROG_ITEMS && (wind = AddMenuItem()) && (trav = AddMenuItem()); i++) {
		snprintf(wind->name, MENU_WIDTH, "OP%d winds:", i + 1);
		wind->data_type = DATA_INT;
		wind->func = WindingProgMenu;
		wind->external_id = i;

		snprintf(trav->name, MENU_WIDTH, "OP%d trav:.", i + 1);
		trav->data_type = DATA_INT;
		trav->func = TravProgMenu;
		trav->external_id = i;
	}
	MenuItem *save = AddMenuItem();
	MenuItem *info = AddMenuItem();
	if (save && info) {
		snprintf(save->name, MENU_WIDTH, "Save Config");
		save->data_type = DATA_NONE;
		save->func = SaveConfigProgMenu;
		snprintf(info->name, MENU_WIDTH, "Num Saves");
		info->data_type = DATA_UINT;
		info->func = SaveCountProgMenu;
	}
}

MenuItem *Menu::AddMenuItem(void)
{
	if (items_used < MAX_ITEMS) {
		return &items[items_used++];
	}
	return NULL;
}

void Menu::DelMenuItem(MenuItem *m)
{
	if (m < &items[0] || m > &items[MAX_ITEMS-1] || items_used <= 0) {
		return;
	}
	/* decrement first, to convert count to index. */
	items_used--;
	if (m != &items[items_used]) {
		memmove(m + 1, m, (&items[items_used] - m) * sizeof(m));
	}
}

MenuItem *Menu::ItemNext(MenuItem *mi)
{
	mi++;
	if (mi >= &items[items_used]) {
		mi = items;
	}
	return mi;
}

MenuItem *Menu::ItemPrev(MenuItem *mi)
{
	if (mi <= items) {
		mi = &items[items_used];
	}
	mi--;
	return mi;
}

void Menu::ConvertData(MenuItem *item, char *buf)
{
	/* Convert data to string */
	switch(item->data_type) {
		case DATA_YESNO:
			if (item->data.dyesno) {
				strcpy(buf, "Y");
			} else {
				strcpy(buf, "N");
			}
			break;
		case DATA_INT:
			snprintf(buf, MENU_WIDTH, "%4.4d", item->data.dint);
			break;
		case DATA_UINT:
			snprintf(buf, MENU_WIDTH, "%u", item->data.duint);
			break;
		case DATA_STRING:
			strncpy(buf, item->data.dstr, MENU_WIDTH);
			break;
		default:
			buf[0] = 0;
			break;
	}
}

void Menu::DrawScreen(LiquidCrystal *lcd)
{
	MenuItem *item;
	char buf[MENU_WIDTH+1];
	int y;

	item = display_top;
	for (y = 0; y < MENU_HEIGHT; y++) {
		/* Call DRAW op to update value. */
		item->func(this, item, MENU_OP_DRAW);
		ConvertData(item, buf);
		if ((item == selected) && editing &&
			(screen_redraws % 30 < 5))	// Blink duty cycle
		{
			buf[0] = 0;
		}
		lcd->setCursor(0, y);
		lcd->printf("%c%s%s               ", (selected == item) ? '>' : ' ', item->name, buf);
		item = ItemNext(item);
	}
	screen_redraws++;
}

void Menu::ResetMenu(void)
{
	editing = 0;
	selected = items;
	display_top = items;
}

int Menu::ItemOnScreen(MenuItem *item)
{
	MenuItem *mi = display_top;
	int i;
	for (i = 0; i < MENU_HEIGHT; i++, mi = ItemNext(mi)) {
		if (item == mi) {
			return 1;
		}
	}
	return 0;
}

int Menu::RunMenu(int encoder_val, int encoder_button)
{
	if (encoder_button) {
		/* Toggle editing mode */
		if (selected->data_type != DATA_NONE) {
			editing = !editing;
		}
		selected->func(this, selected, MENU_OP_SELECT);
	}
	ui_diff = encoder_val - prev_encoder_val;
	if (ui_diff != 0) {
		prev_encoder_val = encoder_val;
		if (!editing) {
			if (ui_diff < 0) {
				selected = ItemNext(selected);
				if (((selected - items) % MENU_HEIGHT) == 0) {
					display_top = selected;
				}
			} else if (ui_diff > 0) {
				selected = ItemPrev(selected);
				if (!ItemOnScreen(selected)) {
					/* Iterate from the start, so it's same interval as menu_height. */
					/* Should be smarter about this. */
					display_top = items;
					int loop = 0;
					while (!ItemOnScreen(selected) && (loop++ < 1000)) { // Crappy exit on while(1)
						int i;
						for (i = 0; i < MENU_HEIGHT; i++) {
							display_top = ItemNext(display_top);
						}
					}
				}
			}
		}
	}
	if (editing) {
		selected->func(this, selected, MENU_OP_ADJUST);
	}

	/* Draw screen */
	DrawScreen(&lcd);
	if (exit_requested) {
		editing = 0;
		exit_requested = 0;
		return 1;
	}
	return 0;
}

Menu g_menu;

/***************************  System Specifics: ********************/


#define SCALE_COUNTER(prev, val, scale) \
	{	static int cnt; \
		if ((prev) != DIG_ASSERTED) { (cnt) = 0; } \
		cnt++; \
		if (cnt > 500) { (val) += (scale) * 20; } else \
		if (cnt > 400) { (val) += (scale) * 5; } else \
		if (cnt > 300) { (val) += (scale) * 1; } else \
		if (cnt > 200) { (val) += (scale) * (cnt & 0x1); } else \
		if (cnt > 100) { (val) += (scale) * ((cnt & 0x3) == 1); } else \
		{ (val) += (scale) * ((cnt & 0x1f) == 1); } \
	}

typedef enum {
	MODE_HOMING,
	MODE_STOP,
	MODE_SET_LIM_L,
	MODE_SET_LIM_R,
	MODE_RUN,
	MODE_MENU,
} sys_mode_t;

const char *sys_state_str[MODE_RUN+1] = {
	"Homing",
	"Stop",
	"LLim",
	"RLim",
	"Run",
};

#define MAX_PROG 10
typedef struct Prog {
	unsigned int	windings;
	unsigned int	traversal;
} Prog;

#define CONFIG_VERSION 1
#define CHECK_VALUE 0x43214321
typedef struct Nvconfig {
	unsigned int version;
	unsigned int check_value;
	unsigned int config_saves;
	int			nv_winding_limit;
	int			nv_lim_l;
	int			nv_lim_r;
	int			nv_program_mode;
	int			nv_number_of_ops;
	Prog		nv_program[MAX_PROG];

	int			unused_padding;
} Nvconfig;

Nvconfig g_nvconfig;

class System {
	sys_mode_t	sys_state = MODE_HOMING;
	int 		program_mode;	// Boolean, runs program steps if enabled.
	volatile int program_counter;
	volatile unsigned int prog_op_winds;
	Prog		program[MAX_PROG];
	int			winding_limit;
	int			winding_count = 0;
	int 		lim_r;
	int			lim_l;
	int			trav_dir;
	int			trav_accumulator;
	int			trav_rate;
	int			trav_rate_tenths;
	int			trav_valid_config;
	volatile int half_winding_events;
#define TRAV_STEP_COST 100
#define MAX_TRAV_RATE 1024 * 2
#define MAX_TRAV_ACCUMULATOR (2 * MAX_TRAV_RATE)
	int			trav_step_cost = TRAV_STEP_COST;
public:
	Axis		*spindle;
	Axis		*trav;
	LiquidCrystal *lcd;
	Menu		*menu;
//public:
	System(Axis *s, Axis *t, LiquidCrystal *l);
	void PostInit(void);
	void RunLoop(void);
	int UpdateTravLim(int sig);
	int TravValidConfig(void);
	int GetTravRate(void)	{ return trav_rate; }
	void UpdateTravRate(int);
	int GetTravRateInTenThou();
	void SetTravRateInTenThou(int ten_thou);
	void DoneCallback(void) { sys_state = MODE_STOP; }
	int HandleStrobe(void);
	void HalfWindingComplete(void);
	void DoneMoveTrav(void);
	void DrawMenu(void);
	int GetProgramMode(void) { return program_mode != 0; }
	void SetProgramMode(int value) { program_mode = value; }
	void SetProgWinding(int id, unsigned int value);
	void SetProgTrav(int id, unsigned int value);
	int GetProgWinding(int id);
	int GetProgTrav(int id);
	void UpdateProgModeWindingLimit(void);
	void ResetProgMode(void);
	int ResumeProgram(void);
	void LoadConfig(void);
	void InitConfig(void);
	void SaveConfig(void);
};

void System::DoneMoveTrav(void)
{
	if (winding_count >= winding_limit) {
		if (TravValidConfig()) {
			trav->MoveTo(lim_l);
		}
	}
}

void SystemDoneCallback(void *arg)
{
	System *s = (System*)arg;
	/* Turn off the LED */
	digitalWrite(pin_map[SIG_STROBE].num, 0);
	s->DoneCallback();
	s->DoneMoveTrav();
}

void TravReversal(void *arg)
{

}

System::System(Axis *s, Axis *t, LiquidCrystal *l)
{
	spindle = s;
	trav = t;
	lcd = l;
	menu = &g_menu;
	s->RegisterDoneCallback(SystemDoneCallback, (void*)this);
	t->RegisterDoneCallback(TravReversal, (void*)this);
	/* Init a few things: */
	winding_limit = 6000;
	lim_l = 30;
	lim_r = 120;
	PostInit();
}

void System::PostInit()
{
	lcd->begin(16, 2);
	lcd->clear();
	lcd->clear();
	lcd->setCursor(0, 0);
	lcd->printf("Frickup winder   ");
	lcd->setCursor(0, 1);
	lcd->printf("Version %d       ", SOFTWARE_VERSION);
	delay(1000);
	LoadConfig();
}

int System::UpdateTravLim(int sig)
{
	int pos;
	/* First, invert the position: */
	pos = 1024 - pin_map[sig].value;
	/* Second scale it */
	pos = pos * trav->GetMaxPos() / 1023;
	trav->MoveTo(pos);
	return pos;
}

int System::TravValidConfig(void)
{
	return (lim_l < lim_r);
}

void System::UpdateTravRate(int sig)
{
	int v = pin_map[sig].value;
	//trav_rate = 10 + ((v > 512) ? v + ((v - 512) * 2) : v);
    if (v > 768) {
		v += 16 * (v - 768);
	}
	trav_rate_tenths = 10 + v / 6;
	SetTravRateInTenThou(trav_rate_tenths);
}

int System::GetTravRateInTenThou(void)
{
	return trav_rate_tenths;
}

void System::SetTravRateInTenThou(int ten_thou)
{
	trav_rate_tenths = ten_thou;
	trav_rate = ten_thou * (trav->GetMaxPos() * TRAV_STEP_COST) / (TRAV_DISTANCE_THOU * 10);
}

void System::RunLoop(void)
{
	/* State independent checks: */
#define SPINDLE_SPEED_FACTOR 5
#define SPINDLE_MIN_SPEED 100
	spindle->SetMaxSpeed(pin_map[SIG_SPIN_SPEED].value * SPINDLE_SPEED_FACTOR + SPINDLE_MIN_SPEED);
	if (sys_state != MODE_HOMING) {
		if (pin_map[SIG_LIMIT_SW1].value == DIG_ASSERTED ||
			pin_map[SIG_LIMIT_SW2].value == DIG_ASSERTED)
		{
			trav->EStop(1);
		}
	}

	switch (sys_state) {
	case MODE_HOMING:
		/* Only the traversal gets homed */
		homing_state_t hs;
		hs = trav->GetHomedStatus();
		if (hs == HOMING_NOT_STARTED) {
			trav->StartHomingCycle();
		}
		if (hs == HOMED) {
			sys_state = MODE_STOP;
			return;
		}
		if (pin_map[SIG_STOP].value == STOP_ASSERTED) {
			trav->EStop(1);
		}
		lcd->setCursor(0, 0);
		lcd->printf("HOMING %d        ", (int)hs);
		lcd->setCursor(0, 1);
		lcd->printf("pos=%d dir=%d     ", trav->GetCurPos(), trav->motor.direction);
		return;
	case MODE_STOP:
		if (pin_map[SIG_MAX_UP].value == DIG_ASSERTED) {
			SCALE_COUNTER(pin_map[SIG_MAX_UP].prev_value, winding_limit, 1);
		}
		if (pin_map[SIG_MAX_DOWN].value == DIG_ASSERTED) {
			SCALE_COUNTER(pin_map[SIG_MAX_DOWN].prev_value, winding_limit, -1);
		}
		if (pin_map[SIG_COUNT_RESET].value == DIG_ASSERTED) {
			/* Clear counter */
			//spindle->ClearCounter();
			winding_count = 0;
			half_winding_events = 0;
			/* Reset program mode vars: */
			ResetProgMode();
		}
		if (pin_map[SIG_START].value == DIG_ASSERTED &&
			spindle->IsStopped())
		{
SetRunMode:
			int move;
			if (winding_count > winding_limit) {
				move = 0;
			} else {
				move = winding_limit - (winding_count);
			}
			if (!pin_map[SIG_WIND_DIR].value) {
				move = -move;
			}
			if (move != 0) {
				/* Change state to run */
				ResumeProgram();
				sys_state = MODE_RUN;
				spindle->MoveTurns(move);
			}
			break;
		}
		/* Check left/right lim set buttons, change state when lifted! */
		if (pin_map[SIG_SET_LIM_L].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_L].value != DIG_ASSERTED)
		{
SetLimLMode:
			sys_state = MODE_SET_LIM_L;
			trav_dir = -1;
			digitalWrite(pin_map[SIG_LLIM_LED].num, 1);
			digitalWrite(pin_map[SIG_RLIM_LED].num, 0);
			break;
		}
		if (pin_map[SIG_SET_LIM_R].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_R].value != DIG_ASSERTED)
		{
SetLimRMode:
			sys_state = MODE_SET_LIM_R;
			trav_dir = 1;
			digitalWrite(pin_map[SIG_RLIM_LED].num, 1);
			digitalWrite(pin_map[SIG_LLIM_LED].num, 0);
			break;
		}
		if (pin_map[SIG_ENCODER_BUTTON].value != DIG_ASSERTED &&
			pin_map[SIG_ENCODER_BUTTON].prev_value == DIG_ASSERTED)
		{
			sys_state = MODE_MENU;
			menu->ResetMenu();
			break;
		}
		break;
	case MODE_SET_LIM_L:
		if (pin_map[SIG_SET_LIM_L].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_L].value != DIG_ASSERTED)
		{
			digitalWrite(pin_map[SIG_LLIM_LED].num, 0);
			sys_state = MODE_STOP;
		} else if (pin_map[SIG_SET_LIM_R].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_R].value != DIG_ASSERTED)
		{
			goto SetLimRMode;
		} else if (pin_map[SIG_START].value == DIG_ASSERTED &&
			spindle->IsStopped())
		{
			digitalWrite(pin_map[SIG_LLIM_LED].num, 0);
			goto SetRunMode;
		} else {
			lim_l = UpdateTravLim(SIG_LLIM);
		}
		break;
	case MODE_SET_LIM_R:
		if (pin_map[SIG_SET_LIM_R].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_R].value != DIG_ASSERTED)
		{
			digitalWrite(pin_map[SIG_RLIM_LED].num, 0);
			sys_state = MODE_STOP;
		} else if (pin_map[SIG_SET_LIM_L].prev_value == DIG_ASSERTED &&
			pin_map[SIG_SET_LIM_L].value != DIG_ASSERTED)
		{
			goto SetLimLMode;
		} else if (pin_map[SIG_START].value == DIG_ASSERTED &&
			spindle->IsStopped())
		{
			digitalWrite(pin_map[SIG_RLIM_LED].num, 0);
			goto SetRunMode;
		} else {
			lim_r = UpdateTravLim(SIG_RLIM);
		}
		break;
	case MODE_RUN:
		strobe_rotation = (int) encoder_knob.read() * 10;
		if (pin_map[SIG_STOP].value == STOP_ASSERTED) {
		  /* Stop spindle, change state to stop */
		  spindle->Stop();
		  sys_state = MODE_STOP;
		  break;
		}
		if (!program_mode) {
			UpdateTravRate(SIG_TRAV_RATE);
		}
		break;
	case MODE_MENU:
		int button = ((pin_map[SIG_ENCODER_BUTTON].value != DIG_ASSERTED) &&
				      (pin_map[SIG_ENCODER_BUTTON].prev_value == DIG_ASSERTED));
		if (menu->RunMenu(encoder_knob.read() / 2, button))
		{
			sys_state = MODE_STOP;
		}
		return;
	}
	
	lcd->setCursor(0,0);
	lcd->printf("%5.5d/%5.5d %drpm ", half_winding_events / 2, winding_limit, spindle->GetRpm());

	lcd->setCursor(0,1);
	if (trav->motor.estopped) {
		lcd->printf(">> ESTOP <<          ");
		return;
	}

	char c;
	if (trav_dir >= 0) {
		c = '>';
	} else {
		c = '<';
	}

	if (sys_state == MODE_SET_LIM_L) {
		lcd->printf("%d SetLeftLimit  ", lim_l);
	} else if (sys_state == MODE_SET_LIM_R) {
		lcd->printf("%d SetRightLimit  ", lim_r);
	} else if (sys_state == MODE_STOP) {
		if (winding_count >= winding_limit) {
			lcd->printf("DONE, hit reset ");
		}
		if (program_mode) {
			lcd->printf("P%d Stop Dir=%s     ", program_counter + 1, 
						pin_map[SIG_WIND_DIR].value ? "CW" : "CCW");
		} else {
			lcd->printf("Stop Dir=%s     ", pin_map[SIG_WIND_DIR].value ? "CW" : "CCW");
		}
	//               1234567890123456
	} else if (TravValidConfig()) {
		if (program_mode) {
			lcd->printf("P%d %c%2.2d t=.%4.4d\"    ", program_counter + 1, c,
				(trav->motor.cmd_pos - lim_l) * 100 / (lim_r - lim_l), 
				GetTravRateInTenThou());
		} else {
			lcd->printf("%c%2.2d %s t=.%4.4d\"    ", c,
				(trav->motor.cmd_pos - lim_l) * 100 / (lim_r - lim_l), 
				sys_state_str[sys_state], GetTravRateInTenThou());
		}
	} else {
		lcd->printf("%s LimitsNotSet", sys_state_str[sys_state]);
	}
	//               1234567890123456
}

void System::ResetProgMode(void)
{
	program_counter = 0;
	prog_op_winds = 0;
	/* If this is not in program mode, trav_rate will be updated anyways: */
	ResumeProgram();
}

int System::ResumeProgram(void)
{
	// Look at winding count, iterate through program to find
	// out where we are.
	int i;
	int count = 0;
	if (!program_mode) {
		return 0;
	}
	for (i = 0; i < MAX_PROG; i++) {
		if (program[i].windings == 0) {
			break;
		}
		count += program[i].windings;
		if (winding_count < count) {
			/* Remining minus how far into this op's winding's we are. */
			prog_op_winds = program[i].windings - (count - winding_count);
			program_counter = i;
			SetTravRateInTenThou(program[i].traversal);
			return 1;
		}
	}
	return 0;
}

int System::HandleStrobe(void)
{
	static unsigned long i;
	static int strobe_pin = pin_map[SIG_STROBE].num;
	unsigned long mod;
	int val = 0;
	
	i++;
	mod = (i + strobe_rotation) % (SPINDLE_STEPS_PER_REV / ((FTM1_CONF & 0x1f) + 1));
	if (sys_state == MODE_RUN) {
		val = (mod <= 5);
	}
	digitalWrite(strobe_pin, val);
	return mod == 0;
}

void System::HalfWindingComplete(void)
{
	int steps = 0, vector, cur_pos, new_pos;

	half_winding_events++;
	if (half_winding_events & 0x1) {
		/* Skip advancing on half windings */
		return;
	} else {
		winding_count++;
		if (program_mode) {
			if (++prog_op_winds >= program[program_counter].windings) {
				/* Advance to next stage. */
				prog_op_winds = 0;
				program_counter++;
				SetTravRateInTenThou(program[program_counter].traversal);
			}
		}
	}
	/* First deal with being outside the window. */
	cur_pos = trav->GetCmdPos();
	if (cur_pos < lim_l || cur_pos > lim_r) {
		if (trav_dir >= 0) {
			trav->MoveTo(lim_r);
		} else {
			trav->MoveTo(lim_l);
		}
		return;
	}
	/* Next accumulate traversal credit */
	trav_accumulator += trav_rate;
	if (trav_accumulator > trav_step_cost) {
		steps = trav_accumulator / trav_step_cost;
	}
	if (steps <= 0) {
		/* Can't move yet. */
		return;
	}
	if (trav_dir == 0) {
		trav_dir = 1;
	}
	vector = (steps * trav_dir);
	new_pos = cur_pos + vector;
	if (vector > 0) {
		if (new_pos >= lim_r) {
			new_pos = lim_r;
			//steps = cur_pos - lim_r;
			trav_dir = -1;
		}
	} else {
		if (new_pos <= lim_l) {
			new_pos = lim_l;
			//steps = lim_l - cur_pos;
			trav_dir = 1;
		}
	}
	trav_accumulator -= steps * trav_step_cost;
	if (trav_accumulator > MAX_TRAV_ACCUMULATOR) {
		trav_accumulator = MAX_TRAV_ACCUMULATOR;
	}
	trav->MoveTo(new_pos);
}

void System::SetProgWinding(int id, unsigned int value)
{
	if (id < 0 || id >= MAX_PROG) {
		return;
	}
	program[id].windings = value;
}

void System::SetProgTrav(int id, unsigned int value)
{
	if (id < 0 || id >= MAX_PROG) {
		return;
	}
	program[id].traversal = value;
}

int System::GetProgWinding(int id)
{
	if (id < 0 || id >= MAX_PROG) {
		return 0;
	}
	return program[id].windings;
}

int System::GetProgTrav(int id)
{
	if (id < 0 || id >= MAX_PROG) {
		return 0;
	}
	return program[id].traversal;
}

void System::UpdateProgModeWindingLimit(void)
{
	if (program_mode) {
		/* In program mode, the winding_limit is the sum of all
		 * program ops until a zero winding op is encountered.
 		 */
		Prog *p;
		winding_limit = 0;
		for (p = program; p < &program[MAX_PROG]; p++) {
			if (!p->windings) {
				break;
			}
			winding_limit += p->windings;
		}
	}
}

void System::InitConfig(void)
{
	Nvconfig *c = &g_nvconfig;
	memset(c, 0, sizeof(*c));
	c->version = CONFIG_VERSION;
	c->check_value = CHECK_VALUE;
	eeprom_initialize();
}

#define NV_EEPROM_ADDR 0
void System::LoadConfig(void)
{
	int i;
	Nvconfig *c = &g_nvconfig;

	/* Overwrite to be sure we've loaded it: */
	c->version = 0;
	c->check_value = 0;
	eeprom_read_block((void*)c, NV_EEPROM_ADDR, (uint32_t)sizeof(*c));

	lcd->setCursor(0, 0);

	if (c->version == CONFIG_VERSION &&
		c->check_value == CHECK_VALUE)
	{
		winding_limit = c->nv_winding_limit;
		lim_l = c->nv_lim_l;
		lim_r = c->nv_lim_r;
		program_mode = c->nv_program_mode;
		lcd->printf("windlim=%d     ", winding_limit);
		lcd->setCursor(0, 1);
		lcd->printf("liml=%d r=%d    ", lim_l, lim_r);
		delay(1500);
		lcd->setCursor(0, 0);
		for (i = 0; i < MAX_PROG; i++) {
			program[i].windings = c->nv_program[i].windings;
			program[i].traversal = c->nv_program[i].traversal;
		}
		lcd->printf("Loaded config   ");
		lcd->setCursor(0, 1);
		lcd->printf("%d saves        ", c->config_saves);
		delay(1000);
	} else {
		lcd->printf("No saved config");
		InitConfig();
		delay(1000);
	}
}

void System::SaveConfig(void)
{
	int i;
	Nvconfig *c = &g_nvconfig;

	/* Update structure */
	c->version = CONFIG_VERSION;
	c->check_value = CHECK_VALUE;
	c->config_saves++;
	c->nv_winding_limit = winding_limit;
	c->nv_lim_l = lim_l;
	c->nv_lim_r = lim_r;
	c->nv_program_mode = program_mode;
	c->nv_number_of_ops = MAX_PROG;
	for (i = 0; i < MAX_PROG; i++) {
		c->nv_program[i].windings = program[i].windings;
		c->nv_program[i].traversal = program[i].traversal;
	}

	/* Write updated structure to eeprom */
	eeprom_write_block((void*)c, NV_EEPROM_ADDR, (uint32_t)sizeof(*c));
}

Axis g_spindle(&SpindleMotFunctions, SPINDLE_STEPS_PER_REV);
Axis g_trav(&TravMotFunctions, TRAV_STEPS_PER_REV);
System g_system(&g_spindle, &g_trav, &lcd);

/************************* Menu -> System callbacks *******/
int ExitSelect(Menu *m, MenuItem *mi, MenuOp op)
{
	if (op == MENU_OP_SELECT) {
		g_system.UpdateProgModeWindingLimit();
		g_system.ResetProgMode();
		m->RequestExit();
	}
	return 0;
}

int ProgramModeSelect(Menu *m, MenuItem *mi, MenuOp op)
{
	if (op == MENU_OP_ADJUST) {
		if (m->ui_diff & 1) {
			mi->data.dyesno = !(mi->data.dyesno);
		}
		g_system.SetProgramMode(mi->data.dyesno);
	} else if (op == MENU_OP_DRAW) {
		mi->data.dyesno = g_system.GetProgramMode();
	}
	return 0;
}

int WindingProgMenu(Menu *m, MenuItem *mi, MenuOp op)
{
	int windings = g_system.GetProgWinding(mi->external_id);
	if (op == MENU_OP_ADJUST) {
		if (pin_map[SIG_MAX_UP].value == DIG_ASSERTED) {
			SCALE_COUNTER(pin_map[SIG_MAX_UP].prev_value, windings, 1);
		}
		if (pin_map[SIG_MAX_DOWN].value == DIG_ASSERTED) {
			SCALE_COUNTER(pin_map[SIG_MAX_DOWN].prev_value, windings, -1);
		}
		g_system.SetProgWinding(mi->external_id, windings);
		mi->data.dint = windings;
	} else if (op == MENU_OP_DRAW) {
		mi->data.dint = windings;
	}
	return 0;
}

int TravProgMenu(Menu *m, MenuItem *mi, MenuOp op)
{
	if (op == MENU_OP_ADJUST) {
		g_system.UpdateTravRate(SIG_TRAV_RATE);
		mi->data.dint = (int)g_system.GetTravRateInTenThou();
		g_system.SetProgTrav(mi->external_id, mi->data.dint);
	} else if (op == MENU_OP_DRAW) {
		mi->data.dint = g_system.GetProgTrav(mi->external_id);
	}
	return 0;
}

int SaveConfigProgMenu(Menu *m, MenuItem *mi, MenuOp op)
{
	if (op == MENU_OP_SELECT) {
		g_system.SaveConfig();
	}
	return 0;
}

int SaveCountProgMenu(Menu *m, MenuItem *mi, MenuOp op)
{
	if (op == MENU_OP_DRAW) {
		mi->data.duint = g_nvconfig.config_saves;
	}
	return 0;
}


/* TODO: Move to it's own file? */
int filterInput(int a, struct filter *f)
{
  f->idx++;
  if (f->idx > FILT_LEN) {
    f->idx = 0; 
  }
  f->buf[f->idx] = a;
  int i, avg;
  for (i = 0, avg = 0; i < FILT_LEN; i++) {
    avg += f->buf[i];
  }
  if (abs(avg - (f->last)) < FILT_LEN) {
    avg = f->last;
  }
  f->last = avg;
  return avg / FILT_LEN;
}

#ifdef OUTSIDE_OF_ARDUINO_IDE
int main(void)
#else
void setup()
#endif
{
	int i;
	pinmap_t *pmap;
	uint8_t mode;

	for (i = 0; i < NUM_PINS; i++) {
		switch (i) {
		case SIG_ENCODER_A:
		case SIG_ENCODER_B:
		case SIG_LCD1:
		case SIG_LCD2:
		case SIG_LCD3:
		case SIG_LCD4:
		case SIG_LCD5:
		case SIG_LCD6:
			continue; 
		}
		pmap = &pin_map[i];
		if (pmap->type == PT_DIGITAL) {
			switch (i) {
			case SIG_ENCODER_A:
			case SIG_ENCODER_B:
				mode = INPUT;
				break;
			default:
				// Assumption that dir is also arduino pin mode:
				mode = (uint8_t)pmap->dir;
				break;
			}
			pinMode(pmap->num, mode);
			if (pmap->dir == PD_OUTPUT) {
				digitalWrite(pmap->num, 0);
			}
		}
	}

	#ifdef USE_SERIAL
	Serial.begin(9600);
	#endif

	g_spindle.SetInitialPeriod(SPINDLE_INITIAL_PERIOD);
	#ifdef OUTSIDE_OF_ARDUINO_IDE

	while (1) {
		loop();
	}
	#endif
}

void update_pins() {
  int i;
  pinmap_t *pmap;
  unsigned int *dig_filter;
  unsigned int new_filter;
  
  for (i = 0; i < NUM_PINS; i++) {
#if 1
    switch (i) {
    case SIG_ENCODER_A:
    case SIG_ENCODER_B:
      continue;
    default:
      break;
    }
#endif
    pmap = &pin_map[i];
    if (pmap->dir != PD_INPUT) {
      continue; 
    }
    pmap->prev_value = pmap->value;
    switch (pmap->type) {
      case PT_DIGITAL:
		dig_filter = (unsigned int*)&pmap->filter;
		new_filter = ((*dig_filter) << 1) + (unsigned int)digitalRead(pmap->num);
		*dig_filter = new_filter;
		new_filter &= DIG_FILTER_MASK;
		if (new_filter == DIG_FILTER_MASK) {
			pmap->value = 1;
		} else if (new_filter == 0) {
			pmap->value = 0;
		}
        break;
      case PT_ANALOG:
        pmap->value = filterInput(analogRead(pmap->num), pmap->filter);
        break;
      default:
        break;
    }
  }
}


void ftm1_isr(void)
{
	if (g_system.HandleStrobe()) {
		g_system.HalfWindingComplete();
	}
	g_spindle.TimerCallback();
}

void TravIsr(void)
{
	travIsrs++;
	g_trav.TimerCallback();
}

void loop()
{
  /* Read inputs */
  update_pins();
  g_system.RunLoop();
}


