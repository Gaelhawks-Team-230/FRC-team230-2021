/*
* Common.h
*
* Created on: Jan 11, 2020
*		Author: Gaelhawks
*/

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

#include "TalonXXI_main.h"

//#define TEST_DRIVE
//#define FALCONTEST   1
//#define BUTTONTEST   1
//#define SHOOTERTEST		1
#define USE_GYRO
//#define TEST_MODE
#define USING_SOLENOID

/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)

#define LOOPTIME                        (0.02)
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)
#define ONE_SEC						(N1SEC)
#define HALF_SEC					((int)(0.5*N1SEC))

typedef enum
{
	PWM_GRABBER_WHEELS		= 0,
	PWM_COLOR_WHEEL 		= 1,
	PWM_SHROUDING 			= 2,
	PWM_TURRET 				= 3,
	PWM_STAR_SPIN_MOTOR 	= 4,
	PWM_CELL_KICK_MOTOR		= 5,
	PWM_CLIMB_EXTEND		= 6,
	PWM_CLIMB_WINCH			= 7,
//	PWM_AUX_GRABBER			= 7,
} pwms;


typedef enum
{
	GRABBER_CURRENT			= 11,
	DEATH_STAR_CURRENT		= 8,
} pdp;


typedef enum
{
    CAN_FRONT_LEFT 		=1,
	CAN_BACK_LEFT		=2,
	CAN_FRONT_RIGHT 	=3,
	CAN_BACK_RIGHT 		=4,
	CAN_SHOOTER			=5,
} can;


typedef enum
{
	TURRET_ANALOG				= 0,
	TURRET_GYRO_ANALOG			= 1,
} analogs;

typedef enum
{
	PCM_GRAB_CELL_1 	 = 0,
	PCM_GRAB_CELL_2 	 = 1,
	PCM_KICK_CELL_1		 = 2,
	PCM_KICK_CELL_2		 = 3,
	PCM_CLIMB_PISTON	 = 4, //remove later?
} pcm;


typedef enum
{
	DIGIN_DEATH_STAR_1					=0,
	DIGIN_DEATH_STAR_2					=1,
	DIGIN_DEATH_STAR_3					=2,
	DIGIN_DEATH_STAR_4					=3,
	DIGIN_DEATH_STAR_5					=4,
	DIGIN_DEATH_STAR_DUTY_CYCLE			=5,
	SHROUD_ANGLE_ENCODER_ONE			=6, 
	SHROUD_ANGLE_ENCODER_TWO			=7,
	WINCH_ENCODER_ONE					=8,
	WINCH_ENCODER_TWO					=9,
	CLIMBER_DUTY_CYCLE					=14,   // MXP DIO4
 	
	//DIGIN_CELL_SENSOR					=102, //REMOVE :)
} digitals;

/*
** List of gamepad (USB_GAMEPAD below) button and axis assignments
*/

typedef enum
{
	ENABLE_GYRO_BUTTON 		= 1,
	CLIMB_TWO_PREP		    = 2, // was 4
	OVERRIDE_BUTTON_RIGHT   = 4, // was 2
	AUTO_PAUSE_BUTTON       = 16,
	TRACKING_BUTTON			= 3,
	CLIMB_CHECK_BUTTON		= 5,
	CLIMB_LEV3_BUTTON		= 14,
	OVERRIDE_MAIN			= 8,
	OVERRIDE_CLIMBER		= 6,
	OVERRIDE_ELEVATOR		= 10,
	OVERRIDE_PIVOT			= 12,
	DRAG_BUTTON 			= 15,
} joystick_buttons;

typedef enum
{
	SPEED_AXIS					= 1,
	//STRAFE_AXIS					= 0,
	ROTATE_AXIS					= 5,
	TRACKING_AXIS				= 2,
	TILT_STOP_LIMIT				= 6,
} joystick_axes;


typedef enum
{
	TURRET_AXIS					= 3,
} gamepad_axes;
/*
typedef enum
{
	
} gamepad_pov;

typedef enum
{

} gamepad_buttons;
*/
#endif /*Common_H_*/