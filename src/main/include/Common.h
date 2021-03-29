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
//#define PRACTICE_BOT

/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)

#define LOOPTIME                        (0.02)//(0.03125)
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)
#define ONE_SEC						(N1SEC)
#define HALF_SEC					((int)(0.5*N1SEC))

//Moved due to #ifDef issues

//deathstar:
#ifdef PRACTICE_BOT
#define DEATH_STAR_OFFSET       (-14.5)//(14.5)
#else
#define DEATH_STAR_OFFSET       (-10.0)//(-14.5)
#endif

//turret:
#ifdef PRACTICE_BOT
#define TURRET_DEGREES_PER_VOLT     (75.0217)//(72.0)
#else
#define TURRET_DEGREES_PER_VOLT     (72.72)//(73.0)
#endif
#ifdef PRACTICE_BOT
#define TURRET_ANALOG_OFFSET        (0.0)//(-43.0)//(181.0)
#else
#define TURRET_ANALOG_OFFSET        (0.0)//(-254.0)
#endif

//limelight:
#ifdef PRACTICE_BOT
#define LIMELIGHT_X_OFFSET_FAR                    (-2.25)//(-2.75)//(-3.25)
#define LIMELIGHT_X_OFFSET_NEAR                   (-4.5)
#define LIMELIGHT_Y_THRESHOLD                        (8.0)
#define LIMELIGHT_X_AUTO_TRENCH_OFFSET             (-1.0)
#else
#define LIMELIGHT_X_OFFSET_FAR                    (0.0)//(-2.75)//(-3.25)
#define LIMELIGHT_X_OFFSET_NEAR                   (-1.0)
#define LIMELIGHT_Y_THRESHOLD                        (15.0)
#define LIMELIGHT_X_AUTO_TRENCH_OFFSET             (-1.0)
#endif

//ShooterTurret
#ifdef PRACTICE_BOT
#define TURRET_MIN_ANGLE                        (40.0)
#else
#define TURRET_MIN_ANGLE                        (82.0)
#endif

#ifdef PRACTICE_BOT
#define TURRET_FACE_BACK_POS                  (55.0)
#define TURRET_FACE_FRONT_POS                (235.0)

#define TURRET_CENTER_AUTO_POS                (250.0)
#define TURRET_TRENCH_AUTO_POS                (234.0)
#define TURRET_FEEDER_AUTO_POS                (280.0)
#define TURRET_TRENCH_FRONT_POS               (234.0)
#define TURRET_SIDE_POS                       (150.0)
#else
#define TURRET_FACE_BACK_POS                  (55.0+COMP_TURRET_OFFSET)
#define TURRET_FACE_FRONT_POS                (235.0+COMP_TURRET_OFFSET)

#define TURRET_CENTER_AUTO_POS                (250.0+COMP_TURRET_OFFSET)
#define TURRET_TRENCH_AUTO_POS                (234.0+COMP_TURRET_OFFSET)
#define TURRET_FEEDER_AUTO_POS                (280.0+COMP_TURRET_OFFSET)
#define TURRET_TRENCH_FRONT_POS               (234.0+COMP_TURRET_OFFSET)
#define TURRET_SIDE_POS                       (150.0+COMP_TURRET_OFFSET)
#endif

//BallShooter
#ifdef PRACTICE_BOT
#define SHROUD_MAX_ANGLE                (0.55)
#else
#define SHROUD_MAX_ANGLE                (0.50)
#endif

#ifdef PRACTICE_BOT
#define SHROUD_K                                (6.0)//(1.0)//(6.0)//
#else
#define SHROUD_K                                (2.0)//(1.0)//(6.0)//
#endif

#ifdef PRACTICE_BOT
#define SHROUD_CALIBRATE_COMMAND              (-0.3)//(-0.2)
#else
#define SHROUD_CALIBRATE_COMMAND              (-0.2)//(-0.4)//(-0.2)
#endif

#ifdef PRACTICE_BOT
#define SHROUD_TIC_THRESHOLD                (0.15)
#else
#define SHROUD_TIC_THRESHOLD                (0.18)//(0.2)
#endif

#ifdef PRACTICE_BOT
#define SHROUD_TIC_BANDWIDTH                (20.0)
#else
#define SHROUD_TIC_BANDWIDTH                (10.0)//(15.0)//(30.0)
#endif

#ifdef PRACTICE_BOT
#define SHROUD_CMD_OFFSET                   (0.05)
#else
#define SHROUD_CMD_OFFSET                   (0.02)
#endif
//TARGETTING
#ifdef PRACTICE_BOT
#define SHROUD_INITIATION_LINE_ANGLE        (0.25)
#define SHROUD_BEHIND_COLOR_WHEEL_ANGLE     (0.466056)//(0.39939)
#define SHROUD_FRONT_TRENCH_ANGLE           (SHROUD_BEHIND_COLOR_WHEEL_ANGLE)
#define SHROUD_OTHER_LINE_ANGLE             (0.43)
#else
#define SHROUD_NEW_NUT_OFFSET				(-0.0*PI/180)//(2.0*PI/180)
#define SHROUD_INITIATION_LINE_ANGLE        (0.25-SHROUD_NEW_NUT_OFFSET)
#define SHROUD_BEHIND_COLOR_WHEEL_ANGLE     ((23.0*PI/180.0) - SHROUD_NEW_NUT_OFFSET)//(0.39939)
#define SHROUD_FRONT_TRENCH_ANGLE           (SHROUD_BEHIND_COLOR_WHEEL_ANGLE - SHROUD_NEW_NUT_OFFSET)
#define SHROUD_OTHER_LINE_ANGLE             (0.43 - SHROUD_NEW_NUT_OFFSET)
#endif
#define SHROUD_BAD_BALL_FAR_ANGLE			(0.330774)

#ifdef PRACTICE_BOT
#define LIMELIGHT_Y_INITIATION_LINE_LOW             (10.0)
#define LIMELIGHT_Y_INITIATIOn_LINE_HIGH            (24.0)
#define LIMELIGHT_Y_FRONT_TRENCH_LOW                (4.0)
#define LIMELIGHT_Y_FRONT_TRENCH_HIGH               (10.0)
#define LIMELIGHT_Y_COLOR_WHEEL_LOW                 (1.2)
#define LIMELIGHT_Y_COLOR_WHEEL_HIGH                (4.0)
#define LIMELIGHT_Y_OTHER_LINE_LOW                  (0.3)
#define LIMELIGHT_Y_OTHER_LINE_HIGH                 (1.2)
#else
/*#define LIMELIGHT_Y_INITIATION_LINE_LOW             (15.0)
#define LIMELIGHT_Y_INITIATIOn_LINE_HIGH            (24.0)
#define LIMELIGHT_Y_FRONT_TRENCH_LOW                (7.5)//(8.5)
#define LIMELIGHT_Y_FRONT_TRENCH_HIGH               (15.0)
#define LIMELIGHT_Y_COLOR_WHEEL_LOW                 (6.0)
#define LIMELIGHT_Y_COLOR_WHEEL_HIGH                (7.5)//(8.5)
#define LIMELIGHT_Y_OTHER_LINE_LOW                  (4.0)
#define LIMELIGHT_Y_OTHER_LINE_HIGH                 (6.0)
*/
#define LIMELIGHT_Y_LIMIT_LOW						(-11.0)
#define LIMELIGHT_Y_LIMIT_HIGH						(12.0)
#define SHOOTER_ZONE_VEL							(500.0)
#define SHROUD_LOW_ANGLE							(0.38)
#define SHROUD_MID_ANGLE							(0.29)
#define SHROUD_HIGH_ANGLE							(0.02)
#define LIMELIGHT_Y_LOW								(-10.4)
#define LIMELIGHT_Y_MID								(-6.7)
#define LIMELIGHT_Y_HIGH							(10.1)


#endif

#ifdef PRACTICE_BOT
#define DRIVE_OMEGA                     (2.0)//(4.0)
#else
#define DRIVE_OMEGA                     (3.0)//(2.5)//(4.0)
#endif

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
} pwms;


typedef enum
{
#ifdef PRACTICE_BOT
	GRABBER_CURRENT			= 11,
	DEATH_STAR_CURRENT		= 8,
#else
	GRABBER_CURRENT			= 5,
	DEATH_STAR_CURRENT		= 4,
#endif
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
} pcm;


typedef enum
{
	DIGIN_DEATH_STAR_0					=0,
	DIGIN_DEATH_STAR_1					=1,
	DIGIN_DEATH_STAR_2					=2,
	DIGIN_DEATH_STAR_3					=3,
	DIGIN_DEATH_STAR_4					=4,
	DIGIN_DEATH_STAR_DUTY_CYCLE			=5,
	SHROUD_ANGLE_ENCODER_ONE			=6, 
	SHROUD_ANGLE_ENCODER_TWO			=7,
	WINCH_ENCODER_ONE					=8,
	WINCH_ENCODER_TWO					=9,
	CLIMBER_DUTY_CYCLE					=14,   // MXP DIO4
 	
	LED_LIGHT_OUTPUT_1 							= 21,//19  //MXP PWM 5 //21  
	LED_LIGHT_OUTPUT_2							= 20,  //MXP PWM 6 //22
	LED_LIGHT_OUTPUT_3							= 19,//21  //MXP PWM 7 //23
} digitals;

/*
** List of gamepad (USB_GAMEPAD below) button and axis assignments
*/

typedef enum
{
	ENABLE_GYRO_BUTTON 		= 1,
	COLOR_SPIN_BUTTON		= 3,
	COLOR_MATCH_BUTTON		= 14,
	LIMELIGHT_SNAPSHOT_BUTTON	= 10,
} joystick_buttons;

typedef enum
{
	SPEED_AXIS					= 1,
	ROTATE_AXIS					= 5,
} joystick_axes;


typedef enum
{
	TURRET_AXIS					= 0,
	SHROUD_AXIS					= 3,
	COLOR_WHEEL_AXIS			= 2,
} gamepad_axes;
/*
typedef enum
{
	CLIMB_EXTEND_UP_POV			= 0,
	CLIMB_EXTEND_MID_POV		= 90,
	CLIMB_EXTEND_DOWN_POV		= 180,
	CLIMB_WINCH_RETRACT_POV		= 270,
} gamepad_pov;*/

typedef enum
{
	CELL_GRAB_BUTTON			= 6,
	CELL_EJECT_BUTTON			= 5,
	DEATH_STAR_CLOCKWISE_SPIN	= 10,
	DEATH_STAR_COUNTER_SPIN		= 9,
	TRACKING_BUTTON				= 7,
	SHOOTING_BUTTON				= 8,
	TURRET_FACE_BACK_BUTTON		= 2,
	TURRET_FACE_SIDE_BUTTON		= 3,
	TURRET_FACE_FRONT_BUTTON	= 4,
	SHOOTER_SPIN_UP_BUTTON		= 1,
} gamepad_buttons;

#endif /*Common_H_*/