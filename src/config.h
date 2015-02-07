#ifndef __CONFIG_H_
#define __CONFIG_H_

#define JOY_PORT_0				0
#define JOY_PORT_1              1
#define JOY_PORT_2              2
#define JOY_PORT_3              3
#define JOY_PORT_4              4

#define JOY_BTN_X               1
#define JOY_BTN_A               2
#define JOY_BTN_B               3
#define JOY_BTN_Y               4

#define JOY_BTN_LBM             5
#define JOY_BTN_RBM             6
#define JOY_BTN_LTG             7
#define JOY_BTN_RTG             8

#define JOY_SPC_BCK             9  // Back button
#define JOY_SPC_STR             10 // Start button
#define JOY_SPC_LST             11 // Push the left stick in
#define JOY_SPC_RST             12 // Push the right stick in

#define JOY_AXIS_LX             0
#define JOY_AXIS_LY             1
#define JOY_AXIS_RX             2
#define JOY_AXIS_RY             3
#define JOY_AXIS_DX             4
#define JOY_AXIS_DY             5

#define FRONT_LEFT_MOTOR_PWM    0
#define FRONT_RIGHT_MOTOR_PWM   1
#define REAR_RIGHT_MOTOR_PWM	2
#define REAR_LEFT_MOTOR_PWM		3
#define LIFT_PWM				4
#define LEFT_CLAW_PWM			5
#define RIGHT_CLAW_PWM			6

//Digital IO

#define TOP_LIMIT_SWITCH		0
#define BOT_LIMIT_SWITCH		1
#define CARRIAGE_SWITCH			2

//Analog in

#define GYRO_PWM				0

#define PCM_NODE_ID				0

//Solenoid stuff

#define SHIFT_UP				0
#define SHIFT_DOWN				1
#define OPEN_CLAW				2
#define CLOSE_CLAW				3

#endif
