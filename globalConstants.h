#ifndef GLOBALCONSTANTS_H
#define GLOBALCONSTANTS_H

#include "mbed.h"

//////////////////////
//***** MACROS *****//
//////////////////////
// Here's how you easily change the mouse's behavior
// by changing certain constants.

#define MAZE_SIZE_X 16
#define MAZE_SIZE_Y 16
#define CENTER_X 8
#define CENTER_Y 8

// Motor Macros
#define MOTOR_PWM_PERIOD 1024
#define MAX_PWM_PULSELENGTH 768


// Encoder PID Macros
#define ENC_KP_VALUE 3
#define ENC_KD_VALUE 4
#define ENC_KI_VALUE 1
#define ENC_INT_CON_DECAY_FACTOR 0.90f

// Rotation PID Macros
#define ROT_TARG_DEC_FAC 0.91f
#define ROT_TARG_KP_VALUE 0.2f
#define ROT_TARG_KD_VALUE 10.0f
#define ROT_TARG_KI_VALUE .50f

#define NUM_PULSES_PER_NINETY 680

// IR Macros
#define IR_KP_VALUE 1.0f
#define IR_KD_VALUE 1.0f
#define IR_KI_VALUE 2.0f
#define IR_INT_CON_DECAY_FACTOR 0.4f

#define IR_KD_DENOM 0.1

#define LL_NEARNESS_SCALING_FACTOR 32
#define RR_NEARNESS_SCALING_FACTOR 32

//IR Reading Macros
#define WALL_DETECT_NUM_PASS 8

#define FL_WALL_NEAR_THRESH 0.3f // IR reading on Front Left receiver when there's a wall in front of the mouse. This Triggers a certain response
#define FR_WALL_NEAR_THRESH 0.30f // same as above but for Front Right receiver

#define LL_WALL_GAP_THRESH 0.10f // IR reading on Left Left receiver when there's no wall to the left
#define RR_WALL_GAP_THRESH 0.10f // same as above but for right right receiver and for right wall


//General Maze Navigation Macros
#define DISTANCE_BETWEEN_CELLS 2000 // distance between the center of one cell and the center of an adjcent cell (in encoder counts)

// MISC
#define MOV_FWD 1
#define MOV_BCK 0
#define FOLLOW_BOTH 0
#define FOLLOW_RIGHT 1
#define FOLLOW_LEFT 2

#define DIR_UP 0
#define DIR_RI 1
#define DIR_DO 2
#define DIR_LE 3


template <class myType>
myType abs (myType a) {
    eturn (a>0?a:-a);
}
#endif
