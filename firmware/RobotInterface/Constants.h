#ifndef CONSTANTS_H
#define CONSTANTS_H

// Wheel Sizes
#define WHEEL_238 0
#define WHEEL_278 1

// Encoder Constants
#define LEFT_ENCODER_A 3
#define LEFT_ENCODER_B 12
#define RIGHT_ENCODER_A 2
#define RIGHT_ENCODER_B 11

#define WHEEL_SIZE WHEEL_278

#if WHEEL_SIZE == WHEEL_278
    // 2-7/8" wheel, rotated 3 times, encoder output: 4192 -> K = 0.00016418 ticks/meter
    #define LEFT_ENCODER_CONST (float)(0.00016418)
    #define RIGHT_ENCODER_CONST (float)(0.00016418)
#elif WHEEL_SIZE == WHEEL_238
    // 2-3/8" wheel, rotated 3 times, encoder output: 4192 -> K = 0.000135627 ticks/meter
    #define LEFT_ENCODER_CONST (float)(0.000135627)
    #define RIGHT_ENCODER_CONST (float)(0.000135627)
#else
    // Define constant as 1 to return raw ticks
    #define LEFT_ENCODER_CONST 1
    #define RIGHT_ENCODER_CONST 1
#endif



// Motor Constants
#define LEFT_ENABLE     10
#define LEFT_A          5
#define LEFT_B          8
#define RIGHT_ENABLE    9
#define RIGHT_A         7
#define RIGHT_B         6

// Timing Constants
#define MILLIS_PER_SECOND 1000
#define LOOP_RATE   100
#define LOOP_PERIOD (unsigned long)((1 * MILLIS_PER_SECOND) / LOOP_RATE)


#endif