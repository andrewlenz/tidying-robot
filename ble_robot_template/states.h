/*
 * states.h
 *
 *  Created on: Sep 22, 2018
 *      Author: shromonaghosh
 */

#ifndef STATES_H_
#define STATES_H_

#include <stdio.h>

typedef enum {
    OFF,
    DRIVING,
    TURNING,
    OBSTACLE_DETECT,
    OBSTACLE_TURN,
    OBSTACLE_BACK,
   	GRAB,
   	DROP,
   	WAIT,
} states;

typedef enum {
    OBSTACLE,
} wait_cases;

#endif /* STATES_H_ */