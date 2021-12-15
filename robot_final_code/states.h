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
    OBS_DRIVE,
    OBS_TURN,
    BACKUP,
    GRAB,
    DROP,
    WAIT,
} states;

typedef enum {
    DRIVE_WAIT,
    READY,
} wait_cases;

#endif /* STATES_H_ */