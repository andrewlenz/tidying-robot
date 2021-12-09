#pragma once

#include "nrf.h"

// // Type for the function pointer to call when the timer expires
// typedef void (*virtual_timer_callback_t)(void);

// Read the current value of the hardware timer counter
// Returns the counter value
static uint32_t read_timer(void);

// Initialize the timer peripheral
void ultras_virtual_timer_init(void);

static void virtual_timer_clear(void);

// Takes a timer_id and cancels that timer such that it stops firing
static void virtual_timer_cancel(uint32_t timer_id);

bool ultras_update_distance();

static float avg();

float ultras_get_distance();
// void start_timer();