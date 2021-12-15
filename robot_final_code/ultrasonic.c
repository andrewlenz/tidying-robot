// Analog accelerometer app
//
// Reads data from the ADXL327 analog accelerometer

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_timer.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "display.h"
#include "buckler.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "nrf.h"
#include "ultrasonic.h"
// #include "virtual_timer_linked_list.h"

uint32_t pinTrig = BUCKLER_GROVE_D0;
uint32_t pinEcho = BUCKLER_GROVE_D1;
const int MAX_INDEX = 10;
static volatile uint32_t tcount = 0;
static volatile float countToUs = 1;
float distance_avg[10];
int arr_index = 0;
int counter = 0;

// Read the current value of the timer counter
static uint32_t read_timer(void) {

  // Should return the value of the internal counter for TIMER4

  NRF_TIMER4->TASKS_CAPTURE[1] |= 1UL;
  return NRF_TIMER4->CC[1]; 
}

// Initialize TIMER4 as a free running timer
// 1) Set to be a 32 bit timer
// 2) Set to count at 1MHz
// 3) Enable the timer peripheral interrupt (look carefully at the INTENSET register!)
// 4) Clear the timer
// 5) Start the timer
void ultras_virtual_timer_init(void) {
  nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
  // Place your timer initialization code here
  NRF_TIMER4->BITMODE |= 3;
  NRF_TIMER4->PRESCALER |= 4;
  NRF_TIMER4->TASKS_CLEAR |= 1UL;
  NRF_TIMER4->TASKS_START |= 1UL;
  NRF_TIMER4->INTENSET |= 1UL << 16;
  NVIC_EnableIRQ(TIMER4_IRQn);
  // virtual_timer_clear();
}

// Clear timer
// 4) Clear the timer
// 5) Start the timer
static void virtual_timer_clear(void) {
  // Place your timer initialization code here
  NRF_TIMER4->TASKS_CLEAR |= 1UL;
  NRF_TIMER4->TASKS_START |= 1UL;
  NRF_TIMER4->INTENSET |= 1UL << 16;
  NVIC_EnableIRQ(TIMER4_IRQn);
}

bool ultras_update_distance() {
  counter++;

  if (arr_index >= MAX_INDEX) {
    arr_index = 0;
  } else {
    arr_index++;
  }
  //clear and sending 10us pulse to initialize
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);
  nrf_gpio_pin_set(pinTrig);
  nrf_delay_us(10);
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);

  while(!nrf_gpio_pin_read(pinEcho)) {
    // display_write("waiting for echo\n", DISPLAY_LINE_0);
    // printf("Finished trigger, waiting\n");
  }
  if (nrf_gpio_pin_read(pinEcho)) {
    // printf("measuring distance\n");
    // display_write("echo started\n", DISPLAY_LINE_0);
    virtual_timer_clear();
  }
  float duration;
  //wait for Echo pin to go low
  while(nrf_gpio_pin_read(pinEcho)) {
    //calculate duration in us
    duration = read_timer();
  }
    //calculate distance in cm
  float distance = duration / 58.82;
  // printf("updated ultra\n");
  if (distance < 400.0) {
    distance_avg[arr_index] = distance;
    return true;
  } else {    
    // display_write("error\n", DISPLAY_LINE_0);
    return false;
  }
  return false;
}

 static float avg(float arr[]) {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += arr[i];
  }

  return sum/10;
}

// void start_timer(void) {
//   NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
//   NRF_TIMER1->TASKS_CLEAR = 1;
//   uint8_t prescalar = 0;
//   NRF_TIMER1->PRESCALER = prescalar;
//   NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
//   uint16_t comp1 = 500;
//   NRF_TIMER1->CC[1] = comp1;
//   countToUs = 0.0625*comp1*(1<<prescalar);
//   printf("timer_tick = %f us\n", countToUs);
//   NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
//   NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
//   NVIC_EnableIRQ(TIMER1_IRQn);
//   NRF_TIMER1->TASKS_START = 1;
// }

//Return true if at the top of the cliff
float ultras_get_distance() {
  float dist;
  if (counter < 10) {
    dist = 100;
  } else {
    dist = avg(distance_avg);
  }
  // printf("distance: %f", avg(distance_avg));
  return dist;
 }


// int main (void) {
//   ret_code_t error_code = NRF_SUCCESS;
//   APP_ERROR_CHECK(error_code);
//   NRF_LOG_DEFAULT_BACKENDS_INIT();
//   printf("Log initialized!\n");

//   nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
//   nrf_drv_spi_config_t spi_config = {
//     .sck_pin = BUCKLER_LCD_SCLK,
//     .mosi_pin = BUCKLER_LCD_MOSI,
//     .miso_pin = BUCKLER_LCD_MISO,
//     .ss_pin = BUCKLER_LCD_CS,
//     .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
//     .orc = 0,
//     .frequency = NRF_DRV_SPI_FREQ_4M,
//     .mode = NRF_DRV_SPI_MODE_2,
//     .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
//   };
//   error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
//   APP_ERROR_CHECK(error_code);
//   display_init(&spi_instance);
//   display_write("Hello, Human!", DISPLAY_LINE_0);
//   printf("Display initialized!\n");

//   ultras_virtual_timer_init();
//   virtual_timer_clear();
//   // nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
//   // nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
  
//   // loop forever
//   while (1) {
//     float dist;
//     if (ultras_get_distance()) {
//       uint8_t str[4];
//       sprintf((char*)str, "%f cm", dist);
//     }
//     nrf_delay_ms(250);
//   }
// }