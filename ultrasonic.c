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

#include "virtual_timer.h"
// #include "virtual_timer_linked_list.h"

uint32_t pinTrig = BUCKLER_GROVE_D0;
uint32_t pinEcho = BUCKLER_GROVE_D1;
static volatile uint32_t tcount = 0;
static volatile float countToUs = 1;

bool getDistance(float* dist) {
  //clear and sending 10us pulse to initialize
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);
  nrf_gpio_pin_set(pinTrig);
  nrf_delay_us(10);
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);

  while(!nrf_gpio_pin_read(pinEcho)) {
    display_write("waiting for echo\n", DISPLAY_LINE_0);
    printf("Finished trigger, waiting\n");
  }
  if (nrf_gpio_pin_read(pinEcho)) {
    printf("measuring distance\n", "%s");
    display_write("echo started\n", DISPLAY_LINE_0);
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
  if (distance < 400.0){
    *dist = distance;
    return true;
  } else {    
    display_write("error\n", DISPLAY_LINE_0);
    return false;
  }
  return false;
}


// Read the current value of the timer counter
uint32_t read_timer(void) {

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
void virtual_timer_init(void) {
  // Place your timer initialization code here
  NRF_TIMER4->BITMODE |= 3;
  NRF_TIMER4->PRESCALER |= 4;
  NRF_TIMER4->TASKS_CLEAR |= 1UL;
  NRF_TIMER4->TASKS_START |= 1UL;
  NRF_TIMER4->INTENSET |= 1UL << 16;
  NVIC_EnableIRQ(TIMER4_IRQn);
}

// Clear timer
// 4) Clear the timer
// 5) Start the timer
void virtual_timer_clear(void) {
  // Place your timer initialization code here
  NRF_TIMER4->TASKS_CLEAR |= 1UL;
  NRF_TIMER4->TASKS_START |= 1UL;
  NRF_TIMER4->INTENSET |= 1UL << 16;
  NVIC_EnableIRQ(TIMER4_IRQn);
}


int main (void) {
  ret_code_t error_code = NRF_SUCCESS;
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // APP_TIMER_INIT(0,4,4,false);
  virtual_timer_init();
  nrf_gpio_pin_dir_set(pinTrig, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEcho, NRF_GPIO_PIN_DIR_INPUT);
  char buf[16];

  printf("entering while loop\n");
  display_write("measuring\n", DISPLAY_LINE_0);
  // loop forever
  while (1) {
    float dist;
    if (getDistance(&dist)) {
      uint8_t str[4];
      snprintf(buf, 16, "%f", dist);
      display_write(buf, DISPLAY_LINE_0);

    }
    nrf_delay_ms(250);
  }
}


