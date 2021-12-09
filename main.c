// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "simple_ble.h"

#include "states.h"

#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "max44009.h"


// NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0005, // TODO: replace with your lab bench number
        .adv_name          = "ROMI", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t angle_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x98,0x99,0xE6,0x32}
}};

static simple_ble_char_t angle_char = {.uuid16 = 0x9999};
static char angle[12] = "test\0";
/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;
float turn_angle = 0;


void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &angle_char)) {
      turn_angle = (float) atof(angle);
    }
}


// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
KobukiSensors_t sensors = {0};

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  printf("%d\n", current_encoder);
  printf("%d\n", previous_encoder);
  const float CONVERSION = 0.0006108;
  if (current_encoder >= previous_encoder) {
    return (current_encoder - previous_encoder) * CONVERSION;
  } else {
    float overflow = pow(2, 16);
    float cur = overflow + current_encoder - 1; 
    return (cur - previous_encoder) * CONVERSION;
  }
  return (current_encoder - previous_encoder) * CONVERSION;
}

void print_state(states current_state){
  switch(current_state){
  case OFF:
    display_write("OFF", DISPLAY_LINE_0);
    break;
  case TURNING:
    display_write("TURNING", DISPLAY_LINE_0);
    break;
  case DRIVING:
    display_write("DRIVING", DISPLAY_LINE_0);
    break;
  }
} 

int main(void) {

  // initialize display
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

  ret_code_t error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

	// initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // // Setup LED GPIO
  // nrf_gpio_cfg_output(BUCKLER_LED0);

  // // Setup BLE
  display_write("BLE Setup", DISPLAY_LINE_0);
  simple_ble_app = simple_ble_init(&ble_config);


  simple_ble_add_service(&angle_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(char) * 12, (char*)&angle,
      &angle_service, &angle_char);

  // Start Advertising
  simple_ble_adv_only_name();
  display_write("BLE Setup done", DISPLAY_LINE_0);
  nrf_delay_ms(1000);
// initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  uint16_t starting_value;
  uint16_t starting_value2;
  float displacement;
  lsm9ds1_measurement_t initial_gyro;
  char buf[16];
  float backup;
  float saved_angle;
  states state = OFF;
  int obstacle;


  // loop forever, running state machine
  while (1) {
    // power_manage();
    // read sensors from robot
    int status = kobukiSensorPoll(&sensors);

    // TODO: complete state machine
    switch(state) {

      case OFF: {
        print_state(state);
        // received angle over BLE connection
        if (fabs(turn_angle) > 0) { //angle received
          state = TURNING;
          lsm9ds1_stop_gyro_integration();
          lsm9ds1_start_gyro_integration();
          saved_angle = turn_angle;
        } else {
          state = OFF;
          kobukiDriveDirect(0, 0);
        }
        break;
      }

      case TURNING: {
        print_state(state);
        initial_gyro = lsm9ds1_read_gyro_integration();
        snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) { // turn off
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          state = OFF;

        } else if (fabs(initial_gyro.z_axis) >= saved_angle) { // reached angle; finished turning
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0,0);
          display_write("TO DRIVE", DISPLAY_LINE_1);
          nrf_delay_ms(500);
          kobukiSensorPoll(&sensors);
          starting_value = sensors.rightWheelEncoder;
          displacement = 0;
          state = DRIVING; 

        } else if (saved_angle >= 0) {
          kobukiDriveDirect(70,-70); // keep turning right
          snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
          display_write(buf, DISPLAY_LINE_1);
          state = TURNING;

       } else {
          kobukiDriveDirect(-70, 70); // keep turning left
          snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
          display_write(buf, DISPLAY_LINE_1);
          state = TURNING;
       }
       break;
     }
     case DRIVING: {
        // transition logics
        displacement = measure_distance(sensors.rightWheelEncoder, starting_value);

        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0);  
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);

        } else if (displacement >= 1) { // test purposes -- drove far enough
          state = OFF;
          kobukiDriveDirect(0,0);
          display_write("FINISHED", DISPLAY_LINE_1);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);

        } else if (fabs(turn_angle) > 20) { //angle was updated
          saved_angle = turn_angle;
          kobukiDriveDirect(0,0);  
          state = TURNING;
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          lsm9ds1_stop_gyro_integration();
          lsm9ds1_start_gyro_integration();

        } else { // drive
          kobukiDriveDirect(75,75);
          display_write("DRIVING", DISPLAY_LINE_0);
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          state = DRIVING;
        }
        break;
      }
   }
  }
}