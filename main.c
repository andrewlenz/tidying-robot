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

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
KobukiSensors_t sensors = {0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0005, // TODO: replace with your lab bench number
        .adv_name          = "KOBUKI", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

//4607eda0-f65e-4d59-a9ff-84420d87a4ca
static simple_ble_service_t robot_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x98,0x99,0xE6,0x32}
}};

static simple_ble_char_t robot_service_char = {.uuid16 = 0xaaaa};
static float angle;
static float turn_angle = 0;

simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &robot_service_char)) {
      turn_angle = angle;
    }
}

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
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&robot_service);

  // TODO: Register your characteristics

  // Start Advertising
  simple_ble_adv_only_name();

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

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
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
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

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  uint16_t starting_value;
  uint16_t starting_value2;
  float displacement;
  lsm9ds1_measurement_t initial_gyro;
  char buf[16];
  float backup;
  states state = OFF;
  display_write("BLE Setup", DISPLAY_LINE_0);
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&robot_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(char) * 12, (float*)&angle,
      &robot_service, &robot_service_char);

  // Start Advertising
  simple_ble_adv_only_name();
  display_write("BLE Setup done", DISPLAY_LINE_0);

  // loop forever, running state machine
  while (1) {
    // power_manage();
    // read sensors from robot
    int status = kobukiSensorPoll(&sensors);

    // TODO: complete state machine
    switch(state) {
      case OFF: {
        print_state(state);

        // transition logic
        if (turn_angle != 0) {
          state = TURNING;
          //state = NEWSTATE;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }
        break; // each case needs to end with break!
      }
      case TURNING: {
        display_write("TURNING", DISPLAY_LINE_0);
        initial_gyro = lsm9ds1_read_gyro_integration();
        snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          break;
        }
        else {
          if (fabs(initial_gyro.z_axis) >=  turn_angle){
            lsm9ds1_stop_gyro_integration();
            kobukiDriveDirect(0,0);
            display_write("TO DRIVE", DISPLAY_LINE_1);
            nrf_delay_ms(500);
            kobukiSensorPoll(&sensors);
            state = DRIVING; 
            starting_value = sensors.rightWheelEncoder;
            displacement = 0;
            turn_angle = 0;

            break;
        } else {
            kobukiDriveDirect(40,-40);
            snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
            display_write(buf, DISPLAY_LINE_1);
            state = TURNING;
            break;
          }
       }
     }
     case DRIVING: {
        // transition logic
        displacement = measure_distance(sensors.rightWheelEncoder, starting_value);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0);  
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
      }
        else if (displacement >= 1) {
          state = OFF;
          kobukiDriveDirect(0,0);
          display_write("FINISHED", DISPLAY_LINE_1);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
        } else {
          // perform state-specific actions here
          kobukiDriveDirect(75,75);
          display_write("DRIVING", DISPLAY_LINE_0);
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          state = DRIVING;
        }
        break; // each case needs to end with break!
      }
   }
}
}

