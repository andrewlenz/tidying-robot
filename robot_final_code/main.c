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
#include "ultrasonic.h"

#include "app_pwm.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"

#define  SERVO_PIN_GRIPPER   BUCKLER_SD_MISO
#define  SERVO_PIN_RAISER   BUCKLER_SD_MOSI
uint32_t pinIn = BUCKLER_GROVE_A0;
#define TIME_PERIOD  20000 // 20 milliseconds


// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x3001, // TODO: replace with your lab bench number
        .adv_name          = "TIDY1", // used in advertisements if there is room
        // .device_id         = 0x3002, // TODO: replace with your lab bench number
        // .adv_name          = "TIDY2", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(50, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
};


// GLOBAL VARIABLES
//sensors
lsm9ds1_measurement_t initial_gyro; //used in angle measurements
float ultra_dist; //distance measured by ultrasonic sensor
int pressure; //1 if holding object, 0 if not
KobukiSensors_t sensors = {0};

// Speed Consants
const drive_speed = 50;
const cautious_drive_speed = 55;
const turn_speed = 45;
const cautious_turn_speed = 45;
const backing_speed = 65;
const obstacle_turn_speed = 55;

//driving
uint16_t starting_value; // used for driving straight
uint16_t starting_value2; // used when driving backwards
float displacement; //distance driven from starting_value or starting_value2

//turning
float turn_angle = 0;
float saved_angle; //angle that robot turns to in TURNING state

//grabbing
float duty_cycle = 4; //duty cycle for gripper to close on object
bool retry_grab = false; //if we are currently retrying a grab
int grab = 0;
int ready = 1;
int picked = 0;
int arrived = 0; //if robot has arrived at the location it was driving to
int drive_cautious = 0;
int wait_count = 0;
int cautious_counter = 0;


bool lowered = 0;

//other
char buf[16]; //used for writing to LCD display
states state = OFF; // state
wait_cases wait_case; //reason we are in the wait case

//obstacle variables
static bool left;
static bool right;
static bool center;

//BLE VARIABLES
simple_ble_app_t* simple_ble_app;

// 32e61089-2b22-4db5-a914-43ce41986c65
static simple_ble_service_t angle_service = {{
    .uuid128 = {0x65,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x98,0x99,0xE6,0x32}
}};

static simple_ble_char_t angle_char = {.uuid16 = 0x1999};
static char angle[12];

// used for camera to tell robot to stop driving and pick up an object
// static simple_ble_char_t grab_char = {.uuid16 = 0x2999};
// static char grab_array[12];

// used for camera to tell robot it is ready for next step
static simple_ble_char_t ready_char = {.uuid16 = 0x3999};
static char ready_array[12];

// used for robot to tell camera that object has been picked up
static simple_ble_char_t picked_char = {.uuid16 = 0x4999};
static char picked_array[12];

// used if robot should stop driving
static simple_ble_char_t arrived_char = {.uuid16 = 0x5999};
static char arrived_array[12];

//NEED A NEW CHARACTERISTIC
static simple_ble_char_t drive_cautious_char = {.uuid16 = 0x6999};
static char drive_cautious_array[12];


// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

//SERVO motor global variables
APP_PWM_INSTANCE(PWM1, 2);


static void pwm_init() {
  // a variable to hold err_code value
  ret_code_t err_code1;

//  create a pwm config struct and pass it the default configurations along with the pin number for pwm and period in microseconds
// here we have configured one channel, we can configure max up to two channels per pwm instance in this library, if we use two channels then we need to pass two pins
  app_pwm_config_t pwm_cfg1 = APP_PWM_DEFAULT_CONFIG_2CH(TIME_PERIOD, SERVO_PIN_GRIPPER, SERVO_PIN_RAISER);
// change the pwm polarity by setting this value so that logic high value is given as active signal and duty is set for the logic high signal
// we can change it to give the active low signal as well for manipulating the logic low duty
  pwm_cfg1.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
  pwm_cfg1.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
// Initialize the pwm and pass it the intance, configurations and handler, we can also write NULL if we don't want to use the handler 
  err_code1 = app_pwm_init(&PWM1, &pwm_cfg1, NULL);
  APP_ERROR_CHECK(err_code1);// check if any error occurred during initialization
// enable the pwm signal so that the pwm is started on the specified pin
  app_pwm_enable(&PWM1);
}

bool get_pressure(int* pressure) {
  *pressure = nrf_gpio_pin_read(pinIn);
  return true;
}

void update_picked() {
  sprintf(picked_array, "%d", picked);
}

//ble stuff
void ble_evt_write(ble_evt_t const* p_ble_evt) {
    if (simple_ble_is_char_event(p_ble_evt, &angle_char)) {
      turn_angle = (float) atof(angle);
      memset(angle, 0, 12);

    } else if (simple_ble_is_char_event(p_ble_evt, &ready_char)) {
      ready = strcmp(ready_array, "0");
    
    } else if (simple_ble_is_char_event(p_ble_evt, &picked_char)) {
    
    } else if (simple_ble_is_char_event(p_ble_evt, &arrived_char)) {
      arrived = strcmp(arrived_array, "0");
    
    } else if (simple_ble_is_char_event(p_ble_evt, &drive_cautious_char)) {
      drive_cautious = strcmp(drive_cautious_array, "0");
    }
}

void ble_evt_disconnected(ble_evt_t const* p_ble_evt) {
	display_write("BLE disconnect", DISPLAY_LINE_1);
  turn_angle = 0;
  state = OFF;
}

void ble_evt_connected(ble_evt_t const* p_ble_evt) {
	display_write("BLE connected", DISPLAY_LINE_1);
}

// functions for distance/obstacles
static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
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

//prints the current state to robot display
void print_state(states current_state){       

  switch(current_state){
  case OFF:
    display_write("OFF", DISPLAY_LINE_0);
    break;
  case TURNING:
    display_write("TURNING", DISPLAY_LINE_0);
    break;
  case DRIVING:
    if (drive_cautious){
      display_write("CAUTIOUS", DISPLAY_LINE_0);
    } else {
      display_write("DRIVING", DISPLAY_LINE_0);
    }
    break;
  case OBS_DRIVE:
    display_write("OBS_DRIVE", DISPLAY_LINE_0);
    break;
  case BACKUP:
    display_write("BACKUP", DISPLAY_LINE_0);
    break;
  case OBS_TURN:
    display_write("OBS_TURN", DISPLAY_LINE_0);
    break;
  case GRAB:
    display_write("GRAB", DISPLAY_LINE_0);
    break;
  case DROP:
    display_write("DROP", DISPLAY_LINE_0);
    break;
  case WAIT:
    if (wait_case == READY) {
      display_write("WAIT READY", DISPLAY_LINE_0);
    }
    else if (wait_case == DRIVE_WAIT) {
      display_write("WAIT DRIVE", DISPLAY_LINE_0);
    }
    break;
  }
} 

// true if robot has near an obstacle or triggered an obstacle
static bool obstacle(KobukiSensors_t sensors) {

  if (sensors.bumps_wheelDrops.bumpLeft) {
    left = true;
  }
  if (sensors.bumps_wheelDrops.bumpRight) {
    right = true;
  }
  if (sensors.bumps_wheelDrops.bumpCenter) {
    center = true;
  }
  if (sensors.bumps_wheelDrops.bumpLeft || sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpRight) {
    return true;
  } else {
    return false;
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

	// initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  nrf_delay_ms(100);

  // Setup BLE
  display_write("BLE Setup", DISPLAY_LINE_0);
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&angle_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(char) * 12, (char*)&angle,
      &angle_service, &angle_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(char) * 12, (char*)&ready_array,
    &angle_service, &ready_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(char) * 12, (char*)&picked_array,
    &angle_service, &picked_char);

  update_picked();

  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(char) * 12, (char*)&arrived_array,
    &angle_service, &arrived_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
    sizeof(char) * 12, (char*)&drive_cautious_array,
    &angle_service, &drive_cautious_char);

  // Start Advertising
  simple_ble_adv_only_name();
  display_write("BLE Setup done", DISPLAY_LINE_0);

  // initialize Kobuki
  kobukiInit();

  // Set up ultrasonic sensor timer
  ultras_virtual_timer_init();

  // Set up servo and pressure code
  pwm_init();
  nrf_gpio_pin_dir_set(pinIn, NRF_GPIO_PIN_DIR_INPUT);

  //GRIPPER SETUP
  // Fully raised
  while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
  lowered = false;
  // Fully open
  while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);

  uint32_t counter = 0;
  // loop forever, running state machine
  while (1) {
    counter++;
    // read sensors from robot
    int status = kobukiSensorPoll(&sensors);
    print_state(state);

    switch(state) {

    	case OFF: {

        // received angle over BLE connection
        if (fabs(turn_angle) > 0) {
        	state = TURNING;
        	lsm9ds1_start_gyro_integration();
        	saved_angle = turn_angle;
        }
        else {
          state = OFF;
          kobukiDriveDirect(0,0); 
        }
        break;
      }

     	case TURNING: {
	      initial_gyro = lsm9ds1_read_gyro_integration();
	      snprintf(buf, 16, "%0.4f %d", fabs(initial_gyro.z_axis), counter);
	      display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) { // turn off
        	kobukiDriveDirect(0,0); 
					nrf_delay_ms(50);
					kobukiSensorPoll(&sensors);
					state = OFF;

				// reached angle
	      } else if (fabs(initial_gyro.z_axis) >= fabs(saved_angle)) { 
					lsm9ds1_stop_gyro_integration();
					kobukiDriveDirect(0,0);
					nrf_delay_ms(50);
					kobukiSensorPoll(&sensors);
					starting_value = sensors.rightWheelEncoder;
					displacement = 0;
					state = DRIVING; 

					//keep turning right
	      } else if (saved_angle >= 0) {
          if (drive_cautious) {
            kobukiDriveDirect(cautious_turn_speed, -cautious_turn_speed);
            state = TURNING;
          } else{
            kobukiDriveDirect(turn_speed, -turn_speed);
            state = TURNING;          
          }
					// keep turning left
	      } else {
          if (drive_cautious) {
            kobukiDriveDirect(-cautious_turn_speed, cautious_turn_speed);
            state = TURNING;
          } else {
            kobukiDriveDirect(-turn_speed, turn_speed);
            state = TURNING;
          }
	      }
	      break;
    	}

      case DRIVING: {
	      displacement = measure_distance(sensors.rightWheelEncoder, starting_value);

        if (is_button_pressed(&sensors)) {
					state = OFF;
					kobukiDriveDirect(0,0);  
					snprintf(buf, 16, "%f", displacement);
					display_write(buf, DISPLAY_LINE_1);
					nrf_delay_ms(50);
					kobukiSensorPoll(&sensors);

					// within 20 cm of the target, speed reduced and angle adjustments smaller
				} else if (drive_cautious) {
            if (!lowered) {
              while(app_pwm_channel_duty_set(&PWM1, 1, 7.6) == NRF_ERROR_BUSY);
              nrf_delay_ms(30);
              kobukiSensorPoll(&sensors);
              lowered = true;
            }
            //lowered

              // arrived at destination
             if (arrived) {
              drive_cautious = false;
              if (picked) { // holding an object currently
                state = DROP;
              } else {
                duty_cycle = 4;
                state = GRAB; // going to grab an object
              }
              kobukiDriveDirect(0, 0);
              
							//angle was updated, reaangling
						}	else if (fabs(turn_angle) > 1) {
							saved_angle = turn_angle;
							turn_angle = 0;
							kobukiDriveDirect(0, 0);
							lsm9ds1_stop_gyro_integration();
							state = TURNING;
							nrf_delay_ms(50);
							kobukiSensorPoll(&sensors);
							lsm9ds1_start_gyro_integration();
							snprintf(buf, 16, "%f", saved_angle);
							display_write(buf, DISPLAY_LINE_0);

							//continue driving slowly
						} else {
							kobukiDriveDirect(cautious_drive_speed, cautious_drive_speed);
              nrf_delay_ms(10);
              state = DRIVING;
						}

					//all below is for driving at normal speed

				} else if (obstacle(sensors)) {
          kobukiDriveDirect(0,0);
          state = BACKUP;
          starting_value2 = sensors.rightWheelEncoder;

        } else if (fabs(turn_angle) > 3) { //angle was updated
					saved_angle = turn_angle;
					turn_angle = 0;
          kobukiDriveDirect(0,0);
					lsm9ds1_stop_gyro_integration();
					state = TURNING;
					nrf_delay_ms(50);
					kobukiSensorPoll(&sensors);
					lsm9ds1_start_gyro_integration();
          kobukiDriveDirect(50, -50);

        } else { // drive
          // Fully raised
          while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
          lowered = false;
					kobukiDriveDirect(drive_speed, drive_speed);
					state = DRIVING;
				}
				break;
	    }

	    // back up 0.1 m from obstacle
	    case BACKUP: {
	    displacement = measure_distance(starting_value2, sensors.rightWheelEncoder);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(50);
          kobukiSensorPoll(&sensors);
          // break;
        }

        //backed up enough
        else if (displacement >= 0.1) {
        	if (retry_grab || wait_case == READY) { // cases where next move is to drive again
            kobukiDriveDirect(0,0);  
        		state = WAIT;
            wait_case = DRIVE_WAIT;
            drive_cautious = 1;

            //Fully raised, going to backup and retry
            while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
            nrf_delay_ms(30);
            lowered = false;

            // Fully open
            while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
            nrf_delay_ms(30);
            kobukiSensorPoll(&sensors);
            duty_cycle = 4;
        	} else { //purpose of backup was to avoid obstacle
          	state = OBS_TURN;
          	kobukiDriveDirect(0,0);  
	          ret_code_t result = lsm9ds1_start_gyro_integration();
        	}

	        //continue backup
        } else {
          kobukiDriveDirect(-65,-65);
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          state = BACKUP;     
        }
        break;
      }
      // turn 45 degrees away from obstacle
      case OBS_TURN: {
        initial_gyro = lsm9ds1_read_gyro_integration();
        snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
        display_write(buf, DISPLAY_LINE_1);

        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(50);
          kobukiSensorPoll(&sensors);

          //turned enough
        } else if (fabs(initial_gyro.z_axis) >=  45) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0,0);
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          state = OBS_DRIVE;
          displacement = 0;
          starting_value = sensors.rightWheelEncoder;
          left = false;
          right = false;
          center = false;

	      } else {
	        if (left || center) {
	          kobukiDriveDirect(40,-40);
	          snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
	          display_write(buf, DISPLAY_LINE_1);
	          state = OBS_TURN;
	        } else {
	          kobukiDriveDirect(-40, 40);
	          snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
	          display_write(buf, DISPLAY_LINE_1);
	          state = OBS_TURN;
	        }
	        break;
	      }
	    }

	    case OBS_DRIVE: {
	    displacement = measure_distance(starting_value, sensors.rightWheelEncoder);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(50);
          kobukiSensorPoll(&sensors);
          break;
        }
         //go back to drive to get new angle
        if (displacement >= 0.1) {
          state = DRIVING;
          kobukiDriveDirect(0,0);  
          nrf_delay_ms(50);
          kobukiSensorPoll(&sensors); 
          starting_value = sensors.rightWheelEncoder;
        } else {
          //drive forward a bit more to avoid obstacle
          kobukiDriveDirect(50, 50);
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          state = OBS_DRIVE;     
        } 

        break;
      }

      case GRAB: {
      	// closing gripper and object not picked up
        get_pressure(&pressure);
        if ((pressure < 1) && (duty_cycle <= 11)) {

          //closing
          while(app_pwm_channel_duty_set(&PWM1, 0, duty_cycle) == NRF_ERROR_BUSY);
					duty_cycle += 0.5;
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);
          state = GRAB;

        // fully closed but didn't pick up object
        } else if (duty_cycle == 11 && (pressure < 1)) {
        	state = BACKUP;
        	retry_grab = true;
          arrived = 0;
          duty_cycle = 4;

        // successfully picked up object, will wait for next angle
        } else {
        	 //Fully raised
          nrf_delay_ms(30);
          while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
          nrf_delay_ms(30);
          kobukiSensorPoll(&sensors);
          lowered = false;
          ready = 0;
          drive_cautious = 0;
          arrived = 0;
        	retry_grab = false;
        	picked = true; // SEND TO CAMERA
          update_picked();
        	wait_case = READY;
        	state = WAIT;
        }
		  	break;
		  }

		  // case to drop an object
		  case DROP: {
       	//Fully open
        while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
        nrf_delay_ms(30);
        kobukiSensorPoll(&sensors);
        picked = false;
        update_picked();
        ready = 0;
        drive_cautious = 0;
        arrived = 0;
        wait_case = READY;
        duty_cycle = 4;
        state = WAIT;
		    break;
		  }

		  // wait for a response from central
		  case WAIT: {
        switch (wait_case) {

          case DRIVE_WAIT: {
            if (wait_count > 40) {
              wait_count = 0;
              state = DRIVING;
              starting_value = sensors.rightWheelEncoder;
              displacement = 0;
            } else {
              wait_count++;
              kobukiDriveDirect(0,0);
              state = WAIT;
            }
            break;
          }

          case READY: {
            get_pressure(&pressure);
            if (picked && pressure < 1) {
              picked = 0;
              update_picked();
              state = BACKUP;
              retry_grab = true;
              starting_value2 = sensors.rightWheelEncoder;
            }
            else if (ready && arrived == 0) { //new angle is ready
              if (picked) {
                state = TURNING;
              } else {
                starting_value2 = sensors.rightWheelEncoder;
                state = BACKUP;
              }
              lsm9ds1_stop_gyro_integration();
              lsm9ds1_start_gyro_integration();
              saved_angle = turn_angle;
              wait_count = 0;
              drive_cautious = 0;
              arrived = 0;

            //just keep waiting
            } else {
              kobukiDriveDirect(0,0);
              state = WAIT;
            }
            break;            
          }
        }
        break;
			}
    }
  }
}

	    