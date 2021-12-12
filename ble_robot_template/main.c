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
// #include "ultrasonic.c"

#include "app_pwm.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"

// NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x2000, // TODO: replace with your lab bench number
        .adv_name          = "TIDY", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(650, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c65
static simple_ble_service_t angle_service = {{
    .uuid128 = {0x65,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x98,0x99,0xE6,0x32}
}};

static simple_ble_char_t angle_char = {.uuid16 = 0x1999};
static char angle[12];

// 32e61089-2b22-4db5-a914-43ce41986c60
// static simple_ble_service_t robot_service = {{
//     .uuid128 = {0x60,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
//                 0xB5,0x4D,0x22,0x2B,0x98,0x99,0xE6,0x32}
// }};

// global variables
KobukiSensors_t sensors = {0};
static simple_ble_char_t grab_char = {.uuid16 = 0x2999};
static bool grab = false;

static simple_ble_char_t obstacle_char = {.uuid16 = 0x3999};
// static bool obstacle = false;
static int obstacle_response;
static bool obstacle_detect;

simple_ble_app_t* simple_ble_app;
float turn_angle = 0;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

//SERVO motor global variables
APP_PWM_INSTANCE(PWM1, 1);
// APP_PWM_INSTANCE(PWM2, 0);

#define  SERVO_PIN_GRIPPER   BUCKLER_SD_MISO
#define  SERVO_PIN_RAISER   BUCKLER_SD_MOSI
uint32_t pinIn = BUCKLER_GROVE_A0;
#define TIME_PERIOD  20000 // 20 milliseconds
bool picked = false;


static void pwm_init() {
  // a variable to hold err_code value
  ret_code_t err_code1;
  // ret_code_t err_code2;

//  create a pwm config struct and pass it the default configurations along with the pin number for pwm and period in microseconds
// here we have configured one channel, we can configure max up to two channels per pwm instance in this library, if we use two channels then we need to pass two pins
  app_pwm_config_t pwm_cfg1 = APP_PWM_DEFAULT_CONFIG_2CH(TIME_PERIOD, SERVO_PIN_GRIPPER, SERVO_PIN_RAISER);
  printf("00");
  // app_pwm_config_t pwm_cfg2 = APP_PWM_DEFAULT_CONFIG_1CH(TIME_PERIOD, SERVO_PIN_RAISER);

// change the pwm polarity by setting this value so that logic high value is given as active signal and duty is set for the logic high signal
// we can change it to give the active low signal as well for manipulating the logic low duty
  pwm_cfg1.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
  pwm_cfg1.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
  printf("11");
// Initialize the pwm and pass it the intance, configurations and handler, we can also write NULL if we don't want to use the handler 
  err_code1 = app_pwm_init(&PWM1, &pwm_cfg1, NULL);
  printf("22");
  // err_code2 = app_pwm_init(&PWM2, &pwm_cfg2, NULL);
  APP_ERROR_CHECK(err_code1);// check if any error occurred during initialization
  // APP_ERROR_CHECK(err_code2);
  printf("33");
// enable the pwm signal so that the pwm is started on the specified pin
  app_pwm_enable(&PWM1);
  // app_pwm_enable(&PWM2);
  printf("44");
}

bool getPressure(float* pressure) {
  *pressure = nrf_gpio_pin_read(pinIn);
  return true;
}

//ble stuff
void ble_evt_write(ble_evt_t const* p_ble_evt) {
	display_write("BLE WRITE", DISPLAY_LINE_1);
    // if (simple_ble_is_char_event(p_ble_evt, &angle_char)) {
			// printf("turn angle updated\n");
      turn_angle = (float) atof(angle);
      printf("new turn angle:%f\n", turn_angle);
      // print("saved angle: %f\n", saved_angle);
      // printf(angle);
      memset(angle, 0, 12);
      // char buf[16] = "";
      // snprintf(buf, 16, "%f", turn_angle);
      // display_write(buf, DISPLAY_LINE_1);
    // }
}

void ble_evt_disconnected(ble_evt_t const* p_ble_evt) {
	display_write("BLE disconnect", DISPLAY_LINE_1);
  printf("ble disconnected\n");
}

void ble_evt_connected(ble_evt_t const* p_ble_evt) {
	display_write("BLE connected", DISPLAY_LINE_1);
  printf("%d\n", p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr_type);
  printf("%d\n", p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr);
  printf("ble connected\n");
}

// functions for distance/obstacles
static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  // printf("current encoder%d\n", current_encoder);
  // printf("prev encoder%d\n", previous_encoder);
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

static float measure_backup_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.0006108;
  if (previous_encoder >= current_encoder) {
    return (previous_encoder - current_encoder) * CONVERSION;
  } else {
    float overflow = pow(2, 16);
    float prev = overflow + previous_encoder - 1; 
    return (prev - current_encoder) * CONVERSION;
  }
}


void print_state(states current_state){
  switch(current_state){
  case OFF:
    display_write("OFF", DISPLAY_LINE_0);
    // printf("off\n");
    break;
  case TURNING:
    display_write("TURNING", DISPLAY_LINE_0);
    // printf("turning\n");
    break;
  case DRIVING:
    display_write("DRIVING", DISPLAY_LINE_0);
    // printf("driving\n");
    break;
  case OBSTACLE_DETECT:
    display_write("OBSTACLE_DETECT", DISPLAY_LINE_0);
    break;
  case OBSTACLE_BACK:
    display_write("OBSTACLE_BACK", DISPLAY_LINE_0);
    break;
  case OBSTACLE_TURN:
    display_write("OBSTACLE_TURN", DISPLAY_LINE_0);
    break;
  case GRAB:
    display_write("GRAB", DISPLAY_LINE_0);
    break;
  case DROP:
    display_write("DROP", DISPLAY_LINE_0);
    break;
  case WAIT:
    display_write("WAIT", DISPLAY_LINE_0);
    break;
  }
} 

void update_grab(bool state) {
	grab = state;
}

// void update_obstacle(bool state) {
// 	obstacle = state;
// }

static bool left;
static bool right;
static bool center;
static bool obstacle(KobukiSensors_t sensors) {
  if (ultras_get_distance() < 5){ //need to make this an array
  	center = true;
  	return true;
  }
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
  printf("Display initialized!\n");

  printf("Will setup servos now\n");
  pwm_init();

	// initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  printf("aa");
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  printf("bb");
  APP_ERROR_CHECK(error_code);
  printf("cc");
  lsm9ds1_init(&twi_mngr_instance);
  printf("dd");
  printf("IMU initialized!\n");


  // Setup BLE
  display_write("BLE Setup", DISPLAY_LINE_0);
  simple_ble_app = simple_ble_init(&ble_config);


  simple_ble_add_service(&angle_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(char) * 12, (char*)&angle,
      &angle_service, &angle_char);

  // simple_ble_add_service(&robot_service);

  // // indicates whether gripper has grabbed object
  // simple_ble_add_characteristic(1, 1, 0, 0,
  //     sizeof(char) * 12, (char*)&grab,
  //     &robot_service, &grab_char);

  // indicates robot senses obstacle
  // simple_ble_add_characteristic(1, 1, 0, 0,
  //     sizeof(char) * 12, (char*)&obstacle_detect,
  //     &robot_service, &obstacle_char);

  // Start Advertising
  simple_ble_adv_only_name();
  display_write("BLE Setup done", DISPLAY_LINE_0);
  nrf_delay_ms(1000);

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // Set up ultrasonic sensor timer
  ultras_virtual_timer_init();


  printf("finished ble setup\n");
  // display_write("measuring\n", DISPLAY_LINE_0);
  // loop forever

  uint16_t starting_value;
  uint16_t starting_value2;
  float displacement;
  lsm9ds1_measurement_t initial_gyro;
  char buf[16];
  float backup;
  float saved_angle;
  states state = OFF;
  int obstacle_state;
  float ultra_dist;
  wait_cases wait_case;

  // Set up servo and pressure code
  

  float pressure;
  nrf_gpio_pin_dir_set(pinIn, NRF_GPIO_PIN_DIR_INPUT);
  printf("finished pwn init\n");
  // loop forever, running state machine
  while (1) {
    // power_manage();
  	// printf("top of while\n");
    // read sensors from robot
    int status = kobukiSensorPoll(&sensors);
    print_state(state);
    // ultras_update_distance();
    // printf("distance:%f\n", ultras_get_distance());

    switch(state) {

    	case OFF: {
        // received angle over BLE connection
        if (fabs(turn_angle) > 0) { //angle received
        	state = TURNING;
        	lsm9ds1_stop_gyro_integration();
        	lsm9ds1_start_gyro_integration();
        	saved_angle = turn_angle;
        	// turn_angle = 0;
        	printf("going to turning\n");
        }
        else {
          state = OFF;
          kobukiDriveDirect(0,0); 
        }

        //TEMP TRIAL CODE. FOR REAL CODE, UNCOMMENT THE STUFF ABOVE THIS

        // state = DROP;
        
        break;
      }

     	case TURNING: {
        // printf("saved angle: %f\n", saved_angle);
	      initial_gyro = lsm9ds1_read_gyro_integration();
	      snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
	      display_write(buf, DISPLAY_LINE_1);
        // printf("initial gyro z axis: %f\n, initial_gyro.z");

        if (is_button_pressed(&sensors)) { // turn off
        	kobukiDriveDirect(0,0); 
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);
					state = OFF;
					// starting_value2 = sensors.rightWheelEncoder;

	      } else if (fabs(initial_gyro.z_axis) >= fabs(saved_angle)) { // reached angle; finished turning
          printf("done turning\n");
					lsm9ds1_stop_gyro_integration();
					kobukiDriveDirect(0,0);
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);
					starting_value = sensors.rightWheelEncoder;
					displacement = 0;
					state = DRIVING; 

          //TEMP CHANGE
          // turn_angle = 0;
          // state = GRAB;

	      } else if (saved_angle >= 0) {
					kobukiDriveDirect(40, -40); // keep turning right
					snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
					display_write(buf, DISPLAY_LINE_1);
					state = TURNING;

	      } else {
					kobukiDriveDirect(-40, 40); // keep turning left
					snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
					display_write(buf, DISPLAY_LINE_1);
					state = TURNING;
	      }
	      break;
    	}

      case DRIVING: {
	      displacement = measure_distance(sensors.rightWheelEncoder, starting_value);
	      // printf("%f\n", displacement);
        if (obstacle(sensors)) {
        	printf("obs\n");
          kobukiDriveDirect(0,0);
          state = OBSTACLE_DETECT;
          starting_value2 = sensors.rightWheelEncoder;
        
        } else if (is_button_pressed(&sensors)) {
        	printf("pressed\n");
					state = OFF;
					kobukiDriveDirect(0,0);  
					snprintf(buf, 16, "%f", displacement);
					display_write(buf, DISPLAY_LINE_1);
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);
					// starting_value2 = sensors.rightWheelEncoder;


	      } else if (displacement >= 2) { // test purposes -- drove far enough
	      	printf("done\n");
					state = OFF;
					kobukiDriveDirect(0,0);
					update_grab(true);
					display_write("FINISHED", DISPLAY_LINE_1);
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);


	      } else if (fabs(turn_angle) > 15) { //angle was updated
	      	printf("reangle, new angle is: %f\n", turn_angle);
					saved_angle = turn_angle;
					turn_angle = 0;
					kobukiDriveDirect(0,0);
					lsm9ds1_stop_gyro_integration();
					state = TURNING;
					nrf_delay_ms(100);
					kobukiSensorPoll(&sensors);
					lsm9ds1_start_gyro_integration();
					snprintf(buf, 16, "%f", saved_angle);
					display_write(buf, DISPLAY_LINE_0);

	      } else { // drive
	      	// printf("driving now\n");
					kobukiDriveDirect(0, 0);
					display_write("DRIVING", DISPLAY_LINE_0);
					snprintf(buf, 16, "%f", displacement);
					display_write(buf, DISPLAY_LINE_1);
					state = DRIVING;
				}
				break;
	    }

	    case OBSTACLE_DETECT: {
	    	state = OBSTACLE_BACK;
	    }
	    // back up 0.1 m from obstacle
	    case OBSTACLE_BACK: {
	    displacement = measure_distance(starting_value2, sensors.rightWheelEncoder);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
          break;
        }
        if (displacement >= 0.1) {
          state = OBSTACLE_TURN;
          kobukiDriveDirect(0,0);  
          ret_code_t result = lsm9ds1_start_gyro_integration();
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);
        } else {
          // perform state-specific actions here
          kobukiDriveDirect(-65,-65);
          snprintf(buf, 16, "%f", displacement);
          display_write(buf, DISPLAY_LINE_1);
          state = OBSTACLE_BACK;     
        }
        break;
      }
      // turn 45 degrees away from obstacle
      case OBSTACLE_TURN: {
        initial_gyro = lsm9ds1_read_gyro_integration();
        snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
        display_write(buf, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          kobukiDriveDirect(0,0); 
          nrf_delay_ms(100);
          kobukiSensorPoll(&sensors);

        } else if (fabs(initial_gyro.z_axis) >=  45) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0,0);
          nrf_delay_ms(500);
          kobukiSensorPoll(&sensors);
          state = DRIVING;
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
	          state = OBSTACLE_TURN;
	        } else {
	          kobukiDriveDirect(-40, 40);
	          snprintf(buf, 16, "%f", fabs(initial_gyro.z_axis));
	          display_write(buf, DISPLAY_LINE_1);
	          state = OBSTACLE_TURN;
	        }
	        break;
	      }
	    }

      case GRAB: {
  	//GRAB STUFF I GUESS
        //TEMP CODE TO TEST GRIPPERS AND PRESSURE SENSOR
        if (picked && turn_angle > 0) {
          saved_angle = turn_angle;
          state = TURNING;
        }
        if (picked && turn_angle == 0) {
          //Fully lowered
          while(app_pwm_channel_duty_set(&PWM1, 1, 9.5) == NRF_ERROR_BUSY);
          nrf_delay_ms(2400);

          //Fully open
          while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
          getPressure(&pressure);
          printf("Pressure is ");
          printf("%f",pressure);
          nrf_delay_ms(2400);

            //Fully raised
          while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
          nrf_delay_ms(2400);

          picked = false;
          state = GRAB;
        }
        if (!picked) {
          //Fully lowered
          while(app_pwm_channel_duty_set(&PWM1, 1, 9.5) == NRF_ERROR_BUSY);
          nrf_delay_ms(2400);
       
          //Fully open
          while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
          getPressure(&pressure);
          printf("Pressure is ");
          printf("%f",pressure);
          nrf_delay_ms(2400);

          //Fully raised
          while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
          nrf_delay_ms(2400);
          picked = true;
          state = GRAB;
        }

		  	break;
		  }

		  case DROP: {
        // TEMP CODE TO SETUP GRIPPERS AND PRESSURE SENSOR
        //Fully raised
        printf("entered drop");
      while(app_pwm_channel_duty_set(&PWM1, 1, 5) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400);

      //Fully open
      while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400);
      getPressure(&pressure);
      printf("Pressure is ");
      printf("%f",pressure);

      state = GRAB;

		  	break;
		  }
		  	  // wait for a response from central
		  case WAIT: {
		  	if (wait_case == OBSTACLE) {
			    if (obstacle_state == 0) {
						state = DRIVING;

			    } else if (obstacle_state == 1) {
			    	nrf_delay_ms(100);
	          kobukiSensorPoll(&sensors);
	          starting_value2 = sensors.rightWheelEncoder;
						state = OBSTACLE_BACK;

	      	} else if (obstacle_state == 2) {
						state = GRAB;
	        
	        } else {
			  		state = WAIT;
			  	}
			  } else {
		  	//insert other reasons for waiting.
			  	break;
			  }
			  break;
			}
    }
  }
}

	    
	    

      

	  

