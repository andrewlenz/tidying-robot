/**
 
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

// Create a pwm instance by passing the name of the handle and the timer to be used for pwm generation
// here 1 means timer 1
APP_PWM_INSTANCE(PWM1, 1);
APP_PWM_INSTANCE(PWM2, 0);


// Select a pin where we want to connect the servo's pin for pwm
#define  SERVO_PIN_GRIPPER   BUCKLER_SD_MISO
#define  SERVO_PIN_RAISER   BUCKLER_SD_MOSI

#define TIME_PERIOD  20000 // 20 milliseconds

static void pwm_init() {
  // a variable to hold err_code value
  ret_code_t err_code1;
  ret_code_t err_code2;

//  create a pwm config struct and pass it the default configurations along with the pin number for pwm and period in microseconds
// here we have configured one channel, we can configure max up to two channels per pwm instance in this library, if we use two channels then we need to pass two pins
  app_pwm_config_t pwm_cfg1 = APP_PWM_DEFAULT_CONFIG_1CH(TIME_PERIOD, SERVO_PIN_GRIPPER);

  app_pwm_config_t pwm_cfg2 = APP_PWM_DEFAULT_CONFIG_1CH(TIME_PERIOD, SERVO_PIN_RAISER);

// change the pwm polarity by setting this value so that logic high value is given as active signal and duty is set for the logic high signal
// we can change it to give the active low signal as well for manipulating the logic low duty
  pwm_cfg1.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
  pwm_cfg2.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

// Initialize the pwm and pass it the intance, configurations and handler, we can also write NULL if we don't want to use the handler 
  err_code1 = app_pwm_init(&PWM1, &pwm_cfg1, NULL);
  err_code2 = app_pwm_init(&PWM2, &pwm_cfg2, NULL);
  APP_ERROR_CHECK(err_code1);// check if any error occurred during initialization
  APP_ERROR_CHECK(err_code2);
// enable the pwm signal so that the pwm is started on the specified pin
  app_pwm_enable(&PWM1);
  app_pwm_enable(&PWM2);
}



int main(void)
{

pwm_init();

// Infinite loop
  while(true)
  {
	  // here we are changing the duty cycle so as to move the servo in specific direction
	  // adjust these acording to your motor specs
	  // set the duty in milliseconds time period to control its position
    // for(int i = 2; i < 11; i++)
    // {
      while(app_pwm_channel_duty_set(&PWM1, 0, 11) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400); // half second delay after every step
    // }

    // for(int i = 10; i>1; i--)
    // {
      while(app_pwm_channel_duty_set(&PWM1, 0, 3) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400);
      
    // }

      while(app_pwm_channel_duty_set(&PWM2, 0, 5) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400);

      while(app_pwm_channel_duty_set(&PWM2, 0, 9.5) == NRF_ERROR_BUSY);
      nrf_delay_ms(2400);
  
  }

  


   
}


/** @} */

// https://devzone.nordicsemi.com/f/nordic-q-a/77884/precise-angle-control-of-servo-motor-using-pwm?Redirected=true
// https://towardsdatascience.com/live-video-streaming-using-multiple-smartphones-with-imagezmq-e260bd081224
// https://www.tutorialspoint.com/draw-a-line-on-an-image-using-opencv#:~:text=Draw%20a%20line%20on%20an%20image%20using%20OpenCV,a%20blue%20line%20is%20drawn%20on%20the%20image.

