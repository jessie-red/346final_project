// Steering-wheel Driver 
// Read from I2C accelerometer on the Microbit to determine steering angle and direction from the Z-axis tilt
// It clamps at ±30°, then prints the value and Left/Right based on sign

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "app_timer.h"

#include "microbit_v2.h"
#include "lsm303agr.h"

NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
APP_TIMER_DEF(sensor_timer);

static void sensor_timer_cb(void* _unused) {
    lsm303agr_measurement_t a = lsm303agr_read_accelerometer();

    // Compute roll angle: board rolling right gives +x acceleration
    // Default flat (z: +1g) - atan2(0,+1) = 0
    float angle_rad = atan2f(a.x_axis, a.z_axis);
    float angle_deg = angle_rad * (180.0f / M_PI);

    // Clamp to ±45°
    if (angle_deg >  45.0f) angle_deg =  45.0f;
    if (angle_deg < -45.0f) angle_deg = -45.0f;

    // Determine direction
    const char* dir = (angle_deg >  1.0f) ? "Right"
                       : (angle_deg < -1.0f) ? "Left"
                       :                       "Center";

    printf("Steering: %+05.1f° [%s]\n", angle_deg, dir);
}

int main(void) {
  printf("Steering driver starting...\n");

  app_timer_init();

  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = I2C_INTERNAL_SCL;
  i2c_config.sda = I2C_INTERNAL_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  lsm303agr_init(&twi_mngr_instance);

  app_timer_create(&sensor_timer,
                    APP_TIMER_MODE_REPEATED,
                    sensor_timer_cb);
  app_timer_start(sensor_timer,
                  APP_TIMER_TICKS(1000),  
                  NULL);

  while (1) {
    nrf_delay_ms(1000);
  }
}

