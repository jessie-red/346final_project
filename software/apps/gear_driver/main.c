// Joystick Y-only gear-shifter on Micro:bit v2
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "app_error.h"

#define ANALOG_Y_IN       NRF_SAADC_INPUT_AIN0  
#define ADC_Y_CHANNEL     0

#define VREF_FULL_SCALE   3.6f                 
#define VCENTER           (VREF_FULL_SCALE/2)   
#define ANGLE_MAX         90.0f                 
#define SHIFT_THRESHOLD   10.0f                
#define MIN_GEAR          1
#define MAX_GEAR          5
#define ANGLE_BIAS        6.2f

static void saadc_evt(nrfx_saadc_evt_t const* _e) {}

static void adc_init(void) {
    nrfx_saadc_config_t cfg = {
        .resolution         = NRF_SAADC_RESOLUTION_12BIT,
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = 4,
        .low_power_mode     = false,
    };
    APP_ERROR_CHECK(nrfx_saadc_init(&cfg, saadc_evt));

    nrf_saadc_channel_config_t ch_cfg =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_Y_IN);
    APP_ERROR_CHECK(nrfx_saadc_channel_init(ADC_Y_CHANNEL, &ch_cfg));
}

static float read_voltage(void) {
    int16_t counts = 0;
    APP_ERROR_CHECK(nrfx_saadc_sample_convert(ADC_Y_CHANNEL, &counts));
    return ((float)counts / 4095.0f) * VREF_FULL_SCALE;
}

int main(void) {
    printf("Gear-shifter starting...\n");
    adc_init();

    int  gear           = MIN_GEAR;
    bool can_shift_up   = true;
    bool can_shift_down = true;

    while (1) {
        // Read Y voltage
        float v = read_voltage();

        // Map to signed angle: (v – center)/center * 90°
        float angle = ((v - VCENTER) / VCENTER) * ANGLE_MAX;
        // Apply bias to angle
        angle += ANGLE_BIAS;
        if (angle >  ANGLE_MAX) angle =  ANGLE_MAX;
        if (angle < -ANGLE_MAX) angle = -ANGLE_MAX;

        // Check thresholds and edge‐detect shifts
        bool shifted = false;
        const char* shift_dir = "";

        // Up if pushed forward (angle > +TH) and allowed
        if (angle > SHIFT_THRESHOLD && can_shift_up) {
            if (gear < MAX_GEAR) {
                gear++;
                shifted = true;
                shift_dir = "Up";
            }
            can_shift_up = false;
        }
        if (angle <= SHIFT_THRESHOLD) {
            can_shift_up = true;
        }

        // Down if pulled back (angle < -TH) and allowed
        if (angle < -SHIFT_THRESHOLD && can_shift_down) {
            if (gear > MIN_GEAR) {
                gear--;
                shifted = true;
                shift_dir = "Down";
            }
            can_shift_down = false;
        }
        if (angle >= -SHIFT_THRESHOLD) {
            can_shift_down = true;
        }

        printf("Angle: %+05.1f°  Gear: %d", angle, gear);
        if (shifted) {
            printf("  [%s]\n", shift_dir);
        } else {
            printf("\n");
        }

        nrf_delay_ms(200);
    }
}
