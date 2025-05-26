// Simple FSR (Gas & Brake) reader on Micro:bit v2
#include <stdint.h>
#include <stdio.h>

#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "app_error.h"

// Map sensors: analog inputs
#define ANALOG_GAS_IN    NRF_SAADC_INPUT_AIN1   // Gas on breakout pin 1
#define ANALOG_BRAKE_IN  NRF_SAADC_INPUT_AIN0   // Brake on breakout pin 0

// SAADC channel indices
#define ADC_GAS_CHANNEL    0
#define ADC_BRAKE_CHANNEL  1

// Dummy SAADC event handler (required by driver)
static void saadc_event_handler(nrfx_saadc_evt_t const* _evt) {}

// Initialize SAADC and two channels
static void adc_init(void) {
    nrfx_saadc_config_t cfg = {
        .resolution         = NRF_SAADC_RESOLUTION_12BIT,
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = 4,
        .low_power_mode     = false,
    };
    APP_ERROR_CHECK(nrfx_saadc_init(&cfg, saadc_event_handler));

    // Channel configs
    nrf_saadc_channel_config_t gas_cfg   = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_GAS_IN);
    nrf_saadc_channel_config_t brake_cfg = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_BRAKE_IN);

    APP_ERROR_CHECK(nrfx_saadc_channel_init(ADC_GAS_CHANNEL,   &gas_cfg));
    APP_ERROR_CHECK(nrfx_saadc_channel_init(ADC_BRAKE_CHANNEL, &brake_cfg));
}

// Blocking read of one channel: volts [0 … 3.6]
static float adc_read_volts(uint8_t channel) {
    int16_t counts = 0;
    APP_ERROR_CHECK(nrfx_saadc_sample_convert(channel, &counts));
    return ((float)counts / 4095.0f) * 3.6f;
}

int main(void) {
    printf("FSR reader starting...\n");
    adc_init();

    while (1) {
        float gas_v   = adc_read_volts(ADC_GAS_CHANNEL);
        float brake_v = adc_read_volts(ADC_BRAKE_CHANNEL);

        printf("Gas:   %.3f V\n", gas_v);
        printf("Brake: %.3f V\n\n", brake_v);
        
        // if above 2.5V, assume pressed
        if (gas_v > 2.5f) {
            printf("Gas\n");}
        if (brake_v > 2.5f) {
            printf("Brake\n");}

        nrf_delay_ms(500);
    }
}
