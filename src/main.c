#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#define ADC_CHANNEL     ADC_CHANNEL_3    // GPIO27 is connected to ADC2 Channel 3
#define ADC_WIDTH       ADC_BITWIDTH_12  // ADC width of 12 bits
#define ADC_ATTEN       ADC_ATTEN_DB_2_5  // ADC attenuation of 11 dB (0-3.9V)
#define SAMPLES         16               // Number of samples for averaging

void app_main() {
    adc_cali_handle_t adc_cali_handle = NULL;
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));

    while (1) {
        int32_t adc_reading = 0;
        for (int i = 0; i < SAMPLES; i++) {
            int raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw));
            adc_reading += raw;
        }
        adc_reading /= SAMPLES;

        /*
            //float voltage = (adc_reading * 1.5) / (1 << ADC_WIDTH);

            // Calibrate based on experiments:
            //voltage -= 0.07;
        */

        int voltage;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage));
        float scaled_voltage = (voltage * 6.0) / 0.5;

        // Calibrate based on experiments. This probably changes based on
        // on how the components in the voltage divider actually perform
        // and loss due to breadboard.
        scaled_voltage /= 1.08;

        printf("Raw ADC value: %ld\tVoltage: %d mV\tScaled Voltage: %.2f mV\n", adc_reading, voltage, scaled_voltage);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle));
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
}
