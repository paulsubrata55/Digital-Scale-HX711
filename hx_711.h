/*
Enhanced with algorithms from:
"A Reference Design for High-Performance, Low-Cost Weigh Scales"
Analog Devices Application Note by Colm Slattery and Mariah Nie

Modified 2026 - Signal processing enhancements
Free to use for any purpose.
 */
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>


#ifndef __HX711_H__
#define __HX711_H__


// Hardware pin configuration
#define ADC_DATA_PORT PORTB
#define ADC_DATA_DDR DDRB
#define ADC_DATA_PIN PINB
#define ADC_DATA_NUM PB7
#define ADC_CLK_PORT PORTB
#define ADC_CLK_DDR DDRB
#define ADC_CLK_NUM PB6

// PGA gain settings
#define GAIN_CHANNEL_A_128 1
#define GAIN_CHANNEL_A_64 3
#define GAIN_CHANNEL_B_32 2
#define GAIN_DEFAULT GAIN_CHANNEL_A_128

// Default calibration values
#define SCALE_DEFAULT 10000
#define OFFSET_DEFAULT 8000000

// Averaging configuration
#define AVG_SAMPLE_COUNT 25
#define CALIBRATION_SAMPLES 5

// Public API functions
extern int32_t adc_read_raw();
extern int32_t adc_read_averaged(int32_t sample_count);
extern int32_t adc_read_filtered();
extern int32_t adc_read_with_offset();
extern int32_t scale_get_weight();
extern void scale_set_gain(uint16_t gain);
extern uint16_t scale_get_gain();
extern void scale_set_factor(int32_t factor);
extern int32_t scale_get_factor();
extern void scale_set_zero_offset(int32_t offset);
extern int32_t scale_get_zero_offset();
extern void scale_tare_zero();
extern void scale_init_hardware();
extern int32_t adc_read_calibration();

#endif