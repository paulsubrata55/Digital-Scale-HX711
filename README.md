# Digital Scale with HX711 Load Cell ADC

A high-precision digital weighing scale firmware for **ATmega16** microcontroller using the **HX711** 24-bit ADC. This project delivers commercial-grade accuracy with professional signal processing algorithms.

![License](https://img.shields.io/badge/license-MIT-blue.svg)

## ✅ Hardware Tested & Verified

This firmware has been **extensively hardware tested over prolonged durations** and delivers:
- **Commercial-grade accuracy** – readings match professional weighing scales
- **Excellent repeatability** – consistent measurements over time
- **Zero display flicker** – stable, professional display output
- **Sub-gram precision** – highly accurate weight detection

## Features

- 🎯 **24-bit ADC Resolution** with 19-21 bit effective resolution
- 📊 **Advanced Signal Processing** based on Analog Devices reference design
- ⚡ **Fast Response Time** – less than 1 second
- 🔧 **Built-in Calibration Mode** – easy 1000g reference calibration
- 💾 **EEPROM Storage** – calibration values persist across power cycles
- 🔋 **Battery Monitoring** – low battery detection (optional)
- 🖥️ **7-Segment Display** – multiplexed 6-digit display
- 🔊 **Buzzer Feedback** – audio confirmation for button presses
- 💤 **Screensaver Mode** – activates after prolonged zero reading

## Signal Processing Algorithms

This implementation follows best practices from the Analog Devices application note:
> *"A Reference Design for High-Performance, Low-Cost Weigh Scales"*  
> by Colm Slattery and Mariah Nie

### 1. Moving Average Filter with Outlier Rejection
- 15-point circular buffer
- Removes min/max outliers before averaging
- Improves noise performance by ~2.3 bits

### 2. Weight Change Detection (Doubled Judging)
- Compares new reading against filter average
- Requires two consecutive readings to exceed threshold
- Eliminates false triggers from single glitches

### 3. Load Cell Settling Compensation
- Refreshes filter buffer for 6 cycles after weight change
- Handles mechanical settling time of load cell

### 4. Display Flicker Reduction
- 1 gram hysteresis threshold
- Prevents flickering between adjacent display values

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Microcontroller | ATmega16 @ 8MHz |
| ADC Module | HX711 24-bit Load Cell ADC |
| Load Cell | Any compatible strain gauge load cell |
| Display | 6-digit 7-segment display (multiplexed) |
| Buttons | 2x tactile switches (Tare & Mode) |
| Buzzer | Active buzzer (optional) |

### Pin Configuration

| Function | Port | Pin |
|----------|------|-----|
| HX711 DATA | PORTB | PB7 |
| HX711 CLK | PORTB | PB6 |
| Display Control | PORTB | PB0-PB5 |
| Digit Control | PORTD | PD0-PD7 |
| Tare Switch | PINC | PC0 |
| Mode Switch | PINC | PC1 |
| Buzzer | PORTC | PC2 |
| Battery ADC | PORTA | PA0 |

## 🖥️ LCD Display Support (16x2)

This project can be easily extended to use a **16x2 LCD display** instead of 7-segment displays. Simply:

1. Add your preferred LCD library (e.g., `lcd.h` for HD44780-based displays)
2. Replace the `print_weight()` and `print_string()` functions with LCD equivalents
3. Connect the LCD to available I/O pins

Example LCD integration:
```c
#include "lcd.h"

void print_weight_lcd(int32_t weight) {
    char buffer[16];
    sprintf(buffer, "Weight: %ld g", weight);
    lcd_clear();
    lcd_puts(buffer);
    _delay_us(500);
}
```

The HX711 library (`hx_711.c` and `hx_711.h`) is completely display-agnostic and works with any output method.

## Usage

### Normal Operation
1. Power on the scale
2. Wait for automatic tare (zeroing)
3. Place items to weigh
4. Press **Tare** button to zero with items on scale

### Calibration Mode
1. Hold **Mode** button during power-up
2. Display shows "CALI" – press **Tare** to continue
3. Display shows "LOAD" – place known 1000g weight, press **Tare**
4. Display shows "1000G" – press **Tare** to confirm
5. Display shows "DONE" – calibration saved to EEPROM

## Building the Project

### Using AVR-GCC
```bash
avr-gcc -mmcu=atmega16 -Os -DF_CPU=8000000UL -o scale.elf main.c hx_711.c
avr-objcopy -O ihex scale.elf scale.hex
```

### Using Atmel Studio / Microchip Studio
1. Create new GCC C Executable Project
2. Select ATmega16 as target device
3. Add `main.c`, `hx_711.c`, and `hx_711.h` to project
4. Build and flash

### Fuse Settings
- **CKSEL**: Internal 8MHz RC oscillator
- **JTAGEN**: Disabled (required for PORTC pins)

## API Reference

### Core Functions

| Function | Description |
|----------|-------------|
| `scale_init_hardware()` | Initialize HX711 pins and gain |
| `scale_tare_zero()` | Set current reading as zero point |
| `scale_get_weight()` | Get filtered weight in grams |
| `scale_set_factor(int32_t)` | Set calibration scale factor |
| `scale_get_factor()` | Get current calibration factor |

### Low-Level ADC Functions

| Function | Description |
|----------|-------------|
| `adc_read_raw()` | Read raw 24-bit ADC value |
| `adc_read_averaged(count)` | Read with outlier rejection |
| `adc_read_filtered()` | Read with full signal processing |
| `adc_read_with_offset()` | Read with tare offset applied |

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| ADC Resolution | 24-bit |
| Effective Resolution | 19-21 bits |
| Sample Rate | 10/80 SPS (HX711 dependent) |
| Response Time | < 1 second |
| Display Update | ~2.4ms per digit |
| Calibration Storage | EEPROM (non-volatile) |

## File Structure

```
Digital-Scale-HX711/
├── main.c          # Main application, display, UI
├── hx_711.c        # HX711 driver with signal processing
├── hx_711.h        # Public API header
├── LICENSE         # License file
└── README.md       # This file
```

## References

- [HX711 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf)
- [Analog Devices: A Reference Design for High-Performance, Low-Cost Weigh Scales](https://www.analog.com/en/resources/analog-dialogue/articles/a-reference-design-for-weigh-scales.html)

## License

This project is licensed under the Apache License - see the [LICENSE](LICENSE) file for details.

## Author

Developed and hardware tested by the project contributors.

---

**Note:** This firmware has been rigorously tested on actual hardware for extended periods. The weighing accuracy and stability are comparable to commercial digital scales, making it suitable for both hobby projects and practical applications.
