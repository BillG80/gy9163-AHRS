#include "drivers/led_indicator.h"
#include <stdio.h>          // For printf in stubs
#include <stdbool.h>        // For bool type
#include <bcm2835.h>
#include "common/common_defs.h" // Make sure StatusEnum is included

#define BLINK_3HZ_PERIOD_US 333333ULL
// --- Module Private Variables ---
static uint8_t _pin1; // Red LED (GPIO12)
static uint8_t _pin2; // Blue LED (GPIO16)
static StatusEnum _current_status = STATUS_OFF;
static uint64_t _last_toggle_time_us = 0;
static bool _led1_state = false; // Current physical state of Red LED
static bool _led2_state = false; // Current physical state of Blue LED

// --- Function Definitions ---

int led_indicator_init(uint8_t pin1, uint8_t pin2) {
    _pin1 = pin1;
    _pin2 = pin2;

    // bcm2835_init() should be called ONCE in main, not here.
    // Ensure it IS called before this function.

    bcm2835_gpio_fsel(_pin1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(_pin2, BCM2835_GPIO_FSEL_OUTP);

    // Start with LEDs off
    bcm2835_gpio_write(_pin1, LOW);
    bcm2835_gpio_write(_pin2, LOW);
    _led1_state = false;
    _led2_state = false;

    _current_status = STATUS_OFF;
    _last_toggle_time_us = bcm2835_st_read(); // Initialize toggle time

    printf("LED Indicator Initialized (Pin1/Red: %d, Pin2/Blue: %d)\n", _pin1, _pin2);
    return 0;
}

void led_indicator_set_status(StatusEnum status) {
    if (status != _current_status) {
        printf("LED Status changing to: %d\n", status);
        _current_status = status;
        // Reset toggle time immediately when status changes to ensure correct first blink/solid state
        _last_toggle_time_us = bcm2835_st_read();
        // Force update to apply new state immediately
        led_indicator_update();
    }
}

void led_indicator_update(void) {
    uint64_t current_time_us = bcm2835_st_read();
    uint64_t time_since_toggle_us = current_time_us - _last_toggle_time_us;

    // Calibration complete: cycle Red->Blue->Purple fast
    if (_current_status == STATUS_CALIB_COMPLETE) {
        // Use 3Hz total cycle (333ms) split into 3 segments
        const uint64_t cycle_us = BLINK_3HZ_PERIOD_US;
        const uint64_t seg = cycle_us / 3;
        uint64_t t = time_since_toggle_us % cycle_us;
        bool red_phase    = (t < seg);
        bool blue_phase   = (t >= seg && t < 2*seg);
        bool purple_phase = (t >= 2*seg);
        bool new_led1 = red_phase || purple_phase; // Red
        bool new_led2 = blue_phase || purple_phase; // Blue
        if (new_led1 != _led1_state) {
            _led1_state = new_led1;
            bcm2835_gpio_write(_pin1, new_led1 ? HIGH : LOW);
        }
        if (new_led2 != _led2_state) {
            _led2_state = new_led2;
            bcm2835_gpio_write(_pin2, new_led2 ? HIGH : LOW);
        }
        return;
    }

    // --- Define Blink Periods (Microseconds) ---
    const uint64_t BLINK_0_5HZ_PERIOD_US = 2000000;
    const uint64_t BLINK_1HZ_PERIOD_US   = 1000000;
    const uint64_t BLINK_2HZ_PERIOD_US   = 500000;
    const uint64_t BLINK_4HZ_PERIOD_US   = 250000;
    const uint64_t BLINK_5HZ_PERIOD_US   = 200000;

    bool target_led1_state = false; // Default Target state for Pin1 (RED)
    bool target_led2_state = false; // Default Target state for Pin2 (BLUE)
    uint64_t current_blink_period_us = 0; // 0 means solid state

    // --- Determine Target LED state and Blink Pattern based on status ---
    // Note: For BLE_ADVERTISING, the actual alternation is handled later
    switch (_current_status) {
        case STATUS_INITIALIZING:       // Fast Blink Blue (2Hz)
            target_led1_state = false; target_led2_state = true;
            current_blink_period_us = BLINK_2HZ_PERIOD_US;
            break;
        case STATUS_IDLE:               // Very Slow Blink Blue (0.5Hz)
            target_led1_state = false; target_led2_state = true;
            current_blink_period_us = BLINK_0_5HZ_PERIOD_US;
            break;
        case STATUS_RUNNING:            // Slow Blink Blue (1Hz)
            target_led1_state = false; target_led2_state = true;
            current_blink_period_us = BLINK_1HZ_PERIOD_US;
            break;
        case STATUS_ERROR_INIT:         // Fast Blink Red (2Hz)
            target_led1_state = true; target_led2_state = false;
            current_blink_period_us = BLINK_2HZ_PERIOD_US;
            break;
        case STATUS_SHUTTING_DOWN:     // Very Fast Blink Red (5Hz)
            target_led1_state = true; target_led2_state = false;
            current_blink_period_us = BLINK_5HZ_PERIOD_US;
            break;
        case STATUS_ERROR_RUNTIME:      // Solid Red
            target_led1_state = true; target_led2_state = false;
            current_blink_period_us = 0;
            break;
        case STATUS_BLE_ADVERTISING:    // Alternating Blue/Red (3Hz cycle)
            // Targets not used directly here, period set for timing logic below
            target_led1_state = false; // Doesn't matter here
            target_led2_state = false; // Doesn't matter here
            current_blink_period_us = BLINK_3HZ_PERIOD_US;
            break;
        case STATUS_BLE_CONNECTED:      // Solid BLUE
            target_led1_state = false; target_led2_state = true;
            current_blink_period_us = 0;
            break;
        case STATUS_BLE_ERROR:          // Fast Blink Purple (4Hz)
            target_led1_state = true; target_led2_state = true;
            current_blink_period_us = BLINK_4HZ_PERIOD_US;
            break;
        case STATUS_OFF:
        default:                        // Both OFF
            target_led1_state = false; target_led2_state = false;
            current_blink_period_us = 0;
            break;
    }

    // --- Calculate Final LED state based on Blink Logic ---
    bool final_led1_state = _led1_state; // Start with current physical state (RED)
    bool final_led2_state = _led2_state; // Start with current physical state (BLUE)

    // *** Special handling for Alternating BLE Advertising ***
    if (_current_status == STATUS_BLE_ADVERTISING) {
        uint64_t half_period = current_blink_period_us / 2;
        if (time_since_toggle_us < half_period) {
            // First half of cycle: Blue ON, Red OFF
            final_led1_state = false;
            final_led2_state = true;
        } else {
            // Second half of cycle: Blue OFF, Red ON
            final_led1_state = true;
            final_led2_state = false;
        }
        // Check if the full period has elapsed to reset the timer
        if (time_since_toggle_us >= current_blink_period_us) {
            _last_toggle_time_us = current_time_us;
        }
    }
    // *** Standard Blink Logic for other statuses ***
    else if (current_blink_period_us > 0) {
        bool current_phase_on = time_since_toggle_us < (current_blink_period_us / 2);
        final_led1_state = target_led1_state && current_phase_on;
        final_led2_state = target_led2_state && current_phase_on;

        if (time_since_toggle_us >= current_blink_period_us) {
            _last_toggle_time_us = current_time_us;
        }
    }
    // *** Solid State Logic ***
    else { // Solid state (or solid off)
        final_led1_state = target_led1_state;
        final_led2_state = target_led2_state;
        _last_toggle_time_us = current_time_us; // Keep timer updated
    }

    // --- Write to GPIO pins ONLY if the calculated state is different ---
    if (final_led1_state != _led1_state) {
        _led1_state = final_led1_state;
        bcm2835_gpio_write(_pin1, _led1_state ? HIGH : LOW);
    }
    if (final_led2_state != _led2_state) {
         _led2_state = final_led2_state;
        bcm2835_gpio_write(_pin2, _led2_state ? HIGH : LOW);
    }
}

void led_indicator_cleanup(void) {
    // Ensure LEDs are off before setting to input
    bcm2835_gpio_write(_pin1, LOW);
    bcm2835_gpio_write(_pin2, LOW);
    // Set pins back to input (good practice)
    bcm2835_gpio_fsel(_pin1, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(_pin2, BCM2835_GPIO_FSEL_INPT);
    printf("LED Indicator Cleaned Up.\n");
    // bcm2835_close(); // Should be called once at the end of the main program, not here.
}