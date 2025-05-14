#ifndef LED_INDICATOR_H_
#define LED_INDICATOR_H_

#include <stdint.h>
#include "common/common_defs.h" // For StatusEnum and GPIO pin definitions

/**
 * @brief Initializes the LED indicator driver.
 * Configures the specified GPIO pins as outputs.
 * @param pin1 GPIO pin number for the first color LED.
 * @param pin2 GPIO pin number for the second color LED.
 * @return 0 on success, -1 on error (e.g., bcm2835 not initialized).
 */
int led_indicator_init(uint8_t pin1, uint8_t pin2);

/**
 * @brief Sets the desired system status to be displayed by the LED.
 * The actual LED state might be updated in led_indicator_update() based on blinking patterns.
 * @param status The system status to indicate.
 */
void led_indicator_set_status(StatusEnum status);

/**
 * @brief Updates the LED state based on the current status and time.
 * This function should be called periodically in the main loop to handle blinking.
 */
void led_indicator_update(void);

/**
 * @brief Cleans up the LED indicator driver.
 * Sets LED pins back to input or a safe state if needed.
 */
void led_indicator_cleanup(void);

#endif // LED_INDICATOR_H_