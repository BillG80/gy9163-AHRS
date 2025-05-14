#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "common/common_types.h" // For status_t

// GPIO Pin Modes (matching bcm2835 for simplicity here)
typedef enum {
    GPIO_MODE_INPUT  = 0, // BCM2835_GPIO_FSEL_INPT
    GPIO_MODE_OUTPUT = 1, // BCM2835_GPIO_FSEL_OUTP
    // Add ALT modes if needed later, e.g., GPIO_MODE_ALT0 = 4, etc.
} gpio_mode_t;

// GPIO Pin State
typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1,
} gpio_state_t;

// GPIO Pull-up/down Resistor Configuration
typedef enum {
    GPIO_PUD_OFF  = 0, // BCM2835_GPIO_PUD_OFF
    GPIO_PUD_DOWN = 1, // BCM2835_GPIO_PUD_DOWN
    GPIO_PUD_UP   = 2, // BCM2835_GPIO_PUD_UP
} gpio_pud_t;

// --- Function Prototypes ---

/**
 * @brief Initializes the underlying GPIO system (if needed).
 *        Currently relies on SPI/I2C drivers initializing bcm2835.
 * @return STATUS_OK or an error code.
 */
status_t gpio_init(void);

/**
 * @brief Sets the mode (Input/Output) for a specific GPIO pin.
 *
 * @param pin The RPI_GPIO_P1_XX number (e.g., RPI_GPIO_P1_11 for GPIO17).
 * @param mode The desired mode (GPIO_MODE_INPUT or GPIO_MODE_OUTPUT).
 * @return STATUS_OK or an error code.
 */
status_t gpio_set_mode(uint8_t pin, gpio_mode_t mode);

/**
 * @brief Sets the output state of a GPIO pin configured as output.
 *
 * @param pin The RPI_GPIO_P1_XX number.
 * @param state The desired state (GPIO_LOW or GPIO_HIGH).
 * @return STATUS_OK or an error code.
 */
status_t gpio_write(uint8_t pin, gpio_state_t state);

/**
 * @brief Reads the input state of a GPIO pin.
 *
 * @param pin The RPI_GPIO_P1_XX number.
 * @return GPIO_HIGH or GPIO_LOW.
 */
gpio_state_t gpio_read(uint8_t pin);

/**
 * @brief Configures the pull-up/pull-down resistor for a GPIO pin.
 *
 * @param pin The RPI_GPIO_P1_XX number.
 * @param pud The desired pull configuration (GPIO_PUD_OFF, GPIO_PUD_UP, GPIO_PUD_DOWN).
 * @return STATUS_OK or an error code.
 */
status_t gpio_set_pud(uint8_t pin, gpio_pud_t pud);

/**
 * @brief Cleans up GPIO resources (if needed).
 *        Currently relies on SPI/I2C drivers cleaning up bcm2835.
 */
void gpio_cleanup(void);

// TODO: Add interrupt handling functions (gpio_set_interrupt, etc.) later if needed.

// Raspberry Pi header pin macros (BCM numbering)
// See: https://pinout.xyz/

// Raspberry Pi header pin macros (BCM numbering)
// See: https://pinout.xyz/


#endif // GPIO_H