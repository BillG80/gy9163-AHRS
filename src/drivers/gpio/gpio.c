#include "drivers/gpio/gpio.h"
#include <bcm2835.h>
#include <stdio.h> // For potential error messages
#include "bcm_manager.h" // Keep manager for is_initialized check

// Note: This driver currently assumes bcm2835_init() has been called
// by another module (e.g., SPI or I2C driver) before these functions
// are used. A more robust implementation would manage initialization centrally.

status_t gpio_init(void) {
    // REMOVED: Call to bcm_manager_init()
    // Assume library is initialized by application.
    // Check if it *is* initialized before returning OK.
    if (!bcm_manager_is_initialized()) {
         fprintf(stderr, "ERROR: gpio_init: bcm2835 library not initialized by application.\n");
         return ERROR_NOT_INITIALIZED;
    }
    return STATUS_OK;
}

status_t gpio_set_mode(uint8_t pin, gpio_mode_t mode) {
    // Check if library is initialized before using it
    if (!bcm_manager_is_initialized()) {
         fprintf(stderr, "ERROR: gpio_set_mode: bcm2835 library not initialized.\n");
         return ERROR_NOT_INITIALIZED;
    }
    bcm2835_gpio_fsel(pin, (uint8_t)mode);
    return STATUS_OK;
}

status_t gpio_write(uint8_t pin, gpio_state_t state) {
    // Check if library is initialized before using it
    if (!bcm_manager_is_initialized()) {
         fprintf(stderr, "ERROR: gpio_write: bcm2835 library not initialized.\n");
         return ERROR_NOT_INITIALIZED;
    }
    bcm2835_gpio_write(pin, (uint8_t)state);
    return STATUS_OK;
}

gpio_state_t gpio_read(uint8_t pin) {
    // Check if library is initialized before using it
    if (!bcm_manager_is_initialized()) {
         fprintf(stderr, "WARN: gpio_read: bcm2835 library not initialized. Returning LOW.\n");
         return GPIO_LOW;
    }
    return (gpio_state_t)bcm2835_gpio_lev(pin);
}

status_t gpio_set_pud(uint8_t pin, gpio_pud_t pud) {
    // Check if library is initialized before using it
    if (!bcm_manager_is_initialized()) {
         fprintf(stderr, "ERROR: gpio_set_pud: bcm2835 library not initialized.\n");
         return ERROR_NOT_INITIALIZED;
    }
    bcm2835_gpio_set_pud(pin, (uint8_t)pud);
    return STATUS_OK;
}

void gpio_cleanup(void) {
    // Does nothing now. Application handles library cleanup.
    // REMOVED: bcm_manager_close();
}

// Removed the i2c_is_bcm2835_initialized() helper function.
// Cleanup logic in SPI/I2C needs simplification. 