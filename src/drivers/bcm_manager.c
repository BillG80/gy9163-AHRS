#include "bcm_manager.h"
#include "common/common_types.h"
#include <bcm2835.h>
#include <stdio.h> // For error messages
#include <stdbool.h>

static bool bcm_initialized = false;

status_t bcm_manager_init(void) {
    if (bcm_initialized) {
        printf("INFO: bcm_manager: Already initialized.\n");
        return STATUS_OK; // Already initialized is not an error
    }

    if (!bcm2835_init()) {
        fprintf(stderr, "ERROR: bcm_manager: bcm2835_init failed. Are you running as root?\n");
        return ERROR_HARDWARE; // Return error code
    }

    bcm_initialized = true;
    printf("INFO: bcm_manager: bcm2835 library initialized successfully.\n");
    return STATUS_OK; // Return success code
}

void bcm_manager_close(void) {
    if (bcm_initialized) {
        bcm2835_close();
        bcm_initialized = false;
        printf("INFO: bcm_manager: bcm2835 library closed.\n");
    } else {
        printf("INFO: bcm_manager: Already closed or never initialized.\n");
    }
}

bool bcm_manager_is_initialized(void) {
    return bcm_initialized;
} 