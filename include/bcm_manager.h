#ifndef BCM_MANAGER_H
#define BCM_MANAGER_H

#include <stdbool.h>
#include "common/common_types.h" // For status_t

/**
 * @brief Initializes the bcm2835 library if not already initialized.
 *        Increments an internal reference counter.
 * @return STATUS_OK on success (or if already initialized), ERROR_HARDWARE on failure.
 */
status_t bcm_manager_init(void);

/**
 * @brief Decrements the internal reference counter. If the counter reaches zero,
 *        closes the bcm2835 library.
 */
void bcm_manager_close(void);

/**
 * @brief Checks if the bcm2835 library is currently initialized.
 * @return True if initialized, false otherwise.
 */
bool bcm_manager_is_initialized(void);

#endif // BCM_MANAGER_H 