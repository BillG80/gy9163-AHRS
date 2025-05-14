#ifndef FILE_LOGGER_H_
#define FILE_LOGGER_H_

#include <stdint.h>
#include <stdarg.h> // For va_list in log_message
#include "common/common_defs.h" // For FusionData_T, NavData_T

// Configuration structure (optional, could pass args directly to init)
typedef struct {
    const char* base_log_directory;
    const char* base_filename_prefix; // e.g., "sensor_log"
    uint32_t max_file_size_kb;      // Max size before rotation
    uint8_t max_backup_files;       // Number of backup files (0 for no rotation)
} FileLoggerConfig_T;

/**
 * @brief Initializes the file logger.
 * Creates the log directory if it doesn't exist.
 * Opens the initial log file. Sets up rotation parameters.
 * @param config Pointer to the logger configuration structure.
 * @return 0 on success, -1 on error (e.g., cannot create directory/file).
 */
int file_logger_init(const FileLoggerConfig_T* config);

/**
 * @brief Logs the combined sensor fusion and navigation data.
 * Formats the data (e.g., as CSV) and writes it to the current log file.
 * Handles log rotation if necessary.
 * @param fusion_data Pointer to the sensor fusion data.
 * @param nav_data Pointer to the navigation data.
 * @return 0 on success, -1 on error (e.g., file not open).
 */
int file_logger_log_data(const FusionData_T* fusion_data, const NavData_T* nav_data);

/**
 * @brief Logs a general formatted message (like printf).
 * Useful for logging events, errors, or status changes.
 * Handles log rotation if necessary.
 * @param format The format string.
 * @param ... Variable arguments for the format string.
 * @return Number of bytes written, or -1 on error.
 */
int file_logger_log_message(const char* format, ...);

/**
 * @brief Cleans up the file logger.
 * Writes any buffered data and closes the log file.
 */
void file_logger_cleanup(void);

#endif // FILE_LOGGER_H_