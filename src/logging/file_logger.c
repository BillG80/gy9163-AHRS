#include "logging/file_logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <sys/stat.h> // For mkdir, stat
#include <errno.h>
#include <unistd.h> // For rename, remove
#include <inttypes.h> // For PRIu64 if needed in messages (not directly for binary)
#include <bcm2835.h> // For bcm2835_st_read

// --- Module Private Variables ---
static FILE* _log_file_handle = NULL;
static char _current_log_filepath[512]; // Full path to current log file
static FileLoggerConfig_T _config;      // Store the config
static uint32_t _current_file_size = 0; // Track approximate size

// --- Record Type Markers ---
#define RECORD_TYPE_DATA    ((uint8_t)0x01)
#define RECORD_TYPE_MESSAGE ((uint8_t)0x02)
#define RECORD_TYPE_HEADER  ((uint8_t)0x00) // Optional: For file version/metadata

// --- Helper Function Declarations ---
static int _ensure_log_directory_exists(const char* dir_path);
static void _rotate_logs_if_needed(void);
static void _perform_log_rotation(void);
// static void _get_timestamp_str(char* buffer, size_t buffer_size); // No longer needed for data

// --- Function Definitions ---

int file_logger_init(const FileLoggerConfig_T* config) {
    if (!config || !config->base_log_directory || !config->base_filename_prefix) {
        fprintf(stderr, "ERROR: Invalid file logger configuration.\n");
        return -1;
    }
    _config = *config; // Copy config
    // Ensure max size is in bytes for binary comparison
    _config.max_file_size_kb *= 1024; // Convert KB to Bytes

    // Ensure log directory exists
    if (_ensure_log_directory_exists(_config.base_log_directory) != 0) {
        return -1;
    }

    // Construct the initial log file path (use .bin extension now)
    snprintf(_current_log_filepath, sizeof(_current_log_filepath), "%s/%s.bin",
             _config.base_log_directory, _config.base_filename_prefix);

    // --- Open the log file in append binary mode ---
    _log_file_handle = fopen(_current_log_filepath, "ab+"); // Append Binary mode
    if (_log_file_handle == NULL) {
        perror("ERROR: Failed to open binary log file");
        fprintf(stderr, "       File path: %s\n", _current_log_filepath);
        return -1;
    }

    // Get initial file size
    fseek(_log_file_handle, 0, SEEK_END);
    _current_file_size = ftell(_log_file_handle);
    rewind(_log_file_handle); // Go back to start for potential header read/write

    printf("Binary File Logger Initialized. Logging to: %s\n", _current_log_filepath);

    // Optional: Write a file header if the file is new
    if (_current_file_size == 0) {
        uint8_t record_type = RECORD_TYPE_HEADER;
        uint16_t version = 1; // Example version number
        time_t now = time(NULL);

        size_t written = 0;
        written += fwrite(&record_type, sizeof(record_type), 1, _log_file_handle);
        written += fwrite(&version, sizeof(version), 1, _log_file_handle);
        written += fwrite(&now, sizeof(now), 1, _log_file_handle);
        // Add other metadata if needed (e.g., struct sizes, field names - complicates things)

        fflush(_log_file_handle);
        _current_file_size = ftell(_log_file_handle); // Update size
        printf("Wrote binary log header (Version: %u).\n", version);
    } else {
        // Optional: Could read header here to check version compatibility if needed
        printf("Opened existing binary log file (Size: %u bytes).\n", _current_file_size);
    }


    file_logger_log_message("--- Log Session Started ---");

    return 0;
}

int file_logger_log_data(const FusionData_T* fusion_data, const NavData_T* nav_data) {
    if (_log_file_handle == NULL || !fusion_data || !nav_data) {
        return -1;
    }

    _rotate_logs_if_needed(); // Check and rotate before writing

    if (_log_file_handle == NULL) { // Check again in case rotation failed critically
         fprintf(stderr, "ERROR: Log file handle became NULL after rotation check.\n");
         return -1;
    }

    size_t items_written = 0;
    size_t total_bytes = 0;
    uint8_t record_type = RECORD_TYPE_DATA;
    // Map bool to uint8_t for consistent size
    uint8_t is_stationary_byte = nav_data->is_stationary ? 1 : 0;

    // --- Write fields sequentially ---
    items_written = fwrite(&record_type, sizeof(record_type), 1, _log_file_handle); total_bytes += sizeof(record_type) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->timestamp_us, sizeof(fusion_data->timestamp_us), 1, _log_file_handle); total_bytes += sizeof(fusion_data->timestamp_us) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->accel_x, sizeof(fusion_data->accel_x), 1, _log_file_handle); total_bytes += sizeof(fusion_data->accel_x) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->accel_y, sizeof(fusion_data->accel_y), 1, _log_file_handle); total_bytes += sizeof(fusion_data->accel_y) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->accel_z, sizeof(fusion_data->accel_z), 1, _log_file_handle); total_bytes += sizeof(fusion_data->accel_z) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->gyro_x, sizeof(fusion_data->gyro_x), 1, _log_file_handle); total_bytes += sizeof(fusion_data->gyro_x) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->gyro_y, sizeof(fusion_data->gyro_y), 1, _log_file_handle); total_bytes += sizeof(fusion_data->gyro_y) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->gyro_z, sizeof(fusion_data->gyro_z), 1, _log_file_handle); total_bytes += sizeof(fusion_data->gyro_z) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->mag_x, sizeof(fusion_data->mag_x), 1, _log_file_handle); total_bytes += sizeof(fusion_data->mag_x) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->mag_y, sizeof(fusion_data->mag_y), 1, _log_file_handle); total_bytes += sizeof(fusion_data->mag_y) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->mag_z, sizeof(fusion_data->mag_z), 1, _log_file_handle); total_bytes += sizeof(fusion_data->mag_z) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->pressure_pa, sizeof(fusion_data->pressure_pa), 1, _log_file_handle); total_bytes += sizeof(fusion_data->pressure_pa) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->temperature_c, sizeof(fusion_data->temperature_c), 1, _log_file_handle); total_bytes += sizeof(fusion_data->temperature_c) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->altitude_m, sizeof(fusion_data->altitude_m), 1, _log_file_handle); total_bytes += sizeof(fusion_data->altitude_m) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->q0, sizeof(fusion_data->q0), 1, _log_file_handle); total_bytes += sizeof(fusion_data->q0) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->q1, sizeof(fusion_data->q1), 1, _log_file_handle); total_bytes += sizeof(fusion_data->q1) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->q2, sizeof(fusion_data->q2), 1, _log_file_handle); total_bytes += sizeof(fusion_data->q2) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&fusion_data->q3, sizeof(fusion_data->q3), 1, _log_file_handle); total_bytes += sizeof(fusion_data->q3) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&nav_data->timestamp_us, sizeof(nav_data->timestamp_us), 1, _log_file_handle); total_bytes += sizeof(nav_data->timestamp_us) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&nav_data->vertical_accel_m_s2, sizeof(nav_data->vertical_accel_m_s2), 1, _log_file_handle); total_bytes += sizeof(nav_data->vertical_accel_m_s2) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&nav_data->vertical_velocity_m_s, sizeof(nav_data->vertical_velocity_m_s), 1, _log_file_handle); total_bytes += sizeof(nav_data->vertical_velocity_m_s) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&nav_data->vertical_distance_m, sizeof(nav_data->vertical_distance_m), 1, _log_file_handle); total_bytes += sizeof(nav_data->vertical_distance_m) * items_written; if(items_written != 1) goto write_error;
    items_written = fwrite(&is_stationary_byte, sizeof(is_stationary_byte), 1, _log_file_handle); total_bytes += sizeof(is_stationary_byte) * items_written; if(items_written != 1) goto write_error;

    fflush(_log_file_handle); // Ensure data is written
    _current_file_size += total_bytes;
    return 0; // Success

write_error:
    {
        int saved_errno = errno; // Save errno immediately
        fprintf(stderr, "ERROR: fwrite failed in file_logger_log_data. errno = %d (%s)\n", saved_errno, strerror(saved_errno));
        perror(" -> perror description"); // Still call perror for its description
        fflush(stderr); // Ensure the error message is flushed
    }
    // Attempt to keep log file usable, but record is incomplete
    fflush(_log_file_handle);
    _current_file_size = ftell(_log_file_handle); // Try to update size accurately
    return -1;
}

int file_logger_log_message(const char* format, ...) {
     if (_log_file_handle == NULL) {
        return -1;
    }

    _rotate_logs_if_needed();

    if (_log_file_handle == NULL) {
         fprintf(stderr, "ERROR: Log file handle became NULL after rotation check.\n");
         return -1;
    }

    // Prepare message buffer
    char message_buffer[1024]; // Adjust size as needed
    va_list args;
    va_start(args, format);
    int msg_len = vsnprintf(message_buffer, sizeof(message_buffer), format, args);
    va_end(args);

    // Check for encoding error or if message was truncated
    if (msg_len < 0 || (size_t)msg_len >= sizeof(message_buffer)) {
        fprintf(stderr, "Warning: Log message formatting error or buffer overflow for: %s\n", format);
         if (msg_len < 0) return -1; // Return error if vsnprintf failed
         // If truncated, adjust length to prevent writing partial data beyond buffer
         msg_len = sizeof(message_buffer) - 1; // Adjust length to what was actually written
    }

    uint8_t record_type = RECORD_TYPE_MESSAGE;
    uint64_t timestamp_us = bcm2835_st_read(); // Get current timestamp
    uint16_t message_len_bytes = (uint16_t)msg_len; // Length of the string itself

    size_t items_written = 0;
    size_t total_bytes = 0;

    items_written = fwrite(&record_type, sizeof(record_type), 1, _log_file_handle); total_bytes += sizeof(record_type) * items_written; if(items_written != 1) goto write_error_msg;
    items_written = fwrite(&timestamp_us, sizeof(timestamp_us), 1, _log_file_handle); total_bytes += sizeof(timestamp_us) * items_written; if(items_written != 1) goto write_error_msg;
    items_written = fwrite(&message_len_bytes, sizeof(message_len_bytes), 1, _log_file_handle); total_bytes += sizeof(message_len_bytes) * items_written; if(items_written != 1) goto write_error_msg;
    items_written = fwrite(message_buffer, sizeof(char), message_len_bytes, _log_file_handle); total_bytes += sizeof(char) * items_written; if(items_written != message_len_bytes) goto write_error_msg;


    fflush(_log_file_handle);
    _current_file_size += total_bytes;
    return msg_len; // Return length of the actual message string

write_error_msg:
    perror("ERROR: Failed to write full message record to binary log file");
    fflush(_log_file_handle);
    _current_file_size = ftell(_log_file_handle);
    return -1;
}

void file_logger_cleanup(void) {
    if (_log_file_handle != NULL) {
        printf("Cleaning up binary file logger...\n");
        file_logger_log_message("--- Log Session Ended ---");
        fclose(_log_file_handle);
        _log_file_handle = NULL;
        printf("Binary log file closed.\n");
    }
}

// --- Helper Function Implementations ---

static int _ensure_log_directory_exists(const char* dir_path) {
    struct stat st = {0};
    if (stat(dir_path, &st) == -1) {
        #ifdef _WIN32
            if (mkdir(dir_path) != 0) {
        #else
            if (mkdir(dir_path, 0755) != 0) {
        #endif
            if (errno != EEXIST) {
                perror("ERROR: Failed to create log directory");
                fprintf(stderr, "       Directory path: %s\n", dir_path);
                return -1;
            }
        }
        printf("Created log directory: %s\n", dir_path);
    } else if (!S_ISDIR(st.st_mode)) {
        fprintf(stderr, "ERROR: Log path exists but is not a directory: %s\n", dir_path);
        return -1;
    }
    return 0;
}

static void _rotate_logs_if_needed(void) {
    if (_log_file_handle == NULL || _config.max_file_size_kb == 0 || _config.max_backup_files == 0) {
        return; // Rotation not enabled or logger not initialized
    }

    // Check size (use config max_file_size_kb which is now in bytes)
    if (_current_file_size >= _config.max_file_size_kb) {
        printf("Log file size (%u bytes) exceeds limit (%u bytes). Rotating.\n",
               _current_file_size, _config.max_file_size_kb);
        _perform_log_rotation();
    }
}

// Performs the actual log rotation
static void _perform_log_rotation(void) {
    if (!_log_file_handle) return;

    // 1. Close the current log file
    fclose(_log_file_handle);
    _log_file_handle = NULL;

    char old_path[512];
    char new_path[512];

    // 2. Delete the oldest backup file if necessary
    snprintf(old_path, sizeof(old_path), "%s/%s.%d.bin",
             _config.base_log_directory, _config.base_filename_prefix, _config.max_backup_files);
    if (remove(old_path) == 0) {
        printf("Removed oldest backup log: %s\n", old_path);
    } else if (errno != ENOENT) { // ENOENT (No such file or directory) is expected if it's the first rotation
        perror("WARNING: Could not remove oldest backup log file");
        fprintf(stderr, "         File path: %s\n", old_path);
        // Continue anyway, maybe permissions issue or file is open elsewhere
    }

    // 3. Rename existing backup files (e.g., .2 -> .3, .1 -> .2)
    for (int i = _config.max_backup_files - 1; i >= 1; --i) {
        snprintf(old_path, sizeof(old_path), "%s/%s.%d.bin",
                 _config.base_log_directory, _config.base_filename_prefix, i);
        snprintf(new_path, sizeof(new_path), "%s/%s.%d.bin",
                 _config.base_log_directory, _config.base_filename_prefix, i + 1);

        // Check if old path exists before trying to rename
        struct stat st;
        if (stat(old_path, &st) == 0) {
             if (rename(old_path, new_path) != 0) {
                perror("WARNING: Failed to rename backup log file");
                fprintf(stderr, "         From: %s To: %s\n", old_path, new_path);
                // Log the error but continue if possible
             } else {
                 printf("Renamed %s to %s\n", old_path, new_path);
             }
        }
    }

    // 4. Rename the current log file to the first backup (.log -> .1.log)
    snprintf(new_path, sizeof(new_path), "%s/%s.1.bin",
             _config.base_log_directory, _config.base_filename_prefix);
    if (rename(_current_log_filepath, new_path) != 0) {
        perror("ERROR: Failed to rename current log file to backup");
        fprintf(stderr, "       From: %s To: %s\n", _current_log_filepath, new_path);
        // Attempt to reopen the original file if renaming failed? Or stop logging?
        // For now, we try to open a new file anyway.
    } else {
         printf("Renamed %s to %s\n", _current_log_filepath, new_path);
    }


    // 5. Reopen the primary log file (will be created new)
    _log_file_handle = fopen(_current_log_filepath, "ab+"); // Append Binary
    if (_log_file_handle == NULL) {
        perror("CRITICAL ERROR: Failed to reopen log file after rotation");
        // Logging stops here if this fails
        _current_file_size = 0;
        return;
    }

    _current_file_size = 0;
    printf("Opened new log file after rotation: %s\n", _current_log_filepath);

    // Optional: Write header to the new file
    if (_current_file_size == 0) {
        uint8_t record_type = RECORD_TYPE_HEADER;
        uint16_t version = 1;
        time_t now = time(NULL);
        fwrite(&record_type, sizeof(record_type), 1, _log_file_handle);
        fwrite(&version, sizeof(version), 1, _log_file_handle);
        fwrite(&now, sizeof(now), 1, _log_file_handle);
        fflush(_log_file_handle);
        _current_file_size = ftell(_log_file_handle);
        printf("Wrote header to new log file.\n");
    }

    file_logger_log_message("--- Log File Rotated ---");
}

/* No longer needed for binary data logging
static void _get_timestamp_str(char* buffer, size_t buffer_size) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts); // Use CLOCK_REALTIME for wall clock time

    time_t seconds = ts.tv_sec;
    long milliseconds = round(ts.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds

    // Handle potential millisecond rollover
    if (milliseconds >= 1000) {
        seconds++;
        milliseconds -= 1000;
    }

    struct tm timeinfo;
    localtime_r(&seconds, &timeinfo); // Thread-safe localtime

    // Format: YYYY-MM-DDTHH:MM:SS.mmmZ
    int offset = strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%S", &timeinfo);
    if (offset > 0 && (buffer_size - offset) > 5) { // Check space for ".mmmZ"
        snprintf(buffer + offset, buffer_size - offset, ".%03ldZ", milliseconds); // Assuming UTC, add 'Z'
    }
    // Fallback if formatting fails or buffer too small
    else {
        snprintf(buffer, buffer_size, "TIMESTAMP_ERROR");
    }
}
*/
