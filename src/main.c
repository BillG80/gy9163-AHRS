#include <bcm2835.h>
#include <stdio.h>
#include <string.h>
#include "common/common_defs.h"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <stdint.h>  // For uint64_t
#include <getopt.h>
#include "core/fusion/calibration.h"

#include "drivers/bus/i2c_bus.h"
#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include "display/screens.h"
#include "display/display_manager.h"
#include "common/common_defs.h"
#include "core/fusion/sensor_fusion.h"
#include "core/navigation/elevator_nav.h"
#include "logging/file_logger.h"
#include "display/rmr_logo.h"

// Expose sensor fusion interface setup
extern int setup_i2c(void);
extern int setup_spi(void);

// Global variables for clean shutdown
static volatile int keep_running = 1;
static ssd1306_handle_t* disp = NULL;
static i2c_bus_handle_t display_bus = NULL;  // I2C bus 0 for display
static const char* APP_VERSION = __DATE__ " " __TIME__;
// Signal handler for clean shutdown
void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("\nReceived SIGINT, shutting down...\n");
        keep_running = 0;
    }
}

// Function to clean up resources
void cleanup(void) {
    if (disp) {
        ssd1306_cleanup(disp);
        disp = NULL;
    }
    
    if (display_bus) {
        i2c_bus_cleanup(display_bus);
        display_bus = NULL;
    }
    
    // Clean up hardware resources
    cleanup_hw();
    
    // Clean up file logger
    file_logger_cleanup();
    
    bcm2835_close();
    printf("Cleanup complete\n");
}

system_state_t system_state = SYSTEM_STATE_LOGO;

int main(int argc, char *argv[]) {
    ElevatorNavMode_T nav_mode = ELEVATOR_NAV_MODE_ELEVATOR; // Default mode
    // Parse CLI for 'general' mode
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "general") == 0 || strcmp(argv[i], "--mode=general") == 0) {
            nav_mode = ELEVATOR_NAV_MODE_GENERAL;
            break;
        }
    }
    // Parse CLI options for calibration
    int do_calib = 0;
    const char *calib_file = "calibration.json";
    struct option long_opts[] = {
        {"calibrate", required_argument, NULL, 'C'},
        {"help",      no_argument,       NULL, 'h'},
        {0,0,0,0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "C:h", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'C': do_calib = 1; calib_file = optarg; break;
            case 'h':
            default:
                printf("Usage: %s [--calibrate <file>] [normal options]\n", argv[0]);
                return (opt=='h') ? 0 : 1;
        }
    }

    printf("Starting elevator_app %s\n", APP_VERSION);
    // Register signal handler for clean shutdown
    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        fprintf(stderr, "Failed to register signal handler\n");
        return 1;
    }
    
    // Initialize BCM2835 library
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835_init failed\n");
        return 1;
    }
    
    // Initialize I2C bus 0 for SSD1306 display at 400 kHz
    i2c_config_t display_cfg = { .bus_num = 0, .speed = I2C_SPEED_400KHZ };
    if (i2c_bus_init(&display_cfg, &display_bus) != STATUS_OK) {
        fprintf(stderr, "Display I2C bus init failed\n");
            bcm2835_close();
            return 1;
        }
    
    // Initialize display on I2C bus 0
    if (ssd1306_init(display_bus, 0x3C, 128, 64, &disp) != STATUS_OK) {
        fprintf(stderr, "SSD1306 init failed\n");
        i2c_bus_cleanup(display_bus);
        bcm2835_close();
        return 1;
    }
    // Boot-logo progressive reveal
 
    ssd1306_clear_buffer(disp);
    ssd1306_draw_bitmap(disp,
                        (SSD1306_WIDTH  - logo_width)/2,
                        (SSD1306_HEIGHT - logo_height)/2,
                        logo_bitmap,
                        logo_width,
                        logo_height);
    ssd1306_update(disp);  // animates via chunked I2C writes
    sleep(3);
    // Clear logo immediately after display
    ssd1306_clear_buffer(disp);
    ssd1306_update(disp);

    // --- GPIO5 boot-hold calibration confirmation ---
    #define CALIB_CONFIRM_TIMEOUT_S 10
    bcm2835_gpio_fsel(CALIB_BUTTON_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(CALIB_BUTTON_PIN, BCM2835_GPIO_PUD_DOWN);
    bcm2835_delay(50);
    int button_initial = bcm2835_gpio_lev(CALIB_BUTTON_PIN);
    int do_calib_confirm = 0;
    if (button_initial) {
        // Show confirm page and countdown
        int last_level = 1, rising_edge = 0;
        for (int t = CALIB_CONFIRM_TIMEOUT_S; t > 0; --t) {
            draw_calib_confirm_screen(disp, t);
            for (int i = 0; i < 10; ++i) { // 100ms x10 = 1s
                int level = bcm2835_gpio_lev(CALIB_BUTTON_PIN);
                if (!last_level && level) rising_edge = 1;
                last_level = level;
                if (rising_edge) break;
                bcm2835_delay(100);
            }
            if (rising_edge) break;
        }
        if (rising_edge) do_calib_confirm = 1;
    }
    // If not calibrating, no further prompts: skip straight to AHRS/normal UI

    // --- Setup sensor interfaces as in test_sensor_fusion ---
    if (setup_i2c() != 0) {
        fprintf(stderr, "Sensor I2C bus init failed\n");
        cleanup();
        return 1;
    }
    if (setup_spi() != 0) {
        fprintf(stderr, "Sensor SPI bus init failed\n");
        cleanup();
        return 1;
    }

    // Initialize sensors and AHRS using I2C bus 1
    if (initialize_sensors(disp) != 0) {
        fprintf(stderr, "Sensor initialization failed\n");
        cleanup();
        return 1;
    }

    // Calibration or load calibration before navigation/AHRS init
    calib_params_t params = {0};
    int calib_mode = do_calib || do_calib_confirm;
    if (calib_mode) {
        system_state = SYSTEM_STATE_CALIBRATION;
        if (run_full_calibration(disp, &params) != 0) {
            fprintf(stderr, "Calibration failed\n"); cleanup(); return 1;
        }
        system_state = SYSTEM_STATE_AHRS;
        if (save_calibration(calib_file, &params) != 0) {
            fprintf(stderr, "Failed to save calibration to %s\n", calib_file);
            cleanup(); return 1;
        }
        printf("Calibration saved to %s\n", calib_file);
        // After calibration, load the calibration file and continue to AHRS/navigation
        if (load_calibration(calib_file, &params) != 0) {
            fprintf(stderr, "Failed to load calibration after saving.\n");
            cleanup();
            return 1;
        }
    } else {
        if (load_calibration(calib_file, &params) != 0) {
            // Calibration file missing or invalid - prompt and start calibration immediately
            draw_calib_confirm_screen(disp, 0); // Show a message (countdown=0 means immediate)
            bcm2835_delay(1000); // Briefly show the message
            calib_params_t params_local = {0};
            system_state = SYSTEM_STATE_CALIBRATION;
calib_restart:
            if (run_full_calibration(disp, &params_local) != 0) {
                fprintf(stderr, "Calibration failed\n"); cleanup(); return 1;
            }
            system_state = SYSTEM_STATE_AHRS;
            if (save_calibration(calib_file, &params_local) != 0) {
                fprintf(stderr, "Failed to save calibration to %s\n", calib_file);
                cleanup(); return 1;
            }
            printf("Calibration saved to %s\n", calib_file);
            if (load_calibration(calib_file, &params_local) != 0) {
                fprintf(stderr, "Failed to load calibration after saving.\n");
                cleanup();
                return 1;
            }
            // At end of calibration, prompt user to confirm or restart
            int countdown = 10;
            int restart = 0;
            for (int t = countdown; t > 0; --t) {
                draw_calib_confirm_screen(disp, t);
                for (int i = 0; i < 10; ++i) { // 100ms x10 = 1s
                    if (bcm2835_gpio_lev(CALIB_BUTTON_PIN)) {
                        restart = 1;
                        break;
                    }
                    bcm2835_delay(100);
                }
                if (restart) break;
            }
            if (restart) {
                goto calib_restart;
            }
            // else, proceed to AHRS/navigation
        }
    }

    // Now initialize elevator navigation (and AHRS if needed)
    if (elevator_nav_init(nav_mode) != 0) {
        fprintf(stderr, "Elevator navigation initialization failed\n");
        cleanup();
        return 1;
    }
    
    // Boot complete: start display updates
    display_manager_init(disp);
    
    // Initialize file logger
    FileLoggerConfig_T logger_config = {
        .base_log_directory = "./logs",
        .base_filename_prefix = "elevator_data",
        .max_file_size_kb = 1024,     // 1MB max file size
        .max_backup_files = 5          // Keep 5 backup files
    };
    
    if (file_logger_init(&logger_config) != 0) {
        fprintf(stderr, "File logger initialization failed\n");
        // Continue execution - logging is non-critical
        file_logger_log_message("WARNING: File logger initialization failed, continuing without logging");
    } else {
        file_logger_log_message("Elevator motion detection system started");
    }
    
    printf("System initialized, starting main loop...\n");
    
    // Data structures for sensor and navigation data
    OUTPUT_DATA_T output_data;
    uint32_t anim_time_ms = 0;
    struct timespec start_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);
    uint64_t last_disp_us = 0;
    const uint64_t DISPLAY_PERIOD_US = 100000; // 10 Hz update

    // If calibration mode, run full routine and save
    if (do_calib) {
        calib_params_t params = {0};
        system_state = SYSTEM_STATE_CALIBRATION;
        if (run_full_calibration(disp, &params) != 0) {
            fprintf(stderr, "Calibration failed\n"); cleanup(); return 1;
        }
        system_state = SYSTEM_STATE_AHRS;
        if (save_calibration(calib_file, &params) != 0) {
            fprintf(stderr, "Failed to save calibration to %s\n", calib_file);
            cleanup(); return 1;
        }
        printf("Calibration saved to %s\n", calib_file);
        cleanup(); return 0;
    }

    // Main loop
    while (keep_running) {
        // Update animation time with high-resolution timer
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        anim_time_ms = (now_ts.tv_sec - start_ts.tv_sec) * 1000 +
                       (now_ts.tv_nsec - start_ts.tv_nsec) / 1000000;
        
        // Run sensor fusion cycle to get latest data
        if (!run_sensor_fusion_cycle(&output_data.fusion_data)) {
            fprintf(stderr, "Sensor fusion cycle failed\n");
            // Continue anyway, we might recover in the next cycle
        }
        
        // Update elevator navigation data with the latest fusion data
        if (elevator_nav_update(&output_data.fusion_data, &output_data.nav_data) != 0) {
            fprintf(stderr, "Elevator navigation update failed\n");
            // Continue anyway, we might recover in the next cycle
        }
        
        // Log and update display at 10 Hz
        struct timespec ts_disp;
        clock_gettime(CLOCK_MONOTONIC, &ts_disp);
        uint64_t now_disp_us = (uint64_t)ts_disp.tv_sec * 1000000ULL + ts_disp.tv_nsec / 1000ULL;
        if (!last_disp_us || now_disp_us - last_disp_us >= DISPLAY_PERIOD_US) {
            file_logger_log_data(&output_data.fusion_data, &output_data.nav_data);
            display_manager_update(
                anim_time_ms,
                output_data.fusion_data.pressure_pa / 100.0f,
                output_data.fusion_data.temperature_c,
                output_data.fusion_data.altitude_m,
                output_data.nav_data.vertical_velocity_m_s,
                output_data.nav_data.vertical_distance_m,
                output_data.nav_data.vertical_velocity_m_s,
                output_data.nav_data.vertical_accel_m_s2,
                output_data.nav_data.is_stationary
            );
            printf("Distance: %.2fm, Velocity: %.2fm/s, Acceleration: %.2fm/s^2\n",
                   output_data.nav_data.vertical_distance_m,
                   output_data.nav_data.vertical_velocity_m_s,
                   output_data.nav_data.vertical_accel_m_s2);
            last_disp_us = now_disp_us;
        }
        
        static StatusEnum _current_status = STATUS_RUNNING;
        FILE *f = fopen("/run/elevator_status", "w");
        if (f) {
          fprintf(f, "%d\n", _current_status);
          fclose(f);
        }
        
        // Yield briefly for sensor ODR
        usleep(25000);  // 25ms pause
    }
    
    // Clean up resources
    display_manager_shutdown();
    cleanup();
    
    printf("Program terminated successfully\n");
    return 0;
}