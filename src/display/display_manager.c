#include "display/display_manager.h"
#include "display/screens.h"  // Your screen drawing functions
#include "drivers/ssd1306/ssd1306.h"
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>

static display_screen_t current_screen = SCREEN_MAIN;
static ssd1306_handle_t* display = NULL;

static pthread_t disp_thread;
static volatile bool disp_thread_running = false;

static void* display_thread_fn(void* arg) {
    display = (ssd1306_handle_t*)arg;
    disp_thread_running = true;
    struct timespec fps_last_ts;
    clock_gettime(CLOCK_MONOTONIC, &fps_last_ts);
    uint32_t fps_count = 0;
    while (disp_thread_running) {
        // count frames
        ssd1306_update(display);
        fps_count++;
        struct timespec fps_now;
        clock_gettime(CLOCK_MONOTONIC, &fps_now);
        if (fps_now.tv_sec - fps_last_ts.tv_sec >= 1) {
            printf("Display FPS: %u\n", fps_count);
            fps_count = 0;
            fps_last_ts = fps_now;
        }
        usleep(40000); // ~25 Hz
    }
    return NULL;
}

void display_manager_init(ssd1306_handle_t* disp) {
    display = disp;
    pthread_create(&disp_thread, NULL, display_thread_fn, disp);
    pthread_detach(disp_thread);
}

void display_manager_shutdown(void) {
    disp_thread_running = false;
}

void display_manager_set_screen(display_screen_t screen) {
    current_screen = screen;
    // Optionally clear display or reset animation timers
}

display_screen_t display_manager_get_screen(void) {
    return current_screen;
}

void display_manager_update(uint32_t ms_since_boot, float pressure, float temperature,
                             float altitude, float speed, float distance,
                             float vvel, float vacc, bool is_stationary) {
    if (!display) return;
    switch (current_screen) {
        case SCREEN_MAIN:
            draw_main_screen(display, ms_since_boot, pressure, temperature,
                             altitude, speed, distance, vvel, vacc, is_stationary);
            break;
        case SCREEN_SETTINGS:
            draw_settings_screen(display, ms_since_boot);
            break;
        case SCREEN_ERROR:
            draw_error_screen(display, ms_since_boot);
            break;
        // Add more cases as needed
    }
}