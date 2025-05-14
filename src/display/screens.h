#ifndef SCREENS_H
#define SCREENS_H

#include "drivers/ssd1306/ssd1306.h"
#include <stdbool.h>

// Existing declarations ...

void draw_calib_confirm_screen(
    ssd1306_handle_t* disp,
    int countdown_s
);

#endif // SCREENS_H
