#ifndef SSD1306_GRAPHICS_H
#define SSD1306_GRAPHICS_H

#include "drivers/ssd1306/ssd1306.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Draw text
void ssd1306_draw_text(ssd1306_handle_t* handle, int x, int y, const char* text, uint8_t color);

// Draw a line (Bresenham's algorithm)
void ssd1306_draw_line(ssd1306_handle_t* handle, int x0, int y0, int x1, int y1, uint8_t color);

// Draw a rectangle (outline)
void ssd1306_draw_rect(ssd1306_handle_t* handle, int x, int y, int w, int h, uint8_t color);

// Draw a filled rectangle
void ssd1306_fill_rect(ssd1306_handle_t* handle, int x, int y, int w, int h, uint8_t color);

// Draw a circle (outline)
void ssd1306_draw_circle(ssd1306_handle_t* handle, int x0, int y0, int r, uint8_t color);

// Draw a filled circle
void ssd1306_fill_circle(ssd1306_handle_t* handle, int x0, int y0, int r, uint8_t color);

// Draw a monochrome bitmap (1 bit per pixel, row-major)
void ssd1306_draw_bitmap(ssd1306_handle_t* handle, int x, int y, const uint8_t* bitmap, int w, int h);

// Compute pixel width of a text string at the given scale
int ssd1306_get_text_width(const char* text, uint8_t scale);

#ifdef __cplusplus
}
#endif

#endif // SSD1306_GRAPHICS_H