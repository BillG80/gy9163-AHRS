#include "drivers/ssd1306/ssd1306.h"
#include "drivers/ssd1306/ssd1306_graphics.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include "fonts/SansCondensed.h"  // use SansCondensed font instead of fzlthc16

// Default space width
#define SANS_SPACE_WIDTH 4
// Extra spacing between characters
#define CHAR_EXTRA_SPACING 4  // Increase spacing between characters
// Maximum size of font data (safe upper bound)
#define MAX_FONT_DATA_SIZE 10000

// Render width policy types
#define RENDER_POLICY_NORMAL     0  // render_width = x_advance
#define RENDER_POLICY_REDUCED_1  1  // render_width = x_advance - 1
#define RENDER_POLICY_REDUCED_2  2  // render_width = x_advance - 2
#define RENDER_POLICY_SPECIAL    3  // render_width = x_advance + 7

// Get render policy for a specific character
uint8_t get_render_policy(char c) {
    // Characters that need special handling (expanded)
    if (c == 'V') {
        return RENDER_POLICY_NORMAL;
    }
    // Characters that need x_advance-2 (more restrictive)
    else if ( c == ':') {
        return RENDER_POLICY_REDUCED_2;
    }
    // Characters that need x_advance-1 (standard reduction)
    else if (c >= '0' && c <= '9') {
        return RENDER_POLICY_REDUCED_1;
    }
    else if (c == 'C' || c == 'E' || c == 'G' || c == 'H' || c == 'I' || c == 'J' || 
             c == 'M' || c == 'N' || c == 'O' || c == 'P' || c == 'Q' || c == 'R' || 
             c == 'S' || c == 'U' || c == 'W' || c == 'Y' || c == 'e' ||
             c == 'a' || c == 'b' || c == 'c' || c == 'd' || c == 'g' || c == 'h' || 
             c == 'i' || c == 'j' || c == 'l' || c == 'm' || c == 'n' || c == 'o' || 
             c == 'p' || c == 'q' || c == 's' || c == 'u' || c == 'w' || c == 'z' || 
             c == 176) {
        return RENDER_POLICY_REDUCED_1;
    }
    // All other characters use normal rendering (no reduction)
    else {
        return RENDER_POLICY_NORMAL;
    }
}

// Get effective render width based on policy and x_advance
int get_render_width(uint8_t policy, uint8_t x_advance) {
    switch (policy) {
        case RENDER_POLICY_NORMAL:
            return x_advance;
        case RENDER_POLICY_REDUCED_1:
            return x_advance - 1;
        case RENDER_POLICY_REDUCED_2:
            return x_advance - 2;
        case RENDER_POLICY_SPECIAL:
            return x_advance + 7;
        default:
            return x_advance;  // Default to reduced-1 if unknown policy
    }
}

// Compute pixel width of a text string at a given scale
int ssd1306_get_text_width(const char* text, uint8_t scale) {
    int width = 0;
    
    // Iterate through each character in the text using index-based approach
    for (int i = 0; text[i] != '\0' && text[i] != '\n'; i++) {
        char c = text[i];
        
        // Check if character is in range
        if (c >= 32 && c < 32 + 224) {
            // Get character index
            int char_idx = c - 32;
            
            // Get character data from jump table
            int offset_idx = char_idx * 4 + 4;  // Skip header (4 bytes)
            
            // Get x_advance from jump table (4th byte in the entry)
            uint8_t x_advance = DejaVu_Sans_Condensed_12[offset_idx + 3];
            
            // Get the render policy for this character
            uint8_t policy = get_render_policy(c);
            
            // Calculate the effective render width based on policy
            int render_width = get_render_width(policy, x_advance);
            
            // Add this character's advance width to the total
            // Adjust for the reduced spacing
            width += render_width * scale;  
        } else {
            // Space or unsupported character
            width += SANS_SPACE_WIDTH * scale;
        }
    }
    
    // Add back the 1 pixel we subtracted from the last character
    // since it doesn't need the reduced spacing
    if (width > 0) {
        width += scale;
    }
    
    return width;
}

// Draw text using SansCondensed font
void ssd1306_draw_text(ssd1306_handle_t* d, int x, int y, const char* text, uint8_t color) {
    // Get font height from header
    uint8_t font_height = DejaVu_Sans_Condensed_12[1];
    int current_x = x;  // Track current x position
    
    // Iterate through each character in the text
    for (int i = 0; text[i] != '\0'; i++) {
        char c = text[i];
        
        // Check if character is in range
        if (c >= 32 && c < 32 + 224) {
            // Get character index
            int char_idx = c - 32;
            
            // Get character data from jump table
            int offset_idx = char_idx * 4 + 4;  // Skip header (4 bytes)
            
            // Extract data from jump table (each entry is 4 bytes)
            uint16_t data_offset = (DejaVu_Sans_Condensed_12[offset_idx] << 8) | DejaVu_Sans_Condensed_12[offset_idx + 1];
            uint8_t width = DejaVu_Sans_Condensed_12[offset_idx + 2];
            uint8_t x_advance = DejaVu_Sans_Condensed_12[offset_idx + 3];
            
            // Skip if character has no data
            if (data_offset == 0xFFFF) {
                current_x += SANS_SPACE_WIDTH;
                continue;
            }
            
            // Calculate actual data offset (jump table size + header)
            int data_start = 4 + (224 * 4) + data_offset;
            
            // Clear the area where we'll draw this character to prevent ghosting
            // Extend the clearing area to prevent any potential overflow
            for (int clear_x = current_x; clear_x < current_x + width + 2; clear_x++) {
                for (int clear_y = y; clear_y < y + font_height; clear_y++) {
                    if (clear_x >= 0 && clear_x < 128 && clear_y >= 0 && clear_y < 64) {
                        ssd1306_draw_pixel(d, clear_x, clear_y, 0);
                    }
                }
            }
            
            // In Horizontal Addressing Mode, we need to draw character column by column
            int pages = (font_height + 7) / 8;
            
            // Get the render policy for this character
            uint8_t policy = get_render_policy(c);
            
            // Calculate the effective render width based on policy
            int render_width = get_render_width(policy, x_advance);
            
            // Draw the character bitmap
            for (int col = 0; col < render_width; col++) {
                // For each page in the character
                for (int page = 0; page < pages; page++) {
                    // Get the byte for this column and page
                    int byte_index = data_start + (col * pages) + page;
                    
                    // Check if we're within bounds of the font data array
                    if (byte_index >= 0 && byte_index < MAX_FONT_DATA_SIZE) {
                        uint8_t pixel_byte = DejaVu_Sans_Condensed_12[byte_index];
                        
                        // Special handling for 'v' character to remove the extra pixels
                        if (c == 'v') {
                            // Clear column 5, page 1 (bottom section)
                            if (col == 5 && page == 1) {
                                pixel_byte = 0x00; // Clear all pixels in this byte
                            }
                            // Also clear any pixels in the bottom row at the last column
                            if (page == pages - 1 && (col == render_width-1)) {
                                pixel_byte = 0x00;
                            }
                        }
                        
                        // Special handling for 'V' character to remove the extra pixels
                        if (c == 'V') {
                            // Clear all pixels in the bottom row (last page) for columns 12 and 13
                            if (col == 6 && page == 1) {
                                pixel_byte = 0x00; // Clear all pixels in this byte
                            }
                        }
                        
                        // Skip drawing if column is beyond the render width
                        if (col >= render_width) {
                            continue;
                        }
                        
                        // Process each bit in the byte (8 vertical pixels)
                        for (int bit = 0; bit < 8; bit++) {
                            // Only process bits that are within the character height
                            if (page * 8 + bit < font_height) {
                                // If this bit is set
                                if (pixel_byte & (1 << bit)) {
                                    // Calculate the x and y coordinates
                                    int x_pos = current_x + col;
                                    int y_pos = y + page * 8 + bit;
                                    
                                    // ADDITIONAL ROW FIX: Skip drawing pixels in the bottom row for certain characters
                                    // This prevents unwanted dots and artifacts in the bottom row
                                    if (page * 8 + bit == font_height - 1) {
                                        // Characters that often have unwanted pixels in the bottom row
                                        if (c == '7' || c == ':' || c == 'j' || c == 'g' || c == 'p' || c == 'q' || c == 'y') {
                                            // Skip drawing these pixels in the last row
                                            continue;
                                        }
                                    }                                    
                                    // Draw the pixel
                                    if (x_pos >= 0 && x_pos < 128 && y_pos >= 0 && y_pos < 64) {
                                        ssd1306_draw_pixel(d, x_pos, y_pos, color);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            // Advance cursor to the next character position
            // Make sure we have a clean break between characters
            current_x += x_advance;
            
            // If this is the last character in the string or the next character is a space,
            // add extra pixels of clearing to prevent ghosting of the next character
            if (text[i+1] == '\0' || text[i+1] == ' ') {
                // Clear several pixels to the right of the character
                // Use an even wider clearing area for the last character
                for (int clear_x = current_x; clear_x < current_x + 8; clear_x++) {
                    for (int clear_y = y - 2; clear_y < y + font_height + 2; clear_y++) {
                        if (clear_x >= 0 && clear_x < 128 && clear_y >= 0 && clear_y < 64) {
                            ssd1306_draw_pixel(d, clear_x, clear_y, 0);
                        }
                    }
                }
                
                // Also clear a few pixels to the left of where the next character would be
                // This helps prevent any partial rendering of the next character
                for (int clear_x = current_x + x_advance - 2; clear_x < current_x + x_advance + 2; clear_x++) {
                    for (int clear_y = y - 2; clear_y < y + font_height + 2; clear_y++) {
                        if (clear_x >= 0 && clear_x < 128 && clear_y >= 0 && clear_y < 64) {
                            ssd1306_draw_pixel(d, clear_x, clear_y, 0);
                        }
                    }
                }
            }
        } else {
            // Unsupported character
            current_x += SANS_SPACE_WIDTH;
        }
    }
}

// Draw a line (Bresenham's algorithm)
void ssd1306_draw_line(ssd1306_handle_t* handle, int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (1) {
        ssd1306_draw_pixel(handle, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Draw rectangle (outline)
void ssd1306_draw_rect(ssd1306_handle_t* handle, int x, int y, int w, int h, uint8_t color) {
    ssd1306_draw_line(handle, x, y, x + w - 1, y, color);
    ssd1306_draw_line(handle, x, y + h - 1, x + w - 1, y + h - 1, color);
    ssd1306_draw_line(handle, x, y, x, y + h - 1, color);
    ssd1306_draw_line(handle, x + w - 1, y, x + w - 1, y + h - 1, color);
}

// Draw filled rectangle
void ssd1306_fill_rect(ssd1306_handle_t* handle, int x, int y, int w, int h, uint8_t color) {
    for (int i = 0; i < h; i++)
        ssd1306_draw_line(handle, x, y + i, x + w - 1, y + i, color);
}

// Draw circle (outline, midpoint algorithm)
void ssd1306_draw_circle(ssd1306_handle_t* handle, int x0, int y0, int r, uint8_t color) {
    int x = r, y = 0, err = 0;
    while (x >= y) {
        ssd1306_draw_pixel(handle, x0 + x, y0 + y, color);
        ssd1306_draw_pixel(handle, x0 + y, y0 + x, color);
        ssd1306_draw_pixel(handle, x0 - y, y0 + x, color);
        ssd1306_draw_pixel(handle, x0 - x, y0 + y, color);
        ssd1306_draw_pixel(handle, x0 - x, y0 - y, color);
        ssd1306_draw_pixel(handle, x0 - y, y0 - x, color);
        ssd1306_draw_pixel(handle, x0 + y, y0 - x, color);
        ssd1306_draw_pixel(handle, x0 + x, y0 - y, color);
        y++;
        if (err <= 0) { err += 2 * y + 1; }
        if (err > 0) { x--; err -= 2 * x + 1; }
    }
}

// Draw filled circle
void ssd1306_fill_circle(ssd1306_handle_t* handle, int x0, int y0, int r, uint8_t color) {
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++)
            if (x * x + y * y <= r * r)
                ssd1306_draw_pixel(handle, x0 + x, y0 + y, color);
}

// Draw a monochrome bitmap (1 bit per pixel, row-major)
void ssd1306_draw_bitmap(ssd1306_handle_t* handle, int x, int y, const uint8_t* bitmap, int w, int h) {
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int byte_idx = (j * w + i) / 8;
            int bit_idx = 7 - ((j * w + i) % 8);
            uint8_t bit = (bitmap[byte_idx] >> bit_idx) & 0x01;
            ssd1306_draw_pixel(handle, x + i, y + j, bit);
        }
    }
}