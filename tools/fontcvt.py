#!/usr/bin/env python3
import os, textwrap
from PIL import Image, ImageFont, ImageDraw

# --- Customize these ---
CHARS    = "0123456789./:°CADFLPTVadehilmorsty"
FONT_SIZE= 13
# Base project directory
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# Adjust this if your TTF name or location changed
TTF_PATH = os.path.join(BASE_DIR, "src", "fonts", "fzlthc.ttf")
OUT_C    = os.path.join(BASE_DIR, "src", "fonts", "fzlthc16_raw.c")
# Ensure the output directory exists
os.makedirs(os.path.dirname(OUT_C), exist_ok=True)
# -------------------------

font   = ImageFont.truetype(TTF_PATH, FONT_SIZE)
bitmaps= []
glyphs = []
offset = 0

def enhance_horizontal_lines(mask_data, width, height):
    """Enhance horizontal lines in the bitmap to ensure continuity."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 1 if mask_data[y * width + x] else 0
    
    # Enhance horizontal lines - fill gaps of length 1
    for y in range(height):
        for x in range(1, width-1):
            if mask_2d[y][x] == 0 and mask_2d[y][x-1] == 1 and mask_2d[y][x+1] == 1:
                mask_2d[y][x] = 1
    
    # Convert back to 1D array
    enhanced_data = []
    for y in range(height):
        for x in range(width):
            enhanced_data.append(mask_2d[y][x])
    
    return enhanced_data

def enhance_bitmap_comprehensive(mask_data, width, height):
    """Comprehensive bitmap enhancement to fix various rendering issues."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 1 if mask_data[y * width + x] else 0
    
    # 1. Fix horizontal discontinuities - fill gaps of length 1
    for y in range(height):
        for x in range(1, width-1):
            if mask_2d[y][x] == 0 and mask_2d[y][x-1] == 1 and mask_2d[y][x+1] == 1:
                mask_2d[y][x] = 1
    
    # 2. Fix vertical alternating pattern - ensure vertical consistency
    for x in range(width):
        for y in range(1, height-1):
            # If a pixel is surrounded by active pixels above and below, activate it
            if mask_2d[y][x] == 0 and mask_2d[y-1][x] == 1 and mask_2d[y+1][x] == 1:
                mask_2d[y][x] = 1
    
    # 3. Fix vertical strips between characters - clean up edges
    # First pass: detect vertical strips (isolated columns)
    for x in range(1, width-1):
        is_isolated = True
        for y in range(height):
            # If any pixel in this column has neighbors, it's not isolated
            if mask_2d[y][x] == 1 and (mask_2d[y][x-1] == 1 or mask_2d[y][x+1] == 1):
                is_isolated = False
                break
        
        # If this is an isolated vertical strip, remove it
        if is_isolated:
            for y in range(height):
                mask_2d[y][x] = 0
    
    # Convert back to 1D array
    enhanced_data = []
    for y in range(height):
        for x in range(width):
            enhanced_data.append(mask_2d[y][x])
    
    return enhanced_data

def enhance_a_character(mask_data, width, height):
    """Special enhancement for the 'A' character to fix the missing right foot."""
    # First apply the comprehensive enhancement
    enhanced_data = enhance_bitmap_comprehensive(mask_data, width, height)
    
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = enhanced_data[y * width + x]
    
    # For 'A', we'll directly modify the bitmap to ensure correct rendering
    
    # First, let's ensure the bitmap uses the full width
    # Find the leftmost and rightmost active pixels
    left_x = width
    right_x = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
    
    # If the character doesn't use the full width, expand it
    target_width = min(14, width)  # Target width of 14 pixels, but don't exceed the available width
    if right_x - left_x + 1 < target_width:
        # Calculate how much we need to expand
        expand_by = min(target_width - (right_x - left_x + 1), width - 1 - right_x)
        
        # Shift all pixels to the right by expand_by
        for y in range(height):
            for x in range(width - 1, left_x - 1, -1):
                if x - expand_by >= 0:
                    mask_2d[y][x] = mask_2d[y][x - expand_by]
                else:
                    mask_2d[y][x] = 0
        
        # Update right_x
        right_x += expand_by
    
    # Now fix the specific issues with 'A'
    
    # 1. Create a proper right foot (not just a dot)
    bottom_y = height - 1
    
    # Find the rightmost vertical stroke
    right_stroke_x = -1
    for x in range(width-1, -1, -1):
        vertical_count = 0
        for y in range(height//2, height-1):
            if mask_2d[y][x] == 1:
                vertical_count += 1
        
        if vertical_count >= 2:  # At least 2 pixels in a vertical line
            right_stroke_x = x
            break
    
    # If we found a right vertical stroke
    if right_stroke_x >= 0:
        # Create a proper foot by extending the stroke to the bottom
        for y in range(height-3, height):
            if y < height:
                mask_2d[y][right_stroke_x] = 1
        
        # Add supporting pixels to make it look like a proper foot
        # Extend the foot further to the right to avoid the "cut off" appearance
        if right_stroke_x < width - 1:
            mask_2d[bottom_y][right_stroke_x+1] = 1  # Extend foot to the right
            
            # Add one more pixel to the right for a more complete foot
            if right_stroke_x < width - 2:
                mask_2d[bottom_y][right_stroke_x+2] = 1
        
        if right_stroke_x > 0:
            mask_2d[bottom_y][right_stroke_x-1] = 1  # Add pixel to the left of foot
    else:
        # If we couldn't find a right stroke, use the rightmost pixel
        # and create a more substantial right foot
        mask_2d[bottom_y][right_x] = 1
        mask_2d[bottom_y-1][right_x] = 1
        
        # Extend the foot to the right
        if right_x < width - 1:
            mask_2d[bottom_y][right_x+1] = 1
            
            # Add one more pixel for a more complete foot
            if right_x < width - 2:
                mask_2d[bottom_y][right_x+2] = 1
        
        if right_x > 0:
            mask_2d[bottom_y][right_x-1] = 1
            
    # After creating the feet, ensure the right side of the character has a proper vertical stroke
    # This prevents the "cut off" appearance
    right_side_x = max(right_stroke_x, right_x)
    
    # Create a proper vertical stroke on the right side
    for y in range(height // 2, height - 1):
        mask_2d[y][right_side_x] = 1
        
        # Add supporting pixels to make the stroke look natural
        if right_side_x < width - 1:
            # Add some pixels to the right in the middle section
            if y > height // 2 + 1 and y < height - 2:
                mask_2d[y][right_side_x+1] = 1
    
    # 2. Ensure the horizontal middle bar is clear and spans the width
    # Position the bar slightly below the middle for better appearance
    middle_y = (height // 2) + 2  # Move bar down by 2 pixels from the middle
    
    # Find the leftmost and rightmost active pixels in the top half
    top_left_x = width
    top_right_x = -1
    
    for y in range(height // 2):
        for x in range(width):
            if mask_2d[y][x] == 1:
                top_left_x = min(top_left_x, x)
                top_right_x = max(top_right_x, x)
    
    # Create a solid horizontal bar in the middle
    for x in range(top_left_x, top_right_x + 1):
        mask_2d[middle_y][x] = 1
    
    # We want a thinner bar, so we'll remove the vertical connections
    # that were previously added to make it thicker
    
    # Ensure the bar connects to the vertical strokes
    # But only at the exact connection points, not above/below
    mask_2d[middle_y][top_left_x] = 1  # Connect to left stroke
    mask_2d[middle_y][top_right_x] = 1  # Connect to right stroke
    
    # 3. Ensure the character has a proper triangular shape
    # Make sure the top of the 'A' narrows appropriately
    top_y = 0
    while top_y < height // 3 and not any(mask_2d[top_y]):
        top_y += 1
    
    if top_y < height // 3:
        # Make the top narrower
        center_x = (top_left_x + top_right_x) // 2
        for x in range(width):
            if abs(x - center_x) > 1:  # Keep only the center 3 pixels
                mask_2d[top_y][x] = 0
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_l_character(mask_data, width, height):
    """Special enhancement for the 'L' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Ensure the vertical stroke is clear and consistent
    for y in range(top_y, bottom_y + 1):
        mask_2d[y][left_x] = 1  # Make sure the leftmost column is solid
    
    # 2. Ensure the horizontal foot is well-defined
    # Make the bottom row solid from left edge to right edge
    for x in range(left_x, right_x + 1):
        mask_2d[bottom_y][x] = 1
    
    # 3. Add some thickness to the horizontal foot for better visibility
    if bottom_y > 0:
        for x in range(left_x + 1, right_x + 1):  # Start from left_x + 1 to keep the corner sharp
            mask_2d[bottom_y - 1][x] = 1
    
    # 4. Ensure the character has proper proportions
    # Make sure the horizontal foot is at least 5 pixels wide
    target_width = min(8, max(5, right_x - left_x + 1))
    if right_x - left_x + 1 < target_width:
        # Extend the horizontal foot
        new_right_x = left_x + target_width - 1
        if new_right_x < width:
            for x in range(right_x + 1, new_right_x + 1):
                mask_2d[bottom_y][x] = 1
                if bottom_y > 0:
                    mask_2d[bottom_y - 1][x] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_t_character(mask_data, width, height):
    """Special enhancement for the 'T' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Ensure the horizontal top bar is clear and consistent
    # Find the center of the top bar
    center_x = (left_x + right_x) // 2
    
    # Make the top bar solid and extend it if needed
    target_width = min(8, max(6, right_x - left_x + 1))
    bar_left = max(0, center_x - target_width // 2)
    bar_right = min(width - 1, bar_left + target_width - 1)
    
    # Create a solid top bar
    for x in range(bar_left, bar_right + 1):
        mask_2d[top_y][x] = 1
        if top_y + 1 < height:  # Add thickness to the top bar
            mask_2d[top_y + 1][x] = 1
    
    # 2. Ensure the vertical stem is clear and consistent
    # Find the center of the vertical stem
    stem_x = center_x
    
    # Make the vertical stem solid
    for y in range(top_y, bottom_y + 1):
        mask_2d[y][stem_x] = 1
        
        # Add some thickness to the stem for better visibility
        if stem_x + 1 < width:
            mask_2d[y][stem_x + 1] = 1
    
    # 3. Ensure the character has proper proportions
    # Make sure the top bar is wider than the stem
    if bar_right - bar_left < 4:  # If top bar is too narrow
        new_bar_left = max(0, center_x - 2)
        new_bar_right = min(width - 1, center_x + 2)
        for x in range(new_bar_left, new_bar_right + 1):
            mask_2d[top_y][x] = 1
            if top_y + 1 < height:
                mask_2d[top_y + 1][x] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_v_character(mask_data, width, height):
    """Special enhancement for the 'V' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Ensure the 'V' has proper diagonal strokes
    # Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # Calculate the width and height of the character
    char_width = min(8, right_x - left_x + 1)
    char_height = bottom_y - top_y + 1
    
    # Create a new 'V' with clean diagonal strokes
    for y in range(top_y, bottom_y + 1):
        # Calculate the position along the diagonal for this y
        progress = (y - top_y) / max(1, char_height - 1)  # Avoid division by zero
        
        # Left diagonal stroke
        left_pos = int(left_x + progress * ((char_width - 1) // 2))
        if left_pos >= 0 and left_pos < width:
            mask_2d[y][left_pos] = 1
        
        # Right diagonal stroke
        right_pos = int(right_x - progress * ((char_width - 1) // 2))
        if right_pos >= 0 and right_pos < width:
            mask_2d[y][right_pos] = 1
    
    # 2. Ensure the bottom point is clear
    # Find the center at the bottom
    bottom_center = (left_x + right_x) // 2
    
    # Make sure the bottom point is a single pixel
    for x in range(width):
        if y == bottom_y:
            mask_2d[bottom_y][x] = 0
    
    # Set the bottom point
    if bottom_center >= 0 and bottom_center < width:
        mask_2d[bottom_y][bottom_center] = 1
    
    # 3. Add some thickness to the strokes for better visibility
    # Create a copy of the current state
    enhanced = [[mask_2d[y][x] for x in range(width)] for y in range(height)]
    
    # Add thickness by setting adjacent pixels
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                # Add pixels to the right for the left diagonal
                if x < bottom_center and x + 1 < width:
                    enhanced[y][x + 1] = 1
                # Add pixels to the left for the right diagonal
                if x > bottom_center and x - 1 >= 0:
                    enhanced[y][x - 1] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(enhanced[y][x])
    
    return final_data

def enhance_0_character(mask_data, width, height):
    """Special enhancement for the '0' (zero) character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # 2. Calculate the center and dimensions of the oval
    center_x = (left_x + right_x) // 2
    center_y = (top_y + bottom_y) // 2
    char_width = min(6, right_x - left_x + 1)
    char_height = bottom_y - top_y + 1
    
    # 3. Create a simple oval shape with equal thickness on both sides
    # Left and right sides
    for y in range(top_y + 1, bottom_y):
        # Left side
        mask_2d[y][left_x] = 1
        
        # Right side
        mask_2d[y][right_x] = 1
    
    # Top and bottom curves
    for x in range(left_x + 1, right_x):
        # Top curve
        mask_2d[top_y][x] = 1
        
        # Bottom curve
        mask_2d[bottom_y][x] = 1
    
    # 4. Round the corners to make it look more like a '0'
    # Top-left corner
    if top_y + 1 < height and left_x + 1 < width:
        mask_2d[top_y + 1][left_x + 1] = 1
        mask_2d[top_y][left_x] = 0
    
    # Top-right corner
    if top_y + 1 < height and right_x - 1 >= 0:
        mask_2d[top_y + 1][right_x - 1] = 1
        mask_2d[top_y][right_x] = 0
    
    # Bottom-left corner
    if bottom_y - 1 >= 0 and left_x + 1 < width:
        mask_2d[bottom_y - 1][left_x + 1] = 1
        mask_2d[bottom_y][left_x] = 0
    
    # Bottom-right corner
    if bottom_y - 1 >= 0 and right_x - 1 >= 0:
        mask_2d[bottom_y - 1][right_x - 1] = 1
        mask_2d[bottom_y][right_x] = 0
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_1_character(mask_data, width, height):
    """Special enhancement for the '1' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # 2. Calculate the center and dimensions
    center_x = (left_x + right_x) // 2
    char_width = min(4, right_x - left_x + 1)  # '1' should be narrower
    
    # 3. Create a clean vertical stroke
    for y in range(top_y, bottom_y + 1):
        mask_2d[y][center_x] = 1
    
    # 4. Add a small serif at the top (typical for '1')
    if top_y + 1 < height:
        mask_2d[top_y][center_x - 1] = 1
    
    # 5. Create a proper base (horizontal line at the bottom)
    base_width = 3
    base_left = center_x - base_width // 2
    base_right = base_left + base_width - 1
    
    for x in range(base_left, base_right + 1):
        if x >= 0 and x < width:
            mask_2d[bottom_y][x] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_3_character(mask_data, width, height):
    """Special enhancement for the '3' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # 2. Calculate the center and dimensions
    center_x = (left_x + right_x) // 2
    center_y = (top_y + bottom_y) // 2
    char_width = min(5, right_x - left_x + 1)
    char_height = bottom_y - top_y + 1
    
    # Calculate new left and right boundaries
    new_left_x = center_x - char_width // 2
    new_right_x = new_left_x + char_width - 1
    
    # Ensure boundaries are within the bitmap
    new_left_x = max(0, new_left_x)
    new_right_x = min(width - 1, new_right_x)
    
    # 3. Draw the top curve (1/3 of height)
    top_curve_height = char_height // 3
    
    # Top horizontal line (slightly shorter on the left for oval appearance)
    for x in range(new_left_x + 1, new_right_x):
        if x >= 0 and x < width:
            mask_2d[top_y][x] = 1
    
    # Top-right corner (oval)
    if top_y + 1 < height and new_right_x - 1 >= 0:
        mask_2d[top_y][new_right_x] = 0  # Remove corner for oval appearance
        mask_2d[top_y + 1][new_right_x] = 1  # Add pixel below
        mask_2d[top_y][new_right_x - 1] = 1  # Add pixel to the left
    
    # Right edge of top curve
    for y in range(top_y + 2, top_y + top_curve_height):
        if new_right_x >= 0 and new_right_x < width:
            mask_2d[y][new_right_x] = 1
    
    # 4. Draw the middle horizontal line (centered, shorter)
    middle_y = top_y + char_height // 2
    middle_width = char_width * 2 // 3
    middle_left = new_right_x - middle_width + 1
    
    for x in range(middle_left, new_right_x):
        if x >= 0 and x < width:
            mask_2d[middle_y][x] = 1
    
    # Middle-right corner (oval)
    if middle_y > 0 and middle_y < height - 1 and new_right_x - 1 >= 0:
        mask_2d[middle_y][new_right_x] = 0  # Remove corner for oval appearance
        mask_2d[middle_y - 1][new_right_x] = 1  # Add pixel above
        mask_2d[middle_y + 1][new_right_x] = 1  # Add pixel below
    
    # 5. Draw the bottom curve (1/3 of height)
    bottom_curve_height = char_height // 3
    bottom_curve_start = bottom_y - bottom_curve_height + 1
    
    # Right edge of bottom curve
    for y in range(bottom_curve_start, bottom_y - 1):
        if new_right_x >= 0 and new_right_x < width:
            mask_2d[y][new_right_x] = 1
    
    # Bottom-right corner (oval)
    if bottom_y - 1 >= 0 and new_right_x - 1 >= 0:
        mask_2d[bottom_y][new_right_x] = 0  # Remove corner for oval appearance
        mask_2d[bottom_y - 1][new_right_x] = 1  # Add pixel above
        mask_2d[bottom_y][new_right_x - 1] = 1  # Add pixel to the left
    
    # Bottom horizontal line
    for x in range(new_left_x + 1, new_right_x):
        if x >= 0 and x < width:
            mask_2d[bottom_y][x] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_7_character(mask_data, width, height):
    """Special enhancement for the '7' character to improve its appearance."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # 2. Calculate the center and dimensions
    center_x = (left_x + right_x) // 2
    center_y = (top_y + bottom_y) // 2
    char_width = min(6, right_x - left_x + 1)
    char_height = bottom_y - top_y + 1
    
    # Calculate new left and right boundaries
    new_left_x = center_x - char_width // 2
    new_right_x = new_left_x + char_width - 1
    
    # Ensure boundaries are within the bitmap
    new_left_x = max(0, new_left_x)
    new_right_x = min(width - 1, new_right_x)
    
    # 3. Draw the top horizontal line
    for x in range(new_left_x, new_right_x + 1):
        if x >= 0 and x < width:
            mask_2d[top_y][x] = 1
    
    # 4. Draw the diagonal line from top-right to bottom-left
    # Calculate the length of the diagonal
    diagonal_length = char_height
    
    # Draw the diagonal with a consistent slope
    for i in range(diagonal_length):
        # Calculate position along the diagonal
        progress = i / diagonal_length
        
        # Start from top-right and move towards bottom-left
        y = top_y + int(progress * char_height)
        x = new_right_x - int(progress * (char_width * 0.8))
        
        if 0 <= y < height and 0 <= x < width:
            mask_2d[y][x] = 1
            
            # Make the diagonal line thicker for better visibility
            if x - 1 >= 0:
                mask_2d[y][x - 1] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def enhance_degree_character(mask_data, width, height):
    """Special enhancement for the degree (°) character to fix the issue with multiple circles."""
    # Convert to 2D array for easier processing
    mask_2d = [[0 for x in range(width)] for y in range(height)]
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = mask_data[y * width + x]
    
    # Find the bounds of the character
    left_x = width
    right_x = -1
    top_y = height
    bottom_y = -1
    
    for y in range(height):
        for x in range(width):
            if mask_2d[y][x] == 1:
                left_x = min(left_x, x)
                right_x = max(right_x, x)
                top_y = min(top_y, y)
                bottom_y = max(bottom_y, y)
    
    # If we couldn't find any pixels, return the original data
    if left_x > right_x or top_y > bottom_y:
        return mask_data
    
    # 1. Clear the existing character
    for y in range(height):
        for x in range(width):
            mask_2d[y][x] = 0
    
    # 2. Calculate the center and dimensions
    center_x = (left_x + right_x) // 2
    
    # Make the degree symbol extremely small - just a single pixel
    # Position it at the top of the character space
    if top_y >= 0 and top_y < height and center_x >= 0 and center_x < width:
        mask_2d[top_y][center_x] = 1
    
    # Convert back to 1D array
    final_data = []
    for y in range(height):
        for x in range(width):
            final_data.append(mask_2d[y][x])
    
    return final_data

def visualize_bitmap(mask_data, width, height, title="Bitmap Visualization"):
    """Visualize a bitmap in ASCII format."""
    print(f"\n{title}:")
    print("+" + "-" * width + "+")
    for y in range(height):
        row = ""
        for x in range(width):
            if mask_data[y * width + x]:
                row += "#"  # Use # for pixels that are on
            else:
                row += " "  # Use space for pixels that are off
        print("|" + row + "|")
    print("+" + "-" * width + "+")

for ch in CHARS:
    # Render glyph to mono mask
    mask = font.getmask(ch, mode="1")
    w,h  = mask.size
    try:
        data = mask.tobytes()  # 0/255 per pixel
    except AttributeError:
        data = bytes(mask)
    
    # Convert to 0/1 row‐major
    row = [1 if b else 0 for b in data]
    
    # Apply appropriate enhancement based on the character
    if ch == 'A':
        row = enhance_a_character(row, w, h)
    elif ch == 'L':
        row = enhance_l_character(row, w, h)
    elif ch == 'T':
        row = enhance_t_character(row, w, h)
    elif ch == 'V':
        row = enhance_v_character(row, w, h)
    elif ch == '0':
        row = enhance_0_character(row, w, h)
    elif ch == '1':
        row = enhance_1_character(row, w, h)
    elif ch == '3':
        row = enhance_3_character(row, w, h)
    elif ch == '7':
        row = enhance_7_character(row, w, h)
    elif ch == '°':
        row = enhance_degree_character(row, w, h)
    else:
        row = enhance_bitmap_comprehensive(row, w, h)
    
    # Apply vertical offset to characters
    if ch == '°':  # Special case for degree symbol - move up by 1 pixel compared to others
        y_offset = 1  # Shift degree symbol down by only 1 pixel (instead of 2)
    else:
        y_offset = 2  # Shift all other characters down by 2 pixels
        
    glyphs.append(dict(w=w, h=h, xOff=0, yOff=y_offset, xAdv=w, off=offset))
    
    bitmaps.extend(row)
    offset += w*h

with open(OUT_C, "w") as f:
    f.write('#include <stdint.h>\n#include <stddef.h>\n#include "fonts/font_glyph.h"\n#include "fonts/fzlthc16.h"\n\n')
    # bitmap
    f.write("const uint8_t FZLTHC16_bitmap[] = {\n")
    for i in range(0, len(bitmaps), 16):
        line = ", ".join(f"0x{v:02X}" for v in bitmaps[i:i+16])
        f.write(f"  {line},\n")
    f.write("};\n\n")
    # glyphs
    f.write("const FontGlyph_t FZLTHC16_glyphs[FZLTHC16_GLYPH_COUNT] = {\n")
    for g in glyphs:
        f.write(f"  {{ .width={g['w']}, .height={g['h']}, .xOffset={g['xOff']}, .yOffset={g['yOff']}, .xAdvance={g['xAdv']}, .bitmap=&FZLTHC16_bitmap[{g['off']}] }},\n")
    f.write("};\n\n")
    # char map with escaped non-ASCII
    f.write("\nconst char FZLTHC16_map[FZLTHC16_GLYPH_COUNT] = { ")
    literals = []
    for c in CHARS:
        code = ord(c)
        if code < 32 or code > 126:
            literals.append(f"'\\x{code:02X}'")
        else:
            literals.append(f"'{c}'")
    f.write(", ".join(literals))
    f.write(" };\n")
    # get glyph
    f.write("\nconst FontGlyph_t* FZLTHC_GetGlyph(char c) {\n")
    f.write("    for (size_t i = 0; i < FZLTHC16_GLYPH_COUNT; ++i) {\n")
    f.write("        if (FZLTHC16_map[i] == c) {\n")
    f.write("            return &FZLTHC16_glyphs[i];\n")
    f.write("        }\n")
    f.write("    }\n")
    f.write("    return &FZLTHC16_glyphs[0];\n")
    f.write("}\n")