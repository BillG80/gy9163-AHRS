#!/usr/bin/env python3
"""
Font Bitmap Viewer for SSD1306 Display
This tool visualizes the bitmap data of font characters using ASCII art to help identify and fix rendering issues.
"""

import sys
import os
import argparse
import traceback

# Font header structure
# Byte 0: Width
# Byte 1: Height
# Byte 2: First Char
# Byte 3: Number of Chars
# Then follows the jump table (4 bytes per character)
# Then follows the bitmap data

def parse_font_header(font_data):
    """Parse the font header information."""
    print(f"DEBUG: Font data length: {len(font_data)}")
    print(f"DEBUG: First few bytes: {font_data[:10]}")
    width = font_data[0]
    height = font_data[1]
    first_char = font_data[2]
    num_chars = font_data[3]
    print(f"DEBUG: Parsed header - Width: {width}, Height: {height}, First Char: {first_char}, Num Chars: {num_chars}")
    return width, height, first_char, num_chars

def extract_character_data(font_data, char_code):
    """Extract the bitmap data for a specific character."""
    width, height, first_char, num_chars = parse_font_header(font_data)
    
    if char_code < first_char or char_code >= first_char + num_chars:
        print(f"Character code {char_code} ({chr(char_code)}) is out of range.")
        return None
    
    # Calculate character index
    char_idx = char_code - first_char
    
    # Get jump table entry (4 bytes per character)
    jump_table_offset = 4 + char_idx * 4
    data_offset = (font_data[jump_table_offset] << 8) | font_data[jump_table_offset + 1]
    char_width = font_data[jump_table_offset + 2]
    x_advance = font_data[jump_table_offset + 3]
    
    if data_offset == 0xFFFF:
        print(f"Character {char_code} ({chr(char_code)}) has no bitmap data.")
        return None
    
    # Calculate actual data offset (jump table size + header)
    data_start = 4 + (num_chars * 4) + data_offset
    
    # Calculate number of pages (8 pixels per page vertically)
    pages = (height + 7) // 8
    
    # Extract bitmap data
    bitmap_data = []
    for col in range(char_width):
        col_data = []
        for page in range(pages):
            byte_index = data_start + (col * pages) + page
            if byte_index < len(font_data):
                pixel_byte = font_data[byte_index]
                col_data.append(pixel_byte)
        bitmap_data.append(col_data)
    
    return {
        'width': char_width,
        'height': height,
        'x_advance': x_advance,
        'bitmap': bitmap_data
    }

def visualize_character_ascii(char_data, char_code, highlight_issues=True):
    """Create an ASCII art representation of the character bitmap."""
    if not char_data:
        return
    
    width = char_data['width']
    height = char_data['height']
    bitmap = char_data['bitmap']
    
    # Create a 2D array to represent the bitmap
    ascii_bitmap = [[' ' for _ in range(width)] for _ in range(height)]
    
    # Fill in the bitmap
    for col in range(width):
        if col >= len(bitmap):
            continue
            
        col_data = bitmap[col]
        for page in range(len(col_data)):
            pixel_byte = col_data[page]
            
            for bit in range(8):
                if page * 8 + bit >= height:
                    continue
                    
                # Check if this bit is set (LSB first)
                if pixel_byte & (1 << bit):
                    # Set the character in the ASCII bitmap
                    y = page * 8 + bit
                    
                    # Use different characters for highlighting potential issues
                    char = '#'
                    
                    # Highlight potential issues
                    if highlight_issues:
                        # Highlight the last column (where '7' dot or ':' semicolon might be)
                        if col == width - 1:
                            # Highlight bottom rows for '7'
                            if page >= len(col_data) - 2:
                                char = 'X'  # Use 'X' for highlighted pixels
                    
                    ascii_bitmap[y][col] = char
    
    # Print the ASCII bitmap
    print(f"ASCII Bitmap for character {chr(char_code)} (code: {char_code}):")
    print("+" + "-" * width + "+")
    for row in ascii_bitmap:
        print("|" + ''.join(row) + "|")
    print("+" + "-" * width + "+")
    
    # Print legend
    if highlight_issues:
        print("\nLegend:")
        print("# - Normal pixel")
        print("X - Highlighted pixel (potential issue)")

def load_font_data(font_file):
    """Load font data from a C header file or binary file."""
    print(f"DEBUG: Attempting to load font data from {font_file}")
    if not os.path.exists(font_file):
        print(f"ERROR: File {font_file} does not exist.")
        return None
    
    if font_file.endswith('.c') or font_file.endswith('.h'):
        # Parse C file
        print(f"DEBUG: Parsing C file {font_file}")
        with open(font_file, 'r') as f:
            content = f.read()
        
        print(f"DEBUG: File size: {len(content)} bytes")
        
        # Extract the font data array
        start = content.find('{')
        end = content.rfind('}')
        if start == -1 or end == -1:
            print("ERROR: Could not find font data array in C file.")
            return None
        
        print(f"DEBUG: Found array bounds: start={start}, end={end}")
        
        # Parse the hex values
        data_str = content[start+1:end]
        data_str = data_str.replace('\n', '').replace('\t', '').replace(' ', '')
        
        print(f"DEBUG: Data string length after cleaning: {len(data_str)}")
        print(f"DEBUG: First 100 chars of data string: {data_str[:100]}")
        
        values = []
        
        i = 0
        while i < len(data_str):
            if data_str[i:i+2] == '0x':
                # Parse hex value
                j = i + 2
                while j < len(data_str) and data_str[j].isalnum():
                    j += 1
                hex_val = data_str[i+2:j]
                if hex_val:
                    try:
                        values.append(int(hex_val, 16))
                    except ValueError:
                        print(f"ERROR: Invalid hex value: {hex_val}")
                i = j
            elif data_str[i] == ',':
                i += 1
            else:
                i += 1
        
        print(f"DEBUG: Extracted {len(values)} values from the font data")
        if len(values) > 0:
            print(f"DEBUG: First few values: {values[:10]}")
        
        return values
    else:
        # Assume binary file
        print(f"DEBUG: Reading binary file {font_file}")
        with open(font_file, 'rb') as f:
            data = list(f.read())
        print(f"DEBUG: Read {len(data)} bytes from binary file")
        return data

def main():
    try:
        parser = argparse.ArgumentParser(description='Font Bitmap Viewer for SSD1306 Display')
        parser.add_argument('font_file', help='Path to the font file (.c, .h, or binary)')
        parser.add_argument('--char', type=str, default='7', help='Character to visualize (default: 7)')
        parser.add_argument('--range', action='store_true', help='Visualize a range of characters')
        parser.add_argument('--start', type=str, default='0', help='Start character for range (default: 0)')
        parser.add_argument('--end', type=str, default='9', help='End character for range (default: 9)')
        
        args = parser.parse_args()
        
        print(f"DEBUG: Arguments: {args}")
        
        # Load font data
        font_data = load_font_data(args.font_file)
        if not font_data:
            print("ERROR: Failed to load font data")
            return
        
        # Parse font header
        width, height, first_char, num_chars = parse_font_header(font_data)
        print(f"Font Info: Width={width}, Height={height}, FirstChar={first_char}, NumChars={num_chars}")
        
        if args.range:
            # Visualize a range of characters
            start_char = ord(args.start) if len(args.start) == 1 else int(args.start)
            end_char = ord(args.end) if len(args.end) == 1 else int(args.end)
            
            print(f"DEBUG: Visualizing character range from {start_char} to {end_char}")
            
            for char_code in range(start_char, end_char + 1):
                char_data = extract_character_data(font_data, char_code)
                if char_data:
                    print(f"\n{'='*40}")
                    print(f"Character {char_code} ({chr(char_code)}): Width={char_data['width']}, X-Advance={char_data['x_advance']}")
                    print(f"{'='*40}")
                    
                    # Print bitmap data in binary format
                    print("Bitmap Data:")
                    for col in range(char_data['width']):
                        if col < len(char_data['bitmap']):
                            col_data = char_data['bitmap'][col]
                            print(f"Column {col}:")
                            for page in range(len(col_data)):
                                pixel_byte = col_data[page]
                                binary = format(pixel_byte, '08b')
                                print(f"  Page {page}: {binary}")
                    
                    # Visualize the character
                    visualize_character_ascii(char_data, char_code)
                    
                    # Ask user to press Enter to continue
                    if char_code < end_char:
                        input("\nPress Enter to view next character...")
        else:
            # Visualize a single character
            char_code = ord(args.char) if len(args.char) == 1 else int(args.char)
            print(f"DEBUG: Visualizing character {char_code} ({chr(char_code)})")
            
            char_data = extract_character_data(font_data, char_code)
            
            if char_data:
                print(f"Character {char_code} ({chr(char_code)}): Width={char_data['width']}, X-Advance={char_data['x_advance']}")
                
                # Print bitmap data in binary format
                print("Bitmap Data:")
                for col in range(char_data['width']):
                    if col < len(char_data['bitmap']):
                        col_data = char_data['bitmap'][col]
                        print(f"Column {col}:")
                        for page in range(len(col_data)):
                            pixel_byte = col_data[page]
                            binary = format(pixel_byte, '08b')
                            print(f"  Page {page}: {binary}")
                
                # Visualize the character
                visualize_character_ascii(char_data, char_code)
    except Exception as e:
        print(f"ERROR: An exception occurred: {e}")
        print("Traceback:")
        traceback.print_exc()

if __name__ == '__main__':
    main()