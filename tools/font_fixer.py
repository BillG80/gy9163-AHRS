#!/usr/bin/env python3
"""
Font Bitmap Fixer for SSD1306 Display
This tool directly edits the font bitmap data to fix rendering issues with specific characters.
"""

import sys
import os
import argparse
import shutil

DEBUG = False

def read_font_file(font_file):
    """Read the font file and return its content."""
    with open(font_file, 'r') as f:
        return f.read()

def write_font_file(font_file, content):
    """Write the modified content back to the font file."""
    # Create a backup of the original file
    backup_file = font_file + '.bak'
    try:
        shutil.copy2(font_file, backup_file)
        print(f"Created backup at {backup_file}")
    except Exception as e:
        print(f"Warning: Could not create backup file: {e}")
    
    # Write the modified content
    with open(font_file, 'w') as f:
        f.write(content)
    
    print(f"Updated font file: {font_file}")

def parse_font_header(content):
    """Parse the font header to get basic information."""
    lines = content.split('\n')
    for line in lines:
        if "Width:" in line and "Height:" in line:
            parts = line.split(',')
            width = int(parts[0].split(':')[1].strip())
            height = int(parts[1].split(':')[1].strip())
            return width, height
    return 11, 15  # Default values if not found

def find_character_data(content, char_code):
    """Find the bitmap data for a specific character in the font file."""
    print(f"Searching for character data for code {char_code}...")
    
    try:
        code = int(char_code)
        char_name = chr(code)
        print(f"Looking for bitmap data for character '{char_name}' (code {code})...")
        
        lines = content.split('\n')
        
        # First, find the jump table entry for this character
        jump_table_entry = None
        for i, line in enumerate(lines):
            if f"// {code}:" in line and "0x" in line:
                jump_table_entry = line
                print(f"Found jump table entry at line {i+1}: {line}")
                break
        
        if not jump_table_entry:
            print(f"Could not find jump table entry for character {char_name} (code {code})")
            return -1, None
        
        # Extract the offset from the jump table entry
        # Format is typically: 0x02, 0xDA, 0x0C, 0x07,  // 97:730
        # where 730 is the offset into the bitmap data
        try:
            offset_str = jump_table_entry.split(':')[1].strip()
            offset = int(offset_str)
            print(f"Extracted offset {offset} from jump table entry")
        except (IndexError, ValueError) as e:
            print(f"Could not extract offset from jump table entry: {e}")
            return -1, None
        
        # Now find the bitmap data at this offset
        # The bitmap data typically spans multiple lines and doesn't have a clear marker
        # We'll look for a line that has many hex values and is near the expected offset
        
        # First, count how many hex values we've seen so far
        hex_count = 0
        bitmap_start_line = -1
        
        for i, line in enumerate(lines):
            if "0x" in line:
                # Count the hex values in this line
                hex_values_in_line = len([p for p in line.split(',') if "0x" in p])
                hex_count += hex_values_in_line
                
                # If we're near or past the offset, this might be our bitmap data
                if hex_count >= offset and bitmap_start_line == -1:
                    bitmap_start_line = i
                    print(f"Found potential bitmap data at line {i+1}, hex_count={hex_count}, offset={offset}")
                    
                    # Look for a line with many hex values (bitmap data typically has many bytes)
                    if hex_values_in_line >= 8:
                        print(f"Found bitmap data line with {hex_values_in_line} hex values")
                        return i, line
        
        # If we couldn't find a good bitmap data line, just return the line we think is closest
        if bitmap_start_line != -1:
            print(f"Returning best guess for bitmap data at line {bitmap_start_line+1}")
            return bitmap_start_line, lines[bitmap_start_line]
        
    except Exception as e:
        print(f"Error finding bitmap data: {e}")
    
    print(f"Could not find bitmap data for character {char_code}")
    return -1, None

def fix_character_bitmap(content, char_code, x_advance):
    """Generic function to fix a character bitmap by clearing all columns beyond x_advance."""
    line_index, line = find_character_data(content, char_code)
    if line_index == -1:
        print(f"Could not find data for character '{chr(int(char_code))}'")
        return content
    
    print(f"Found '{chr(int(char_code))}' character data: {line}")
    
    # Split the line into parts
    parts = line.split(',')
    
    # Count how many hex values we have
    hex_values = [p for p in parts if "0x" in p or "0X" in p]
    total_bytes = len(hex_values)
    
    print(f"Character '{chr(int(char_code))}' has {total_bytes} bytes, {total_bytes//2} columns")
    print(f"Will clear columns from {x_advance} to {total_bytes//2-1}")
    
    # For any character, we want to clear all columns after x_advance
    # Each column has 2 bytes (page 0 and page 1)
    start_clear_index = x_advance * 2  # Start clearing from column after x_advance
    end_clear_index = total_bytes  # Clear all the way to the end
    
    # Find the actual indices in the parts list
    clear_indices = []
    hex_count = 0
    for i, part in enumerate(parts):
        if "0x" in part or "0X" in part:
            if hex_count >= start_clear_index and hex_count < end_clear_index:
                clear_indices.append(i)
            hex_count += 1
    
    print(f"Will clear {len(clear_indices)} bytes at indices: {clear_indices}")
    
    # Clear the specified columns
    modified = False
    for i in clear_indices:
        if i < len(parts):
            original_value = parts[i]
            if parts[i] != "0x00":
                parts[i] = "0x00"
                modified = True
                print(f"Changed byte at index {i} from {original_value} to 0x00")
    
    if not modified:
        print(f"No modification needed for character '{chr(int(char_code))}'")
        return content
    
    # Join the parts back together
    modified_line = ','.join(parts)
    
    # For debugging, print the before and after
    print(f"Original line: {line[:100]}...")
    print(f"Modified line: {modified_line[:100]}...")
    
    # Replace the line in the content
    lines = content.split('\n')
    original_line = lines[line_index]
    lines[line_index] = modified_line
    
    # Verify the line was actually changed
    if original_line == modified_line:
        print("WARNING: Line content did not change despite modifications!")
    else:
        print(f"Successfully cleared pixels after x_advance for character '{chr(int(char_code))}'")
    
    return '\n'.join(lines)

def fix_character_a(content):
    """Fix the 'a' character (code 97) by removing pixels beyond x_advance."""
    return fix_character_bitmap(content, "97", 7)

def fix_character_b(content):
    """Fix the 'b' character (code 98) by removing pixels beyond x_advance."""
    return fix_character_bitmap(content, "98", 7)

def fix_character_c(content):
    """Fix the 'c' character (code 99) by removing pixels beyond x_advance."""
    return fix_character_bitmap(content, "99", 6)

def fix_character_7(content):
    """Fix the '7' character (code 55) by removing the dot at the bottom right."""
    return fix_character_bitmap(content, "55", 7)

def fix_character_e(content):
    """Fix the 'e' character (code 101) by completely clearing columns 6-11."""
    line_index, line = find_character_data(content, "101")
    if line_index == -1:
        print(f"Could not find data for character 'e'")
        return content
    
    print(f"Found 'e' character data: {line}")
    
    # Split the line into parts
    parts = line.split(',')
    
    # Count how many hex values we have
    hex_values = [p for p in parts if "0x" in p or "0X" in p]
    total_bytes = len(hex_values)
    
    # For 'e', we specifically want to clear columns 6-11
    # Each column has 2 bytes (page 0 and page 1)
    start_clear_index = 6 * 2  # Start clearing from column 6
    end_clear_index = min(12 * 2, total_bytes)  # Clear up to column 11 or the end
    
    # Find the actual indices in the parts list
    clear_indices = []
    hex_count = 0
    for i, part in enumerate(parts):
        if "0x" in part or "0X" in part:
            if hex_count >= start_clear_index and hex_count < end_clear_index:
                clear_indices.append(i)
            hex_count += 1
    
    # Clear the specified columns
    for i in clear_indices:
        if i < len(parts):
            parts[i] = "0x00"
    
    # Join the parts back together
    modified_line = ','.join(parts)
    
    # Replace the line in the content
    lines = content.split('\n')
    lines[line_index] = modified_line
    return '\n'.join(lines)

def fix_character_v(content):
    """Fix the 'v' character (code 118) by removing pixels beyond x_advance."""
    return fix_character_bitmap(content, "118", 7)

def fix_all_lowercase_letters(content):
    """Fix all lowercase letters (a-z) by removing pixels beyond their x_advance."""
    for code in range(97, 123):  # ASCII codes for 'a' through 'z'
        content = fix_character_bitmap(content, str(code), 7)  # Assume x_advance of 7 for all
    return content

def fix_all_characters(content):
    """Apply fixes to all problematic characters."""
    # Fix specific characters with known issues
    content = fix_character_7(content)
    content = fix_character_e(content)
    content = fix_character_v(content)
    
    # Fix all lowercase letters
    content = fix_all_lowercase_letters(content)
    
    return content

def main():
    parser = argparse.ArgumentParser(description='Fix bitmap issues in SSD1306 font files')
    parser.add_argument('font_file', help='Path to the font file (.c)')
    parser.add_argument('--char', type=str, help='Specific character to fix (e.g., "7", "e", "v")')
    parser.add_argument('--all', action='store_true', help='Fix all known problematic characters')
    parser.add_argument('--no-backup', action='store_true', help='Skip creating backup file')
    parser.add_argument('--output', help='Output file path (default: overwrite input file)')
    parser.add_argument('--test', action='store_true', help='Test mode - find character data but don\'t modify')
    parser.add_argument('--debug', action='store_true', help='Enable verbose debug output')
    parser.add_argument('--direct-edit', action='store_true', help='Directly edit specific bytes in the font file')
    parser.add_argument('--restore', help='Restore a character from a good version of the font file')
    parser.add_argument('--restore-file', help='Path to the good version of the font file to restore from')
    
    args = parser.parse_args()
    
    # Enable debug mode if requested
    global DEBUG
    DEBUG = args.debug
    if DEBUG:
        print("Debug mode enabled")
    
    # Check if the font file exists
    if not os.path.exists(args.font_file):
        print(f"Error: Font file {args.font_file} does not exist.")
        return
    
    # Restore mode - restore a character from a good version of the font file
    if args.restore and args.restore_file:
        if not os.path.exists(args.restore_file):
            print(f"Error: Restore file {args.restore_file} does not exist.")
            return
            
        try:
            # Read both files
            with open(args.font_file, 'r') as f:
                content = f.read()
                
            with open(args.restore_file, 'r') as f:
                restore_content = f.read()
                
            # Find the bitmap data for the character in both files
            char_code = ord(args.restore)
            print(f"Restoring character '{args.restore}' (code {char_code}) from {args.restore_file}")
            
            # Find the bitmap data in the restore file
            restore_bitmap_line = None
            restore_lines = restore_content.split('\n')
            for i, line in enumerate(restore_lines):
                if f"// {char_code}" in line and "0x" in line:
                    restore_bitmap_line = line
                    print(f"Found bitmap data in restore file: {restore_bitmap_line[:50]}...")
                    break
                    
            if not restore_bitmap_line:
                print(f"Could not find bitmap data for character '{args.restore}' in restore file")
                return
                
            # Find the bitmap data in the current file
            current_bitmap_line = None
            current_bitmap_index = -1
            lines = content.split('\n')
            for i, line in enumerate(lines):
                if f"// {char_code}" in line and "0x" in line:
                    current_bitmap_line = line
                    current_bitmap_index = i
                    print(f"Found bitmap data in current file: {current_bitmap_line[:50]}...")
                    break
                    
            if current_bitmap_index == -1:
                print(f"Could not find bitmap data for character '{args.restore}' in current file")
                return
                
            # Replace the bitmap data
            lines[current_bitmap_index] = restore_bitmap_line
            modified_content = '\n'.join(lines)
            
            # Write the modified content back
            output_file = args.output if args.output else args.font_file
            with open(output_file, 'w') as f:
                f.write(modified_content)
                
            print(f"Successfully restored character '{args.restore}' from {args.restore_file}")
            return
            
        except Exception as e:
            print(f"Error in restore mode: {e}")
            return
    
    # Create backup if needed
    if not args.no_backup:
        backup_file = f"{args.font_file}.bak"
        try:
            shutil.copy2(args.font_file, backup_file)
            print(f"Created backup at {backup_file}")
        except Exception as e:
            print(f"Warning: Could not create backup file: {e}")
    
    # Direct edit mode - manually specify which bytes to edit
    if args.direct_edit:
        if not args.char:
            print("Error: --char is required with --direct-edit")
            return
            
        try:
            # Read the file as text
            with open(args.font_file, 'r') as f:
                content = f.read()
                
            # Define specific edits for each character
            if args.char == 'a':
                # We need to find the actual bitmap data in the C source file
                # Since we're editing a text file, not a binary file, we need to find the hex values
                # in the text and replace them with "0x00"
                
                # Find the bitmap data for character 'a'
                # The jump table entry for 'a' should look like: 0x02, 0xDA, 0x0C, 0x07,  // 97:730
                a_jump_entry = None
                lines = content.split('\n')
                for i, line in enumerate(lines):
                    if "// 97:" in line:
                        a_jump_entry = line
                        print(f"Found 'a' jump table entry: {a_jump_entry}")
                        break
                
                if not a_jump_entry:
                    print("Could not find jump table entry for 'a'")
                    return
                
                # Extract the offset
                try:
                    offset_str = a_jump_entry.split(':')[1].strip()
                    offset = int(offset_str)
                    print(f"Bitmap data for 'a' starts at offset {offset}")
                except Exception as e:
                    print(f"Error extracting offset: {e}")
                    return
                
                # Now find the actual bitmap data
                # This is tricky because we need to count hex values in the file
                hex_count = 0
                bitmap_line_index = -1
                
                for i, line in enumerate(lines):
                    hex_values = [part for part in line.split(',') if "0x" in part]
                    hex_count += len(hex_values)
                    
                    if hex_count >= offset and bitmap_line_index == -1:
                        bitmap_line_index = i
                        print(f"Found bitmap data for 'a' at line {i+1}")
                        break
                
                if bitmap_line_index == -1:
                    print("Could not find bitmap data for 'a'")
                    return
                
                # Now we need to modify the bitmap data
                # For 'a', we want to clear all columns after column 6
                # Each column is 2 bytes (2 hex values)
                # So we need to replace the hex values for columns 7+ with 0x00
                
                # Get the bitmap data line
                bitmap_line = lines[bitmap_line_index]
                print(f"Original bitmap line: {bitmap_line}")
                
                # Split by commas to get individual hex values
                parts = bitmap_line.split(',')
                
                # Find the hex values (parts containing "0x")
                hex_parts = [i for i, part in enumerate(parts) if "0x" in part]
                
                # Calculate which parts to clear (columns 7+, each column is 2 bytes)
                start_clear_index = 6 * 2  # Column 7 starts at index 12 (0-indexed)
                
                # Clear the hex values
                modified = False
                for i in range(start_clear_index, len(hex_parts)):
                    if i < len(hex_parts):
                        idx = hex_parts[i]
                        original_value = parts[idx]
                        parts[idx] = "0x00"
                        print(f"Replaced {original_value} with 0x00 at index {idx}")
                        modified = True
                
                if not modified:
                    print("No modifications needed for 'a'")
                    return
                
                # Join the parts back together
                modified_line = ','.join(parts)
                print(f"Modified bitmap line: {modified_line}")
                
                # Replace the line in the content
                lines[bitmap_line_index] = modified_line
                modified_content = '\n'.join(lines)
                
                # Write the modified content back
                output_file = args.output if args.output else args.font_file
                with open(output_file, 'w') as f:
                    f.write(modified_content)
                
                print(f"Successfully edited character 'a' in {output_file}")
                return
                
            elif args.char == 'e':
                # Direct fix for 'e' character (code 101)
                print("Directly fixing 'e' character (code 101)")
                
                # The bitmap data for 'e' is on line 306 (0-indexed)
                # 0x00,0x00,0xC0,0x07,0x20,0x09,0x20,0x09,0x60,0x09,0xC0,0x09, // 101
                
                lines = content.split('\n')
                
                # Find the line with the bitmap data for 'e'
                e_bitmap_line = None
                bitmap_line_index = -1
                
                # First, try to find the bitmap data line by looking for a line with "// 101" that has many hex values
                for i, line in enumerate(lines):
                    if "0x" in line and line.strip().endswith("// 101"):
                        # Count the number of hex values in this line
                        hex_count = line.count("0x")
                        if hex_count > 4:  # Jump table entries usually have 4 or fewer hex values
                            e_bitmap_line = line
                            bitmap_line_index = i
                            print(f"Found 'e' bitmap data at line {i+1}: {line}")
                            break
                
                # If we didn't find it, try a more direct approach - look for line 306
                if not e_bitmap_line and len(lines) > 306:
                    line = lines[306]
                    if "// 101" in line:
                        e_bitmap_line = line
                        bitmap_line_index = 306
                        print(f"Found 'e' bitmap data at line 307 (hardcoded): {line}")
                
                # If we still didn't find it, try a more general approach
                if not e_bitmap_line:
                    for i, line in enumerate(lines):
                        if "0x00,0x00,0xC0,0x07" in line and "// 101" in line:
                            e_bitmap_line = line
                            bitmap_line_index = i
                            print(f"Found 'e' bitmap data at line {i+1} (pattern match): {line}")
                            break
                
                if not e_bitmap_line:
                    print("Could not find bitmap data for 'e'")
                    return
                
                # Split by commas to get individual hex values
                parts = e_bitmap_line.split(',')
                
                # Find the hex values (parts containing "0x")
                hex_parts = []
                for i, part in enumerate(parts):
                    if "0x" in part:
                        hex_parts.append(i)
                
                print(f"Found {len(hex_parts)} hex values in the bitmap data")
                
                # The 'e' character bitmap should have 12 hex values (6 columns)
                # We want to keep the first 10 hex values (5 columns) and clear the rest
                # This will remove the dot at the top right
                if len(hex_parts) > 10:
                    # Clear columns 5+ (starting from 0)
                    start_clear_index = 10
                    
                    # Clear the hex values
                    modified = False
                    for i in range(start_clear_index, len(hex_parts)):
                        idx = hex_parts[i]
                        if idx < len(parts):
                            original = parts[idx]
                            parts[idx] = "0x00"
                            print(f"Replaced {original} with 0x00 at index {idx}")
                            modified = True
                    
                    if modified:
                        # Join the parts back together
                        modified_line = ','.join(parts)
                        print(f"Modified bitmap line: {modified_line}")
                        
                        # Replace the line in the content
                        lines[bitmap_line_index] = modified_line
                        modified_content = '\n'.join(lines)
                        
                        # Write the modified content back
                        output_file = args.output if args.output else args.font_file
                        with open(output_file, 'w') as f:
                            f.write(modified_content)
                        
                        print(f"Successfully edited character 'e' in {output_file}")
                    else:
                        print("No modifications needed for 'e'")
                else:
                    print("Bitmap data for 'e' is already small enough, no need to modify")
                
                return

            elif args.char == '7':
                # Find the bitmap data for character '7'
                # The jump table entry for '7' should look like: 0x0X, 0xXX, 0xXX, 0xXX,  // 55:XXX
                seven_jump_entry = None
                lines = content.split('\n')
                for i, line in enumerate(lines):
                    if "// 55:" in line:
                        seven_jump_entry = line
                        print(f"Found '7' jump table entry: {seven_jump_entry}")
                        break
                
                if not seven_jump_entry:
                    print("Could not find jump table entry for '7'")
                    return
                
                # Extract the offset
                try:
                    offset_str = seven_jump_entry.split(':')[1].strip()
                    offset = int(offset_str)
                    print(f"Bitmap data for '7' starts at offset {offset}")
                except Exception as e:
                    print(f"Error extracting offset: {e}")
                    return
                
                # Now find the actual bitmap data
                hex_count = 0
                bitmap_line_index = -1
                
                for i, line in enumerate(lines):
                    hex_values = [part for part in line.split(',') if "0x" in part]
                    hex_count += len(hex_values)
                    
                    if hex_count >= offset and bitmap_line_index == -1:
                        bitmap_line_index = i
                        print(f"Found bitmap data for '7' at line {i+1}")
                        break
                
                if bitmap_line_index == -1:
                    print("Could not find bitmap data for '7'")
                    return
                
                # Now we need to modify the bitmap data
                # For '7', we want to clear the dot at the bottom right
                # This is typically in the last byte of the bitmap
                
                # Get the bitmap data line
                bitmap_line = lines[bitmap_line_index]
                print(f"Original bitmap line: {bitmap_line}")
                
                # Split by commas to get individual hex values
                parts = bitmap_line.split(',')
                
                # Find the hex values (parts containing "0x")
                hex_parts = [i for i, part in enumerate(parts) if "0x" in part]
                
                # The dot is typically in the last byte of the bitmap
                # For '7', we'll clear the last byte
                if hex_parts:
                    last_idx = hex_parts[-1]
                    original = parts[last_idx]
                    parts[last_idx] = "0x00"
                    print(f"Replaced {original} with 0x00 at index {last_idx}")
                    
                    # Join the parts back together
                    modified_line = ','.join(parts)
                    print(f"Modified bitmap line: {modified_line}")
                    
                    # Replace the line in the content
                    lines[bitmap_line_index] = modified_line
                    modified_content = '\n'.join(lines)
                    
                    # Write the modified content back
                    output_file = args.output if args.output else args.font_file
                    with open(output_file, 'w') as f:
                        f.write(modified_content)
                    
                    print(f"Successfully edited character '7' in {output_file}")
                else:
                    print("Could not find hex values in bitmap line for '7'")
                
                return
                
            else:
                print(f"No direct edit defined for character '{args.char}'")
                return

        except Exception as e:
            print(f"Error in direct edit mode: {e}")
            return
    
    # Read the font file
    content = read_font_file(args.font_file)
    
    width, height = parse_font_header(content)
    print(f"Font dimensions: {width}x{height}")
    
    # Apply fixes
    if args.test:
        print("TEST MODE: Will find character data but not modify it")
        if args.char:
            try:
                code = ord(args.char)
                line_index, line = find_character_data(content, str(code))
                if line_index != -1:
                    print(f"Found character '{args.char}' (code {code}) at line {line_index+1}")
                    print(f"Line content: {line}")
                    
                    # Count how many hex values we have
                    parts = line.split(',')
                    hex_values = [p for p in parts if "0x" in p or "0X" in p]
                    total_bytes = len(hex_values)
                    print(f"Character has {total_bytes} bytes, {total_bytes//2} columns")
                    
                    # Simulate the fix
                    x_advance = 7  # Default x_advance
                    start_clear_index = x_advance * 2
                    clear_indices = []
                    hex_count = 0
                    for i, part in enumerate(parts):
                        if "0x" in part or "0X" in part:
                            if hex_count >= start_clear_index:
                                clear_indices.append((i, part))
                            hex_count += 1
                    
                    print(f"Would clear {len(clear_indices)} bytes:")
                    for i, value in clear_indices:
                        print(f"  Index {i}: {value} -> 0x00")
                else:
                    print(f"Could not find character '{args.char}' (code {code})")
            except Exception as e:
                print(f"Error in test mode: {e}")
        else:
            print("Please specify a character with --char in test mode")
        return
    
    if args.all:
        print("Fixing all known problematic characters...")
        content = fix_all_characters(content)
    elif args.char:
        print(f"Fixing character '{args.char}'...")
        if args.char == '7':
            content = fix_character_7(content)
        elif args.char == 'e':
            content = fix_character_e(content)
        elif args.char == 'v':
            content = fix_character_v(content)
        elif args.char == 'a':
            content = fix_character_a(content)
        elif args.char == 'b':
            content = fix_character_b(content)
        elif args.char == 'c':
            content = fix_character_c(content)
        else:
            print(f"No specific fix available for character '{args.char}'")
    else:
        print("No characters specified to fix. Use --char or --all.")
        return
    
    # Write the modified content back to the font file
    write_font_file(args.font_file, content)
    print("Font bitmap fixing completed.")

if __name__ == '__main__':
    main()