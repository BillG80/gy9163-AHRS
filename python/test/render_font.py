#!/usr/bin/env python3
import re

# Path to your raw font C
RAW_C = "src/fonts/fzlthc16_raw.c"

with open(RAW_C, "r", encoding="latin-1") as f:
    src = f.read()

# extract the flat bitmap
bm_block = re.search(r"const uint8_t FZLTHC16_bitmap\[\] = \{([^}]+)\};", src, re.S)
bytes_list = [int(x,16) for x in re.findall(r"0x[0-9A-Fa-f]+", bm_block.group(1))]

# extract glyph definitions: w,h,xOff,yOff,xAdv,offset
glyphs = re.findall(
    r"\{\s*(\d+),\s*(\d+),\s*([-0-9]+),\s*([-0-9]+),\s*(\d+),\s*&FZLTHC16_bitmap\[(\d+)\]\s*\},",
    src
)

for i,(w,h,xOff,yOff,xAdv,off) in enumerate(glyphs):
    w,h,xOff,yOff,xAdv,off = map(int,(w,h,xOff,yOff,xAdv,off))
    code = 32 + i
    ch = chr(code) if 32 <= code < 127 else f"#{code}"
    print(f"Glyph {i} → '{ch}'  size={w}x{h}  advance={xAdv}  offset={off}")
    for row in range(h):
        line = ""
        for col in range(w):
            byte = bytes_list[off + row*w + col]
            line += "█" if byte else " "
        print(line)
    print()