# Convert 320 x 240 pixels PNG image to RGB565 bitmap header

import png
import sys
import os

if len(sys.argv) < 3:
    print("Usage: convert_to_rgb565.py image.png image.h")
    sys.exit(1)

inp = sys.argv[1]
out = sys.argv[2]

if not os.path.exists(inp):
    print(f"Error: {inp} not found")
    sys.exit(1)

reader = png.Reader(inp)
width, height, pixels, metadata = reader.read()

print(f'Image size: {width}x{height}')
print(f'Planes: {metadata.get("planes")} (4=RGBA, 3=RGB)')

pixel_list = list(pixels)
planes = metadata.get("planes", 3)

def to565(r, g, b, a=255):
    if a < 128:
        return 0x0000
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

flat_pixels = []
for row in pixel_list:
    for i in range(0, len(row), planes):
        r = row[i]
        g = row[i+1] if i+1 < len(row) else 0
        b = row[i+2] if i+2 < len(row) else 0
        a = row[i+3] if planes == 4 and i+3 < len(row) else 255
        flat_pixels.append(to565(r, g, b, a))

with open(out, 'w') as f:
    f.write('#pragma once\n')
    f.write('#include <stdint.h>\n')
    f.write('#include <pgmspace.h>\n\n')
    f.write(f'// Generated from: {inp}\n')
    f.write(f'// Image size: {width}x{height}, {len(flat_pixels)} pixels\n')
    f.write(f'constexpr uint16_t IMAGE_WIDTH = {width}u;\n')
    f.write(f'constexpr uint16_t IMAGE_HEIGHT = {height}u;\n\n')
    f.write(f'const uint16_t image[IMAGE_WIDTH * IMAGE_HEIGHT] PROGMEM = {{\n')
    for i, val in enumerate(flat_pixels):
        if i % 12 == 0:
            f.write('    ')
        f.write(f'0x{val:04X},')
        if (i+1) % 12 == 0:
            f.write('\n')
    if len(flat_pixels) % 12 != 0:
        f.write('\n')
    f.write('};\n')

print(f'✓ Wrote {out} with {len(flat_pixels)} pixels ({width}x{height})')
