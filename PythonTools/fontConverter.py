from PIL import Image, ImageFont, ImageDraw
import sys

# --- CONFIG --- #
ttf_file    = "RobotoMono-Bold.ttf"   # name of your font file here
font_size   = 32                     # pixel height of font you want
out_c_name  = "FontUbuntu22.c"       # output C filename
font_name   = "FontUbuntu22"         # name used inside C file
ascii_start = 32
ascii_end   = 126

# --- SCRIPT --- #
font = ImageFont.truetype(ttf_file, font_size)
glyphs = []

# determine max width
max_width = max(font.getbbox(chr(c))[2] for c in range(ascii_start, ascii_end+1))

for c in range(ascii_start, ascii_end+1):
    ch = chr(c)
    bbox = font.getbbox(ch)
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    img = Image.new("1", (max_width, font_size), 0)
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), ch, 1, font=font)

    pixels = list(img.getdata())
    rows = []
    for y in range(font_size):
        row = 0
        for x in range(max_width):
            row <<= 1
            row |= pixels[y * max_width + x]
        rows.append(row)

    glyphs.append(rows)

with open(out_c_name, "w") as f:
    f.write(f"// Generated from {ttf_file} at {font_size}px\n")
    f.write(f"const uint16_t {font_name}_width = {max_width};\n")
    f.write(f"const uint16_t {font_name}_height = {font_size};\n")
    f.write(f"const uint16_t {font_name}_first = {ascii_start};\n")
    f.write(f"const uint16_t {font_name}_last = {ascii_end};\n")
    f.write(f"const uint32_t {font_name}_data[] = {{\n")
    for rows in glyphs:
        f.write("    ")
        f.write(", ".join(f"0x{row:04x}" for row in rows))
        f.write(",\n")
    f.write("};\n")

print(f"✔ Wrote {out_c_name}.")
