from PIL import Image
from pathlib import Path

INFILE = "logo.png"
INFILE = "AnotherFancyLogo.png"

OUTFILE = "HB9IIUlogo.h"
ARRAY_NAME = "logo_bits"
THRESHOLD = 128  # 0..255, lower = more black

def main():
    here = Path(__file__).resolve().parent
    in_path = here / INFILE
    out_path = here / OUTFILE

    if not in_path.exists():
        raise FileNotFoundError(f"Missing {INFILE} in: {here}")

    # Convert to grayscale then threshold to 1-bit
    img = Image.open(in_path).convert("L")
    w, h = img.size
    px = img.load()

    bytes_per_row = (w + 7) // 8
    data = bytearray(bytes_per_row * h)

    for y in range(h):
        for x in range(w):
            # pixel is "on" if darker than threshold
            on = 1 if px[x, y] < THRESHOLD else 0
            if on:
                i = y * bytes_per_row + (x // 8)
                bit = 7 - (x % 8)  # MSB-first
                data[i] |= (1 << bit)

    # Write header
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n")
        f.write('#include <stdint.h>\n\n')
        f.write(f"// Auto-generated from {INFILE}\n")
        f.write(f"// Size: {w}x{h}, 1bpp, MSB-first, row-major\n\n")
        f.write(f"#define {ARRAY_NAME.upper()}_W {w}\n")
        f.write(f"#define {ARRAY_NAME.upper()}_H {h}\n\n")
        f.write(f"static const uint8_t {ARRAY_NAME}[{len(data)}] = {{\n")

        # Hex dump, 16 bytes per line
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            f.write("  " + ", ".join(f"0x{b:02X}" for b in chunk) + ",\n")

        f.write("};\n")

    print(f"OK: wrote {OUTFILE}")
    print(f"Image size: {w}x{h}")
    print(f"Bytes: {len(data)} (bytes_per_row={bytes_per_row})")

if __name__ == "__main__":
    main()
