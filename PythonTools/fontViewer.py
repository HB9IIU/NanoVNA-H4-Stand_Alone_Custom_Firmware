import re, os
import matplotlib.pyplot as plt

def parse_font_file(filename):
    """Return list of glyph byte lists from a .c font file."""
    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()
    entries = re.findall(r'\{([0-9xXa-fA-F,\s]+)\}', content)
    bitmaps = []
    for e in entries:
        vals = [int(x.strip(), 16) for x in e.split(',') if x.strip()]
        bitmaps.append(vals)
    return bitmaps

def get_font_size_from_name(name):
    m = re.search(r'(\d+)x(\d+)', name)
    if m:
        return int(m.group(1)), int(m.group(2))
    return 8, 8

def render_text(bitmap, width, height, text):
    """Return a 2-D array (list of lists) for given text using font bitmap."""
    spacing = 1
    glyphs = []
    for ch in text:
        idx = ord(ch) - 32
        if 0 <= idx < len(bitmap):
            g = bitmap[idx]
            g_rows = [[(val >> (width - 1 - i)) & 1 for i in range(width)] for val in g]
        else:
            g_rows = [[0]*width for _ in range(height)]
        glyphs.append(g_rows)
    # join glyphs horizontally
    out = []
    for row in range(height):
        out_row = []
        for g in glyphs:
            out_row.extend(g[row] + [0]*spacing)
        out.append(out_row)
    return out

def main():
    files = [f for f in os.listdir('.') if f.lower().endswith('.c')]
    sample = "Hello 123.45 MHz"
    if not files:
        print("❌ No font files found.")
        return

    rows = len(files)
    fig, axes = plt.subplots(rows, 1, figsize=(10, 2*rows))
    if rows == 1:
        axes = [axes]

    for ax, fname in zip(axes, files):
        w, h = get_font_size_from_name(fname)
        data = parse_font_file(fname)
        bmp = render_text(data, w, h, sample)
        ax.imshow(bmp, cmap="Greys", interpolation="nearest")
        ax.set_title(f"{fname}  ({w}x{h})  →  \"{sample}\"")
        ax.axis("off")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
