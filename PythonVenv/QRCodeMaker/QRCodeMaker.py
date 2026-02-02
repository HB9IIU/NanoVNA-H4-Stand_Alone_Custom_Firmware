# pip install qrcode[pil]
import qrcode

TEXT = "https://github.com/HB9IIU/NanoVNA-MLA-ToolBox"
BORDER = 1          # modules of quiet zone (to get 31x31 for version=3)
VERSION = 3         # 29x29 modules
ECC = qrcode.constants.ERROR_CORRECT_L

qr = qrcode.QRCode(
    version=VERSION,
    error_correction=ECC,
    box_size=1,
    border=BORDER
)
qr.add_data(TEXT)
qr.make(fit=False)

m = qr.get_matrix()         # True=black, False=white
h = len(m); w = len(m[0])
assert w == h

# Pack: 1 bit per pixel, MSB first in each byte
# Choose polarity: many bitmap renderers use 1=white. Match whatever your draw code expects.
ONE_IS_WHITE = True

out = []
for y in range(h):
    row = m[y]
    byte = 0
    bitpos = 7
    for x in range(w):
        is_black = row[x]
        bit = (0 if is_black else 1) if ONE_IS_WHITE else (1 if is_black else 0)
        byte |= (bit << bitpos)
        bitpos -= 1
        if bitpos < 0:
            out.append(byte)
            byte = 0
            bitpos = 7
    if bitpos != 7:  # flush partial byte
        out.append(byte)

print(f"// {w}x{h} QR bitmap, {len(out)} bytes")
print("static const uint8_t qr_code_map[] = {")
for i, b in enumerate(out):
    print(f"  0x{b:02x},", end="\n" if (i % 12) == 11 else "")
print("\n};")
