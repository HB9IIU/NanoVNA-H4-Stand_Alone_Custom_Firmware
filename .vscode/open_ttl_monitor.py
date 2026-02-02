import sys, subprocess
from serial.tools import list_ports

BAUD = "115200"

def pick_port():
    ports = list(list_ports.comports())

    # Prefer FTDI adapters
    ftdi = [p for p in ports if (p.manufacturer or "").lower() == "ftdi"]
    if len(ftdi) == 1:
        return ftdi[0]

    # If multiple FTDI exist, narrow by description text
    ftdi_usbserial = [p for p in ftdi if "usb serial port" in (p.description or "").lower()]
    if len(ftdi_usbserial) == 1:
        return ftdi_usbserial[0]

    # Fallback: try any port whose description contains "FTDI"
    any_ftdi = [p for p in ports if "ftdi" in (p.description or "").lower()]
    if len(any_ftdi) == 1:
        return any_ftdi[0]

    print("Could not uniquely pick the FTDI port. Available ports:")
    for p in ports:
        print(f"  {p.device}: {p.description} (manufacturer={p.manufacturer})")
    return None

p = pick_port()
if not p:
    sys.exit(1)

print(f"Opening {p.device} ({p.description}) at {BAUD}...")
subprocess.call([sys.executable, "-m", "serial.tools.miniterm", p.device, BAUD, "--eol", "LF"])