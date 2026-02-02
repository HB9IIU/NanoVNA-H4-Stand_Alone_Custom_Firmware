import sys, os, time, threading, argparse
from datetime import datetime
from serial import Serial
from serial.tools import list_ports

# ---- Optional colors (works great in VS Code terminal). If unavailable, no color.
try:
    import colorama
    colorama.just_fix_windows_console()
    COLOR_OK = True
except Exception:
    COLOR_OK = False

def c(s, color):
    if not COLOR_OK:
        return s
    return color + s + ("\x1b[0m")

COL_RED = "\x1b[31m"
COL_YEL = "\x1b[33m"
COL_CYA = "\x1b[36m"
COL_DIM = "\x1b[2m"

def pick_ftdi_port():
    ports = list(list_ports.comports())

    # Prefer FTDI manufacturer
    ftdi = [p for p in ports if (p.manufacturer or "").lower() == "ftdi"]
    if len(ftdi) == 1:
        return ftdi[0].device, ftdi[0].description

    # Fall back to any description containing FTDI
    any_ftdi = [p for p in ports if "ftdi" in (p.description or "").lower()]
    if len(any_ftdi) == 1:
        return any_ftdi[0].device, any_ftdi[0].description

    # Otherwise show options
    raise RuntimeError("Could not uniquely pick the FTDI port.\n" +
                       "\n".join([f"  {p.device}: {p.description} (manufacturer={p.manufacturer})" for p in ports]))

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def make_logfile(logdir, port):
    os.makedirs(logdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"NanoVNA_{port}_{stamp}.log"
    return os.path.join(logdir, filename)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--logdir", default=os.path.join(os.path.expanduser("~"), "serialTerminal", "terminalLog"))
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--mute", action="append", default=[], help="Substring to suppress (can repeat)")
    ap.add_argument("--eol", default="\r", help="EOL to send on Enter (default CR)")
    args = ap.parse_args()

    global COLOR_OK
    if args.no_color:
        COLOR_OK = False

    port, desc = pick_ftdi_port()
    print(f"Opening {port} ({desc}) at {args.baud}...  (Ctrl+C to quit)")

    logfile = None
    f = None
    if not args.no_log:
        logfile = make_logfile(args.logdir, port.replace(":", "_"))
        f = open(logfile, "a", encoding="utf-8", newline="\n")
        print(c(f"Logging to: {logfile}", COL_DIM))

    ser = Serial(port=port, baudrate=args.baud, timeout=0.1)

    stop = threading.Event()

    def write_log(line):
        if f:
            f.write(line + "\n")
            f.flush()

    def reader():
        buf = bytearray()
        while not stop.is_set():
            chunk = ser.read(1024)
            if not chunk:
                continue
            buf.extend(chunk)

            # Normalize on LF; keep CR-only lines too
            while True:
                nl = buf.find(b"\n")
                cr = buf.find(b"\r")
                cut = -1
                if nl != -1:
                    cut = nl
                    sep_len = 1
                elif cr != -1:
                    cut = cr
                    sep_len = 1
                else:
                    break

                raw = bytes(buf[:cut])
                del buf[:cut + sep_len]

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Mute filters
                if any(m in line for m in args.mute):
                    continue

                # Coloring heuristics
                out = line
                if "HardFault" in line or "FAULT" in line or "error" in line.lower():
                    out = c(line, COL_RED)
                elif "[MLA]" in line:
                    out = c(line, COL_CYA)
                elif "WARN" in line or "warning" in line.lower():
                    out = c(line, COL_YEL)

                stamped = f"{ts()}  {out}"
                print(stamped)
                write_log(f"{ts()}  {line}")

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            # Interactive input: type shell commands here
            s = sys.stdin.readline()
            if s == "":
                break
            s = s.rstrip("\r\n")
            ser.write(s.encode("utf-8") + args.eol.encode("utf-8"))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            ser.close()
        except Exception:
            pass
        if f:
            f.close()

if __name__ == "__main__":
    main()
