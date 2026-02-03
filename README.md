# NanoVNA-H4 Custom Firmware — Magnetic Loop Antenna Tuning

Custom firmware for the **NanoVNA-H4**, designed for **fast analysis and practical tuning** of **magnetic loop antennas**.

This firmware focuses on what you actually need at the antenna: quick resonance finding, easy-to-read values, and a tuning helper that works in real time.

---

## ✨ Key Features

- **Automatic plot centering** of **S11** and **SWR** curves  
- **Large, easy-to-read on-screen values**, including:
  - Current **resonance frequency**
  - **SWR**
  - **Theoretical reflected power (%)**
  - **Bandwidth at SWR = 2**
  - **Q factor**
- **Tuning Helper Page**
  - Set a **target frequency**
  - Manually adjust your capacitor while watching **SWR update live** in real time

---

## ⚠️ Important Notes / Disclaimer

- ✅ **Works only on NanoVNA-H4**
- ✅ Flashing is done the **same way as the official (stock) firmware**
- During development I flashed the device **well over 100 times** without bricking it.  
  **Still: use at your own risk. I cannot take responsibility for any damage.**

---

## 🎥 Demo Video

- YouTube: https://www.youtube.com/watch?v=NZVbPHmsltQ

---

## ⬇️ Download (Latest Binary)

- **Latest build:** `H4.bin`  
  https://raw.githubusercontent.com/HB9IIU/NanoVNA-H4-Stand_Alone_Custom_Firmware/main/build/H4.bin

---

## 🔥 Flashing

Flash it the same way you normally update NanoVNA-H4 firmware (DFU method).  
If you’ve flashed stock firmware before, you already know the process.

> Tip: Use a good USB cable and avoid hubs if you run into detection issues.

---

## 🧑‍💻 Developer Notes

### Keyboard Shortcuts (Custom)

| Shortcut | Action | Script | Description |
|---------:|--------|--------|-------------|
| Ctrl + Alt + B | Build | `00_HB9IIUscripts/buildonly.sh` | Clean + compile for STM32F303 |
| Ctrl + Alt + F | Flash | `00_HB9IIUscripts/flashonly.sh` | Flash `build/H4.bin` using `dfu-util` |
| Ctrl + Alt + Q | Build + Flash | `00_HB9IIUscripts/build_and_flash_h4.sh` | Full build then flash |
| Ctrl + Alt + C | Clean | `00_HB9IIUscripts/makeclean.sh` | Remove build artifacts |

### Location of Scripts

All automation scripts are stored in:

```
00_HB9IIUscripts/
```

Each script begins by returning to the project root:

```bash
cd "$(dirname "$0")/.."
```

This ensures the `Makefile` is found correctly.

### Build Output

The compiled firmware can be found here:

```
build/H4.bin
```

---

## 📌 Status

Visible: **0% → 100%**
