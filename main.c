/*
 * Copyright (c) 2019-2024, Dmitry (DiSlord) dislordlive@gmail.com
 * Based on TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, * Boston, MA 02110-1301, USA.
 */

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"

#include <chprintf.h>
#include <string.h>

#ifdef __USE_SERIAL_CONSOLE__
// Serial Shell commands output
int serial_shell_printf(const char *fmt, ...)
{
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}
#endif
// UART logging macro for MLA debug (UART SD1 via serial_shell_printf)
#ifdef __USE_SERIAL_CONSOLE__
#define MLA_UART_LOG(...) serial_shell_printf(__VA_ARGS__)
#else
#define MLA_UART_LOG(...) \
  do                      \
  {                       \
  } while (0)
#endif
void ui_close_all(void);
bool doOnceOnly_In_Mode2;

// =======================================================================
// FUNCTION PROTOTYPES FOR main.c
// =======================================================================
int touch_checkHB9IIU(void);
int HB9II_mode;
bool targetting_mode = true;
uint32_t target_frequency;
// ---- Sweep Engine ----
static bool sweep(bool break_on_operation, uint16_t ch_mask);

// ---- MLA Autotune ----
static void mla_update_good_dip_flag(void);
static void mla_update_Q_and_band(void);
static void compute_first_resonance(void);

// ---- Serial Command Handlers ----
static void cmd_reson(int argc, char *argv[]);

// ---- Thread Entry Point ----
static THD_FUNCTION(Thread1, arg);

// DANIEL: simple welcome / splash screen (NanoVNA-H4 compatible)
static void draw_welcome_screen(void)
{
  // Set colors (palette index 0 = black, 1 = white)
  lcd_set_background(LCD_BG_COLOR);
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_foreground(LCD_TRACE_3_COLOR); // green (default palette) :contentReference[oaicite:3]{index=3}

  // Clear screen
  lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);

  // Draw text
  lcd_drawstring_size("MLA ToolBox Project", 50, 40, 3);
  chThdSleepMilliseconds(500);
  lcd_set_foreground(LCD_TRACE_1_COLOR); // green (default palette) :contentReference[oaicite:3]{index=3}

  lcd_drawstring_size("'Find the dip. Trust the dip.'", 60, 120, 2);
  chThdSleepMilliseconds(500);

  lcd_set_foreground(LCD_FG_COLOR);

  lcd_drawstring_size("Version: BETA 1.0", 135, 210, 2);
  lcd_drawstring_size("By HB9IIU", 180, 245, 2);

  // Pause 2 seconds
  chThdSleepMilliseconds(2000);
}

// From ui.c
// extern void auto_scale_current_trace(void);

// ui.c: autoscale the currently selected trace
extern void menu_auto_scale_cb(int item);

static BaseSequentialStream *shell_stream = 0;
threads_queue_t shell_thread;

// Shell new line
#define VNA_SHELL_NEWLINE_STR "\r\n"
// Shell command promt
#define VNA_SHELL_PROMPT_STR "ch> "
// Shell max arguments
#define VNA_SHELL_MAX_ARGUMENTS 4
// Shell max command line size
#define VNA_SHELL_MAX_LENGTH 64
// Shell frequency printf format
// #define VNA_FREQ_FMT_STR         "%lu"
#define VNA_FREQ_FMT_STR "%u"

// Shell command functions prototypes
typedef void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) \
  static void command_name(int argc, char *argv[])

// Shell command line buffer, args, nargs, and function ptr
static char shell_line[VNA_SHELL_MAX_LENGTH];
static char *shell_args[VNA_SHELL_MAX_ARGUMENTS + 1];
static uint16_t shell_nargs;
static volatile vna_shellcmd_t shell_function = 0;

#define ENABLED_DUMP_COMMAND
// Allow get threads debug info
// #define ENABLE_THREADS_COMMAND
// Enable vbat_offset command, allow change battery voltage correction in config
#define ENABLE_VBAT_OFFSET_COMMAND
// Info about NanoVNA, need fore soft
#define ENABLE_INFO_COMMAND
// Enable color command, allow change config color for traces, grid, menu
#define ENABLE_COLOR_COMMAND
// Enable transform command
#define ENABLE_TRANSFORM_COMMAND
// Enable sample command
// #define ENABLE_SAMPLE_COMMAND
// Enable I2C command for send data to AIC3204, used for debug
// #define ENABLE_I2C_COMMAND
// Enable LCD command for send data to LCD screen, used for debug
// #define ENABLE_LCD_COMMAND
// Enable output debug data on screen on hard fault
// #define ENABLE_HARD_FAULT_HANDLER_DEBUG
// Enable test command, used for debug
// #define ENABLE_TEST_COMMAND
// Enable stat command, used for debug
// #define ENABLE_STAT_COMMAND
// Enable gain command, used for debug
// #define ENABLE_GAIN_COMMAND
// Enable port command, used for debug
// #define ENABLE_PORT_COMMAND
// Enable si5351 register write, used for debug
// #define ENABLE_SI5351_REG_WRITE
// Enable i2c timing command, used for debug
// #define ENABLE_I2C_TIMINGS
// Enable band setting command, used for debug
// #define ENABLE_BAND_COMMAND
// Enable scan_bin command (need use ex scan in future)
#define ENABLE_SCANBIN_COMMAND
// Enable debug for console command
// #define DEBUG_CONSOLE_SHOW
// Enable usart command
#define ENABLE_USART_COMMAND
// Enable config command
#define ENABLE_CONFIG_COMMAND
#ifdef __USE_SD_CARD__
// Enable SD card console command
#define ENABLE_SD_CARD_COMMAND
#endif

static void apply_CH0_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2]);
static void apply_CH1_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2]);
static void cal_interpolate(int idx, freq_t f, float data[CAL_TYPE_COUNT][2]);
static uint16_t get_sweep_mask(void);
static void update_frequencies(void);
static int set_frequency(freq_t freq);
static void set_frequencies(freq_t start, freq_t stop, uint16_t points);
static bool sweep(bool break_on_operation, uint16_t ch_mask);
static void transform_domain(uint16_t ch_mask);

freq_t getFrequency(uint16_t idx);
int shell_printf(const char *fmt, ...);

// We will use this to return to the original wide sweep
void reset_sweep_frequency(void);

// Index and frequency of the detected resonance dip.
// mla_res_idx = -1 → no usable resonance detected.
int16_t mla_res_idx = -1;
freq_t mla_res_freq = 0;
float mla_res_swr = 0.0f;

// True if the detected dip is deep enough to count as a real resonance.
// (Set by mla_update_good_dip_flag().)
static bool mla_has_good_resonance = false;

// ---------------------------------------------------------------------------
// MLA: Calculate resonance after a completed sweep (fixed span continuous mode)
//
// Resonance is defined here as the sweep point with the minimum |Γ| on CH0 (S11),
// where Γ = measured[0][i] = {re, im}.
// Updates existing globals:
//   - mla_res_idx  (index of best match, -1 if none)
//   - mla_res_freq (frequency in Hz at that index, 0 if none)
// Also logs the result in the existing [MLA] console style.
// ---------------------------------------------------------------------------
static void mla_calc_resonance_after_sweep(void)
{
  int best = -1;
  float best_mag2 = 0.0f;

  for (int i = 0; i < sweep_points; i++)
  {
    const freq_t f = getFrequency((uint16_t)i);
    if (f == 0)
      continue;

    const float re = measured[0][i][0];
    const float im = measured[0][i][1];
    const float mag2 = (re * re) + (im * im);

    if (best < 0 || mag2 < best_mag2)
    {
      best = i;
      best_mag2 = mag2;
    }
  }

  if (best >= 0)
  {
    mla_res_idx = (int16_t)best;
    mla_res_freq = getFrequency((uint16_t)best);

    MLA_UART_LOG("[MLA] Resonance Freq: " VNA_FREQ_FMT_STR " Hz (idx=%d)\r\n",
                 (unsigned long)mla_res_freq,
                 (int)mla_res_idx);
  }
  else
  {
    mla_res_idx = -1;
    mla_res_freq = 0;

    MLA_UART_LOG("[MLA] Resonance Freq: none\r\n");
  }
}

// -----------------------------------------------------------------------
// 2. RESONANCE STABILITY CHECKING
// Ensures that a dip remains stable across multiple sweeps.
// -----------------------------------------------------------------------

// Once true, the system stops hunting for other dips.
bool mla_resonance_locked = false;

// Number of consecutive sweeps that stayed within drift tolerance.
static uint8_t mla_res_stable_count = 0;

// Frequency used as stability reference.
static freq_t mla_res_ref_freq = 0;

// Requirements for stability.
#define MLA_STABLE_SWEEPS_REQUIRED 3
#define MLA_RESONANCE_STABILITY_HZ 20000U // 20 kHz max drift

// -----------------------------------------------------------------------
// 3. NO-RESONANCE DETECTION
// Detects missing antenna or completely flat response.
// -----------------------------------------------------------------------
static uint8_t mla_no_dip_count = 0;
#define MLA_NO_DIP_LIMIT 2

// -----------------------------------------------------------------------
// 4. AUTO-ZOOM / REFINEMENT STATE MACHINE
// Stages:
//   0 → wide sweep
//   1 → narrow sweep
//   2 → bandwidth (SWR2) sweep
//   3 → autoscale + finalize
// -----------------------------------------------------------------------
static uint8_t mla_autozone_stage = 0;
static bool mla_autoscale_pending = false;
bool drawTheSWRLines = false;

// -----------------------------------------------------------------------
// 5. BANDWIDTH & Q-FACTOR ESTIMATION
// Computed after final resonance lock.
// -----------------------------------------------------------------------
static freq_t mla_bw_low = 0;  // SWR<=2 lower edge
static freq_t mla_bw_high = 0; // SWR<=2 upper edge
                               // static freq_t mla_bw_swr2 = 0; // BW in Hz (mla_bw_high - mla_bw_low)
freq_t mla_bw_swr2 = 0;        // BW in Hz (mla_bw_high - mla_bw_low)

static float mla_Q_factor = 0.0f;      // Q = f_res / BW
static float mla_bw_rel = 0.0f;        // Relative BW = BW / f_res
static char mla_band_name[12] = "n/a"; // Simple band label (“40m”, etc.)

// Sweep operating mode (SWEEP_ENABLE = continuous sweep).
uint8_t sweep_mode = SWEEP_ENABLE;

// Sweep point index while collecting data.
static uint16_t p_sweep = 0;

// Complex measurement buffer:
// measured[ch][point][0] = real
// measured[ch][point][1] = imag
// ch=0: S11, ch=1: S21
float measured[2][SWEEP_POINTS_MAX][2];

// =======================================================================
// CALIBRATION & DSP (REAL VARIABLES ONLY)
// Error-correction parameters for S11/S21 processing.
// DO NOT touch macro-based calibration arrays in current_props.
// =======================================================================

// Additional phase delay correction.
float electrical_delay = 0.0f;

// S11 (reflection) correction terms.
float es = 0.0f;  // Source match
float esd = 0.0f; // Directivity
float ed = 0.0f;  // Reflection tracking
float e00 = 0.0f; // Reflection offset

// S21 (transmission) correction terms.
float et = 0.0f;  // Transmission tracking
float ese = 0.0f; // Transmission leakage/isolation

// Calibration level flag (0 = off, 1 = full).
uint16_t cal_level = 0;

// =======================================================================
// UI / TOUCH / USER INTERACTION STATE
// Touchscreen state, UI modes, marker handling, trace selection.
// =======================================================================

// Redraw request bitmask (0 = none).
uint8_t redraw_request = 0;

// Last detected touch coordinates.
int16_t touch_x = -1;
int16_t touch_y = -1;

// Last stable touch coordinates.
int16_t last_touch_x = -1;
int16_t last_touch_y = -1;

// Touch state (0=none, 1=pressed, 2=dragging).
uint8_t touch_status = 0;

// UI mode (menu, sweep, config, markers...).
uint8_t ui_mode = 0;

// Plot markers and trace selection.
uint8_t marker_count = 0;          // Number of active markers
uint8_t measurement_selection = 0; // S11/S21/etc.
uint8_t trace_selection = 0;       // Which trace is active

// -----------------------------------------------------------------------
// MLA bandwidth/Q thresholds (static classification rules)
// -----------------------------------------------------------------------

// SWR2 bandwidth thresholds for sanity checking.
#define MLA_BW_TOO_NARROW_HZ 3000U // < 3 kHz → suspiciously sharp
#define MLA_BW_TOO_WIDE_HZ 300000U // > 300 kHz → too wide/flat

// Q-factor classification ranges.
#define MLA_Q_TOO_LOW 20.0f
#define MLA_Q_LOW 50.0f
#define MLA_Q_GOOD 150.0f
#define MLA_Q_HIGH 400.0f
#define MLA_Q_TOO_HIGH 1200.0f

// HB9IIU
// ---------------------------------------------------------------------------
// MLA HB9IIU: generic resonance finder on S11 (CH0)
// ---------------------------------------------------------------------------
// Returns true if a minimum was found, false otherwise.
// search_start / search_stop in Hz:
//   - if both 0 -> use full sweep (except edges)
//   - otherwise restrict search to [search_start, search_stop],
//     clipped to the actual sweep range.
// ---------------------------------------------------------------------------
static bool getResonanceHB9IIU(freq_t search_start,
                               freq_t search_stop,
                               freq_t *out_freq,
                               uint16_t *out_idx,
                               float *out_mag2)
{
  if (sweep_points < 3)
    return false;

  uint16_t i_start = 1;              // skip index 0 (edge)
  uint16_t i_end = sweep_points - 2; // skip last index (edge)

  // If caller gave a search window, map it to index range
  if (search_start != 0 || search_stop != 0)
  {
    freq_t f0 = getFrequency(0);
    freq_t f1 = getFrequency(sweep_points - 1);

    if (search_start == 0)
      search_start = f0;
    if (search_stop == 0)
      search_stop = f1;

    if (search_start < f0)
      search_start = f0;
    if (search_stop > f1)
      search_stop = f1;

    // Move i_start forward until we are inside the window
    while (i_start < i_end && getFrequency(i_start) < search_start)
      i_start++;

    // Move i_end backward until we are inside the window
    while (i_end > i_start && getFrequency(i_end) > search_stop)
      i_end--;
  }

  float best_mag2 = 1e30f;
  uint16_t best_idx = i_start;
  bool found = false;

  for (uint16_t i = i_start; i <= i_end; i++)
  {
    float re = measured[0][i][0];
    float im = measured[0][i][1];
    float mag2 = re * re + im * im;

    // Ignore completely zero data (uninitialized / invalid)
    if (mag2 == 0.0f)
      continue;

    if (!found || mag2 < best_mag2)
    {
      best_mag2 = mag2;
      best_idx = i;
      found = true;
    }
  }

  if (!found)
    return false;

  if (out_freq)
    *out_freq = getFrequency(best_idx);
  if (out_idx)
    *out_idx = best_idx;
  if (out_mag2)
    *out_mag2 = best_mag2;

  return true;
}

// ---------------------------------------------------------------------------
// Convert complex S11 at index i into SWR
// SWR = (1 + |Γ|) / (1 - |Γ|),   Γ = S11
// If |Γ| is invalid or >= 1, return a very large SWR.
// ---------------------------------------------------------------------------
static float getSWRfromIndex(uint16_t i)
{
  if (i >= sweep_points)
    return 9999.0f;

  float re = measured[0][i][0];
  float im = measured[0][i][1];
  float mag2 = re * re + im * im;

  if (mag2 <= 0.0f)
    return 9999.0f;

  float mag = vna_sqrtf(mag2); // |Γ|

  if (mag >= 0.9999f)
    return 9999.0f;

  return (1.0f + mag) / (1.0f - mag);
}

// ---------------------------------------------------------------------------
// MLA: Calculate SWR at the current resonance index after a completed sweep
//
// Requires mla_res_idx to already be valid (e.g. set by mla_calc_resonance_after_sweep()).
// Logs in the same [MLA] console style.
// Returns: SWR value, or 0.0f if invalid.
// ---------------------------------------------------------------------------

static float mla_calc_swr_at_resonance_after_sweep(void)
{
  // Need a valid resonance index
  if (mla_res_idx < 0 || mla_res_idx >= (int16_t)sweep_points)
  {
    MLA_UART_LOG("[MLA] Resonance SWR: invalid resonance index (%d)\r\n", (int)mla_res_idx);
    return 0.0f;
  }

  float swr = getSWRfromIndex((uint16_t)mla_res_idx);

  // Basic sanity (similar limits used elsewhere in your MLA code)
  if (swr <= 0.0f || swr > 50.0f)
  {
    MLA_UART_LOG("[MLA] Resonance SWR: invalid (%.2f) at idx=%d\r\n",
                 swr, (int)mla_res_idx);
    return 0.0f;
  }

  MLA_UART_LOG("[MLA] Resonance SWR: %.2f (idx=%d)\r\n", swr, (int)mla_res_idx);
  return swr;
}

// ---------------------------------------------------------------------------
// MLA: check if the current sweep has a "good" S11 dip
//
// Uses ONLY MLA_UART_LOG, same as the other MLA messages.
// No new printf functions, no new includes.
// ---------------------------------------------------------------------------
static void mla_update_good_dip_flag(void)
{
  freq_t f_res;
  uint16_t idx;
  float mag2;

  // Try to find the best resonance (minimum |S11|^2)
  if (!getResonanceHB9IIU(0, 0, &f_res, &idx, &mag2))
  {
    mla_has_good_resonance = false;
    MLA_UART_LOG("[MLA][good_dip] no resonance found (getResonanceHB9IIU failed)\r\n");
    return;
  }

  float swr_min = getSWRfromIndex(idx);

  // Sanity: if SWR is completely nonsense, bail out
  if (swr_min <= 0.0f || swr_min > 50.0f)
  {
    mla_has_good_resonance = false;
    MLA_UART_LOG("[MLA][good_dip] invalid SWR at resonance -> NO CLEAR RESONANCE\r\n");
    return;
  }

  // Find the maximum SWR over the sweep
  float max_swr = 0.0f;
  for (uint16_t i = 0; i < sweep_points; i++)
  {
    float swr = getSWRfromIndex(i);
    if (swr > 0.0f && swr < 9999.0f && swr > max_swr)
      max_swr = swr;
  }

  // If everything is flat (no contrast), treat as no dip
  if (max_swr <= 0.0f)
  {
    mla_has_good_resonance = false;
    MLA_UART_LOG("[MLA][good_dip] flat SWR curve -> NO CLEAR RESONANCE\r\n");
    return;
  }

  float ratio = max_swr / swr_min;

  // Heuristic thresholds:
  //   ratio < 1.5  → very shallow / noisy, ignore
  //   swr_min > 10 → "best point" is still terrible, ignore
  if (ratio < 1.5f || swr_min > 10.0f)
  {
    mla_has_good_resonance = false;
    MLA_UART_LOG("[MLA][good_dip] NO CLEAR RESONANCE\r\n");
  }
  else
  {
    mla_has_good_resonance = true;
    MLA_UART_LOG("[MLA][good_dip] GOOD DIP DETECTED\r\n");
  }
}

// ---------------------------------------------------------------------------
// MLA: compute bandwidth around resonance where SWR <= 2.0
// Updates:
//   mla_bw_low, mla_bw_high, mla_bw_swr2
// Logs result in single-line [MLA] style.
//
// This version adds linear interpolation at the SWR=limit crossings:
// - Left edge: between first point >limit and last point <=limit
// - Right edge: between last point <=limit and first point >limit
// ---------------------------------------------------------------------------
static void mla_update_bandwidth_swr2(void)
{
  const float swr_bw_limit = 2.0f;

  // Default outputs
  mla_bw_low = 0;
  mla_bw_high = 0;
  mla_bw_swr2 = 0;

  // Need a valid resonance lock and center index
  if (!mla_resonance_locked)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (not locked)\r\n", swr_bw_limit);
    return;
  }

  if (sweep_points < 3)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (sweep_points=%u)\r\n",
                 swr_bw_limit, sweep_points);
    return;
  }

  uint16_t center = (uint16_t)mla_res_idx;
  if (center >= sweep_points)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (center idx out of range: %u)\r\n",
                 swr_bw_limit, center);
    return;
  }

  // Sanity check at center
  float swr_center = getSWRfromIndex(center);
  if (swr_center <= 0.0f || swr_center > 50.0f)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (bad center SWR=%.2f idx=%u)\r\n",
                 swr_bw_limit, swr_center, center);
    return;
  }

  // --- Search left (contiguous region around center) ---
  int16_t left_idx = (int16_t)center;
  int16_t last_good_left = -1;
  while (left_idx >= 0)
  {
    float swr = getSWRfromIndex((uint16_t)left_idx);
    if (swr > 0.0f && swr <= swr_bw_limit)
    {
      last_good_left = left_idx;
      left_idx--;
    }
    else
    {
      break; // first "bad" point (SWR>limit or invalid)
    }
  }
  // After loop:
  // last_good_left = last index with SWR<=limit
  // left_idx       = first index just outside band (SWR>limit) OR -1

  // --- Search right (contiguous region around center) ---
  int16_t right_idx = (int16_t)center;
  int16_t last_good_right = -1;
  while (right_idx < (int16_t)sweep_points)
  {
    float swr = getSWRfromIndex((uint16_t)right_idx);
    if (swr > 0.0f && swr <= swr_bw_limit)
    {
      last_good_right = right_idx;
      right_idx++;
    }
    else
    {
      break; // first "bad" point
    }
  }
  // After loop:
  // last_good_right = last index with SWR<=limit
  // right_idx       = first index just outside band (SWR>limit) OR sweep_points

  if (last_good_left < 0 || last_good_right < 0 || last_good_right <= last_good_left)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (left=%d right=%d)\r\n",
                 swr_bw_limit, last_good_left, last_good_right);
    return;
  }

  // --------- Interpolate LEFT edge frequency (SWR crosses limit) ----------
  // Good point is last_good_left (<=limit)
  // Bad point is last_good_left-1 (== left_idx) if available and valid
  float f_low_f = (float)getFrequency((uint16_t)last_good_left);

  if (last_good_left > 0)
  {
    int16_t bad_i = last_good_left - 1;
    float swr_bad = getSWRfromIndex((uint16_t)bad_i);
    float swr_good = getSWRfromIndex((uint16_t)last_good_left);

    // We only interpolate when we truly bracket the limit:
    // swr_bad  > limit
    // swr_good <= limit
    if (swr_bad > swr_bw_limit && swr_good > 0.0f && swr_good <= swr_bw_limit && swr_bad != swr_good)
    {
      float f_bad = (float)getFrequency((uint16_t)bad_i);
      float f_good = (float)getFrequency((uint16_t)last_good_left);

      // Linear interpolation in SWR vs frequency
      float t = (swr_bw_limit - swr_bad) / (swr_good - swr_bad); // 0..1
      f_low_f = f_bad + t * (f_good - f_bad);
    }
  }

  // --------- Interpolate RIGHT edge frequency (SWR crosses limit) ----------
  // Good point is last_good_right (<=limit)
  // Bad point is last_good_right+1 (== right_idx) if available and valid
  float f_high_f = (float)getFrequency((uint16_t)last_good_right);

  if (last_good_right < (int16_t)sweep_points - 1)
  {
    int16_t bad_i = last_good_right + 1;
    float swr_bad = getSWRfromIndex((uint16_t)bad_i);
    float swr_good = getSWRfromIndex((uint16_t)last_good_right);

    if (swr_bad > swr_bw_limit && swr_good > 0.0f && swr_good <= swr_bw_limit && swr_bad != swr_good)
    {
      float f_bad = (float)getFrequency((uint16_t)bad_i);
      float f_good = (float)getFrequency((uint16_t)last_good_right);

      float t = (swr_bw_limit - swr_good) / (swr_bad - swr_good); // 0..1
      f_high_f = f_good + t * (f_bad - f_good);
    }
  }

  // Convert interpolated floats back to integer Hz (rounded)
  mla_bw_low = (freq_t)(f_low_f + 0.5f);
  mla_bw_high = (freq_t)(f_high_f + 0.5f);

  if (mla_bw_high <= mla_bw_low)
  {
    MLA_UART_LOG("[MLA] BW(SWR<=%.1f): n/a (f_high<=f_low)\r\n", swr_bw_limit);
    mla_bw_low = mla_bw_high = mla_bw_swr2 = 0;
    return;
  }

  mla_bw_swr2 = mla_bw_high - mla_bw_low;
  const float bw_khz = (float)mla_bw_swr2 / 1000.0f;

  // Keep your original log format (idx=... remain the last_good indices)
  MLA_UART_LOG("[MLA] BW(SWR<=%.1f): f_low=" VNA_FREQ_FMT_STR " Hz (idx=%d), "
               "f_high=" VNA_FREQ_FMT_STR " Hz (idx=%d), BW=" VNA_FREQ_FMT_STR " Hz (%.2f kHz)\r\n",
               swr_bw_limit,
               (unsigned long)mla_bw_low, last_good_left,
               (unsigned long)mla_bw_high, last_good_right,
               (unsigned long)mla_bw_swr2, bw_khz);
}

// ---------------------------------------------------------------------------
// MLA: Calculate Q based on BW where SWR <= 2.0  (Q@SWR2, not 3dB-Q)
//
// Uses:
//   - mla_res_freq  (Hz)  resonance frequency
//   - mla_bw_swr2   (Hz)  bandwidth where SWR <= 2.0
//
// Returns: Q@SWR2 (float). 0.0f if not available.
// ---------------------------------------------------------------------------
static float mla_calc_q_after_sweep(void)
{
  if (mla_res_freq == 0)
  {
    MLA_UART_LOG("[MLA] Q@SWR2: n/a (resonance freq not set)\r\n");
    return 0.0f;
  }
  if (mla_bw_swr2 == 0)
  {
    MLA_UART_LOG("[MLA] Q@SWR2: n/a (BW(SWR<=2)=0)\r\n");
    return 0.0f;
  }

  const float f0 = (float)mla_res_freq;
  const float bw = (float)mla_bw_swr2;

  const float q = f0 / bw;
  const float bw_over_f0 = (bw / f0) * 100.0f;

  MLA_UART_LOG("[MLA] Q@SWR2: f0=" VNA_FREQ_FMT_STR " Hz, BW(SWR<=2)=" VNA_FREQ_FMT_STR
               " Hz, Q=%.1f, BW/f0=%.3f %%\r\n",
               (unsigned long)mla_res_freq,
               (unsigned long)mla_bw_swr2,
               q,
               bw_over_f0);

  return q;
}

// ---------------------------------------------------------------------------
// MLA: Rate Q@SWR2 (heuristic)
// IMPORTANT: Uses Q computed from BW(SWR<=2), so label is Q@SWR2.
//
// Returns: rating string pointer ("EXCELLENT", ...), or "N/A".
// ---------------------------------------------------------------------------
static const char *mla_rate_q_after_sweep(void)
{
  const float q = mla_calc_q_after_sweep(); // compute once, reuse
  if (q <= 0.0f)
  {
    MLA_UART_LOG("[MLA] Q rating: N/A\r\n");
    return "N/A";
  }

  const char *rating;
  if (q >= 500.0f)
    rating = "EXCELLENT";
  else if (q >= 350.0f)
    rating = "VERY GOOD";
  else if (q >= 200.0f)
    rating = "GOOD";
  else if (q >= 100.0f)
    rating = "FAIR";
  else
    rating = "POOR";

  MLA_UART_LOG("[MLA] Q rating: %s (Q@SWR2=%.1f)\r\n", rating, q);
  return rating;
}

// ---------------------------------------------------------------------------
// MLA: compute Q-factor and HF band name from current resonance & BW(SWR<=2)
// Updates:
//   - mla_Q_factor
//   - mla_bw_rel
//   - mla_band_name[]
//   - Logs summary via MLA_UART_LOG()
// ---------------------------------------------------------------------------
static void mla_update_Q_and_band(void)
{
  // Default / error state
  mla_Q_factor = 0.0f;
  mla_bw_rel = 0.0f;
  strncpy(mla_band_name, "n/a", sizeof(mla_band_name));
  mla_band_name[sizeof(mla_band_name) - 1] = '\0';

  // Need valid resonance and bandwidth
  if (!mla_resonance_locked || mla_res_freq == 0 || mla_bw_swr2 == 0)
  {
    MLA_UART_LOG("[MLA][Q] no valid resonance/BW, skip Q & band calc\r\n");
    return;
  }

  const freq_t f0_hz = mla_res_freq;
  const freq_t bw_hz = mla_bw_swr2;

  if (f0_hz == 0 || bw_hz == 0)
  {
    MLA_UART_LOG("[MLA][Q] invalid f0 or BW\r\n");
    return;
  }

  float f0 = (float)f0_hz;
  float bw = (float)bw_hz;

  // Compute Q = f0 / BW and relative bandwidth
  mla_Q_factor = f0 / bw;
  mla_bw_rel = bw / f0; // dimensionless (~ 1/Q)

  // -----------------------------------------------------------------------
  // HF amateur band classification
  // -----------------------------------------------------------------------
  const char *name = NULL;

  if (f0_hz >= 1800000 && f0_hz <= 2000000)
    name = "160m";
  else if (f0_hz >= 3500000 && f0_hz <= 4000000)
    name = "80m";
  else if (f0_hz >= 5300000 && f0_hz <= 5500000)
    name = "60m";
  else if (f0_hz >= 7000000 && f0_hz <= 7300000)
    name = "40m";
  else if (f0_hz >= 10100000 && f0_hz <= 10150000)
    name = "30m";
  else if (f0_hz >= 13900000 && f0_hz <= 14350000)
    name = "20m"; // extended range
  else if (f0_hz >= 18068000 && f0_hz <= 18168000)
    name = "17m";
  else if (f0_hz >= 21000000 && f0_hz <= 21450000)
    name = "15m";
  else if (f0_hz >= 24890000 && f0_hz <= 24990000)
    name = "12m";
  else if (f0_hz >= 28000000 && f0_hz <= 29700000)
    name = "10m";
  else if (f0_hz >= 50000000 && f0_hz <= 54000000)
    name = "6m";
  else if (f0_hz >= 144000000 && f0_hz <= 148000000)
    name = "2m";
  else if (f0_hz >= 430000000 && f0_hz <= 440000000)
    name = "70cm";

  bool outside_band = false;

  if (name != NULL)
  {
    strncpy(mla_band_name, name, sizeof(mla_band_name));
  }
  else
  {
    strncpy(mla_band_name, "outside", sizeof(mla_band_name));
    outside_band = true;
  }
  mla_band_name[sizeof(mla_band_name) - 1] = '\0';

  // -----------------------------------------------------------------------
  // Log summary to UART
  // -----------------------------------------------------------------------
  if (outside_band)
  {
    MLA_UART_LOG("[MLA][Q] f0      = " VNA_FREQ_FMT_STR " Hz (outside band)\r\n", f0_hz);
  }
  else
  {
    MLA_UART_LOG("[MLA][Q] f0      = " VNA_FREQ_FMT_STR " Hz (%s band)\r\n", f0_hz, mla_band_name);
  }

  MLA_UART_LOG("[MLA][Q] BW(SWR<=2) = " VNA_FREQ_FMT_STR " Hz\r\n", bw_hz);
  MLA_UART_LOG("[MLA][Q] Q ≈ %.1f,  BW/f0 ≈ %.6f  (%.3f %%)\r\n",
               mla_Q_factor,
               mla_bw_rel,
               mla_bw_rel * 100.0f);

  // -----------------------------------------------------------------------
  // Optional: classify Q-factor quality for quick reference
  // -----------------------------------------------------------------------
  const char *q_class = "OK";
  if (bw_hz < MLA_BW_TOO_NARROW_HZ || mla_Q_factor > MLA_Q_TOO_HIGH)
    q_class = "ULTRA-NARROW";
  else if (mla_Q_factor >= MLA_Q_HIGH)
    q_class = "HIGH-Q";
  else if (mla_Q_factor >= MLA_Q_GOOD)
    q_class = "GOOD";
  else if (mla_Q_factor >= MLA_Q_LOW)
    q_class = "LOW-Q";
  else
    q_class = "VERY-LOW-Q";

  MLA_UART_LOG("[MLA][Q] class: %s\r\n", q_class);
}

bool mla_is_resonance_centered(void)
{
  const int EDGE_POINTS = sweep_points / 5; // aggressive, safe
  bool centered =
      (mla_res_idx > EDGE_POINTS) &&
      (mla_res_idx < (sweep_points - EDGE_POINTS));

  MLA_UART_LOG(
      "[MLA] centered=%d  res_idx=%d  sweep_points=%d  edge=%d\r\n",
      centered,
      mla_res_idx,
      sweep_points,
      EDGE_POINTS);

  return centered;
}

// -----------------------------------------------------------------------------
// Called once per completed sweep, after compute_first_resonance()
// If several consecutive sweeps show no valid resonance,
// display the "No Clear Resonance" popup.
// -----------------------------------------------------------------------------
void ui_show_no_resonance_hint(void);
void ui_hide_no_resonance_hint(void);

void ui_show_check_amtenna_hint(void);

void ui_hide_check_amtenna_hint(void);

// ---------------------------------------------------------------------------
// MLA HB9IIU: auto-zoom around first resonance and keep it centered
//
// Stage machine:
//   0 -> 1 : wide sweep → fixed 0.5 MHz span around resonance
//   1 -> 2 : SWR-based window, then *symmetric* around resonance index
//   2 -> 3 : one-shot AUTO SCALE (S11 + SWR) on final window
// ---------------------------------------------------------------------------
static void compute_first_resonance(void)
{
  // -------------------------------------------------------------
  // STAGE 2: Final sweep finished → autoscale and done
  // -------------------------------------------------------------

  if (mla_autozone_stage == 2)
  {
    // LOG ONLY TO UART (if enabled), NO USB SHELL
    MLA_UART_LOG("[MLA] stage 2: applying AUTO SCALE on final sweep\r\n");

    // Instead of running autoscale here, just mark it as pending.
    // It will be executed in Thread1 after the sweep is completed.
    mla_autoscale_pending = true;

    // After final, narrow sweep:
    // 1) compute and log SWR<=2 bandwidth
    // 2) compute and log Q-factor and band name
    mla_update_bandwidth_swr2();
    mla_update_Q_and_band();

    request_to_redraw(REDRAW_BACKUP | REDRAW_AREA); // ensure full screen refresh

    mla_autozone_stage = 3; // fully done
    return;
  }

  // After final stage, do nothing
  if (mla_autozone_stage >= 3)
    return;

  freq_t f_res = 0;
  uint16_t idx = 0;
  float mag2 = 0.0f;

  // Always work on the *current* sweep data
  if (!getResonanceHB9IIU(0, 0, &f_res, &idx, &mag2))
    return;

  // Save result for later (cmd_reson, display, etc.)
  mla_res_freq = f_res;
  mla_res_idx = idx;
  mla_resonance_locked = true;

  // Current sweep window
  freq_t cur_start = get_sweep_frequency(ST_START);
  freq_t cur_stop = get_sweep_frequency(ST_STOP);

  // -------- Log only via UART (optional) ----------
  MLA_UART_LOG("[MLA] resonance result (stage %u):\r\n", mla_autozone_stage);
  MLA_UART_LOG("  sweep_points = %u\r\n", sweep_points);
  MLA_UART_LOG("  f_start      = " VNA_FREQ_FMT_STR " Hz\r\n", cur_start);
  MLA_UART_LOG("  f_stop       = " VNA_FREQ_FMT_STR " Hz\r\n", cur_stop);
  MLA_UART_LOG("  best_idx     = %u\r\n", idx);
  MLA_UART_LOG("  f_res        = " VNA_FREQ_FMT_STR " Hz\r\n", f_res);
  MLA_UART_LOG("  |S11|^2      = %f\r\n", mag2);

  // -------------------------------------------------------------
  // STAGE 0: Wide sweep → center and narrow with fixed span
  // -------------------------------------------------------------
  if (mla_autozone_stage == 0)
  {
    const freq_t span = 500000; // 0.5 MHz total
    const freq_t half = span / 2;

    freq_t start = (f_res > half) ? (f_res - half) : FREQUENCY_MIN;
    freq_t stop = start + span;

    if (stop > FREQUENCY_MAX)
      stop = FREQUENCY_MAX;
    if (start >= stop)
      return;

    // UART-only debug
    MLA_UART_LOG(
        "[MLA][stage 0 → 1] fixed-span auto-zoom:\r\n"
        "  span        = " VNA_FREQ_FMT_STR " Hz\r\n"
        "  new sweep   = " VNA_FREQ_FMT_STR " Hz .. " VNA_FREQ_FMT_STR " Hz\r\n"
        "  center_f    = " VNA_FREQ_FMT_STR " Hz\r\n",
        span, start, stop, f_res);

    // Apply first (coarse) auto-zoom: this triggers a new sweep
    set_sweep_frequency(ST_START, start);
    set_sweep_frequency(ST_STOP, stop);

    mla_autozone_stage = 1;
    return;
  }

  // -------------------------------------------------------------
  // STAGE 1: Pre-centered narrow sweep → SWR-limited, symmetric window
  // -------------------------------------------------------------
  if (mla_autozone_stage == 1)
  {
    const float swr_limit = 3.0f; // LIMIT for SWR-based window  XXXX
    uint16_t left = idx;
    uint16_t right = idx;

    float swr_center = getSWRfromIndex(idx);

    // Expand to the left while SWR <= limit
    while (left > 0)
    {
      float swr = getSWRfromIndex(left - 1);
      if (swr > swr_limit)
        break;
      left--;
    }

    // Expand to the right while SWR <= limit
    while (right + 1 < sweep_points)
    {
      float swr = getSWRfromIndex(right + 1);
      if (swr > swr_limit)
        break;
      right++;
    }

    // Optional: add one extra bin of padding on each side if possible
    if (left > 0)
      left--;
    if (right + 1 < sweep_points)
      right++;

    // Make sweep symmetric around resonance index (same logic you already had)
    int left_span = (int)idx - (int)left;
    int right_span = (int)right - (int)idx;
    int half_span = (left_span > right_span) ? left_span : right_span;

    int sym_left = (int)idx - half_span;
    int sym_right = (int)idx + half_span;

    if (sym_left < 0)
      sym_left = 0;
    if (sym_right >= (int)sweep_points)
      sym_right = (int)sweep_points - 1;
    if (sym_left >= sym_right)
      return;

    freq_t start = getFrequency((uint16_t)sym_left);
    freq_t stop = getFrequency((uint16_t)sym_right);

    if (start < FREQUENCY_MIN)
      start = FREQUENCY_MIN;
    if (stop > FREQUENCY_MAX)
      stop = FREQUENCY_MAX;
    if (start >= stop)
      return;

    float swr_left = getSWRfromIndex((uint16_t)sym_left);
    float swr_right = getSWRfromIndex((uint16_t)sym_right);

    // UART-only debug
    MLA_UART_LOG(
        "[MLA] stage 1 → 2: SWR-based symmetric zoom (limit=%.2f):\r\n"
        "  left_idx    = %u (f = " VNA_FREQ_FMT_STR " Hz, SWR=%.2f)\r\n"
        "  center_idx  = %u (f = " VNA_FREQ_FMT_STR " Hz, SWR=%.2f)\r\n"
        "  right_idx   = %u (f = " VNA_FREQ_FMT_STR " Hz, SWR=%.2f)\r\n"
        "  final sweep = " VNA_FREQ_FMT_STR " Hz .. " VNA_FREQ_FMT_STR " Hz\r\n",
        swr_limit,
        (uint16_t)sym_left, start, swr_left,
        idx, f_res, swr_center,
        (uint16_t)sym_right, stop, swr_right,
        start, stop);

    set_sweep_frequency(ST_START, start);
    set_sweep_frequency(ST_STOP, stop);

    // Done: next sweep will be "final" (stage 2)
    mla_autozone_stage = 2;
    return;
  }
}

#undef VERSION
#define VERSION "Beta 28.12.2025"

// Version text, displayed in Config->Version menu, also sent by info command
const char *info_about[] = {
    "Dummy Line",
    "Based on 2019-2024 NanoVNA-D by DiSlord",
    "(original work by edy555)",
    "Status: Work In Progress",
    "Licensed under GPL.",
    "Source code will be distributed once polished, and readable.",
    "Version: " VERSION,
    "Build Time: " __DATE__ " - " __TIME__,
    "Kernel: " CH_KERNEL_VERSION,
    "Compiler: " PORT_COMPILER_NAME,
    "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
    "Port Info: " PORT_INFO,
    "Platform: " PLATFORM_NAME,

    0 // sentinel
};

// Allow draw some debug on LCD
#ifdef DEBUG_CONSOLE_SHOW
void my_debug_log(int offs, char *log)
{
  static uint16_t shell_line_y = 0;
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(FREQUENCIES_XPOS1, shell_line_y, LCD_WIDTH - FREQUENCIES_XPOS1, 2 * FONT_GET_HEIGHT);
  lcd_drawstring(FREQUENCIES_XPOS1 + offs, shell_line_y, log);
  shell_line_y += FONT_STR_HEIGHT;
  if (shell_line_y >= LCD_HEIGHT - FONT_STR_HEIGHT * 4)
    shell_line_y = 0;
}
#define DEBUG_LOG(offs, text) my_debug_log(offs, text);
#else
#define DEBUG_LOG(offs, text)
#endif

#ifdef __USE_SMOOTH__
static float arifmetic_mean(float v0, float v1, float v2)
{
  return (v0 + 2 * v1 + v2) / 4;
}

static float geometry_mean(float v0, float v1, float v2)
{
  float v = vna_cbrtf(vna_fabsf(v0 * v1 * v2));
  if (v0 + v1 + v2 < 0)
    v = -v;
  return v;
}

uint8_t smooth_factor = 0;
void set_smooth_factor(uint8_t factor)
{
  if (factor > 8)
    factor = 8;
  smooth_factor = factor;
  request_to_redraw(REDRAW_CAL_STATUS);
}

uint8_t get_smooth_factor(void)
{
  return smooth_factor;
}

// Allow smooth complex data point array (this remove noise, smooth power depend form count)
// see https://terpconnect.umd.edu/~toh/spectrum/Smoothing.html
static void measurementDataSmooth(uint16_t ch_mask)
{
  int j;
  //  ch_mask = 2;
  //  memcpy(measured[0], measured[1], sizeof(measured[0]));
  float (*smooth_func)(float v0, float v1, float v2) = VNA_MODE(VNA_MODE_SMOOTH) ? arifmetic_mean : geometry_mean;
  for (int ch = 0; ch < 2; ch++, ch_mask >>= 1)
  {
    if ((ch_mask & 1) == 0)
      continue;
    int count = 1 << (smooth_factor - 1), n;
    float *data = measured[ch][0];
    for (n = 0; n < count; n++)
    {
      float prev_re = data[2 * 0];
      float prev_im = data[2 * 0 + 1];
      // first point smooth (use first and second points), disabled it made phase shift
      //      data[0] = smooth_func(prev_re, prev_re, data[2  ]);
      //      data[1] = smooth_func(prev_im, prev_im, data[2+1]);
      // simple data smooth on 3 points
      for (j = 1; j < sweep_points - 1; j++)
      {
        float old_re = data[2 * j]; // save current data point for next point smooth
        float old_im = data[2 * j + 1];
        data[2 * j] = smooth_func(prev_re, data[2 * j], data[2 * j + 2]);
        data[2 * j + 1] = smooth_func(prev_im, data[2 * j + 1], data[2 * j + 3]);
        prev_re = old_re;
        prev_im = old_im;
      }
      // last point smooth, disabled it made phase shift
      //      data[2*j  ] = smooth_func(data[2*j  ], data[2*j  ], prev_re);
      //      data[2*j+1] = smooth_func(data[2*j+1], data[2*j+1], prev_im);
    }
  }
}
#endif

static THD_WORKING_AREA(waThread1, 1024);

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#define SWR_MIN 1.0f
#define SWR_MAX 3.5f

#define BAR_SEGMENTS 40
#define BAR_GAP_PX 2

#define BAR_HEIGHT_PX 45
#define BAR_Y_POS (LCD_HEIGHT * 3 / 4)

#define BAR_LEFT_MARGIN 20
#define BAR_RIGHT_MARGIN 20

void drawBarGraph(float value)
{
  static int prevActive = -1; // force init on first call
  if (doOnceOnly_In_Mode2 == true)
  {
    prevActive = -1;
  }

  if (value < SWR_MIN)
    value = SWR_MIN;
  if (value > SWR_MAX)
    value = SWR_MAX;

  int barWidth = LCD_WIDTH - BAR_LEFT_MARGIN - BAR_RIGHT_MARGIN;
  int segmentWidth =
      (barWidth - (BAR_SEGMENTS - 1) * BAR_GAP_PX) / BAR_SEGMENTS;

  int x0 = BAR_LEFT_MARGIN;
  int y0 = BAR_Y_POS;

  /* compute active segments */
  float norm = (value - SWR_MIN) / (SWR_MAX - SWR_MIN);
  int active = (int)(norm * BAR_SEGMENTS + 0.5f);
  if (active < 0)
    active = 0;
  if (active > BAR_SEGMENTS)
    active = BAR_SEGMENTS;

  /* first call: clear bar and draw frame ONCE */
  if (prevActive < 0)
  {
    /* clear bar area */
    lcd_set_colors(LCD_BG_COLOR, LCD_BG_COLOR);
    lcd_fill(x0, y0, barWidth, BAR_HEIGHT_PX);

    /* draw 1px frame using fills */
    lcd_set_colors(LCD_BG_COLOR, LCD_GRID_COLOR);

    /* top */
    lcd_fill(x0 - 1, y0 - 1, barWidth + 2, 1);
    /* bottom */
    lcd_fill(x0 - 1, y0 + BAR_HEIGHT_PX, barWidth + 2, 1);
    /* left */
    lcd_fill(x0 - 1, y0 - 1, 1, BAR_HEIGHT_PX + 2);
    /* right */
    lcd_fill(x0 + barWidth, y0 - 1, 1, BAR_HEIGHT_PX + 2);

    prevActive = 0;
  }

  /* value increased → draw new segments */
  if (active > prevActive)
  {
    for (int i = prevActive; i < active; i++)
    {
      float segValue =
          SWR_MIN +
          ((float)(i + 1) / BAR_SEGMENTS) * (SWR_MAX - SWR_MIN);

      uint16_t color;
      if (segValue <= 1.5f)
        color = LCD_TRACE_3_COLOR; // green
      else if (segValue <= 3.0f)
        color = LCD_TRACE_1_COLOR; // yellow
      else
        color = LCD_TRACE_5_COLOR; // red

      int x = x0 + i * (segmentWidth + BAR_GAP_PX);

      lcd_set_colors(LCD_BG_COLOR, color);
      lcd_fill(x, y0, segmentWidth, BAR_HEIGHT_PX);
    }
  }
  /* value decreased → clear removed segments */
  else if (active < prevActive)
  {
    for (int i = active; i < prevActive; i++)
    {
      int x = x0 + i * (segmentWidth + BAR_GAP_PX);

      lcd_set_colors(LCD_BG_COLOR, LCD_BG_COLOR);
      lcd_fill(x, y0, segmentWidth, BAR_HEIGHT_PX);
    }
  }

  prevActive = active;
}

void displaySWR(float swr)
{

  static int last_swr_int = -1;
  static int label_drawn = 0;
  static uint16_t last_color = 0xFFFF;

  if (doOnceOnly_In_Mode2)
  {
    label_drawn = 0;
  }

  /* 250ms averaging + rate limit (NanoVNA / ChibiOS ticks) */
  static float acc = 0.0f;
  static uint16_t n = 0;
  static systime_t t0 = 0;

  systime_t now = chVTGetSystemTimeX();
  if (t0 == 0)
    t0 = now;

  /* accumulate */
  acc += swr;
  if (n != 0xFFFF)
    n++;

  /* only update every 250ms (assuming 1kHz system tick = 1ms) */
  if ((systime_t)(now - t0) < (systime_t)250)
    return;

  /* compute average, reset window */
  float avg = (n ? (acc / (float)n) : swr);
  acc = 0.0f;
  n = 0;
  t0 = now;

  /* draw label ONCE */
  if (!label_drawn)
  {
    lcd_set_background(LCD_BG_COLOR);
    lcd_set_foreground(LCD_FG_COLOR);
    lcd_drawstring_size("SWR", 140, (LCD_HEIGHT / 2) - 20, 3);
    label_drawn = 1;
  }

  /* overflow / invalid filter */
  int overflow = 0;
  if (!(avg > 0.0f) || (avg != avg) || (avg > 9.9f)) /* <=0, NaN, or too large */
    overflow = 1;

  /* choose color (keep your bar-graph thresholds) */
  uint16_t color;
  if (overflow)
    color = LCD_TRACE_5_COLOR; // red
  else if (avg <= 1.5f)
    color = LCD_TRACE_3_COLOR; // green
  else if (avg <= 3.0f)
    color = LCD_TRACE_1_COLOR; // yellow
  else
    color = LCD_TRACE_5_COLOR; // red

  /* quantize only if not overflow */
  int swr_int = overflow ? -9999 : (int)(avg * 10.0f + 0.5f);

  /* do nothing if nothing changed */
  if (!overflow)
  {
    if ((swr_int == last_swr_int) && (color == last_color))
      return;
  }
  else
  {
    /* if already showing overflow in red, no need to redraw */
    if ((last_swr_int == -9999) && (last_color == color))
      return;
  }

  last_swr_int = swr_int;
  last_color = color;

  /* clear ONLY number area */
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(250, (LCD_HEIGHT / 2) - 30, 140, 50);

  lcd_set_foreground(color);

  if (overflow)
  {
    lcd_drawstring_size("!!!", 240, (LCD_HEIGHT / 2) - 20, 3);
    return;
  }

  /* split value */
  int swr_whole = swr_int / 10; /* 0..9 */
  int swr_frac = swr_int % 10;  /* 0..9 */

  char buf[4];
  buf[0] = '0' + swr_whole;
  buf[1] = '.';
  buf[2] = '0' + swr_frac;
  buf[3] = 0;

  lcd_drawstring_size(buf, 240, (LCD_HEIGHT / 2) - 20, 3);
}

#define FREQ_X_BASE 200 // must match lcd_drawstring_size(buf, X, yp, 3)
#define FREQ_Y_BASE 35  // or use yp
#define FREQ_SCALE 3

#define HIT_TOP_H 18    // height of upper (increment) zone
#define HIT_BOT_H 18    // height of lower (decrement) zone
#define HIT_GAP 4       // gap between top and bottom zones
#define HIT_Y_OFFSET -6 // move zones up/down to align with digits

#define HIT_W 14       // width of one digit hit zone (tune!)
#define HIT_SPACING 16 // distance between digit zones (tune!)

// Draw debug hitboxes ABOVE and BELOW each digit in the rendered frequency string.
// Uses the same font metrics and scale as lcd_drawstring_size().
// Draw red hit-boxes ABOVE and BELOW each DIGIT of the rendered frequency string.
// Draw debug hitboxes above/below each DIGIT of the already-rendered frequency string.
// Uses existing NanoVNA font cell macros + the same scale as lcd_drawstring_size().
static void drawTargetFreqHitboxes(const char *s, int xBase, int yBase, int scale)
{
  bool showBoxes = false; // <-- set true to show red debug rectangles

  const int cw = FONT_WIDTH * scale;
  const int ch = FONT_GET_HEIGHT * scale;

  const int gap = 2;
  const int boxH = (ch * 2) / 3;
  const int boxW = cw - 2;

  const int topY = yBase - boxH - gap;
  const int botY = yBase + ch + gap;

  // triangle sizing/placement (tuned for scale=3)
  const int triH = 6;                 // triangle height in pixels
  const int triW = 10;                // triangle base width in pixels (must be even-ish)
  const int triUpY = topY + 2;        // inside top zone
  const int triDnY = botY + boxH - 2; // inside bottom zone

  int x = xBase;

  while (*s && *s != ' ')
  {
    if (*s >= '0' && *s <= '9')
    {
      if (showBoxes)
      {
        lcd_set_colors(LCD_BG_COLOR, LCD_TRACE_5_COLOR);
        lcd_fill(x + 1, topY, boxW, boxH);
        lcd_fill(x + 1, botY, boxW, boxH);
      }

      // Draw triangles in foreground color
      lcd_set_colors(LCD_MENU_ACTIVE_COLOR, LCD_MENU_ACTIVE_COLOR);
      int cx = x + (cw / 2);

      // --- UP triangle (filled, drawn with horizontal fills) ---
      // apex at (cx, triUpY)
      for (int i = 0; i < triH; i++)
      {
        int half = (triW * i) / (2 * triH); // grows from 0 to triW/2
        lcd_fill(cx - half, triUpY + i, 2 * half + 1, 1);
      }

      // --- DOWN triangle (filled) ---
      // apex at (cx, triDnY)
      for (int i = 0; i < triH; i++)
      {
        int half = (triW * i) / (2 * triH);
        lcd_fill(cx - half, triDnY - i, 2 * half + 1, 1);
      }
    }

    // Keep your proven spacing ('.' narrower)
    if (*s == '.')
      x += (cw / 2);
    else
      x += cw;

    s++;
  }
}

extern volatile int g_touch_px;
extern volatile int g_touch_py;

void displayTargetFrequency(uint32_t target_frequency)
{
  static uint32_t last_freq = 0;
  static int label_drawn = 0;
  if (doOnceOnly_In_Mode2 == true)
  {
    label_drawn = 0;
  }
  int yp = 35;

  /* draw label ONCE */
  if (!label_drawn)
  {
    lcd_set_background(LCD_BG_COLOR);
    lcd_set_foreground(LCD_FG_COLOR);

    char label[16] = "Target  "; // 2 spaces after Target
    label[8] = 0x18;             // put glyph AFTER the spaces
    label[9] = 0;                // terminator

    lcd_drawstring_size(label, 10, yp, 3);
    label_drawn = 1;
  }

  /* do nothing if value did not change */
  if (target_frequency == last_freq)
    return;

  last_freq = target_frequency;

  /* split frequency */
  uint32_t mhz = target_frequency / 1000000UL;
  uint32_t khz = (target_frequency / 1000UL) % 1000UL;
  uint32_t hz = target_frequency % 1000UL;

  /* clear ONLY number area */
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(200, 30 - 5, 260, yp);

  lcd_set_foreground(LCD_FG_COLOR);

  /* format manually: "14.123.456 MHz" */
  char buf[16];

  buf[0] = '0' + (mhz / 10);
  buf[1] = '0' + (mhz % 10);
  buf[2] = '.';
  buf[3] = '0' + (khz / 100);
  buf[4] = '0' + ((khz / 10) % 10);
  buf[5] = '0' + (khz % 10);
  buf[6] = '.';
  buf[7] = '0' + (hz / 100);
  buf[8] = '0' + ((hz / 10) % 10);
  buf[9] = '0' + (hz % 10);
  buf[10] = ' ';
  buf[11] = 'H';
  buf[12] = 'z';
  buf[13] = 0;

  lcd_drawstring_size(buf, 200, yp, 3);
  // DEBUG overlay rectangles (temporary)
  drawTargetFreqHitboxes(buf, 200, yp, 3);
}

// Returns signed delta in Hz depending on which hitbox was tapped.
// +delta = upper box (increment), -delta = lower box (decrement), 0 = no hit.
static int32_t freq_hitbox_delta_hz(int px, int py, int xBase, int yBase, int scale)
{
  const int cw = FONT_WIDTH * scale;
  const int ch = FONT_GET_HEIGHT * scale;

  // MUST match drawTargetFreqHitboxes()
  const int gap = 2;
  const int boxH = (ch * 2) / 3;
  const int boxW = cw - 2;

  const int topY = yBase - boxH - gap;
  const int botY = yBase + ch + gap;

  int dir = 0;
  if (py >= topY && py < topY + boxH)
    dir = +1; // upper = increment
  else if (py >= botY && py < botY + boxH)
    dir = -1; // lower = decrement
  else
    return 0;

  // Digit steps for "DD.DDD.DDD" (8 digits total):
  // 10MHz, 1MHz, 100k, 10k, 1k, 100, 10, 1
  static const int32_t stepHz[8] = {
      10000000L, 1000000L, 100000L, 10000L, 1000L, 100L, 10L, 1L};

  int x = xBase;
  int digitIdx = -1;

  // Walk the 10 symbols: D D . D D D . D D D
  for (int sym = 0; sym < 10; sym++)
  {
    const int isDot = (sym == 2) || (sym == 6);

    if (!isDot)
    {
      digitIdx++;

      int bx0 = x + 1;
      int bx1 = bx0 + boxW;

      if (px >= bx0 && px < bx1)
      {
        if (digitIdx >= 0 && digitIdx < 8)
          return dir * stepHz[digitIdx];
        return 0;
      }

      x += cw;
    }
    else
    {
      x += (cw / 2); // '.' is narrower (same as your working hitbox code)
    }
  }

  return 0;
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("sweep");

  /* ------------------------------------------------------------
   * UI and plot initialization
   * ------------------------------------------------------------ */
  ui_init();
  lcd_show_logo();

  redraw_request |= REDRAW_ALL;

  plot_init();

  /* ------------------------------------------------------------
   * MLA Toolbox: disable ALL markers at startup
   * ------------------------------------------------------------ */
  for (int m = 0; m < MARKERS_MAX; m++)
    markers[m].enabled = FALSE;

  active_marker = MARKER_INVALID;
  request_to_redraw(REDRAW_MARKER | REDRAW_AREA);

  // XXXXXXXXXXXXXXXXXXXXXXX   FINAL TENTATIVE XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  HB9II_mode = 1;
  targetting_mode = false; // to ignore screen touch on mode 2 , double usage, could be replaced by HB9II_mode
  doOnceOnly_In_Mode2 = true;

  while (1)
  {

    if (HB9II_mode == 1)
    {
      doOnceOnly_In_Mode2 = true;
      bool completed = false;
      uint16_t mask = get_sweep_mask();

      /* ----------------------------------------------------------
       * Sweep execution
       * ---------------------------------------------------------- */
      if (sweep_mode & (SWEEP_ENABLE | SWEEP_ONCE))
      {
        completed = sweep(true, mask);
        sweep_mode &= ~SWEEP_ONCE;
      }
      else
      {
        __WFI();
      }

      /* ----------------------------------------------------------
       * Run shell commands in sweep thread
       * ---------------------------------------------------------- */
      while (shell_function)
      {
        shell_function(shell_nargs - 1, &shell_args[1]);
        shell_function = 0;
        osalThreadDequeueNextI(&shell_thread, MSG_OK);
      }

      /* ----------------------------------------------------------
       * UI processing
       * ---------------------------------------------------------- */
      sweep_mode |= SWEEP_UI_MODE;
      ui_process();
      sweep_mode &= ~SWEEP_UI_MODE;

      /* ----------------------------------------------------------
       * Post-sweep processing
       * ---------------------------------------------------------- */
      if (completed)
      {

        /* --------------------------------------------------------
         * 2LA calculations after sweep
         * -------------------------------------------------------- */
        mla_calc_resonance_after_sweep();
        target_frequency = mla_res_freq;
        mla_res_swr = mla_calc_swr_at_resonance_after_sweep();
        mla_update_bandwidth_swr2();
        mla_calc_q_after_sweep();
        mla_rate_q_after_sweep();

        /* --------------------------------------------------------
         * Resonance detection / locking logic
         * -------------------------------------------------------- */
        if (!mla_resonance_locked)
        {
          if (mla_no_dip_count < MLA_NO_DIP_LIMIT)
            ui_show_check_amtenna_hint();

          mla_update_good_dip_flag();

          if (mla_has_good_resonance)
          {
            uint32_t f = mla_res_freq;

            mla_no_dip_count = 0;
            ui_hide_no_resonance_hint();

            if (mla_res_stable_count == 0)
            {
              mla_res_ref_freq = f;
              mla_res_stable_count = 1;
            }
            else
            {
              uint32_t delta =
                  (f > mla_res_ref_freq) ? (f - mla_res_ref_freq) : (mla_res_ref_freq - f);

              if (delta <= MLA_RESONANCE_STABILITY_HZ)
              {
                if (mla_res_stable_count < MLA_STABLE_SWEEPS_REQUIRED)
                  mla_res_stable_count++;
              }
              else
              {
                mla_res_ref_freq = f;
                mla_res_stable_count = 1;
              }
            }

            if (mla_res_stable_count >= MLA_STABLE_SWEEPS_REQUIRED)
            {
              mla_resonance_locked = true;
              mla_res_freq = mla_res_ref_freq;

              ui_hide_check_amtenna_hint();
              ui_hide_no_resonance_hint();
            }
          }
          else
          {
            mla_res_stable_count = 0;

            if (mla_no_dip_count < MLA_NO_DIP_LIMIT)
              mla_no_dip_count++;

            if (mla_no_dip_count >= MLA_NO_DIP_LIMIT)
            {
              ui_hide_check_amtenna_hint();
              ui_show_no_resonance_hint();
            }
          }
        }

        /* --------------------------------------------------------
         * MLA refinement stages
         * -------------------------------------------------------- */
        if (mla_resonance_locked && mla_autozone_stage < 3)
          compute_first_resonance();

        /* --------------------------------------------------------
         * AUTOSCALE — EVERY completed sweep (no flags)
         * -------------------------------------------------------- */

        /* === S11 autoscale (trace 0) === */
        {
          const uint8_t t = 0;
          int type = trace[t].type;
          get_value_cb_t cb = trace_info_list[type].get_value_cb;
          if (cb)
          {
            int ch = trace[t].channel;
            float (*array)[2] = measured[ch];

            float min_v = 0.0f, max_v = 0.0f;
            bool first = true;

            for (int i = 0; i < sweep_points; i++)
            {
              float v = cb(i, array[i]);
              if (vna_fabsf(v) == infinityf())
                break;

              if (first)
              {
                min_v = max_v = v;
                first = false;
              }
              else
              {
                if (v < min_v)
                  min_v = v;
                if (v > max_v)
                  max_v = v;
              }
            }

            float span = max_v - min_v;
            if (span < 1e-6f)
              span = 2.0f;

            float y_max = (1.0f - 0.10f) * NGRIDY;
            float y_min = 0.02f * NGRIDY;
            float dy = y_max - y_min;
            if (dy < 0.1f)
              dy = 0.1f;

            float scale = span / dy;
            float refpos = y_max - (max_v / scale);

            if (trace[t].scale <= 0.0f ||
                vna_fabsf(scale - trace[t].scale) > trace[t].scale * 0.03f)
              set_trace_scale(t, scale);

            if (vna_fabsf(refpos - trace[t].refpos) > 0.02f)
              set_trace_refpos(t, refpos);
          }
        }

        /* === SWR autoscale (trace 3) === */
        {
          const uint8_t t = 3;
          int type = trace[t].type;
          get_value_cb_t cb = trace_info_list[type].get_value_cb;
          if (cb)
          {
            int ch = trace[t].channel;
            float (*array)[2] = measured[ch];

            float min_p = 0.0f, max_p = 0.0f;
            bool first = true;

            for (int i = 0; i < sweep_points; i++)
            {
              float swr = cb(i, array[i]);
              if (vna_fabsf(swr) == infinityf())
                break;

              if (swr < 1.0f)
                swr = 1.0f;
              float p = swr - 1.0f;

              if (first)
              {
                min_p = max_p = p;
                first = false;
              }
              else
              {
                if (p < min_p)
                  min_p = p;
                if (p > max_p)
                  max_p = p;
              }
            }

            float span = max_p - min_p;
            if (span < 1e-6f)
              span = 0.5f;

            float y_max = (1.0f - 0.10f) * NGRIDY;
            float y_min = 0.05f * NGRIDY;
            float dy = y_max - y_min;
            if (dy < 0.1f)
              dy = 0.1f;

            float scale = span / dy;
            float refpos = y_max - (max_p / scale);

            if (trace[t].scale <= 0.0f ||
                vna_fabsf(scale - trace[t].scale) > trace[t].scale * 0.03f)
              set_trace_scale(t, scale);

            if (vna_fabsf(refpos - trace[t].refpos) > 0.02f)
              set_trace_refpos(t, refpos);
          }

          drawTheSWRLines = false;
        }

        request_to_redraw(REDRAW_BACKUP | REDRAW_AREA);

        /* --------------------------------------------------------
         * Plot freeze logic + plotting
         * -------------------------------------------------------- */
        bool in_mla_stability_phase =
            (!mla_resonance_locked &&
             (mla_res_stable_count > 0) &&
             (mla_res_stable_count < MLA_STABLE_SWEEPS_REQUIRED));

        if (!in_mla_stability_phase && (mla_no_dip_count == 0))
        {
          request_to_redraw(REDRAW_PLOT);
          draw_all();

          bool centered = mla_is_resonance_centered();

          if (!centered)
          {
            /* ----------------------------------------------------------
             * Force sweep window around resonance
             * ---------------------------------------------------------- */

            const uint32_t SPAN_HZ = 1 * 1000000UL; // e.g. 1 MHz
            uint32_t start, stop;

            start = mla_res_freq - (SPAN_HZ / 2);
            stop = mla_res_freq + (SPAN_HZ / 2);

            set_sweep_frequency(ST_START, start);
            set_sweep_frequency(ST_STOP, stop);

            mla_autozone_stage = 1;
          }

          // chThdSleepMilliseconds(1000);
        }
      }
    }

    //*************************************************************************************************************************** */
    if (HB9II_mode == 2)
    {

      if (doOnceOnly_In_Mode2 == true)
      {
        targetting_mode = true;
        MLA_UART_LOG("[MLA] We are in Mode 2\r\n");
        set_sweep_frequency(ST_START, target_frequency);
        set_sweep_frequency(ST_STOP, target_frequency);

        // Make sure only one point is used
        sweep_points = 1;

        lcd_set_colors(LCD_BG_COLOR, LCD_BG_COLOR);
        lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);
      }

      bool completed = false;
      uint16_t mask = get_sweep_mask();
      completed = sweep(true, mask);
      displayTargetFrequency(target_frequency);

      if (completed)
      {
        float re = measured[0][0][0];
        float im = measured[0][0][1];

        float mag = sqrtf(re * re + im * im);
        float rl = (mag > 0.0f) ? -20.0f * log10f(mag) : 0.0f;
        float swr = (mag < 0.999f) ? (1.0f + mag) / (1.0f - mag) : 9999.0f;

        // MLA_UART_LOG("F=14.200 MHz | SWR=%.2f | RL=%.1f dB\r\n",swr, rl);

        drawBarGraph(swr);
        displaySWR(swr);
        doOnceOnly_In_Mode2 = false;
      }
      int evt = touch_checkHB9IIU();

      if (evt == 2) // PRESSED
      {

#define SWR_TOUCH_X0 0
#define SWR_TOUCH_Y0 ((LCD_HEIGHT / 2) - 30)
#define SWR_TOUCH_W LCD_WIDTH
#define SWR_TOUCH_H 50

        int px = g_touch_px;
        int py = g_touch_py;

        // 1) SWR area click?
        if (px >= SWR_TOUCH_X0 && px < (SWR_TOUCH_X0 + SWR_TOUCH_W) &&
            py >= SWR_TOUCH_Y0 && py < (SWR_TOUCH_Y0 + SWR_TOUCH_H))
        {
          MLA_UART_LOG("SWR AREA PRESSED  x=%d y=%d\r\n", px, py);

          HB9II_mode = 1;
          targetting_mode = false;

          /* 1) HARD RESET of sweep/page state (DO NOT OR into old flags) */
          sweep_mode = 0; // <-- critical: purge stale sweep flags
          drawTheSWRLines = false;
          redraw_request = REDRAW_ALL; // <-- critical: don't OR with old redraw bits

          /* 2) Reset resonance / autozone gating so no “go to resonance point” blocks you */
          mla_resonance_locked = false;
          mla_res_ref_freq = 0;
          mla_res_stable_count = 0;
          mla_no_dip_count = 0;
          mla_autozone_stage = 0;
          mla_autoscale_pending = false;

          /* 3) Set the full-band sweep params BEFORE plot_init() (plot may scale from them) */
          sweep_points = 401;
          set_sweep_points(401);
          set_sweep_frequency(ST_START, 3000000); // 3 MHz
          set_sweep_frequency(ST_STOP, 30000000); // 30 MHz

          lcd_clear_screen();
          ui_close_all();
          plot_init();
          /* FORCE menu to be closed (menu may be logically open even if not drawn) */

          /* 5) Now arm + run exactly one sweep */
          sweep_mode = (SWEEP_ENABLE | SWEEP_ONCE);
          // 5) Force redraw AFTER geometry + init
          redraw_request |= REDRAW_ALL;
        }
        else
        {
          // 2) Otherwise, check frequency digit hotspots
          int32_t dHz = freq_hitbox_delta_hz(px, py, 200, 35, 3);

          if (dHz != 0)
          {
            int64_t nf = (int64_t)target_frequency + dHz;

            if (nf < 8000)
              nf = 8000;
            if (nf > 160000000)
              nf = 160000000;

            target_frequency = (uint32_t)nf;

            set_sweep_frequency(ST_START, target_frequency);
            set_sweep_frequency(ST_STOP, target_frequency);

            MLA_UART_LOG("NEW FREQ = %lu (dHz=%ld)\r\n",
                         (unsigned long)target_frequency, (long)dHz);
          }
        }
      }

      chThdSleepMilliseconds(40);
    }
  }

  // BAR GRAPH TEST
  /*
  while (1)
  {
      static float value = 1.0f;
      static float step  = 0.05f;

      MLA_UART_LOG("TEST LOOP: value = %.2f\r\n", value);

      drawBarGraph(value);

      value += step;

      if (value >= 5.0f)
      {
          value = 5.0f;
          step  = -step;
          MLA_UART_LOG("Direction: DOWN\r\n");
      }
      else if (value <= 1.0f)
      {
          value = 1.0f;
          step  = -step;
          MLA_UART_LOG("Direction: UP\r\n");
      }

      chThdSleepMilliseconds(10);
  }

  */
  // TEST------------------------------------------------------------------

  // --- FIXED FREQUENCY TEST MODE ---

  targetting_mode = true;

  set_sweep_frequency(ST_START, target_frequency);
  set_sweep_frequency(ST_STOP, target_frequency);

  // Make sure only one point is used
  sweep_points = 1;

  // fixed frequency already set before while(1)
  lcd_set_colors(LCD_BG_COLOR, LCD_BG_COLOR);
  lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);
  // displayTargetFrequency(target_frequency);

  while (1)
  {
    bool completed = false;
    uint16_t mask = get_sweep_mask();
    completed = sweep(true, mask);
    displayTargetFrequency(target_frequency);

    if (completed)
    {
      float re = measured[0][0][0];
      float im = measured[0][0][1];

      float mag = sqrtf(re * re + im * im);
      float rl = (mag > 0.0f) ? -20.0f * log10f(mag) : 0.0f;
      float swr = (mag < 0.999f) ? (1.0f + mag) / (1.0f - mag) : 9999.0f;

      // MLA_UART_LOG("F=14.200 MHz | SWR=%.2f | RL=%.1f dB\r\n",swr, rl);

      drawBarGraph(swr);
      displaySWR(swr);
    }
    int evt = touch_checkHB9IIU();

    if (evt == 2) // PRESSED
    {

#define SWR_TOUCH_X0 0
#define SWR_TOUCH_Y0 ((LCD_HEIGHT / 2) - 30)
#define SWR_TOUCH_W LCD_WIDTH
#define SWR_TOUCH_H 50

      int px = g_touch_px;
      int py = g_touch_py;

      // 1) SWR area click?
      if (px >= SWR_TOUCH_X0 && px < (SWR_TOUCH_X0 + SWR_TOUCH_W) &&
          py >= SWR_TOUCH_Y0 && py < (SWR_TOUCH_Y0 + SWR_TOUCH_H))
      {
        MLA_UART_LOG("SWR AREA PRESSED  x=%d y=%d\r\n", px, py);
      }
      else
      {
        // 2) Otherwise, check frequency digit hotspots
        int32_t dHz = freq_hitbox_delta_hz(px, py, 200, 35, 3);

        if (dHz != 0)
        {
          int64_t nf = (int64_t)target_frequency + dHz;

          if (nf < 8000)
            nf = 8000;
          if (nf > 160000000)
            nf = 160000000;

          target_frequency = (uint32_t)nf;

          set_sweep_frequency(ST_START, target_frequency);
          set_sweep_frequency(ST_STOP, target_frequency);

          MLA_UART_LOG("NEW FREQ = %lu (dHz=%ld)\r\n",
                       (unsigned long)target_frequency, (long)dHz);
        }
      }
    }

    chThdSleepMilliseconds(20);
  }

  /* ------------------------------------------------------------
   * Main sweep thread loop
   * ------------------------------------------------------------ */
  while (1)
  {
    bool completed = false;
    uint16_t mask = get_sweep_mask();

    /* ----------------------------------------------------------
     * Sweep execution
     * ---------------------------------------------------------- */
    if (sweep_mode & (SWEEP_ENABLE | SWEEP_ONCE))
    {
      completed = sweep(true, mask);
      sweep_mode &= ~SWEEP_ONCE;
    }
    else
    {
      __WFI();
    }

    /* ----------------------------------------------------------
     * Run shell commands in sweep thread
     * ---------------------------------------------------------- */
    while (shell_function)
    {
      shell_function(shell_nargs - 1, &shell_args[1]);
      shell_function = 0;
      osalThreadDequeueNextI(&shell_thread, MSG_OK);
    }

    /* ----------------------------------------------------------
     * UI processing
     * ---------------------------------------------------------- */
    sweep_mode |= SWEEP_UI_MODE;
    ui_process();
    sweep_mode &= ~SWEEP_UI_MODE;

    /* ----------------------------------------------------------
     * Post-sweep processing
     * ---------------------------------------------------------- */
    if (completed)
    {
      /* --------------------------------------------------------
       * MLA calculations after sweep
       * -------------------------------------------------------- */
      mla_calc_resonance_after_sweep();
      mla_res_swr = mla_calc_swr_at_resonance_after_sweep();
      mla_update_bandwidth_swr2();
      mla_calc_q_after_sweep();
      mla_rate_q_after_sweep();

      /* --------------------------------------------------------
       * Resonance detection / locking logic
       * -------------------------------------------------------- */
      if (!mla_resonance_locked)
      {
        if (mla_no_dip_count < MLA_NO_DIP_LIMIT)
          ui_show_check_amtenna_hint();

        mla_update_good_dip_flag();

        if (mla_has_good_resonance)
        {
          uint32_t f = mla_res_freq;

          mla_no_dip_count = 0;
          ui_hide_no_resonance_hint();

          if (mla_res_stable_count == 0)
          {
            mla_res_ref_freq = f;
            mla_res_stable_count = 1;
          }
          else
          {
            uint32_t delta =
                (f > mla_res_ref_freq) ? (f - mla_res_ref_freq) : (mla_res_ref_freq - f);

            if (delta <= MLA_RESONANCE_STABILITY_HZ)
            {
              if (mla_res_stable_count < MLA_STABLE_SWEEPS_REQUIRED)
                mla_res_stable_count++;
            }
            else
            {
              mla_res_ref_freq = f;
              mla_res_stable_count = 1;
            }
          }

          if (mla_res_stable_count >= MLA_STABLE_SWEEPS_REQUIRED)
          {
            mla_resonance_locked = true;
            mla_res_freq = mla_res_ref_freq;

            ui_hide_check_amtenna_hint();
            ui_hide_no_resonance_hint();
          }
        }
        else
        {
          mla_res_stable_count = 0;

          if (mla_no_dip_count < MLA_NO_DIP_LIMIT)
            mla_no_dip_count++;

          if (mla_no_dip_count >= MLA_NO_DIP_LIMIT)
          {
            ui_hide_check_amtenna_hint();
            ui_show_no_resonance_hint();
          }
        }
      }

      /* --------------------------------------------------------
       * MLA refinement stages
       * -------------------------------------------------------- */
      if (mla_resonance_locked && mla_autozone_stage < 3)
        compute_first_resonance();

      /* --------------------------------------------------------
       * AUTOSCALE — EVERY completed sweep (no flags)
       * -------------------------------------------------------- */

      /* === S11 autoscale (trace 0) === */
      {
        const uint8_t t = 0;
        int type = trace[t].type;
        get_value_cb_t cb = trace_info_list[type].get_value_cb;
        if (cb)
        {
          int ch = trace[t].channel;
          float (*array)[2] = measured[ch];

          float min_v = 0.0f, max_v = 0.0f;
          bool first = true;

          for (int i = 0; i < sweep_points; i++)
          {
            float v = cb(i, array[i]);
            if (vna_fabsf(v) == infinityf())
              break;

            if (first)
            {
              min_v = max_v = v;
              first = false;
            }
            else
            {
              if (v < min_v)
                min_v = v;
              if (v > max_v)
                max_v = v;
            }
          }

          float span = max_v - min_v;
          if (span < 1e-6f)
            span = 2.0f;

          float y_max = (1.0f - 0.10f) * NGRIDY;
          float y_min = 0.02f * NGRIDY;
          float dy = y_max - y_min;
          if (dy < 0.1f)
            dy = 0.1f;

          float scale = span / dy;
          float refpos = y_max - (max_v / scale);

          if (trace[t].scale <= 0.0f ||
              vna_fabsf(scale - trace[t].scale) > trace[t].scale * 0.03f)
            set_trace_scale(t, scale);

          if (vna_fabsf(refpos - trace[t].refpos) > 0.02f)
            set_trace_refpos(t, refpos);
        }
      }

      /* === SWR autoscale (trace 3) === */
      {
        const uint8_t t = 3;
        int type = trace[t].type;
        get_value_cb_t cb = trace_info_list[type].get_value_cb;
        if (cb)
        {
          int ch = trace[t].channel;
          float (*array)[2] = measured[ch];

          float min_p = 0.0f, max_p = 0.0f;
          bool first = true;

          for (int i = 0; i < sweep_points; i++)
          {
            float swr = cb(i, array[i]);
            if (vna_fabsf(swr) == infinityf())
              break;

            if (swr < 1.0f)
              swr = 1.0f;
            float p = swr - 1.0f;

            if (first)
            {
              min_p = max_p = p;
              first = false;
            }
            else
            {
              if (p < min_p)
                min_p = p;
              if (p > max_p)
                max_p = p;
            }
          }

          float span = max_p - min_p;
          if (span < 1e-6f)
            span = 0.5f;

          float y_max = (1.0f - 0.10f) * NGRIDY;
          float y_min = 0.05f * NGRIDY;
          float dy = y_max - y_min;
          if (dy < 0.1f)
            dy = 0.1f;

          float scale = span / dy;
          float refpos = y_max - (max_p / scale);

          if (trace[t].scale <= 0.0f ||
              vna_fabsf(scale - trace[t].scale) > trace[t].scale * 0.03f)
            set_trace_scale(t, scale);

          if (vna_fabsf(refpos - trace[t].refpos) > 0.02f)
            set_trace_refpos(t, refpos);
        }

        drawTheSWRLines = false;
      }

      request_to_redraw(REDRAW_BACKUP | REDRAW_AREA);

      /* --------------------------------------------------------
       * Plot freeze logic + plotting
       * -------------------------------------------------------- */
      bool in_mla_stability_phase =
          (!mla_resonance_locked &&
           (mla_res_stable_count > 0) &&
           (mla_res_stable_count < MLA_STABLE_SWEEPS_REQUIRED));

      if (!in_mla_stability_phase && (mla_no_dip_count == 0))
      {
        request_to_redraw(REDRAW_PLOT);
        draw_all();

        bool centered = mla_is_resonance_centered();

        if (!centered)
        {
          /* ----------------------------------------------------------
           * Force sweep window around resonance
           * ---------------------------------------------------------- */

          const uint32_t SPAN_HZ = 1 * 1000000UL; // e.g. 1 MHz
          uint32_t start, stop;

          start = mla_res_freq - (SPAN_HZ / 2);
          stop = mla_res_freq + (SPAN_HZ / 2);

          set_sweep_frequency(ST_START, start);
          set_sweep_frequency(ST_STOP, stop);

          mla_autozone_stage = 1;
        }

        // chThdSleepMilliseconds(1000);
      }
    }
  }
}

void pause_sweep(void)
{
  sweep_mode &= ~SWEEP_ENABLE;
}

static inline void
resume_sweep(void)
{
  sweep_mode |= SWEEP_ENABLE;
}

void toggle_sweep(void)
{
  sweep_mode ^= SWEEP_ENABLE;
}

//
//  Optimized Kaiser window functions for transform domain
//
//       Zero-order modified Bessel function
//                 (x/2)^(2n)
// Bessel I0 = 1 + ---------- => For bessel_I0_ext(z) set input as z = (x/2)^2 (input range 0 .. beta*beta/4)
//                   (n!)^2
//
//       z^n         z    z^2   z^3   z^4    z^5              z^n
// 1 + ------ = 1 + --- + --- + --- + --- + -----  ...... + ------
//     (n!)^2        1     4     36   576   14400           (n!)^2
float bessel_I0_ext(float z)
{
// Set calculated elements count, more size - less error but longer (bigger beta also need more size for less error)
// Use BESSEL_SIZE = 12 (last used n = 12), use constant size faster then check every time limits in float
// For beta =  6 and BESSEL_SIZE = 12 max error 4.2e-7
// For beta = 13 and BESSEL_SIZE = 12 max error 2.5e-4
#define BESSEL_SIZE 12
  int i = BESSEL_SIZE - 1;
  // Precalculated multipliers: 1 / (n!^2)
  static const float besseli0_k[BESSEL_SIZE - 1] = {
      //  1.0000000000000000000000000000000e+00,    // 1 / ( 1!^2)
      2.5000000000000000000000000000000e-01, // 1 / ( 2!^2)
      2.7777777777777777777777777777778e-02, // 1 / ( 3!^2)
      1.7361111111111111111111111111111e-03, // 1 / ( 4!^2)
      6.9444444444444444444444444444444e-05, // 1 / ( 5!^2)
      1.9290123456790123456790123456790e-06, // 1 / ( 6!^2)
      3.9367598891408415217939027462837e-08, // 1 / ( 7!^2)
      6.1511873267825648778029730410683e-10, // 1 / ( 8!^2)
      7.5940584281266233059295963469979e-12, // 1 / ( 9!^2)
      7.5940584281266233059295963469979e-14, // 1 / (10!^2)
      6.2760813455591928148178482206594e-16, // 1 / (11!^2)
      4.3583898233049950102901723754579e-18, // 1 / (12!^2)
                                             //  2.5789288895295828463255457842946e-20,    // 1 / (13!^2)
                                             //  1.3157800456783585950640539715789e-22,    // 1 / (14!^2)
                                             //  5.8479113141260382002846843181284e-25,    // 1 / (15!^2)
                                             //  2.2843403570804836719862048117689e-27,    // 1 / (16!^2)
                                             //  7.9042918930120542283259682068128e-30,    // 1 / (17!^2)
  };
  float term = z, ret = 1.0f + z;
  do
  {
    ret += (term *= z) * besseli0_k[BESSEL_SIZE - 1 - i];
  } while (--i);
  return ret;
}
//                        Kaiser window
//           bessel_I0(beta*sqrt(1 - (2*k/N - 1)^2))
// Kaiser = -------------------------------------
//                   bessel_I0(beta)
// Move out constant divider: bessel_I0(beta) = bessel_I0_ext(beta * beta / 4)
// Made calculation optimization (in integer)
// x = (2*k)/(n-1) - 1 = (set n=n-1) = 2*k/n - 1 = (2*k-n)/n
// calculate kaiser window vs bessel_I0(w) there:
//                                      n*n - (2*k-n)*(2*k-n)                4*k*(n-k)
// w = beta*sqrt(1 - x*x) = beta * sqrt(---------------------) = beta * sqrt(---------)
//                                             n*n                              n*n
// bessel_I0(w) = bessel_I0_ext(z) (there z = (w*w)/4 for speed)
//     w^2                  k*(n-k)
// z = --- = beta * beta * (-------)
//      4                     n*n
// return = bessel_I0_ext(z)
static float kaiser_window_ext(uint32_t k, uint32_t n, uint16_t beta)
{
  if (beta == 0)
    return 1.0f;
  n = n - 1;
  k = k * (n - k) * beta * beta;
  n = n * n;
  return bessel_I0_ext((float)k / n);
}

static void
transform_domain(uint16_t ch_mask)
{
  // use spi_buffer as temporary buffer and calculate ifft for time domain
  // Need 2 * sizeof(float) * FFT_SIZE bytes for work
#if 2 * 4 * FFT_SIZE > (SPI_BUFFER_SIZE * LCD_PIXEL_SIZE)
#error "Need increase spi_buffer or use less FFT_SIZE value"
#endif
  int i;
  uint16_t offset = 0;
  uint8_t is_lowpass = FALSE;
  switch (domain_func)
  {
    //  case TD_FUNC_BANDPASS:
    //    break;
  case TD_FUNC_LOWPASS_IMPULSE:
  case TD_FUNC_LOWPASS_STEP:
    is_lowpass = TRUE;
    offset = sweep_points;
    break;
  }
  uint16_t window_size = sweep_points + offset;
  uint16_t beta = 0;
  switch (domain_window)
  {
    //    case TD_WINDOW_MINIMUM:
    //    beta = 0;  // this is rectangular
    //      break;
  case TD_WINDOW_NORMAL:
    beta = 6;
    break;
  case TD_WINDOW_MAXIMUM:
    beta = 13;
    break;
  }
  // Add amplitude correction for not full size FFT data and also add computed default scale
  // recalculate the scale factor if any window details are changed. The scale factor is to compensate for windowing.
  // Add constant multiplier for kaiser_window_ext use 1.0f / bessel0_ext(beta*beta/4.0f)
  // Add constant multiplier 1.0f / FFT_SIZE
  static float window_scale = 0.0f;
  static uint16_t td_cache = 0;
  // Check mode cache data
  uint16_t td_check = (props_mode & (TD_WINDOW | TD_FUNC)) | (sweep_points << 5);
  if (td_cache != td_check)
  {
    td_cache = td_check;
    if (domain_func == TD_FUNC_LOWPASS_STEP)
      window_scale = FFT_SIZE * bessel_I0_ext(beta * beta / 4.0f);
    else
    {
      window_scale = 0.0f;
      for (int i = 0; i < sweep_points; i++)
        window_scale += kaiser_window_ext(i + offset, window_size, beta);
      if (domain_func == TD_FUNC_LOWPASS_IMPULSE)
        window_scale *= 2.0f;
      //    window_scale/= FFT_SIZE                      // add correction from kaiser_window summ
      //    window_scale*= FFT_SIZE                      // add default from FFT_SIZE
      //    window_scale/= bessel_I0_ext(beta*beta/4.0f) // for get result as kaiser_window summ
      //    window_scale*= bessel_I0_ext(beta*beta/4.0f) // for set correction on calculated kaiser_window for value
    }
    window_scale = 1.0f / window_scale;
#ifdef USE_FFT_WINDOW_BUFFER
    // Cache window function data to static buffer
    static float kaiser_data[FFT_SIZE];
    for (i = 0; i < sweep_points; i++)
      kaiser_data[i] = kaiser_window_ext(i + offset, window_size, beta) * window_scale;
#endif
  }
  // Made Time Domain Calculations
  for (int ch = 0; ch < 2; ch++, ch_mask >>= 1)
  {
    if ((ch_mask & 1) == 0)
      continue;
    // Prepare data in tmp buffer (use spi_buffer), apply window function and constant correction factor
    float *tmp = (float *)spi_buffer;
    float *data = measured[ch][0];
    for (i = 0; i < sweep_points; i++)
    {
#ifdef USE_FFT_WINDOW_BUFFER
      float w = kaiser_data[i];
#else
      float w = kaiser_window_ext(i + offset, window_size, beta) * window_scale;
#endif
      tmp[i * 2 + 0] = data[i * 2 + 0] * w;
      tmp[i * 2 + 1] = data[i * 2 + 1] * w;
    }
    // Fill zeroes last
    for (; i < FFT_SIZE; i++)
    {
      tmp[i * 2 + 0] = 0.0f;
      tmp[i * 2 + 1] = 0.0f;
    }
    // For lowpass mode swap
    if (is_lowpass)
    {
      for (i = 1; i < sweep_points; i++)
      {
        tmp[(FFT_SIZE - i) * 2 + 0] = tmp[i * 2 + 0];
        tmp[(FFT_SIZE - i) * 2 + 1] = -tmp[i * 2 + 1];
      }
    }
    // Made iFFT in temp buffer
    fft_inverse((float (*)[2])tmp);
    // set img part as zero
    if (is_lowpass)
    {
      for (i = 0; i < sweep_points; i++)
        tmp[i * 2 + 1] = 0.0f;
    }
    if (domain_func == TD_FUNC_LOWPASS_STEP)
    {
      for (i = 1; i < sweep_points; i++)
        tmp[i * 2 + 0] += tmp[i * 2 + 0 - 2];
    }
    // Copy data back
    memcpy(measured[ch], tmp, sizeof(measured[0]));
  }
}

// Shell commands output
int shell_printf(const char *fmt, ...)
{
  if (shell_stream == NULL)
    return 0;
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf(shell_stream, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}

static void shell_write(const void *buf, uint32_t size) { streamWrite(shell_stream, buf, size); }
static int shell_read(void *buf, uint32_t size) { return streamRead(shell_stream, buf, size); }
// static void shell_put(uint8_t c)                      {streamPut(shell_stream, c);}
// static uint8_t shell_getc(void)                       {return streamGet(shell_stream);}

// HB9IIU
VNA_SHELL_FUNCTION(cmd_reson)
{
  (void)argc;
  (void)argv;

  if (!mla_resonance_locked)
  {
    shell_printf("Resonance not computed yet" VNA_SHELL_NEWLINE_STR);
    return;
  }

  shell_printf("MLA resonance: f = %u Hz (idx=%u)" VNA_SHELL_NEWLINE_STR,
               (unsigned)mla_res_freq,
               (unsigned)mla_res_idx);
}

VNA_SHELL_FUNCTION(cmd_pause)
{
  (void)argc;
  (void)argv;
  pause_sweep();
}

VNA_SHELL_FUNCTION(cmd_resume)
{
  (void)argc;
  (void)argv;

  // restore frequencies array and cal
  update_frequencies();
  resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
  (void)argc;
  (void)argv;
#ifdef __DFU_SOFTWARE_MODE__
  if (argc == 1)
  {
    if (get_str_index(argv[0], "dfu") == 0)
    {
      shell_printf("Performing reset to DFU mode" VNA_SHELL_NEWLINE_STR);
      ui_enter_dfu();
      return;
    }
  }
#endif
  shell_printf("Performing reset" VNA_SHELL_NEWLINE_STR);
  NVIC_SystemReset();
}

#ifdef __USE_SMOOTH__
VNA_SHELL_FUNCTION(cmd_smooth)
{
  if (argc != 1)
  {
    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR
                 "current: %u" VNA_SHELL_NEWLINE_STR,
                 "smooth {0-8}", smooth_factor);
    return;
  }
  set_smooth_factor(my_atoui(argv[0]));
}
#endif

#ifdef ENABLE_CONFIG_COMMAND
VNA_SHELL_FUNCTION(cmd_config)
{
  static const char cmd_mode_list[] =
      "auto"
#ifdef __USE_SMOOTH__
      "|avg"
#endif
#ifdef __USE_SERIAL_CONSOLE__
      "|connection"
#endif
      "|mode"
      "|grid"
      "|dot"
#ifdef __USE_BACKUP__
      "|bk"
#endif
#ifdef __FLIP_DISPLAY__
      "|flip"
#endif
#ifdef __DIGIT_SEPARATOR__
      "|separator"
#endif
#ifdef __SD_CARD_DUMP_TIFF__
      "|tif"
#endif
      ;
  int idx;
  if (argc == 2 && (idx = get_str_index(argv[0], cmd_mode_list)) >= 0)
  {
    apply_VNA_mode(idx, my_atoui(argv[1]));
  }
  else
    shell_printf("usage: config {%s} [0|1]" VNA_SHELL_NEWLINE_STR, cmd_mode_list);
}
#endif

#ifdef __VNA_MEASURE_MODULE__
VNA_SHELL_FUNCTION(cmd_measure)
{
  static const char cmd_measure_list[] =
      "none"
#ifdef __USE_LC_MATCHING__
      "|lc" // Add LC match function
#endif
#ifdef __S21_MEASURE__
      "|lcshunt"  // Enable LC shunt measure option
      "|lcseries" // Enable LC series  measure option
      "|xtal"     // Enable XTAL measure option
      "|filter"   // Enable filter measure option
#endif
#ifdef __S11_CABLE_MEASURE__
      "|cable" // Enable S11 cable measure option
#endif
#ifdef __S11_RESONANCE_MEASURE__
      "|resonance" // Enable S11 resonance search option
#endif
      ;
  int idx;
  if (argc == 1 && (idx = get_str_index(argv[0], cmd_measure_list)) >= 0)
    plot_set_measure_mode(idx);
  else
    shell_printf("usage: measure {%s}" VNA_SHELL_NEWLINE_STR, cmd_measure_list);
}
#endif

#ifdef USE_VARIABLE_OFFSET
VNA_SHELL_FUNCTION(cmd_offset)
{
  if (argc != 1)
  {
    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR
                 "current: %u" VNA_SHELL_NEWLINE_STR,
                 "offset {frequency offset(Hz)}", IF_OFFSET);
    return;
  }
  si5351_set_frequency_offset(my_atoi(argv[0]));
}
#endif

VNA_SHELL_FUNCTION(cmd_freq)
{
  if (argc != 1)
  {
    shell_printf("usage: freq {frequency(Hz)}" VNA_SHELL_NEWLINE_STR);
    return;
  }
  uint32_t freq = my_atoui(argv[0]);
  pause_sweep();
  set_frequency(freq);
  return;
}

void set_power(uint8_t value)
{
  request_to_redraw(REDRAW_CAL_STATUS);
  if (value > SI5351_CLK_DRIVE_STRENGTH_8MA)
    value = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  if (current_props._power == value)
    return;
  current_props._power = value;
  // Update power if pause, need for generation in CW mode
  if (!(sweep_mode & SWEEP_ENABLE))
    si5351_set_power(value);
}

VNA_SHELL_FUNCTION(cmd_power)
{
  if (argc != 1)
  {
    shell_printf("usage: power {0-3}|{255 - auto}" VNA_SHELL_NEWLINE_STR
                 "power: %d" VNA_SHELL_NEWLINE_STR,
                 current_props._power);
    return;
  }
  set_power(my_atoi(argv[0]));
}

#ifdef __USE_RTC__
VNA_SHELL_FUNCTION(cmd_time)
{
  (void)argc;
  (void)argv;
  uint32_t dt_buf[2];
  dt_buf[0] = rtc_get_tr_bcd(); // TR should be read first for sync
  dt_buf[1] = rtc_get_dr_bcd(); // DR should be read second
  static const uint8_t idx_to_time[] = {6, 5, 4, 2, 1, 0};
  static const char time_cmd[] = "y|m|d|h|min|sec|ppm";
  //            0    1   2       4      5     6
  // time[] ={sec, min, hr, 0, day, month, year, 0}
  uint8_t *time = (uint8_t *)dt_buf;
  if (argc == 3 && get_str_index(argv[0], "b") == 0)
  {
    rtc_set_time(my_atoui(argv[1]), my_atoui(argv[2]));
    return;
  }
  if (argc != 2)
    goto usage;
  int idx = get_str_index(argv[0], time_cmd);
  if (idx == 6)
  {
    rtc_set_cal(my_atof(argv[1]));
    return;
  }
  uint32_t val = my_atoui(argv[1]);
  if (idx < 0 || val > 99)
    goto usage;
  // Write byte value in struct
  time[idx_to_time[idx]] = ((val / 10) << 4) | (val % 10); // value in bcd format
  rtc_set_time(dt_buf[1], dt_buf[0]);
  return;
usage:
  shell_printf("20%02x/%02x/%02x %02x:%02x:%02x" VNA_SHELL_NEWLINE_STR
               "usage: time {[%s] 0-99} or {b 0xYYMMDD 0xHHMMSS}" VNA_SHELL_NEWLINE_STR,
               time[6], time[5], time[4], time[2], time[1], time[0], time_cmd);
}
#endif

#ifdef __VNA_ENABLE_DAC__
VNA_SHELL_FUNCTION(cmd_dac)
{
  if (argc != 1)
  {
    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR
                 "current: %u" VNA_SHELL_NEWLINE_STR,
                 "dac {value(0-4095)}", config._dac_value);
    return;
  }
  dac_setvalue_ch2(my_atoui(argv[0]) & 0xFFF);
}
#endif

VNA_SHELL_FUNCTION(cmd_threshold)
{
  uint32_t value;
  if (argc != 1)
  {
    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR
                 "current: %u" VNA_SHELL_NEWLINE_STR,
                 "threshold {frequency in harmonic mode}", config._harmonic_freq_threshold);
    return;
  }
  value = my_atoui(argv[0]);
  config._harmonic_freq_threshold = value;
}

VNA_SHELL_FUNCTION(cmd_saveconfig)
{
  (void)argc;
  (void)argv;
  config_save();
  shell_printf("Config saved" VNA_SHELL_NEWLINE_STR);
}

VNA_SHELL_FUNCTION(cmd_clearconfig)
{
  if (argc != 1)
  {
    shell_printf("usage: clearconfig {protection key}" VNA_SHELL_NEWLINE_STR);
    return;
  }

  if (get_str_index(argv[0], "1234") != 0)
  {
    shell_printf("Key unmatched." VNA_SHELL_NEWLINE_STR);
    return;
  }

  clear_all_config_prop_data();
  shell_printf("Config and all cal data cleared." VNA_SHELL_NEWLINE_STR
               "Do reset manually to take effect. Then do touch cal and save." VNA_SHELL_NEWLINE_STR);
}

VNA_SHELL_FUNCTION(cmd_data)
{
  int i;
  int sel = 0;
  float (*array)[2];
  if (argc == 1)
    sel = my_atoi(argv[0]);
  if (sel < 0 || sel >= 7)
    goto usage;

  array = sel < 2 ? measured[sel] : cal_data[sel - 2];

  for (i = 0; i < sweep_points; i++)
    shell_printf("%f %f" VNA_SHELL_NEWLINE_STR, array[i][0], array[i][1]);
  return;
usage:
  shell_printf("usage: data [array]" VNA_SHELL_NEWLINE_STR);
}

#ifdef __CAPTURE_RLE8__
void capture_rle8(void)
{
  static const struct
  {
    uint16_t header;
    uint16_t width, height;
    uint8_t bit_per_pixel, compression;
  } screenshot_header = {
      0x4D42,
      LCD_WIDTH, LCD_HEIGHT,
      8, 1};

  uint16_t size = sizeof(config._lcd_palette);
  shell_write(&screenshot_header, sizeof(screenshot_header)); // write header
  shell_write(&size, sizeof(uint16_t));                       // write palette block size
  shell_write(config._lcd_palette, size);                     // write palette block
  uint16_t *data = &spi_buffer[32];                           // most bad pack situation increase on 1 byte every 128, so put not compressed data on 64 byte offset
  for (int y = 0, idx = 0; y < LCD_HEIGHT; y++)
  {
    lcd_read_memory(0, y, LCD_WIDTH, 1, data); // read in 16bpp format
    for (int x = 0; x < LCD_WIDTH; x++)
    { // convert to palette mode
      if (config._lcd_palette[idx] != data[x])
      { // search color in palette
        for (idx = 0; idx < MAX_PALETTE && config._lcd_palette[idx] != data[x]; idx++)
          ;
        if (idx >= MAX_PALETTE)
          idx = 0;
      }
      ((uint8_t *)data)[x] = idx; // put palette index
    }
    spi_buffer[0] = packbits((char *)data, (char *)&spi_buffer[1], LCD_WIDTH); // pack
    shell_write(spi_buffer, spi_buffer[0] + sizeof(uint16_t));
  }
}
#endif

VNA_SHELL_FUNCTION(cmd_capture)
{
  (void)argc;
  (void)argv;
#ifdef __CAPTURE_RLE8__
  if (argc > 0)
  {
    capture_rle8();
    return;
  }
#endif
// Check buffer limits, if less possible reduce rows count
#define READ_ROWS 2
#if (SPI_BUFFER_SIZE * LCD_PIXEL_SIZE) < (LCD_RX_PIXEL_SIZE * LCD_WIDTH * READ_ROWS)
#error "Low size of spi_buffer for cmd_capture"
#endif
  // read 2 row pixel time
  for (int y = 0; y < LCD_HEIGHT; y += READ_ROWS)
  {
    // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
    lcd_read_memory(0, y, LCD_WIDTH, READ_ROWS, (uint16_t *)spi_buffer);
    shell_write(spi_buffer, READ_ROWS * LCD_WIDTH * sizeof(uint16_t));
  }
}

#if 0
VNA_SHELL_FUNCTION(cmd_gamma)
{
  float gamma[2];
  (void)argc;
  (void)argv;
  
  pause_sweep();
  chMtxLock(&mutex);
  wait_dsp(4);  
  calculate_gamma(gamma);
  chMtxUnlock(&mutex);

  shell_printf("%d %d" VNA_SHELL_NEWLINE_STR, gamma[0], gamma[1]);
}
#endif

static void (*sample_func)(float *gamma) = calculate_gamma;
#ifdef ENABLE_SAMPLE_COMMAND
VNA_SHELL_FUNCTION(cmd_sample)
{
  if (argc != 1)
    goto usage;
  //                                         0    1   2
  static const char cmd_sample_list[] = "gamma|ampl|ref";
  switch (get_str_index(argv[0], cmd_sample_list))
  {
  case 0:
    sample_func = calculate_gamma;
    return;
  case 1:
    sample_func = fetch_amplitude;
    return;
  case 2:
    sample_func = fetch_amplitude_ref;
    return;
  default:
    break;
  }
usage:
  shell_printf("usage: sample {%s}" VNA_SHELL_NEWLINE_STR, cmd_sample_list);
}
#endif

config_t config = {
    .magic = CONFIG_MAGIC,
    ._harmonic_freq_threshold = FREQUENCY_THRESHOLD,
    ._IF_freq = FREQUENCY_OFFSET,
    ._touch_cal = DEFAULT_TOUCH_CONFIG,
    ._vna_mode = 0, // USB mode, search max
    ._brightness = 100,
    ._dac_value = 1922,
    ._vbat_offset = 420,
    ._bandwidth = BANDWIDTH_1000,
    ._lcd_palette = LCD_DEFAULT_PALETTE,
    ._serial_speed = SERIAL_DEFAULT_BITRATE,
    ._xtal_freq = XTALFREQ,
    ._measure_r = MEASURE_DEFAULT_R,
    ._lever_mode = LM_MARKER,
    ._band_mode = 0,
};

properties_t current_props;

// NanoVNA Default settings
static const trace_t def_trace[TRACES_MAX] = { // enable, type, channel, smith format, scale, refpos
    {TRUE, TRC_LOGMAG, 0, MS_RX, 10.0, NGRIDY - 1},
    {TRUE, TRC_LOGMAG, 1, MS_REIM, 10.0, NGRIDY - 1},
    {TRUE, TRC_SMITH, 0, MS_RX, 1.0, 0},
    {TRUE, TRC_PHASE, 1, MS_REIM, 90.0, NGRIDY / 2}};

static const marker_t def_markers[MARKERS_MAX] = {
    {FALSE, 0, 10 * SWEEP_POINTS_MAX / 100 - 1, 0},
#if MARKERS_MAX > 1
    {FALSE, 0, 20 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif

#if MARKERS_MAX > 2
    {FALSE, 0, 30 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
#if MARKERS_MAX > 3
    {FALSE, 0, 40 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
#if MARKERS_MAX > 4
    {FALSE, 0, 50 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
#if MARKERS_MAX > 5
    {FALSE, 0, 60 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
#if MARKERS_MAX > 6
    {FALSE, 0, 70 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
#if MARKERS_MAX > 7
    {FALSE, 0, 80 * SWEEP_POINTS_MAX / 100 - 1, 0},
#endif
};

// Load propeties default settings
static void load_default_properties(void)
{
  // Magic add on caldata_save
  current_props.magic = PROPERTIES_MAGIC;
  current_props._frequency0 = 50000;     // start =  50kHz
  current_props._frequency1 = 900000000; // end   = 900MHz
  current_props._var_freq = 0;
  current_props._sweep_points = 401;         // Set default points count
  current_props._cal_frequency0 = 50000;     // calibration start =  50kHz
  current_props._cal_frequency1 = 900000000; // calibration end   = 900MHz
  current_props._cal_sweep_points = 401;     // Set calibration default points count
  current_props._cal_status = 0;
  //=============================================
  memcpy(current_props._trace, def_trace, sizeof(def_trace));
  memcpy(current_props._markers, def_markers, sizeof(def_markers));
  //=============================================
  current_props._electrical_delay[0] = 0.0f;
  current_props._electrical_delay[1] = 0.0f;
  current_props._var_delay = 0.0f;
  current_props._s21_offset = 0.0f;
  current_props._portz = 50.0f;
  current_props._cal_load_r = 50.0f;
  current_props._velocity_factor = 70;
  current_props._current_trace = 0;
  current_props._active_marker = MARKER_INVALID;
  current_props._previous_marker = MARKER_INVALID;
  current_props._mode = 0;
  current_props._reserved = 0;
  current_props._power = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  current_props._cal_power = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  current_props._measure = 0;
  // This data not loaded by default
  // current_props._cal_data[5][POINTS_COUNT][2];
  // Checksum add on caldata_save
  // current_props.checksum = 0;
}

//
// Backup registers support, allow save data on power off (while vbat power enabled)
//
#ifdef __USE_BACKUP__
#if SWEEP_POINTS_MAX > 511 || SAVEAREA_MAX > 15
#error "Check backup data limits!!"
#endif

// backup_0 bitfield
typedef union
{
  struct
  {
    uint32_t points : 9;     //  9 !! limit 511 points!!
    uint32_t bw : 9;         // 18 !! limit 511
    uint32_t id : 4;         // 22 !! 15 save slots
    uint32_t leveler : 3;    // 25
    uint32_t brightness : 7; // 32
  };
  uint32_t v;
} backup_0;

void update_backup_data(void)
{
  backup_0 bk = {
      .points = sweep_points,
      .bw = config._bandwidth,
      .id = lastsaveid,
      .leveler = lever_mode,
      .brightness = config._brightness};
  set_backup_data32(0, bk.v);
  set_backup_data32(1, frequency0);
  set_backup_data32(2, frequency1);
  set_backup_data32(3, var_freq);
  set_backup_data32(4, config._vna_mode);
}

static void load_settings(void)
{
  load_default_properties(); // Load default settings
  if (config_recall() == 0 && VNA_MODE(VNA_MODE_BACKUP))
  { // Config loaded ok and need restore backup if enabled
    backup_0 bk = {.v = get_backup_data32(0)};
    if (bk.v != 0)
    { // if backup data valid
      if (bk.id < SAVEAREA_MAX && caldata_recall(bk.id) == 0)
      {                           // Slot valid and Load ok
        sweep_points = bk.points; // Restore settings depend from calibration data
        frequency0 = get_backup_data32(1);
        frequency1 = get_backup_data32(2);
        var_freq = get_backup_data32(3);
      }
      else
        caldata_recall(0);
      // Here need restore settings not depend from cal data
      config._brightness = bk.brightness;
      lever_mode = bk.leveler;
      config._vna_mode = get_backup_data32(4) | (1 << VNA_MODE_BACKUP); // refresh backup settings
      set_bandwidth(bk.bw);
    }
    else
      caldata_recall(0); // Try load 0 slot
  }
  else
    caldata_recall(0); // Try load 0 slot
  update_frequencies();
#ifdef __VNA_MEASURE_MODULE__
  plot_set_measure_mode(current_props._measure);
#endif
}
#else
static void load_settings(void)
{
  load_default_properties();
  config_recall();
  load_properties(0);
}
#endif

int load_properties(uint32_t id)
{
  int r = caldata_recall(id);
  update_frequencies();
#ifdef __VNA_MEASURE_MODULE__
  plot_set_measure_mode(current_props._measure);
#endif
  return r;
}

#ifdef ENABLED_DUMP_COMMAND
audio_sample_t *dump_buffer;
volatile int16_t dump_len = 0;
int16_t dump_selection = 0;
static void
duplicate_buffer_to_dump(audio_sample_t *p, size_t n)
{
  p += dump_selection;
  while (n)
  {
    if (dump_len == 0)
      return;
    dump_len--;
    *dump_buffer++ = *p;
    p += 2;
    n -= 2;
  }
}
#endif

// DMA i2s callback function, called on get 'half' and 'full' buffer size data need for process data, while DMA fill next buffer
static systime_t ready_time = 0;
// sweep operation variables
volatile uint16_t wait_count = 0;
// i2s buffer must be 2x size (for process one while next buffer filled by DMA)
static audio_sample_t rx_buffer[AUDIO_BUFFER_LEN * 2];

void i2s_lld_serve_rx_interrupt(uint32_t flags)
{
  // if ((flags & (STM32_DMA_ISR_TCIF|STM32_DMA_ISR_HTIF)) == 0) return;
  uint16_t wait = wait_count;
  if (wait == 0 || chVTGetSystemTimeX() < ready_time)
    return;
  uint16_t count = AUDIO_BUFFER_LEN;
  audio_sample_t *p = (flags & STM32_DMA_ISR_TCIF) ? rx_buffer + AUDIO_BUFFER_LEN : rx_buffer; // Full or Half transfer complete
  if (wait >= config._bandwidth + 2)                                                           // At this moment in buffer exist noise data, reset and wait next clean buffer
    reset_dsp_accumerator();
  else
    dsp_process(p, count);
#ifdef ENABLED_DUMP_COMMAND
  duplicate_buffer_to_dump(p, count);
#endif
  --wait_count;
}

#ifdef ENABLE_SI5351_TIMINGS
extern uint16_t timings[16];
#undef DELAY_CHANNEL_CHANGE
#undef DELAY_SWEEP_START
#define DELAY_CHANNEL_CHANGE timings[3]
#define DELAY_SWEEP_START timings[4]
#endif

#define DSP_START(delay)                       \
  {                                            \
    ready_time = chVTGetSystemTimeX() + delay; \
    wait_count = config._bandwidth + 2;        \
  }
#define DSP_WAIT     \
  while (wait_count) \
  {                  \
    __WFI();         \
  }
#define RESET_SWEEP \
  {                 \
    p_sweep = 0;    \
  }

#define SWEEP_CH0_MEASURE (1 << 0)
#define SWEEP_CH1_MEASURE (1 << 1)
#define SWEEP_APPLY_EDELAY_S11 (1 << 2)
#define SWEEP_APPLY_EDELAY_S21 (1 << 3)
#define SWEEP_APPLY_S21_OFFSET (1 << 4)
#define SWEEP_APPLY_CALIBRATION (1 << 5)
#define SWEEP_USE_INTERPOLATION (1 << 6)
#define SWEEP_USE_RENORMALIZATION (1 << 7)

static uint16_t get_sweep_mask(void)
{
  uint16_t ch_mask = 0;
#if 0
  // Sweep only used channels (FIXME strange bug in 400-500M on switch channels if only 1 Trace)
  int t;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if ((trace[t].channel&1) == 0) ch_mask|= SWEEP_CH0_MEASURE;
    else/*if (trace[t].channel == 1)*/ ch_mask|= SWEEP_CH1_MEASURE;
  }
#else
  // sweep 2 channels in any case
  ch_mask |= SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE;
#endif

#ifdef __VNA_MEASURE_MODULE__
  // For measure calculations need data
  ch_mask |= plot_get_measure_channels();
#endif
#ifdef __VNA_Z_RENORMALIZATION__
  if (current_props._portz != cal_load_r)
    ch_mask |= SWEEP_USE_RENORMALIZATION;
#endif
  if (cal_status & CALSTAT_APPLY)
    ch_mask |= SWEEP_APPLY_CALIBRATION;
  if (cal_status & CALSTAT_INTERPOLATED)
    ch_mask |= SWEEP_USE_INTERPOLATION;
  if (electrical_delayS11)
    ch_mask |= SWEEP_APPLY_EDELAY_S11;
  if (electrical_delayS21)
    ch_mask |= SWEEP_APPLY_EDELAY_S21;
  if (s21_offset)
    ch_mask |= SWEEP_APPLY_S21_OFFSET;
  return ch_mask;
}

static void applyEDelay(float w, float data[2])
{
  float s, c;
  float real = data[0];
  float imag = data[1];
  vna_sincosf(w, &s, &c);
  data[0] = real * c - imag * s;
  data[1] = imag * c + real * s;
}

static void applyOffset(float data[2], float offset)
{
  data[0] *= offset;
  data[1] *= offset;
}

#ifdef __VNA_Z_RENORMALIZATION__
#include "vna_modules/vna_renorm.c"
#endif

// main loop for measurement
static bool sweep(bool break_on_operation, uint16_t mask)
{
  if (p_sweep >= sweep_points || break_on_operation == false)
    RESET_SWEEP;
  if (break_on_operation && mask == 0)
    return false;
  float data[4];
  float c_data[CAL_TYPE_COUNT][2];
  // Blink LED while scanning
  palClearPad(GPIOC, GPIOC_LED);
  int delay = 0;
  float offset = vna_expf(s21_offset * (logf(10.0f) / 20.0f));
  //  START_PROFILE;
  lcd_set_background(LCD_SWEEP_LINE_COLOR);
  // Wait some time for stable power
  int st_delay = DELAY_SWEEP_START;
  int bar_start = 0;
  int interpolation_idx;

  for (; p_sweep < sweep_points; p_sweep++)
  {
    freq_t frequency = getFrequency(p_sweep);
    // Need made measure - set frequency
    if (mask & (SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE))
    {
      delay = set_frequency(frequency);
      interpolation_idx = mask & SWEEP_USE_INTERPOLATION ? -1 : p_sweep;
    }
    // CH0:REFLECTION, reset and begin measure
    if (mask & SWEEP_CH0_MEASURE)
    {
      tlv320aic3204_select(0);
      DSP_START(delay + st_delay);
      delay = DELAY_CHANNEL_CHANGE;
      // Get calibration data
      if (mask & SWEEP_APPLY_CALIBRATION)
        cal_interpolate(interpolation_idx, frequency, c_data);
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(&data[0]);           // calculate reflection coefficient
      if (mask & SWEEP_APPLY_CALIBRATION) // Apply calibration
        apply_CH0_error_term(data, c_data);
    }
    // CH1:TRANSMISSION, reset and begin measure
    if (mask & SWEEP_CH1_MEASURE)
    {
      tlv320aic3204_select(1);
      DSP_START(delay + st_delay);
      // Get calibration data, only if not do this in 0 channel wait
      if ((mask & SWEEP_APPLY_CALIBRATION) && !(mask & SWEEP_CH0_MEASURE))
        cal_interpolate(interpolation_idx, frequency, c_data);
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(&data[2]);           // Measure transmission coefficient
      if (mask & SWEEP_APPLY_CALIBRATION) // Apply calibration
        apply_CH1_error_term(data, c_data);
    }
#ifdef __VNA_Z_RENORMALIZATION__
    if (mask & SWEEP_USE_RENORMALIZATION)
      apply_renormalization(data, mask);
#endif
    if (p_sweep < SWEEP_POINTS_MAX)
    {
      if (mask & SWEEP_CH0_MEASURE)
      {
        if (mask & SWEEP_APPLY_EDELAY_S11)
          applyEDelay(electrical_delayS11 * frequency, &data[0]); // Apply e-delay
        measured[0][p_sweep][0] = data[0];
        measured[0][p_sweep][1] = data[1];
      }
      if (mask & SWEEP_CH1_MEASURE)
      {
        if (mask & SWEEP_APPLY_EDELAY_S21)
          applyEDelay(electrical_delayS21 * frequency, &data[2]); // Apply e-delay
        if (mask & SWEEP_APPLY_S21_OFFSET)
          applyOffset(&data[2], offset);
        measured[1][p_sweep][0] = data[2];
        measured[1][p_sweep][1] = data[3];
      }
    }
    if (operation_requested && break_on_operation)
      break;
    st_delay = 0;
    // Display SPI made noise on measurement (can see in CW mode), use reduced update
    if (config._bandwidth >= BANDWIDTH_100)
    {
      int current_bar = (p_sweep * WIDTH) / (sweep_points - 1);
      if (current_bar - bar_start > 0)
      {
        lcd_fill(OFFSETX + CELLOFFSETX + bar_start, OFFSETY, current_bar - bar_start, 1);
        bar_start = current_bar;
      }
    }
  }
  if (bar_start)
  {
    lcd_set_background(LCD_GRID_COLOR);
    lcd_fill(OFFSETX + CELLOFFSETX, OFFSETY, bar_start, 1);
  }

  //  STOP_PROFILE;
  // blink LED while scanning
  palSetPad(GPIOC, GPIOC_LED);
  return p_sweep == sweep_points;
}

#ifdef ENABLED_DUMP_COMMAND
VNA_SHELL_FUNCTION(cmd_dump)
{
  int i, j;
  audio_sample_t dump[96 * 2];
  dump_buffer = dump;
  dump_len = ARRAY_COUNT(dump);
  int len = dump_len;
  if (argc == 1)
    dump_selection = my_atoi(argv[0]) == 1 ? 0 : 1;

  tlv320aic3204_select(0);
  DSP_START(DELAY_SWEEP_START);
  while (dump_len > 0)
  {
    __WFI();
  }
  for (i = 0, j = 0; i < len; i++)
  {
    shell_printf("%6d ", dump[i]);
    if (++j == 12)
    {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      j = 0;
    }
  }
}
#endif

#ifdef ENABLE_GAIN_COMMAND
VNA_SHELL_FUNCTION(cmd_gain)
{
  int rvalue = 0;
  int lvalue = 0;
  if (argc == 0 && argc > 2)
  {
    shell_printf("usage: gain {lgain(0-95)} [rgain(0-95)]" VNA_SHELL_NEWLINE_STR);
    return;
  };
  lvalue = rvalue = my_atoui(argv[0]);
  if (argc == 3)
    rvalue = my_atoui(argv[1]);
  tlv320aic3204_set_gain(lvalue, rvalue);
}
#endif

static int set_frequency(freq_t freq)
{
  return si5351_set_frequency(freq, current_props._power);
}

void set_bandwidth(uint16_t bw_count)
{
  config._bandwidth = bw_count & 0x1FF;
  request_to_redraw(REDRAW_BACKUP | REDRAW_FREQUENCY);
}

uint32_t get_bandwidth_frequency(uint16_t bw_freq)
{
  return (AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT) / (bw_freq + 1);
}

#define MAX_BANDWIDTH (AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT)
#define MIN_BANDWIDTH ((AUDIO_ADC_FREQ / AUDIO_SAMPLES_COUNT) / 512 + 1)

VNA_SHELL_FUNCTION(cmd_bandwidth)
{
  uint16_t user_bw;
  if (argc == 1)
    user_bw = my_atoui(argv[0]);
  else if (argc == 2)
  {
    uint16_t f = my_atoui(argv[0]);
    if (f > MAX_BANDWIDTH)
      user_bw = 0;
    else if (f < MIN_BANDWIDTH)
      user_bw = 511;
    else
      user_bw = ((AUDIO_ADC_FREQ + AUDIO_SAMPLES_COUNT / 2) / AUDIO_SAMPLES_COUNT) / f - 1;
  }
  else
    goto result;
  set_bandwidth(user_bw);
result:
{
  uint32_t bw_hz = get_bandwidth_frequency(config._bandwidth);
  float bw_khz = (float)bw_hz / 1000.0f;
  shell_printf("bandwidth %d (%u Hz, %.2f kHz)" VNA_SHELL_NEWLINE_STR,
               config._bandwidth, bw_hz, bw_khz);
}
}

void set_sweep_points(uint16_t points)
{
  if (points > SWEEP_POINTS_MAX)
    points = SWEEP_POINTS_MAX;
  if (points < SWEEP_POINTS_MIN)
    points = SWEEP_POINTS_MIN;
  if (points == sweep_points)
    return;
  sweep_points = points;
  update_frequencies();
}

/*
 * Frequency list functions
 */
#ifdef __USE_FREQ_TABLE__
static freq_t frequencies[SWEEP_POINTS_MAX];
static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  uint32_t i;
  freq_t step = (points - 1);
  freq_t span = stop - start;
  freq_t delta = span / step;
  freq_t error = span % step;
  freq_t f = start, df = step >> 1;
  for (i = 0; i <= step; i++, f += delta)
  {
    frequencies[i] = f;
    if ((df += error) >= step)
    {
      f++;
      df -= step;
    }
  }
  // disable at out of sweep range
  for (; i < SWEEP_POINTS_MAX; i++)
    frequencies[i] = 0;
}
#define _c_start frequencies[0]
#define _c_stop frequencies[sweep_points - 1]
#define _c_points (sweep_points)

freq_t getFrequency(uint16_t idx) { return frequencies[idx]; }
#else
static freq_t _f_start;
static freq_t _f_delta;
static freq_t _f_error;
static uint16_t _f_points;

static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  freq_t span = stop - start;
  _f_start = start;
  _f_points = (points - 1);
  _f_delta = span / _f_points;
  _f_error = span % _f_points;
}
freq_t getFrequency(uint16_t idx) { return _f_start + _f_delta * idx + (_f_points / 2 + _f_error * idx) / _f_points; }
freq_t getFrequencyStep(void) { return _f_delta; }
#endif

static bool needInterpolate(freq_t start, freq_t stop, uint16_t points)
{
  return start != cal_frequency0 || stop != cal_frequency1 || points != cal_sweep_points;
}

#define SCAN_MASK_OUT_FREQ 0b00000001
#define SCAN_MASK_OUT_DATA0 0b00000010
#define SCAN_MASK_OUT_DATA1 0b00000100
#define SCAN_MASK_NO_CALIBRATION 0b00001000
#define SCAN_MASK_NO_EDELAY 0b00010000
#define SCAN_MASK_NO_S21OFFS 0b00100000
#define SCAN_MASK_BINARY 0b10000000

VNA_SHELL_FUNCTION(cmd_scan)
{
  freq_t start, stop;
  uint16_t points = sweep_points;
  if (argc < 2 || argc > 4)
  {
    shell_printf("usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]" VNA_SHELL_NEWLINE_STR);
    return;
  }

  start = my_atoui(argv[0]);
  stop = my_atoui(argv[1]);
  if (start == 0 || stop == 0 || start > stop)
  {
    shell_printf("frequency range is invalid" VNA_SHELL_NEWLINE_STR);
    return;
  }
  if (argc >= 3)
  {
    points = my_atoui(argv[2]);
    if (points == 0 || points > SWEEP_POINTS_MAX)
    {
      shell_printf("sweep points exceeds range " define_to_STR(SWEEP_POINTS_MAX) VNA_SHELL_NEWLINE_STR);
      return;
    }
    sweep_points = points;
  }
  uint16_t mask = 0;
  uint16_t sweep_ch = SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE;

#ifdef ENABLE_SCANBIN_COMMAND
  if (argc == 4)
  {
    mask = my_atoui(argv[3]);
    if (sweep_mode & SWEEP_BINARY)
      mask |= SCAN_MASK_BINARY;
    sweep_ch = (mask >> 1) & 3;
  }
  sweep_mode &= ~(SWEEP_BINARY);
#else
  if (argc == 4)
  {
    mask = my_atoui(argv[3]);
    sweep_ch = (mask >> 1) & 3;
  }
#endif

  if ((cal_status & CALSTAT_APPLY) && !(mask & SCAN_MASK_NO_CALIBRATION))
    sweep_ch |= SWEEP_APPLY_CALIBRATION;
  if (electrical_delayS11 && !(mask & SCAN_MASK_NO_EDELAY))
    sweep_ch |= SWEEP_APPLY_EDELAY_S11;
  if (electrical_delayS21 && !(mask & SCAN_MASK_NO_EDELAY))
    sweep_ch |= SWEEP_APPLY_EDELAY_S21;
  if (s21_offset && !(mask & SCAN_MASK_NO_S21OFFS))
    sweep_ch |= SWEEP_APPLY_S21_OFFSET;

  if (needInterpolate(start, stop, sweep_points))
    sweep_ch |= SWEEP_USE_INTERPOLATION;

  sweep_points = points;
  set_frequencies(start, stop, points);
  if (sweep_ch & (SWEEP_CH0_MEASURE | SWEEP_CH1_MEASURE))
    sweep(false, sweep_ch);
  pause_sweep();
  // Output data after if set (faster data receive)
  if (mask)
  {
    if (mask & SCAN_MASK_BINARY)
    {
      shell_write(&mask, sizeof(uint16_t));
      shell_write(&points, sizeof(uint16_t));
      for (int i = 0; i < points; i++)
      {
        if (mask & SCAN_MASK_OUT_FREQ)
        {
          freq_t f = getFrequency(i);
          shell_write(&f, sizeof(freq_t));
        } // 4 bytes .. frequency
        if (mask & SCAN_MASK_OUT_DATA0)
          shell_write(&measured[0][i][0], sizeof(float) * 2); // 4+4 bytes .. S11 real/imag
        if (mask & SCAN_MASK_OUT_DATA1)
          shell_write(&measured[1][i][0], sizeof(float) * 2); // 4+4 bytes .. S21 real/imag
      }
    }
    else
    {
      for (int i = 0; i < points; i++)
      {
        if (mask & SCAN_MASK_OUT_FREQ)
          shell_printf(VNA_FREQ_FMT_STR " ", getFrequency(i));
        if (mask & SCAN_MASK_OUT_DATA0)
          shell_printf("%f %f ", measured[0][i][0], measured[0][i][1]);
        if (mask & SCAN_MASK_OUT_DATA1)
          shell_printf("%f %f ", measured[1][i][0], measured[1][i][1]);
        shell_printf(VNA_SHELL_NEWLINE_STR);
      }
    }
  }
}

#ifdef ENABLE_SCANBIN_COMMAND
VNA_SHELL_FUNCTION(cmd_scan_bin)
{
  sweep_mode |= SWEEP_BINARY;
  cmd_scan(argc, argv);
  sweep_mode &= ~(SWEEP_BINARY);
}
#endif

VNA_SHELL_FUNCTION(cmd_tcxo)
{
  if (argc != 1)
  {
    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR
                 "current: %u" VNA_SHELL_NEWLINE_STR,
                 "tcxo {TCXO frequency(Hz)}", config._xtal_freq);
    return;
  }
  si5351_set_tcxo(my_atoui(argv[0]));
}

void set_marker_index(int m, int idx)
{
  if (m == MARKER_INVALID || (uint32_t)idx >= sweep_points)
    return;
  markers[m].frequency = getFrequency(idx);
  if (markers[m].index == idx)
    return;
  request_to_draw_marker(markers[m].index); // Mark old marker position for erase
  markers[m].index = idx;                   // Set new position
  request_to_redraw(REDRAW_MARKER);
}

freq_t get_marker_frequency(int marker)
{
  if ((uint32_t)marker >= MARKERS_MAX)
    return 0;
  return markers[marker].frequency;
}

static void
update_marker_index(freq_t fstart, freq_t fstop, uint16_t points)
{
  int m, idx;
  for (m = 0; m < MARKERS_MAX; m++)
  {
    // Update index for all markers !!
    freq_t f = markers[m].frequency;
    if (f == 0)
      idx = markers[m].index; // Not need update index in no freq
    else if (f <= fstart)
      idx = 0;
    else if (f >= fstop)
      idx = points - 1;
    else
    { // Search frequency index for marker frequency
#if 0
      for (idx = 1; idx < points; idx++) {
        if (frequencies[idx] <= f) continue;
        if (f < (frequencies[idx-1]/2 + frequencies[idx]/2)) idx--; // Correct closest idx
        break;
      }
#else
      float r = ((float)(f - fstart)) / (fstop - fstart);
      idx = r * (points - 1);
#endif
    }
    set_marker_index(m, idx);
  }
}

static void
update_frequencies(void)
{
  freq_t start = get_sweep_frequency(ST_START);
  freq_t stop = get_sweep_frequency(ST_STOP);

  set_frequencies(start, stop, sweep_points);

  update_marker_index(start, stop, sweep_points);
  // set grid layout
  update_grid(start, stop);
  // Update interpolation flag
  if (needInterpolate(start, stop, sweep_points))
    cal_status |= CALSTAT_INTERPOLATED;
  else
    cal_status &= ~CALSTAT_INTERPOLATED;

  request_to_redraw(REDRAW_BACKUP | REDRAW_PLOT | REDRAW_CAL_STATUS | REDRAW_FREQUENCY | REDRAW_AREA);
  RESET_SWEEP;
}

void set_sweep_frequency(uint16_t type, freq_t freq)
{
  // Check frequency for out of bounds (minimum SPAN can be any value)
  if (type < ST_SPAN && freq < FREQUENCY_MIN)
    freq = FREQUENCY_MIN;
  // One point step input, so change stop freq or span depend from mode
  if (type == ST_STEP)
  {
    freq *= (sweep_points - 1);
    type = FREQ_IS_CENTERSPAN() ? ST_SPAN : ST_STOP;
    if (type == ST_STOP)
      freq += frequency0;
  }
  if (freq > FREQUENCY_MAX)
    freq = FREQUENCY_MAX;
  freq_t center, span;
  switch (type)
  {
  case ST_START:
    FREQ_STARTSTOP();
    frequency0 = freq;
    // if start > stop then make start = stop
    if (frequency1 < freq)
      frequency1 = freq;
    break;
  case ST_STOP:
    FREQ_STARTSTOP()
    frequency1 = freq;
    // if start > stop then make start = stop
    if (frequency0 > freq)
      frequency0 = freq;
    break;
  case ST_CENTER:
    FREQ_CENTERSPAN();
    center = freq;
    span = (frequency1 - frequency0 + 1) >> 1;
    if (span > center - FREQUENCY_MIN)
      span = (center - FREQUENCY_MIN);
    if (span > FREQUENCY_MAX - center)
      span = (FREQUENCY_MAX - center);
    frequency0 = center - span;
    frequency1 = center + span;
    break;
  case ST_SPAN:
    FREQ_CENTERSPAN();
    center = get_sweep_frequency(ST_CENTER);
    span = freq >> 1;
    if (center < FREQUENCY_MIN + span)
      center = FREQUENCY_MIN + span;
    if (center > FREQUENCY_MAX - span)
      center = FREQUENCY_MAX - span;
    frequency0 = center - span;
    frequency1 = center + span;
    break;
  case ST_CW:
    FREQ_CENTERSPAN();
    frequency0 = freq;
    frequency1 = freq;
    break;
  case ST_VAR:
    var_freq = freq;
    request_to_redraw(REDRAW_BACKUP);
    return;
  }
  update_frequencies();
}

void reset_sweep_frequency(void)
{
  frequency0 = cal_frequency0;
  frequency1 = cal_frequency1;
  sweep_points = cal_sweep_points;
  update_frequencies();
}

VNA_SHELL_FUNCTION(cmd_sweep)
{
  if (argc == 0)
  {
    shell_printf(VNA_FREQ_FMT_STR " " VNA_FREQ_FMT_STR " %d" VNA_SHELL_NEWLINE_STR, get_sweep_frequency(ST_START), get_sweep_frequency(ST_STOP), sweep_points);
    return;
  }
  else if (argc > 3)
  {
    goto usage;
  }
  freq_t value0 = 0;
  freq_t value1 = 0;
  uint32_t value2 = 0;
  if (argc >= 1)
    value0 = my_atoui(argv[0]);
  if (argc >= 2)
    value1 = my_atoui(argv[1]);
  if (argc >= 3)
    value2 = my_atoui(argv[2]);
#if MAX_FREQ_TYPE != 5
#error "Sweep mode possibly changed, check cmd_sweep function"
#endif
  // Parse sweep {start|stop|center|span|cw|step|var} {freq(Hz)}
  // get enum ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW, ST_STEP, ST_VAR
  static const char sweep_cmd[] = "start|stop|center|span|cw|step|var";
  if (argc == 2 && value0 == 0)
  {
    int type = get_str_index(argv[0], sweep_cmd);
    if (type == -1)
      goto usage;
    set_sweep_frequency(type, value1);
    return;
  }
  //  Parse sweep {start(Hz)} [stop(Hz)]
  if (value0)
    set_sweep_frequency(ST_START, value0);
  if (value1)
    set_sweep_frequency(ST_STOP, value1);
  if (value2)
    set_sweep_points(value2);
  return;
usage:
  shell_printf("usage: sweep {start(Hz)} [stop(Hz)] [points]" VNA_SHELL_NEWLINE_STR
               "\tsweep {%s} {freq(Hz)}" VNA_SHELL_NEWLINE_STR,
               sweep_cmd);
}

static void
eterm_set(int term, float re, float im)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    cal_data[term][i][0] = re;
    cal_data[term][i][1] = im;
  }
}

static void
eterm_copy(int dst, int src)
{
  memcpy(cal_data[dst], cal_data[src], sizeof cal_data[dst]);
}

static void
eterm_calc_es(void)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao for effeiciency
#if 0
    float c = 50e-15;
    //float c = 1.707e-12;
    float z0 = 50;
    float z = 2 * VNA_PI * frequencies[i] * c * z0;
    float sq = 1 + z*z;
    float s11aor = (1 - z*z) / sq;
    float s11aoi = 2*z / sq;
#else
    float s11aor = 1.0f;
    float s11aoi = 0.0f;
#endif
    // S11mo’= S11mo - Ed
    // S11ms’= S11ms - Ed
    float s11or = cal_data[CAL_OPEN][i][0] - cal_data[ETERM_ED][i][0];
    float s11oi = cal_data[CAL_OPEN][i][1] - cal_data[ETERM_ED][i][1];
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    // Es = (S11mo'/s11ao + S11ms’)/(S11mo' - S11ms’)
    float numr = s11sr + s11or * s11aor - s11oi * s11aoi;
    float numi = s11si + s11oi * s11aor + s11or * s11aoi;
    float denomr = s11or - s11sr;
    float denomi = s11oi - s11si;
    float d = denomr * denomr + denomi * denomi;
    cal_data[ETERM_ES][i][0] = (numr * denomr + numi * denomi) / d;
    cal_data[ETERM_ES][i][1] = (numi * denomr - numr * denomi) / d;
  }
  cal_status &= ~CALSTAT_OPEN;
  cal_status |= CALSTAT_ES;
}

static void
eterm_calc_er(int sign)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // Er = sign*(1-sign*Es)S11ms'
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    float esr = cal_data[ETERM_ES][i][0];
    float esi = cal_data[ETERM_ES][i][1];
    if (sign > 0)
    {
      esr = -esr;
      esi = -esi;
    }
    esr = 1 + esr;
    float err = esr * s11sr - esi * s11si;
    float eri = esr * s11si + esi * s11sr;
    if (sign < 0)
    {
      err = -err;
      eri = -eri;
    }
    cal_data[ETERM_ER][i][0] = err;
    cal_data[ETERM_ER][i][1] = eri;
  }
  cal_status &= ~CALSTAT_SHORT;
  cal_status |= CALSTAT_ER;
}

// CAUTION: Et is inversed for efficiency
static void
eterm_calc_et(void)
{
  int i;
  for (i = 0; i < sweep_points; i++)
  {
    // Et = 1/(S21mt - Ex)
    float etr = cal_data[CAL_THRU][i][0] - cal_data[CAL_ISOLN][i][0];
    float eti = cal_data[CAL_THRU][i][1] - cal_data[CAL_ISOLN][i][1];
    float sq = etr * etr + eti * eti;
    float invr = etr / sq;
    float invi = -eti / sq;
    cal_data[ETERM_ET][i][0] = invr;
    cal_data[ETERM_ET][i][1] = invi;
  }
  cal_status &= ~CALSTAT_THRU;
  cal_status |= CALSTAT_ET;
}

#if 0
void apply_error_term(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
  }
}

static void apply_error_term_at(int i)
{
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
#if 1
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = 0 - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
#else
    // Not made CH1 correction by CH0 data
    float s21ar = s21mr * cal_data[ETERM_ET][i][0] - s21mi * cal_data[ETERM_ET][i][1];
    float s21ai = s21mi * cal_data[ETERM_ET][i][0] + s21mr * cal_data[ETERM_ET][i][1];
#endif
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
}
#endif

static void apply_CH0_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2])
{
  // S11m' = S11m - Ed
  // S11a = S11m' / (Er + Es S11m')
  float s11mr = data[0] - c_data[ETERM_ED][0];
  float s11mi = data[1] - c_data[ETERM_ED][1];
  float err = c_data[ETERM_ER][0] + s11mr * c_data[ETERM_ES][0] - s11mi * c_data[ETERM_ES][1];
  float eri = c_data[ETERM_ER][1] + s11mr * c_data[ETERM_ES][1] + s11mi * c_data[ETERM_ES][0];
  float sq = err * err + eri * eri;
  data[0] = (s11mr * err + s11mi * eri) / sq;
  data[1] = (s11mi * err - s11mr * eri) / sq;
}

static void apply_CH1_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2])
{
  // CAUTION: Et is inversed for efficiency
  // S21a = (S21m - Ex) * Et`
  float s21mr = data[2] - c_data[ETERM_EX][0];
  float s21mi = data[3] - c_data[ETERM_EX][1];
  // Not made CH1 correction by CH0 data
  data[2] = s21mr * c_data[ETERM_ET][0] - s21mi * c_data[ETERM_ET][1];
  data[3] = s21mi * c_data[ETERM_ET][0] + s21mr * c_data[ETERM_ET][1];
  if (cal_status & CALSTAT_ENHANCED_RESPONSE)
  {
    // S21a*= 1 - Es * S11a
    float esr = 1.0f - (c_data[ETERM_ES][0] * data[0] - c_data[ETERM_ES][1] * data[1]);
    float esi = 0.0f - (c_data[ETERM_ES][1] * data[0] + c_data[ETERM_ES][0] * data[1]);
    float re = data[2];
    float im = data[3];
    data[2] = esr * re - esi * im;
    data[3] = esi * re + esr * im;
  }
}

void cal_collect(uint16_t type)
{
  uint16_t dst, src;

  static const struct
  {
    uint16_t set_flag;
    uint16_t clr_flag;
    uint8_t dst;
    uint8_t src;
  } calibration_set[] = {
      //    type       set data flag                              reset flag  destination source
      [CAL_LOAD] = {CALSTAT_LOAD, ~(CALSTAT_APPLY), CAL_LOAD, 0},
      [CAL_OPEN] = {CALSTAT_OPEN, ~(CALSTAT_ES | CALSTAT_ER | CALSTAT_APPLY), CAL_OPEN, 0},    // Reset Es and Er state
      [CAL_SHORT] = {CALSTAT_SHORT, ~(CALSTAT_ES | CALSTAT_ER | CALSTAT_APPLY), CAL_SHORT, 0}, // Reset Es and Er state
      [CAL_THRU] = {CALSTAT_THRU, ~(CALSTAT_ET | CALSTAT_APPLY), CAL_THRU, 1},                 // Reset Et state
      [CAL_ISOLN] = {CALSTAT_ISOLN, ~(CALSTAT_APPLY), CAL_ISOLN, 1},
  };
  if (type >= ARRAY_COUNT(calibration_set))
    return;

  // reset old calibration if frequency range/points not some
  if (needInterpolate(frequency0, frequency1, sweep_points))
  {
    cal_status = 0;
    cal_frequency0 = frequency0;
    cal_frequency1 = frequency1;
    cal_sweep_points = sweep_points;
  }
  cal_power = current_props._power;

  cal_status &= calibration_set[type].clr_flag;
  cal_status |= calibration_set[type].set_flag;
  dst = calibration_set[type].dst;
  src = calibration_set[type].src;

  // Run sweep for collect data (use minimum BANDWIDTH_30, or bigger if set)
  uint8_t bw = config._bandwidth; // store current setting
  if (bw < BANDWIDTH_100)
    config._bandwidth = BANDWIDTH_100;

  // Set MAX settings for sweep_points on calibrate
  //  if (sweep_points != POINTS_COUNT)
  //    set_sweep_points(POINTS_COUNT);
  uint16_t mask = (src == 0) ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE;
  //  if (electrical_delayS11) mask|= SWEEP_APPLY_EDELAY_S11;
  //  if (electrical_delayS21) mask|= SWEEP_APPLY_EDELAY_S21;
  // Measure calibration data
  sweep(false, mask);
  // Copy calibration data
  memcpy(cal_data[dst], measured[src], sizeof measured[0]);

  // Made average if need
  int count = 1, i, j;
  for (i = 1; i < count; i++)
  {
    sweep(false, (src == 0) ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE);
    for (j = 0; j < sweep_points; j++)
    {
      cal_data[dst][j][0] += measured[src][j][0];
      cal_data[dst][j][1] += measured[src][j][1];
    }
  }
  if (i != 1)
  {
    float k = 1.0f / i;
    for (j = 0; j < sweep_points; j++)
    {
      cal_data[dst][j][0] *= k;
      cal_data[dst][j][1] *= k;
    }
  }

  config._bandwidth = bw; // restore
  request_to_redraw(REDRAW_CAL_STATUS);
}

void cal_done(void)
{
  // Set Load/Ed to default if not calculated
  if (!(cal_status & CALSTAT_LOAD))
    eterm_set(ETERM_ED, 0.0, 0.0);
  // Set Isoln/Ex to default if not measured
  if (!(cal_status & CALSTAT_ISOLN))
    eterm_set(ETERM_EX, 0.0, 0.0);

  // Precalculate Es and Er from Short and Open (and use Load/Ed data)
  if ((cal_status & CALSTAT_SHORT) && (cal_status & CALSTAT_OPEN))
  {
    eterm_calc_es();
    eterm_calc_er(-1);
  }
  else if (cal_status & CALSTAT_OPEN)
  {
    eterm_copy(CAL_SHORT, CAL_OPEN);
    cal_status &= ~CALSTAT_OPEN;
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(1);
  }
  else if (cal_status & CALSTAT_SHORT)
  {
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(-1);
  }

  // Apply Et
  if (cal_status & CALSTAT_THRU)
    eterm_calc_et();

  // Set other fields to default if not set
  if (!(cal_status & CALSTAT_ET))
    eterm_set(ETERM_ET, 1.0, 0.0);
  if (!(cal_status & CALSTAT_ER))
    eterm_set(ETERM_ER, 1.0, 0.0);
  if (!(cal_status & CALSTAT_ES))
    eterm_set(ETERM_ES, 0.0, 0.0);

  cal_status |= CALSTAT_APPLY;
  lastsaveid = NO_SAVE_SLOT;
  request_to_redraw(REDRAW_BACKUP | REDRAW_CAL_STATUS);
}

static void cal_interpolate(int idx, freq_t f, float data[CAL_TYPE_COUNT][2])
{
  int eterm;
  uint16_t src_points = cal_sweep_points - 1;
  if (idx >= 0)
    goto copy_point;
  if (f <= cal_frequency0)
  {
    idx = 0;
    goto copy_point;
  }
  if (f >= cal_frequency1)
  {
    idx = src_points;
    goto copy_point;
  }
  // Calculate k for linear interpolation
  freq_t span = cal_frequency1 - cal_frequency0;
  idx = (uint64_t)(f - cal_frequency0) * (uint64_t)src_points / span;
  uint64_t v = (uint64_t)span * idx + src_points / 2;
  freq_t src_f0 = cal_frequency0 + (v) / src_points;
  freq_t src_f1 = cal_frequency0 + (v + span) / src_points;

  freq_t delta = src_f1 - src_f0;
  // Not need interpolate
  if (f == src_f0)
    goto copy_point;

  float k = (delta == 0) ? 0.0f : (float)(f - src_f0) / delta;
  // avoid glitch between freqs in different harmonics mode
  uint32_t hf0 = si5351_get_harmonic_lvl(src_f0);
  if (hf0 != si5351_get_harmonic_lvl(src_f1))
  {
    // f in prev harmonic, need extrapolate from prev 2 points
    if (hf0 == si5351_get_harmonic_lvl(f))
    {
      if (idx < 1)
        goto copy_point; // point limit
      idx--;
      k += 1.0f;
    }
    // f in next harmonic, need extrapolate from next 2 points
    else
    {
      if (idx >= src_points)
        goto copy_point; // point limit
      idx++;
      k -= 1.0f;
    }
  }
  // Interpolate by k
  for (eterm = 0; eterm < CAL_TYPE_COUNT; eterm++)
  {
    data[eterm][0] = cal_data[eterm][idx][0] + k * (cal_data[eterm][idx + 1][0] - cal_data[eterm][idx][0]);
    data[eterm][1] = cal_data[eterm][idx][1] + k * (cal_data[eterm][idx + 1][1] - cal_data[eterm][idx][1]);
  }
  return;
  // Direct point copy
copy_point:
  for (eterm = 0; eterm < CAL_TYPE_COUNT; eterm++)
  {
    data[eterm][0] = cal_data[eterm][idx][0];
    data[eterm][1] = cal_data[eterm][idx][1];
  }
  return;
}

VNA_SHELL_FUNCTION(cmd_cal)
{
  static const char *items[] = {"load", "open", "short", "thru", "isoln", "Es", "Er", "Et", "cal'ed"};

  if (argc == 0)
  {
    int i;
    for (i = 0; i < 9; i++)
    {
      if (cal_status & (1 << i))
        shell_printf("%s ", items[i]);
    }
    shell_printf(VNA_SHELL_NEWLINE_STR);
    return;
  }
  request_to_redraw(REDRAW_CAL_STATUS);
  //                                     0    1     2    3     4    5  6   7     8
  static const char cmd_cal_list[] = "load|open|short|thru|isoln|done|on|off|reset";
  switch (get_str_index(argv[0], cmd_cal_list))
  {
  case 0:
    cal_collect(CAL_LOAD);
    return;
  case 1:
    cal_collect(CAL_OPEN);
    return;
  case 2:
    cal_collect(CAL_SHORT);
    return;
  case 3:
    cal_collect(CAL_THRU);
    return;
  case 4:
    cal_collect(CAL_ISOLN);
    return;
  case 5:
    cal_done();
    return;
  case 6:
    cal_status |= CALSTAT_APPLY;
    return;
  case 7:
    cal_status &= ~CALSTAT_APPLY;
    return;
  case 8:
    cal_status = 0;
    return;
  default:
    break;
  }
  shell_printf("usage: cal [%s]" VNA_SHELL_NEWLINE_STR, cmd_cal_list);
}

VNA_SHELL_FUNCTION(cmd_save)
{
  uint32_t id;
  if (argc == 1 && (id = my_atoui(argv[0])) < SAVEAREA_MAX)
  {
    caldata_save(id);
    request_to_redraw(REDRAW_CAL_STATUS);
    return;
  }
  shell_printf("usage: %s 0..%d" VNA_SHELL_NEWLINE_STR, SAVEAREA_MAX - 1, "save");
}

VNA_SHELL_FUNCTION(cmd_recall)
{
  uint32_t id;
  if (argc == 1 && (id = my_atoui(argv[0])) < SAVEAREA_MAX)
  {
    if (load_properties(id)) // Check for success
      shell_printf("Err, default load" VNA_SHELL_NEWLINE_STR);
    return;
  }
  shell_printf("usage: %s 0..%d" VNA_SHELL_NEWLINE_STR, SAVEAREA_MAX - 1, "recall");
}

static const char *const trc_channel_name[] = {
    "S11", "S21"};

const char *get_trace_chname(int t)
{
  return trc_channel_name[trace[t].channel & 1];
}

void set_trace_type(int t, int type, int channel)
{
  channel &= 1;
  bool update = trace[t].type != type || trace[t].channel != channel;
  if (!update)
    return;
  if (trace[t].type != type)
  {
    trace[t].type = type;
    // Set default trace refpos
    set_trace_refpos(t, trace_info_list[type].refpos);
    // Set default trace scale
    set_trace_scale(t, trace_info_list[type].scale_unit);
    request_to_redraw(REDRAW_AREA | REDRAW_PLOT | REDRAW_BACKUP); // need for update grid
  }
  set_trace_channel(t, channel);
}

void set_trace_channel(int t, int channel)
{
  channel &= 1;
  if (trace[t].channel != channel)
  {
    trace[t].channel = channel;
    request_to_redraw(REDRAW_MARKER | REDRAW_PLOT);
  }
}

void set_active_trace(int t)
{
  if (current_trace == t)
    return;
  current_trace = t;
  request_to_redraw(REDRAW_MARKER | REDRAW_GRID_VALUE);
}

void set_trace_scale(int t, float scale)
{
  if (trace[t].scale != scale)
  {
    trace[t].scale = scale;
    request_to_redraw(REDRAW_MARKER | REDRAW_GRID_VALUE | REDRAW_PLOT);
  }
}

void set_trace_refpos(int t, float refpos)
{
  if (trace[t].refpos != refpos)
  {
    trace[t].refpos = refpos;
    request_to_redraw(REDRAW_REFERENCE | REDRAW_GRID_VALUE | REDRAW_PLOT);
  }
}

void set_trace_enable(int t, bool enable)
{
  trace[t].enabled = enable;
  current_trace = enable ? t : TRACE_INVALID;
  if (!enable)
  {
    for (int i = 0; i < TRACES_MAX; i++) // set first enabled as current trace
      if (trace[i].enabled)
      {
        set_active_trace(i);
        break;
      }
  }
  request_to_redraw(REDRAW_AREA);
}

void set_electrical_delay(int ch, float seconds)
{
  if (current_props._electrical_delay[ch] == seconds)
    return;
  current_props._electrical_delay[ch] = seconds;
  request_to_redraw(REDRAW_MARKER);
}

float get_electrical_delay(void)
{
  if (current_trace == TRACE_INVALID)
    return 0.0f;
  int ch = trace[current_trace].channel;
  return current_props._electrical_delay[ch];
}

void set_s21_offset(float offset)
{
  if (s21_offset != offset)
  {
    s21_offset = offset;
    request_to_redraw(REDRAW_MARKER);
  }
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  uint32_t t;
  if (argc == 0)
  {
    for (t = 0; t < TRACES_MAX; t++)
    {
      if (trace[t].enabled)
      {
        const char *type = get_trace_typename(trace[t].type, 0);
        const char *channel = get_trace_chname(t);
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        shell_printf("%d %s %s %f %f" VNA_SHELL_NEWLINE_STR, t, type, channel, scale, refpos);
      }
    }
    return;
  }

  if (get_str_index(argv[0], "all") == 0 &&
      argc > 1 && get_str_index(argv[1], "off") == 0)
  {
    for (t = 0; t < TRACES_MAX; t++)
      set_trace_enable(t, false);
    return;
  }

  t = (uint32_t)my_atoi(argv[0]);
  if (t >= TRACES_MAX)
    goto usage;
  if (argc == 1)
  {
    const char *type = get_trace_typename(trace[t].type, 0);
    const char *channel = get_trace_chname(t);
    shell_printf("%d %s %s" VNA_SHELL_NEWLINE_STR, t, type, channel);
    return;
  }
  if (get_str_index(argv[1], "off") == 0)
  {
    set_trace_enable(t, false);
    return;
  }
#if MAX_TRACE_TYPE != 30
#error "Trace type enum possibly changed, check cmd_trace function"
#endif
  // enum TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Z, TRC_ZPHASE,
  //      TRC_G, TRC_B, TRC_Y, TRC_Rp, TRC_Xp, TRC_sC, TRC_sL, TRC_pC, TRC_pL, TRC_Q, TRC_Rser, TRC_Xser, TRC_Zser, TRC_Rsh, TRC_Xsh, TRC_Zsh, TRC_Qs21
  static const char cmd_type_list[] = "logmag|phase|delay|smith|polar|linear|swr|real|imag|r|x|z|zp|g|b|y|rp|xp|cs|ls|cp|lp|q|rser|xser|zser|rsh|xsh|zsh|q21";
  int type = get_str_index(argv[1], cmd_type_list);
  if (type >= 0)
  {
    int src = trace[t].channel;
    if (argc > 2)
    {
      src = my_atoi(argv[2]);
      if ((uint32_t)src > 1)
        goto usage;
    }
    set_trace_type(t, type, src);
    set_trace_enable(t, true);
    return;
  }

  static const char cmd_marker_smith[] = "lin|log|ri|rx|rlc|gb|glc|rpxp|rplc|rxsh|rlcsh|rxser|rlcser";
  // Set marker smith format
  int format = get_str_index(argv[1], cmd_marker_smith);
  if (format >= 0)
  {
    trace[t].smith_format = format;
    return;
  }
  //                                            0      1
  static const char cmd_scale_ref_list[] = "scale|refpos";
  if (argc >= 3)
  {
    switch (get_str_index(argv[1], cmd_scale_ref_list))
    {
    case 0:
      set_trace_scale(t, my_atof(argv[2]));
      break;
    case 1:
      set_trace_refpos(t, my_atof(argv[2]));
      break;
    default:
      goto usage;
    }
  }
  return;
usage:
  shell_printf("trace {0|1|2|3|all} [%s] [src]" VNA_SHELL_NEWLINE_STR
               "trace {0|1|2|3} [%s]" VNA_SHELL_NEWLINE_STR
               "trace {0|1|2|3} {%s} {value}" VNA_SHELL_NEWLINE_STR,
               cmd_type_list, cmd_marker_smith, cmd_scale_ref_list);
}

VNA_SHELL_FUNCTION(cmd_edelay)
{
  int ch = 0;
  float value;
  static const char cmd_edelay_list[] = "s11|s21";
  if (argc >= 1)
  {
    int idx = get_str_index(argv[0], cmd_edelay_list);
    if (idx == -1)
      value = my_atof(argv[0]);
    else
    {
      ch = idx;
      if (argc != 2)
        goto usage;
      value = my_atof(argv[0]);
    }
    set_electrical_delay(ch, value * 1e-12); // input value in seconds
    return;
  }
usage:
  shell_printf("%f" VNA_SHELL_NEWLINE_STR, current_props._electrical_delay[ch] * (1.0f / 1e-12f)); // return in picoseconds
}

VNA_SHELL_FUNCTION(cmd_s21offset)
{
  if (argc != 1)
  {
    shell_printf("%f" VNA_SHELL_NEWLINE_STR, s21_offset); // return in dB
    return;
  }
  set_s21_offset(my_atof(argv[0])); // input value in dB
}

VNA_SHELL_FUNCTION(cmd_marker)
{
  static const char cmd_marker_list[] = "on|off";
  int t;
  if (argc == 0)
  {
    for (t = 0; t < MARKERS_MAX; t++)
    {
      if (markers[t].enabled)
      {
        shell_printf("%d %d " VNA_FREQ_FMT_STR "" VNA_SHELL_NEWLINE_STR, t + 1, markers[t].index, markers[t].frequency);
      }
    }
    return;
  }
  request_to_redraw(REDRAW_MARKER | REDRAW_AREA);
  // Marker on|off command
  int enable = get_str_index(argv[0], cmd_marker_list);
  if (enable >= 0)
  { // string found: 0 - on, 1 - off
    active_marker = enable == 1 ? MARKER_INVALID : 0;
    for (t = 0; t < MARKERS_MAX; t++)
      markers[t].enabled = enable == 0;
    return;
  }

  t = my_atoi(argv[0]) - 1;
  if (t < 0 || t >= MARKERS_MAX)
    goto usage;
  if (argc == 1)
  {
    shell_printf("%d %d " VNA_FREQ_FMT_STR "" VNA_SHELL_NEWLINE_STR, t + 1, markers[t].index, markers[t].frequency);
    active_marker = t;
    // select active marker
    markers[t].enabled = TRUE;
    return;
  }

  switch (get_str_index(argv[1], cmd_marker_list))
  {
  case 0:
    markers[t].enabled = TRUE;
    active_marker = t;
    return;
  case 1:
    markers[t].enabled = FALSE;
    if (active_marker == t)
      active_marker = MARKER_INVALID;
    return;
  default:
    // select active marker and move to index
    markers[t].enabled = TRUE;
    int index = my_atoi(argv[1]);
    set_marker_index(t, index);
    active_marker = t;
    return;
  }
usage:
  shell_printf("marker [n] [%s|{index}]" VNA_SHELL_NEWLINE_STR, cmd_marker_list);
}

VNA_SHELL_FUNCTION(cmd_touchcal)
{
  (void)argc;
  (void)argv;
  shell_printf("first touch upper left, then lower right...");
  ui_touch_cal_exec();
  shell_printf("done" VNA_SHELL_NEWLINE_STR
               "touch cal params: %d %d %d %d" VNA_SHELL_NEWLINE_STR,
               config._touch_cal[0], config._touch_cal[1], config._touch_cal[2], config._touch_cal[3]);
  request_to_redraw(REDRAW_ALL);
}

VNA_SHELL_FUNCTION(cmd_touchtest)
{
  (void)argc;
  (void)argv;
  ui_touch_draw_test();
}

VNA_SHELL_FUNCTION(cmd_frequencies)
{
  int i;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++)
  {
    shell_printf(VNA_FREQ_FMT_STR VNA_SHELL_NEWLINE_STR, getFrequency(i));
  }
}

#ifdef ENABLE_TRANSFORM_COMMAND
static void
set_domain_mode(int mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (props_mode & DOMAIN_MODE))
  {
    props_mode = (props_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    request_to_redraw(REDRAW_FREQUENCY | REDRAW_MARKER);
    lever_mode = LM_MARKER;
  }
}

static inline void
set_timedomain_func(uint32_t func) // accept TD_FUNC_LOWPASS_IMPULSE, TD_FUNC_LOWPASS_STEP or TD_FUNC_BANDPASS
{
  props_mode = (props_mode & ~TD_FUNC) | func;
}

static inline void
set_timedomain_window(uint32_t func) // accept TD_WINDOW_MINIMUM/TD_WINDOW_NORMAL/TD_WINDOW_MAXIMUM
{
  props_mode = (props_mode & ~TD_WINDOW) | func;
}

VNA_SHELL_FUNCTION(cmd_transform)
{
  int i;
  if (argc == 0)
  {
    goto usage;
  }
  //                                         0   1       2    3        4       5      6       7
  static const char cmd_transform_list[] = "on|off|impulse|step|bandpass|minimum|normal|maximum";
  for (i = 0; i < argc; i++)
  {
    switch (get_str_index(argv[i], cmd_transform_list))
    {
    case 0:
      set_domain_mode(DOMAIN_TIME);
      break;
    case 1:
      set_domain_mode(DOMAIN_FREQ);
      break;
    case 2:
      set_timedomain_func(TD_FUNC_LOWPASS_IMPULSE);
      break;
    case 3:
      set_timedomain_func(TD_FUNC_LOWPASS_STEP);
      break;
    case 4:
      set_timedomain_func(TD_FUNC_BANDPASS);
      break;
    case 5:
      set_timedomain_window(TD_WINDOW_MINIMUM);
      break;
    case 6:
      set_timedomain_window(TD_WINDOW_NORMAL);
      break;
    case 7:
      set_timedomain_window(TD_WINDOW_MAXIMUM);
      break;
    default:
      goto usage;
    }
  }
  return;
usage:
  shell_printf("usage: transform {%s} [...]" VNA_SHELL_NEWLINE_STR, cmd_transform_list);
}
#endif

#ifdef ENABLE_TEST_COMMAND
VNA_SHELL_FUNCTION(cmd_test)
{
  (void)argc;
  (void)argv;

#if 0
  int i;
  for (i = 0; i < 100; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(10000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);

    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(90000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  int i;
  int mode = 0;
  if (argc >= 1)
    mode = my_atoi(argv[0]);

  for (i = 0; i < 20; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    ili9341_test(mode);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  //extern adcsample_t adc_samples[2];
  //shell_printf("adc: %d %d" VNA_SHELL_NEWLINE_STR, adc_samples[0], adc_samples[1]);
  int i;
  int x, y;
  for (i = 0; i < 50; i++) {
    test_touch(&x, &y);
    shell_printf("adc: %d %d" VNA_SHELL_NEWLINE_STR, x, y);
    chThdSleepMilliseconds(200);
  }
  //extern int touch_x, touch_y;
  //shell_printf("adc: %d %d" VNA_SHELL_NEWLINE_STR, touch_x, touch_y);
#endif
#if 0
  while (argc > 1) {
    int16_t x, y;
    touch_position(&x, &y);
    shell_printf("touch: %d %d" VNA_SHELL_NEWLINE_STR, x, y);
    chThdSleepMilliseconds(200);
  }
#endif
}
#endif

#ifdef ENABLE_PORT_COMMAND
VNA_SHELL_FUNCTION(cmd_port)
{
  int port;
  if (argc != 1)
  {
    shell_printf("usage: port {0:TX 1:RX}" VNA_SHELL_NEWLINE_STR);
    return;
  }
  port = my_atoi(argv[0]);
  tlv320aic3204_select(port);
}
#endif

#ifdef ENABLE_STAT_COMMAND
static struct
{
  int16_t rms[2];
  int16_t ave[2];
#if 0
  int callback_count;
  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
#endif
} stat;

VNA_SHELL_FUNCTION(cmd_stat)
{
  int16_t *p = &rx_buffer[0];
  int32_t acc0, acc1;
  int32_t ave0, ave1;
  //  float sample[2], ref[2];
  //  minr, maxr,  mins, maxs;
  int32_t count = AUDIO_BUFFER_LEN;
  int i;
  (void)argc;
  (void)argv;
  for (int ch = 0; ch < 2; ch++)
  {
    tlv320aic3204_select(ch);
    DSP_START(4);
    DSP_WAIT;
    //    reset_dsp_accumerator();
    //    dsp_process(&p[               0], AUDIO_BUFFER_LEN);
    //    dsp_process(&p[AUDIO_BUFFER_LEN], AUDIO_BUFFER_LEN);

    acc0 = acc1 = 0;
    for (i = 0; i < AUDIO_BUFFER_LEN * 2; i += 2)
    {
      acc0 += p[i];
      acc1 += p[i + 1];
    }
    ave0 = acc0 / count;
    ave1 = acc1 / count;
    acc0 = acc1 = 0;
    //    minr  = maxr = 0;
    //    mins  = maxs = 0;
    for (i = 0; i < AUDIO_BUFFER_LEN * 2; i += 2)
    {
      acc0 += (p[i] - ave0) * (p[i] - ave0);
      acc1 += (p[i + 1] - ave1) * (p[i + 1] - ave1);
      //      if (minr < p[i  ]) minr = p[i  ];
      //      if (maxr > p[i  ]) maxr = p[i  ];
      //      if (mins < p[i+1]) mins = p[i+1];
      //      if (maxs > p[i+1]) maxs = p[i+1];
    }
    stat.rms[0] = vna_sqrtf(acc0 / count);
    stat.rms[1] = vna_sqrtf(acc1 / count);
    stat.ave[0] = ave0;
    stat.ave[1] = ave1;
    shell_printf("Ch: %d" VNA_SHELL_NEWLINE_STR, ch);
    shell_printf("average:   r: %6d s: %6d" VNA_SHELL_NEWLINE_STR, stat.ave[0], stat.ave[1]);
    shell_printf("rms:       r: %6d s: %6d" VNA_SHELL_NEWLINE_STR, stat.rms[0], stat.rms[1]);
    //    shell_printf("min:     ref %6d ch %6d" VNA_SHELL_NEWLINE_STR, minr, mins);
    //    shell_printf("max:     ref %6d ch %6d" VNA_SHELL_NEWLINE_STR, maxr, maxs);
  }
  // shell_printf("callback count: %d" VNA_SHELL_NEWLINE_STR, stat.callback_count);
  // shell_printf("interval cycle: %d" VNA_SHELL_NEWLINE_STR, stat.interval_cycles);
  // shell_printf("busy cycle: %d" VNA_SHELL_NEWLINE_STR, stat.busy_cycles);
  // shell_printf("load: %d" VNA_SHELL_NEWLINE_STR, stat.busy_cycles * 100 / stat.interval_cycles);
  //  extern int awd_count;
  //  shell_printf("awd: %d" VNA_SHELL_NEWLINE_STR, awd_count);
}
#endif

#ifndef VERSION
#define VERSION "unknown"
#endif

const char NANOVNA_VERSION[] = VERSION;

VNA_SHELL_FUNCTION(cmd_version)
{
  (void)argc;
  (void)argv;
  shell_printf("%s" VNA_SHELL_NEWLINE_STR, NANOVNA_VERSION);
}

VNA_SHELL_FUNCTION(cmd_vbat)
{
  (void)argc;
  (void)argv;
  shell_printf("%d m" S_VOLT VNA_SHELL_NEWLINE_STR, adc_vbat_read());
}

#ifdef ENABLE_VBAT_OFFSET_COMMAND
VNA_SHELL_FUNCTION(cmd_vbat_offset)
{
  if (argc != 1)
  {
    shell_printf("%d" VNA_SHELL_NEWLINE_STR, config._vbat_offset);
    return;
  }
  config._vbat_offset = (int16_t)my_atoi(argv[0]);
}
#endif

#ifdef ENABLE_SI5351_TIMINGS
VNA_SHELL_FUNCTION(cmd_si5351time)
{
  (void)argc;
  int idx = my_atoui(argv[0]);
  uint16_t value = my_atoui(argv[1]);
  si5351_set_timing(idx, value);
}
#endif

#ifdef ENABLE_SI5351_REG_WRITE
VNA_SHELL_FUNCTION(cmd_si5351reg)
{
#if 0
  (void) argc;
  uint32_t reg = my_atoui(argv[0]);
  uint8_t buf[1] = {0xAA};
  if (si5351_bulk_read(reg, buf, 1))
    shell_printf("si reg[%d] = 0x%02x" VNA_SHELL_NEWLINE_STR, reg, buf[0]);
#else
  if (argc != 2)
  {
    shell_printf("usage: si reg data" VNA_SHELL_NEWLINE_STR);
    return;
  }
  uint8_t reg = my_atoui(argv[0]);
  uint8_t dat = my_atoui(argv[1]);
  uint8_t buf[] = {reg, dat};
  si5351_bulk_write(buf, 2);
#endif
}
#endif

#ifdef ENABLE_I2C_TIMINGS
VNA_SHELL_FUNCTION(cmd_i2ctime)
{
  (void)argc;
  uint32_t tim = STM32_TIMINGR_PRESC(0U) |
                 STM32_TIMINGR_SCLDEL(my_atoui(argv[0])) | STM32_TIMINGR_SDADEL(my_atoui(argv[1])) |
                 STM32_TIMINGR_SCLH(my_atoui(argv[2])) | STM32_TIMINGR_SCLL(my_atoui(argv[3]));
  set_I2C_timings(tim);
}
#endif

#ifdef ENABLE_INFO_COMMAND
VNA_SHELL_FUNCTION(cmd_info)
{
  (void)argc;
  (void)argv;
  int i = 0;
  while (info_about[i])
    shell_printf("%s" VNA_SHELL_NEWLINE_STR, info_about[i++]);
}
#endif

#ifdef ENABLE_COLOR_COMMAND
VNA_SHELL_FUNCTION(cmd_color)
{
  uint32_t color;
  uint16_t i;
  if (argc != 2)
  {
    shell_printf("usage: color {id} {rgb24}" VNA_SHELL_NEWLINE_STR);
    for (i = 0; i < MAX_PALETTE; i++)
    {
      color = GET_PALTETTE_COLOR(i);
      color = HEXRGB(color);
      shell_printf(" %2d: 0x%06x" VNA_SHELL_NEWLINE_STR, i, color);
    }
    return;
  }
  i = my_atoui(argv[0]);
  if (i >= MAX_PALETTE)
    return;
  color = RGBHEX(my_atoui(argv[1]));
  config._lcd_palette[i] = color;
  // Redraw all
  request_to_redraw(REDRAW_ALL);
}
#endif

#ifdef ENABLE_I2C_COMMAND
VNA_SHELL_FUNCTION(cmd_i2c)
{
  if (argc != 3)
  {
    shell_printf("usage: i2c page reg data" VNA_SHELL_NEWLINE_STR);
    return;
  }
  uint8_t page = my_atoui(argv[0]);
  uint8_t reg = my_atoui(argv[1]);
  uint8_t data = my_atoui(argv[2]);
  tlv320aic3204_write_reg(page, reg, data);
}
#endif

#ifdef ENABLE_BAND_COMMAND
VNA_SHELL_FUNCTION(cmd_band)
{
  static const char cmd_sweep_list[] = "mode|freq|div|mul|omul|pow|opow|l|r|lr|adj";
  if (argc != 3)
  {
    shell_printf("cmd error" VNA_SHELL_NEWLINE_STR);
    return;
  }
  int idx = my_atoui(argv[0]);
  int pidx = get_str_index(argv[1], cmd_sweep_list);
  si5351_update_band_config(idx, pidx, my_atoui(argv[2]));
}
#endif

#ifdef ENABLE_LCD_COMMAND
VNA_SHELL_FUNCTION(cmd_lcd)
{
  uint8_t d[VNA_SHELL_MAX_ARGUMENTS];
  if (argc == 0)
    return;
  for (int i = 0; i < argc; i++)
    d[i] = my_atoui(argv[i]);
  uint32_t ret = lcd_send_register(d[0], argc - 1, &d[1]);
  shell_printf("ret = 0x%08X" VNA_SHELL_NEWLINE_STR, ret);
  chThdSleepMilliseconds(5);
}
#endif

#ifdef ENABLE_THREADS_COMMAND
#if CH_CFG_USE_REGISTRY == FALSE
#error "Threads Requite enabled CH_CFG_USE_REGISTRY in chconf.h"
#endif
VNA_SHELL_FUNCTION(cmd_threads)
{
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;
  (void)argc;
  (void)argv;
  shell_printf("stklimit|   stack|stk free|    addr|refs|prio|    state|        name" VNA_SHELL_NEWLINE_STR);
  tp = chRegFirstThread();
  do
  {
    uint32_t max_stack_use = 0U;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#if CH_DBG_FILL_THREADS == TRUE
    uint8_t *p = (uint8_t *)tp->wabase;
    while (p[max_stack_use] == CH_DBG_STACK_FILL_VALUE)
      max_stack_use++;
#endif
#else
    uint32_t stklimit = 0U;
#endif
    shell_printf("%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s" VNA_SHELL_NEWLINE_STR,
                 stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
                 (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
                 tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

#ifdef __USE_SERIAL_CONSOLE__
#ifdef ENABLE_USART_COMMAND
VNA_SHELL_FUNCTION(cmd_usart_cfg)
{
  if (argc != 1)
  {
    //    shell_printf("usage: %s" VNA_SHELL_NEWLINE_STR "current: %u" VNA_SHELL_NEWLINE_STR, "usart_cfg {baudrate}", config._serial_speed);
    shell_printf("Serial: %u baud" VNA_SHELL_NEWLINE_STR, config._serial_speed);
    return;
  }
  uint32_t speed = my_atoui(argv[0]);
  if (speed < 300)
    speed = 300;
  shell_update_speed(speed);
}

VNA_SHELL_FUNCTION(cmd_usart)
{
  uint32_t time = MS2ST(200); // 200ms wait answer by default
  if (argc == 0 || argc > 2 || VNA_MODE(VNA_MODE_CONNECTION))
    return; // Not work in serial mode
  if (argc == 2)
    time = MS2ST(my_atoui(argv[1]));
  sdWriteTimeout(&SD1, (uint8_t *)argv[0], strlen(argv[0]), time);
  sdWriteTimeout(&SD1, (uint8_t *)VNA_SHELL_NEWLINE_STR, sizeof(VNA_SHELL_NEWLINE_STR) - 1, time);
  uint32_t size;
  uint8_t buffer[64];
  while ((size = sdReadTimeout(&SD1, buffer, sizeof(buffer), time)))
    streamWrite(&SDU1, buffer, size);
}
#endif
#endif

#ifdef __REMOTE_DESKTOP__
void send_region(remote_region_t *rd, uint8_t *buf, uint16_t size)
{
  if (SDU1.config->usbp->state == USB_ACTIVE)
  {
    shell_write(rd, sizeof(remote_region_t));
    shell_write(buf, size);
    shell_write(VNA_SHELL_PROMPT_STR VNA_SHELL_NEWLINE_STR, 6);
  }
  else
    sweep_mode &= ~SWEEP_REMOTE;
}

VNA_SHELL_FUNCTION(cmd_refresh)
{
  static const char cmd_enable_list[] = "on|off";
  if (argc != 1)
    return;
  int enable = get_str_index(argv[0], cmd_enable_list);
  if (enable == 0)
    sweep_mode |= SWEEP_REMOTE;
  else if (enable == 1)
    sweep_mode &= ~SWEEP_REMOTE;
  // redraw all on screen
  request_to_redraw(REDRAW_FREQUENCY | REDRAW_CAL_STATUS | REDRAW_AREA | REDRAW_BATTERY);
}

VNA_SHELL_FUNCTION(cmd_touch)
{
  if (argc != 2)
    return;
  remote_touch_set(REMOTE_PRESS, my_atoi(argv[0]), my_atoi(argv[1]));
}

VNA_SHELL_FUNCTION(cmd_release)
{
  int16_t x = -1, y = -1;
  if (argc == 2)
  {
    x = my_atoi(argv[0]);
    y = my_atoi(argv[1]);
  }
  remote_touch_set(REMOTE_RELEASE, x, y);
}
#endif

#ifdef ENABLE_SD_CARD_COMMAND
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_COMMAND"
#endif

static FRESULT cmd_sd_card_mount(void)
{
  const FRESULT res = f_mount(fs_volume, "", 1);
  if (res != FR_OK)
    shell_printf("err: no card" VNA_SHELL_NEWLINE_STR);
  return res;
}

VNA_SHELL_FUNCTION(cmd_sd_list)
{
  DIR dj;
  FILINFO fno;
  if (cmd_sd_card_mount() != FR_OK)
    return;
  switch (argc)
  {
  case 0:
    dj.pat = "*.*";
    break;
  case 1:
    dj.pat = argv[0];
    break;
  default:
    shell_printf("usage: sd_list {pattern}" VNA_SHELL_NEWLINE_STR);
    return;
  }
  if (f_opendir(&dj, "") == FR_OK)
  {
    while (f_findnext(&dj, &fno) == FR_OK && fno.fname[0])
      shell_printf("%s %u" VNA_SHELL_NEWLINE_STR, fno.fname, fno.fsize);
  }
  f_closedir(&dj);
}

VNA_SHELL_FUNCTION(cmd_sd_read)
{
  char *buf = (char *)spi_buffer;
  if (argc != 1)
  {
    shell_printf("usage: sd_read {filename}" VNA_SHELL_NEWLINE_STR);
    return;
  }
  const char *filename = argv[0];
  if (cmd_sd_card_mount() != FR_OK)
    return;

  if (f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ) != FR_OK)
  {
    shell_printf("err: no file" VNA_SHELL_NEWLINE_STR);
    return;
  }
  // shell_printf("sd_read: %s" VNA_SHELL_NEWLINE_STR, filename);
  // number of bytes to follow (file size)
  uint32_t filesize = f_size(fs_file);
  shell_write(&filesize, 4);
  UINT size = 0;
  // file data (send all data from file)
  while (f_read(fs_file, buf, 512, &size) == FR_OK && size > 0)
    shell_write(buf, size);

  f_close(fs_file);
  return;
}

VNA_SHELL_FUNCTION(cmd_sd_delete)
{
  FRESULT res;
  if (argc != 1)
  {
    shell_printf("usage: sd_delete {filename}" VNA_SHELL_NEWLINE_STR);
    return;
  }
  if (cmd_sd_card_mount() != FR_OK)
    return;
  const char *filename = argv[0];
  res = f_unlink(filename);
  shell_printf("delete: %s %s" VNA_SHELL_NEWLINE_STR, filename, res == FR_OK ? "OK" : "err");
  return;
}
#endif

#ifdef __SD_CARD_LOAD__
VNA_SHELL_FUNCTION(cmd_msg)
{
  if (argc == 0)
  {
    shell_printf("usage: msg delay [text] [header]" VNA_SHELL_NEWLINE_STR);
    return;
  }
  uint32_t delay = my_atoui(argv[0]);
  char *header = 0, *text = 0;
  if (argc > 1)
    text = argv[1];
  if (argc > 2)
    header = argv[2];
  ui_message_box(header, text, delay);
}
#endif

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
typedef struct
{
  const char *sc_name;
  vna_shellcmd_t sc_function;
  uint16_t flags;
} VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only in sweep thread, not in main cycle
#define CMD_WAIT_MUTEX 1
// Command execution need in sweep thread, and need break sweep for run
#define CMD_BREAK_SWEEP 2
// Command can run in shell thread (if sweep thread process UI, not sweep)
#define CMD_RUN_IN_UI 4
// Command can run in load script
#define CMD_RUN_IN_LOAD 8

static const VNAShellCommand commands[] =
    {
        {"scan", cmd_scan, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP},
#ifdef ENABLE_SCANBIN_COMMAND
        {"scan_bin", cmd_scan_bin, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP},
#endif
        {"data", cmd_data, 0},
        {"frequencies", cmd_frequencies, 0},
        {"freq", cmd_freq, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
        {"sweep", cmd_sweep, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
        {"power", cmd_power, CMD_RUN_IN_LOAD},
#ifdef USE_VARIABLE_OFFSET
        {"offset", cmd_offset, CMD_WAIT_MUTEX | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#endif
        {"bandwidth", cmd_bandwidth, CMD_RUN_IN_LOAD},
#ifdef __USE_RTC__
        {"time", cmd_time, CMD_RUN_IN_UI},
#endif
#ifdef ENABLE_SD_CARD_COMMAND
        {"sd_list", cmd_sd_list, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
        {"sd_read", cmd_sd_read, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
        {"sd_delete", cmd_sd_delete, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
#endif
#ifdef __VNA_ENABLE_DAC__
        {"dac", cmd_dac, CMD_RUN_IN_LOAD},
#endif
        {"saveconfig", cmd_saveconfig, CMD_RUN_IN_LOAD},
        {"clearconfig", cmd_clearconfig, CMD_RUN_IN_LOAD},
#ifdef ENABLED_DUMP_COMMAND
        {"dump", cmd_dump, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP},
#endif
#ifdef ENABLE_PORT_COMMAND
        {"port", cmd_port, CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_STAT_COMMAND
        {"stat", cmd_stat, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_GAIN_COMMAND
        {"gain", cmd_gain, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_SAMPLE_COMMAND
        {"sample", cmd_sample, 0},
#endif
#ifdef ENABLE_TEST_COMMAND
        {"test", cmd_test, 0},
#endif
        {"touchcal", cmd_touchcal, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP},
        {"touchtest", cmd_touchtest, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP},
        {"pause", cmd_pause, CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
        {"resume", cmd_resume, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#ifdef __SD_CARD_LOAD__
        {"msg", cmd_msg, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_LOAD},
#endif
        {"cal", cmd_cal, CMD_WAIT_MUTEX},
        {"save", cmd_save, CMD_RUN_IN_LOAD},
        {"recall", cmd_recall, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
        {"trace", cmd_trace, CMD_RUN_IN_LOAD},
        {"marker", cmd_marker, CMD_RUN_IN_LOAD},
        {"edelay", cmd_edelay, CMD_RUN_IN_LOAD},
        {"s21offset", cmd_s21offset, CMD_RUN_IN_LOAD},
        {"capture", cmd_capture, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
#ifdef __VNA_MEASURE_MODULE__
        {"measure", cmd_measure, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#endif
#ifdef __REMOTE_DESKTOP__
        {"refresh", cmd_refresh, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
        {"touch", cmd_touch, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
        {"release", cmd_release, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI},
#endif
        {"vbat", cmd_vbat, CMD_RUN_IN_LOAD},
        {"tcxo", cmd_tcxo, CMD_RUN_IN_LOAD},
        {"reset", cmd_reset, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_LOAD},
#ifdef __USE_SMOOTH__
        {"smooth", cmd_smooth, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_CONFIG_COMMAND
        {"config", cmd_config, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#endif
#ifdef __USE_SERIAL_CONSOLE__
#ifdef ENABLE_USART_COMMAND
        {"usart_cfg", cmd_usart_cfg, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
        {"usart", cmd_usart, CMD_WAIT_MUTEX | CMD_BREAK_SWEEP | CMD_RUN_IN_UI | CMD_RUN_IN_LOAD},
#endif
#endif
#ifdef ENABLE_VBAT_OFFSET_COMMAND
        {"vbat_offset", cmd_vbat_offset, CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_TRANSFORM_COMMAND
        {"transform", cmd_transform, CMD_RUN_IN_LOAD},
#endif
        {"threshold", cmd_threshold, CMD_RUN_IN_LOAD},
        {"help", cmd_help, 0},
#ifdef ENABLE_INFO_COMMAND
        {"info", cmd_info, 0},
#endif
        {"version", cmd_version, 0},
        {"reson", cmd_reson, 0},

#ifdef ENABLE_COLOR_COMMAND
        {"color", cmd_color, CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_I2C_COMMAND
        {"i2c", cmd_i2c, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_SI5351_REG_WRITE
        {"si", cmd_si5351reg, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_LCD_COMMAND
        {"lcd", cmd_lcd, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_THREADS_COMMAND
        {"threads", cmd_threads, 0},
#endif
#ifdef ENABLE_SI5351_TIMINGS
        {"t", cmd_si5351time, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_I2C_TIMINGS
        {"i", cmd_i2ctime, CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_BAND_COMMAND
        {"b", cmd_band, CMD_WAIT_MUTEX},
#endif
        {NULL, NULL, 0}};

VNA_SHELL_FUNCTION(cmd_help)
{
  (void)argc;
  (void)argv;
  const VNAShellCommand *scp = commands;
  shell_printf("Commands:");
  while (scp->sc_name != NULL)
  {
    shell_printf(" %s", scp->sc_name);
    scp++;
  }
  shell_printf(VNA_SHELL_NEWLINE_STR);
  return;
}

/*
 * VNA shell functions
 */

// Check Serial connection requirements
#ifdef __USE_SERIAL_CONSOLE__
#if HAL_USE_SERIAL == FALSE
#error "For serial console need HAL_USE_SERIAL as TRUE in halconf.h"
#endif

// Before start process command from shell, need select input stream
// #define PREPARE_STREAM shell_stream = VNA_MODE(VNA_MODE_CONNECTION) ? (BaseSequentialStream *)&SD1 : (BaseSequentialStream *)&SDU1;
#define PREPARE_STREAM shell_stream = (BaseSequentialStream *)&SD1;

// Update Serial connection speed and settings
void shell_update_speed(uint32_t speed)
{
  config._serial_speed = speed;
  // Update Serial speed settings
  sdSetBaudrate(&SD1, speed);
}

// Check USB connection status
static bool usb_IsActive(void)
{
  return usbGetDriverStateI(&USBD1) == USB_ACTIVE;
}

// Reset shell I/O queue
void shell_reset_console(void)
{
  // Reset I/O queue over USB (for USB need also connect/disconnect)
  if (usb_IsActive())
  {
    if (VNA_MODE(VNA_MODE_CONNECTION))
      sduDisconnectI(&SDU1);
    else
      sduConfigureHookI(&SDU1);
  }
  // Reset I/O queue over Serial
  qResetI(&SD1.oqueue);
  qResetI(&SD1.iqueue);
  // Prepare I/O for shell_stream
  PREPARE_STREAM;
}

// Check active connection for Shell
static bool shell_check_connect(void)
{
  /*
   // Serial connection always active
   if (VNA_MODE(VNA_MODE_CONNECTION))
     return true;
   // USB connection can be USB_SUSPENDED
   return usb_IsActive();
 */
  return true;
}

static void shell_init_connection(void)
{
  osalThreadQueueObjectInit(&shell_thread);
  /*
   * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  SerialConfig s_config = {config._serial_speed, 0, USART_CR2_STOP1_BITS, 0};
  sdStart(&SD1, &s_config);
  /*
   * Set Serial speed settings for SD1
   */
  shell_update_speed(config._serial_speed);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  shell_reset_console();
}

#else
// Only USB console, shell_stream always on USB
#define PREPARE_STREAM shell_stream = (BaseSequentialStream *)&SDU1;

// Check connection as Active, if no suspend input
static bool shell_check_connect(void)
{
  return SDU1.config->usbp->state == USB_ACTIVE;
}

// Init shell I/O connection over USB
static void shell_init_connection(void)
{
  /*
   * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  /*
   *  Set I/O stream SDU1 for shell
   */
  PREPARE_STREAM;
}
#endif

static const VNAShellCommand *VNAShell_parceLine(char *line)
{
  // Parse and execute line
  shell_nargs = parse_line(line, shell_args, ARRAY_COUNT(shell_args));
  if (shell_nargs > ARRAY_COUNT(shell_args))
  {
    shell_printf("too many arguments, max " define_to_STR(VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
    return NULL;
  }
  if (shell_nargs > 0)
  {
    const VNAShellCommand *scp;
    for (scp = commands; scp->sc_name != NULL; scp++)
      if (get_str_index(scp->sc_name, shell_args[0]) == 0)
        return scp;
  }
  return NULL;
}

//
// Read command line from shell_stream
//
static const char backspace[] = {0x08, 0x20, 0x08, 0x00};
static int VNAShell_readLine(char *line, int max_size)
{
  // send backspace, space for erase, backspace again
  uint8_t c;
  uint16_t j = 0;
  // Return 0 only if stream not active
  while (shell_read(&c, 1))
  {
    // Backspace or Delete
    if (c == 0x08 || c == 0x7f)
    {
      if (j > 0)
      {
        shell_write(backspace, sizeof(backspace));
        j--;
      }
      continue;
    }
    // New line (Enter)
    if (c == '\r')
    {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      line[j] = 0;
      return 1;
    }
    // Others (skip) or too long - skip
    if (c < ' ' || j >= max_size - 1)
      continue;
    shell_write(&c, 1); // Echo
    line[j++] = (char)c;
  }
  return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
  DEBUG_LOG(0, line); // debug console log
  // Execute line
  const VNAShellCommand *scp = VNAShell_parceLine(line);
  if (scp)
  {
    uint16_t cmd_flag = scp->flags;
    // Skip wait mutex if process UI
    if ((cmd_flag & CMD_RUN_IN_UI) && (sweep_mode & SWEEP_UI_MODE))
      cmd_flag &= ~CMD_WAIT_MUTEX;
    // Break current sweep operation
    if (scp->flags & CMD_BREAK_SWEEP)
      operation_requested |= OP_CONSOLE;
    // Add function for run on sweep end or on break sweep
    if (cmd_flag & CMD_WAIT_MUTEX)
    {
      shell_function = scp->sc_function;
      // Wait execute command in sweep thread
      osalThreadEnqueueTimeoutS(&shell_thread, TIME_INFINITE);
      //      do {
      //        chThdSleepMilliseconds(10);
      //      } while (shell_function);
    }
    else
      scp->sc_function(shell_nargs - 1, &shell_args[1]);
    //  DEBUG_LOG(10, "ok");
  }
  else if (**shell_args) // unknown command (not empty), ignore <CR>
    shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

void VNAShell_executeCMDLine(char *line)
{
  // Disable shell output (not allow shell_printf write, but not block other output!!)
  shell_stream = NULL;
  const VNAShellCommand *scp = VNAShell_parceLine(line);
  if (scp && (scp->flags & CMD_RUN_IN_LOAD))
    scp->sc_function(shell_nargs - 1, &shell_args[1]);
  PREPARE_STREAM;
}

#ifdef __SD_CARD_LOAD__
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use __SD_CARD_LOAD__"
#endif
bool sd_card_load_config(void)
{
  // Mount card
  if (f_mount(fs_volume, "", 1) != FR_OK)
    return FALSE;

  if (f_open(fs_file, "config.ini", FA_OPEN_EXISTING | FA_READ) != FR_OK)
    return FALSE;

  // Disable shell output (not allow shell_printf write, but not block other output!!)
  shell_stream = NULL;
  char *buf = (char *)spi_buffer;
  UINT size = 0;

  uint16_t j = 0, i;
  while (f_read(fs_file, buf, 512, &size) == FR_OK && size > 0)
  {
    i = 0;
    while (i < size)
    {
      uint8_t c = buf[i++];
      // New line (Enter)
      if (c == '\r')
      {
        //        shell_line[j  ] = '\r';
        //        shell_line[j+1] = '\n';
        //        shell_line[j+2] = 0;
        //        shell_printf(shell_line);
        shell_line[j] = 0;
        j = 0;
        const VNAShellCommand *scp = VNAShell_parceLine(shell_line);
        if (scp && (scp->flags & CMD_RUN_IN_LOAD))
          scp->sc_function(shell_nargs - 1, &shell_args[1]);
        continue;
      }
      // Others (skip)
      if (c < 0x20)
        continue;
      // Store
      if (j < VNA_SHELL_MAX_LENGTH - 1)
        shell_line[j++] = (char)c;
    }
  }
  f_close(fs_file);
  PREPARE_STREAM;
  return TRUE;
}
#endif

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */ 442);
THD_FUNCTION(myshellThread, p)
{
  (void)p;
  chRegSetThreadName("shell");
  while (true)
  {
    shell_printf(VNA_SHELL_PROMPT_STR);
    if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
      VNAShell_executeLine(shell_line);
    else // Putting a delay in order to avoid an endless loop trying to read an unavailable stream.
      chThdSleepMilliseconds(100);
  }
}
#endif

// Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
// Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
// Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40 bytes
//
int main(void)
{
  /*
   * Initialize ChibiOS systems
   */
  halInit();
  chSysInit();

  /*
   * Init used hardware
   */
  /*
   *  Init DMA channels (used for direct send data, used for i2s and spi)
   */
  rccEnableDMA1(false);
// rccEnableDMA2(false);

/*
 * Init GPIO (pin control)
 */
#if HAL_USE_PAL == FALSE
  initPal();
#endif

/*
 * Initialize RTC library (not used ChibiOS RTC module)
 */
#ifdef __USE_RTC__
  rtc_init();
#endif

/*
 * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
 */
#if defined(__VNA_ENABLE_DAC__) || defined(__LCD_BRIGHTNESS__)
  dac_init();
#endif

  /*
   * restore config and calibration 0 slot from flash memory, also if need use backup data
   */
  load_settings();

  /*
   * MLA Toolbox: force startup sweep range to 3–30 MHz
   */
  set_sweep_frequency(ST_START, 3000000); // 3 MHz
  set_sweep_frequency(ST_STOP, 30000000); // 30 MHz
  set_sweep_points(401);
  /*
   * MLA Toolbox: enable only S11 logmag & S21 phase at startup
   * trace 0: S11 logmag (keep enabled)
   * trace 1: S21 logmag (disable)
   * trace 2: S11 smith  (disable)
   * trace 3: S21 phase  (keep enabled)
   */
  set_trace_enable(1, false); // disable S21 logmag
  set_trace_enable(2, false); // disable S11 smith
  set_active_trace(0);        // make sure S11 logmag is the active trace

  /*
   * MLA Toolbox: purple trace (trace 3) = SWR of S11
   */
  set_trace_type(3, TRC_SWR, 0); // TRC_SWR on channel 0 (S11)

  /*
   * I2C bus
   */
  i2c_start();

  /*
   * Start si5351
   */
  si5351_init();

/*
 * Set frequency offset
 */
#ifdef USE_VARIABLE_OFFSET
  si5351_set_frequency_offset(IF_OFFSET);
#endif
  /*
   * Init Shell console connection data
   */
  shell_init_connection();

#ifdef __USE_SERIAL_CONSOLE__
  serial_shell_printf("\r\n[MLA ToolBox] NanoVNA H4 Booting...\r\n");
#endif

  /*
   * SPI bus and LCD Initialize
   */
  lcd_init();

  /*





   * tlv320aic Initialize (audio codec)
   */
  tlv320aic3204_init();
  chThdSleepMilliseconds(200); // Wait for aic codec start
                               /*
                                * I2S Initialize
                                */
  initI2S(rx_buffer, ARRAY_COUNT(rx_buffer) * sizeof(audio_sample_t) / sizeof(int16_t));

/*
 * SD Card init (if inserted) allow fix issues
 * Some card after insert work in SDIO mode and can corrupt SPI exchange (need switch it to SPI)
 */
#ifdef __USE_SD_CARD__
  disk_initialize(0);
#endif

  /*
   * I2C bus run on work speed
   */
  i2c_set_timings(STM32_I2C_TIMINGR);

  /*
   * Startup sweep thread
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

  while (1)
  {
    if (shell_check_connect())
    {
      shell_printf(VNA_SHELL_NEWLINE_STR "NanoVNA Shell" VNA_SHELL_NEWLINE_STR);
#ifdef VNA_SHELL_THREAD
#if CH_CFG_USE_WAITEXIT == FALSE
#error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
#endif
      thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2),
                                            NORMALPRIO + 1,
                                            myshellThread, NULL);
      chThdWait(shelltp);
#else
      do
      {
        shell_printf(VNA_SHELL_PROMPT_STR);
        if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
          VNAShell_executeLine(shell_line);
        else
          chThdSleepMilliseconds(200);
      } while (shell_check_connect());
#endif
    }
    chThdSleepMilliseconds(1000);
  }
}
