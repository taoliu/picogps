# picogps
The mini GPS built around Raspberry Pico

## Intro

It's my hobby project. I plan to learn some basic micropython programming on my raspberry pico (1 and 2 gen). I purchased a GPS module NEO-6MV2 from aliexpress for $2 and a tiny 0.91" OLED display (SSD1306) from aliexpress as well for another $2. After I wired them up, and plugged in the a powerbank with 3 18650 batteries, this whole device worked pretty well. It will display on the 0.91" the longitude, the latitude, the moving speed and the number of saterlites it can detect.

## Project overview

| Aspect                | Implementation                                                                                                                                                                                                                                                                      |
| --------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Platform**          | Raspberry Pi Pico (RP2040) running MicroPython                                                                                                                                                                                                                                      |
| **Peripherals**       | • **GPS** on UART-1 (GP4 TX, GP5 RX, 9600 bps)  <br>• **128 × 32 SSD1306 OLED** on I²C-0 (GP0 SDA, GP1 SCL, addr 0x3C)                                                                                                                                                              |
| **Display layout**    | Two fixed lines, 8 × 8 font (16 chars/line) <br>`Lat        speed (m/s)`<br>`Lon        🛰 sat_count`                                                                                                                                                                               |
| **GPS data**          | *Latitude*, *longitude*, *UTC time*, *satellites* parsed from `$GxGGA` / `$GxRMC` sentences                                                                                                                                                                                         |
| **Ground speed**      | Great-circle distance (haversine) between successive fixes ÷ Δt                                                                                                                                                                                                                     |
| **Reliability**       | • Non-blocking UART read  <br>• Robust parser ignores incomplete / malformed sentences  <br>• 8 s watchdog → automatic reset on lock-up  <br>• Heartbeat LED toggles each loop  <br>• Unhandled exceptions logged to `exception.log`; LED holds solid for 2 s, then program resumes |
| **Power-off logging** | `exception.log` remains in flash; inspect after reconnecting USB                                                                                                                                                                                                                    |
| **No-fix handling**   | OLED shows “NO FIX” and live satellite count until a valid fix appears                                                                                                                                                                                                              |

---

## `main.py` (final version)

```python
# main.py  –  Pico GPS-OLED: two-line readout + heartbeat, watchdog, crash log
# Wiring:
#   GP0 SDA  | GP1 SCL  → 128×32 SSD1306 (I2C-0, addr 0x3C)
#   GP4 TX   | GP5 RX   → GPS @ 9600 bps  (UART-1)

from machine import Pin, I2C, UART, WDT
import utime, math, gc, framebuf, ssd1306

# --- pins & settings --------------------------------------------------
SDA_PIN, SCL_PIN, OLED_ADDR = 0, 1, 0x3C
UART_ID,  TX_PIN, RX_PIN    = 1, 4, 5
GPS_BAUD                    = 9600
WATCHDOG_MS                 = 8000          # 8 s
LOG_PATH                    = "exception.log"
CHAR_W                      = 8             # 8×8 font char width
# ----------------------------------------------------------------------

i2c  = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=OLED_ADDR)
uart = UART(UART_ID, GPS_BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN), timeout=300)
led  = Pin(25, Pin.OUT)      # on-board LED
wdt  = WDT(timeout=WATCHDOG_MS)

# ---------- helpers ---------------------------------------------------
def dm_to_dd(dm, hemi):
    if not dm:
        return None
    dot = dm.find('.')
    deg = int(dm[:dot - 2])
    minutes = float(dm[dot - 2:])
    dd = deg + minutes / 60
    return -dd if hemi in ('S', 'W') else dd

def fmt_deg(dd, hemi_pair=('N', 'S')):
    return '--' if dd is None else f"{abs(dd):.4f}{hemi_pair[0] if dd >= 0 else hemi_pair[1]}"

def safe_time_to_sec(t):
    if not t or len(t) < 6 or not t[:6].isdigit():
        return None
    try:
        hh, mm, ss = int(t[:2]), int(t[2:4]), float(t[4:])
        return hh*3600 + mm*60 + ss
    except ValueError:
        return None

def parse_nmea(line):
    """Return dict(lat, lon, time_s, sats, fix_valid)."""
    if not line.startswith('$'):
        return {}
    p, tag, d = line.split(','), line[-3:], {}
    try:
        if tag == 'GGA' and len(p) >= 10:
            d['sats'] = int(p[7] or 0)
            d['fix_valid'] = p[6] != '0'
            if d['fix_valid']:
                d['lat'] = dm_to_dd(p[2], p[3]) if p[2] and p[3] else None
                d['lon'] = dm_to_dd(p[4], p[5]) if p[4] and p[5] else None
                d['time_s'] = safe_time_to_sec(p[1])
        elif tag == 'RMC' and len(p) >= 7:
            d['fix_valid'] = p[2] == 'A'
            if d['fix_valid']:
                d['lat'] = dm_to_dd(p[3], p[4]) if p[3] and p[4] else None
                d['lon'] = dm_to_dd(p[5], p[6]) if p[5] and p[6] else None
                d['time_s'] = safe_time_to_sec(p[1])
    except (ValueError, IndexError):
        return {}
    return d

def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    dφ = φ2 - φ1
    dλ = math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def sat_icon(buf, x, y):
    buf.fill_rect(x+1, y+1, 3, 3, 1)
    buf.pixel(x, y+1, 1); buf.pixel(x, y+2, 1); buf.pixel(x, y+3, 1)
    buf.pixel(x+4, y+1, 1); buf.pixel(x+4, y+2, 1); buf.pixel(x+4, y+3, 1)
    buf.pixel(x+2, y, 1); buf.pixel(x+2, y+4, 1)

def log_exc(e):
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f"{utime.time()} {repr(e)}\n")
    except OSError:
        pass
# ----------------------------------------------------------------------

state = dict(lat=None, lon=None, sats=0, fix=False,
             prev_lat=None, prev_lon=None, prev_t=None,
             speed=0.0)

# ---------- main loop --------------------------------------------------
while True:
    try:
        # UART read (non-blocking)
        rb = uart.readline()
        if rb:
            try:
                raw = rb.decode().strip()
            except UnicodeError:
                raw = ''
            u = parse_nmea(raw)
            if 'sats' in u and u['sats'] is not None:
                state['sats'] = u['sats']
            if u.get('fix_valid'):
                lat, lon, t = u.get('lat'), u.get('lon'), u.get('time_s')
                if all(v is not None for v in (lat, lon, t)):
                    if state['prev_lat'] is not None:
                        d = haversine(state['prev_lat'], state['prev_lon'], lat, lon)
                        dt = t - state['prev_t']
                        if dt < 0:
                            dt += 86400      # midnight rollover
                        state['speed'] = d/dt if dt else 0.0
                    state['prev_lat'], state['prev_lon'], state['prev_t'] = lat, lon, t
                state['lat'], state['lon'], state['fix'] = lat, lon, True

        # ---------------- OLED ----------------
        oled.fill(0)
        if not state['fix']:
            oled.text('NO FIX', 0, 12)
            oled.text(f'Sat:{state["sats"]}', 0, 22)
        else:
            # row 1
            oled.text(fmt_deg(state['lat'], ('N','S')), 0, 0)
            sp = f'{state["speed"]:.1f}m/s'
            oled.text(sp, 128 - CHAR_W*len(sp), 0)
            # row 2
            oled.text(fmt_deg(state['lon'], ('E','W')), 0, 16)
            sat_s = str(state['sats'])
            icon_x = 128 - CHAR_W*len(sat_s) - 7
            sat_icon(oled, icon_x, 16)
            oled.text(sat_s, icon_x + 7, 16)
        oled.show()

        # housekeeping
        led.toggle()     # heartbeat
        wdt.feed()
        gc.collect()
        utime.sleep_ms(60)

    except Exception as e:
        log_exc(e)
        led.on()                       # solid LED for 2 s
        oled.fill(0); oled.text('ERR', 0, 12); oled.show()
        utime.sleep(2)
        led.off()
```

**Usage**

* Save `main.py` and `ssd1306.py` on the Pico; power-cycle.
* Monitor the LED: blinking = healthy loop; solid = error logged.
* Read `exception.log` over USB if any issues arise.

The project now provides a compact, battery-friendly GPS speedometer with robust self-recovery and on-device diagnostics.
