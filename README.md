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
# main.py  –  Pico GPS-OLED with heartbeat, watchdog & crash-log
# Pins (change if wired differently):
#   I²C-0   GP0 SDA, GP1 SCL  → SSD1306 (0x3C)
#   UART-1  GP4 TX,  GP5 RX   → GPS @ 9600 bps

from machine import Pin, I2C, UART, WDT
import utime, math, gc, framebuf, ssd1306

# ── configurable pins & constants ────────────────────────────────
SDA_PIN, SCL_PIN, OLED_ADDR = 0, 1, 0x3C
UART_ID,  TX_PIN, RX_PIN    = 1, 4, 5
GPS_BAUD                    = 9600
LOG_PATH                    = "exception.log"
WATCHDOG_MS                 = 8000          # 8 s watchdog
# ─────────────────────────────────────────────────────────────────

# peripherals
i2c  = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)
oled = ssd1306.SSD1306_I2C(128, 32, i2c, addr=OLED_ADDR)
uart = UART(UART_ID, GPS_BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN), timeout=300)
led  = Pin(25, Pin.OUT)        # on-board LED
wdt  = WDT(timeout=WATCHDOG_MS)

CHAR_W = 8                     # width of 8×8 font

# ── helper functions ────────────────────────────────────────────
def dm_to_dd(dm: str, hemi: str):
    if not dm:
        return None
    dot = dm.find('.')
    deg = int(dm[:dot - 2])
    minutes = float(dm[dot - 2:])
    dd = deg + minutes / 60
    return -dd if hemi in ('S', 'W') else dd

def fmt_deg(dd: float, hemi=('N', 'S')):
    return '--' if dd is None else f"{abs(dd):.4f}{hemi[0] if dd >= 0 else hemi[1]}"

# ---------- robust NMEA parser ------------------------------------------
def safe_time_to_sec(timestr):
    """
    Convert hhmmss[.sss] to seconds after midnight.
    Returns None if the string is malformed.
    """
    if not timestr or len(timestr) < 6 or not timestr[0:6].isdigit():
        return None
    try:
        hh = int(timestr[0:2])
        mm = int(timestr[2:4])
        ss = float(timestr[4:])
        return hh * 3600 + mm * 60 + ss
    except ValueError:
        return None


def parse_nmea(line: str):
    """
    Parse $GxGGA / $GxRMC.
    Returns dict(lat, lon, time_s, sats, fix_valid); keys absent if unknown.
    Never raises ValueError or IndexError.
    """
    if not line.startswith("$"):
        return {}

    p   = line.split(",")
    tag = p[0][-3:]
    d   = {}

    try:
        if tag == "GGA" and len(p) >= 10:
            d["sats"]      = int(p[7] or 0)
            d["fix_valid"] = p[6] != "0"
            if d["fix_valid"]:
                d["lat"] = dm_to_dd(p[2], p[3]) if p[2] and p[3] else None
                d["lon"] = dm_to_dd(p[4], p[5]) if p[4] and p[5] else None
                tsec = safe_time_to_sec(p[1])
                if tsec is not None:
                    d["time_s"] = tsec

        elif tag == "RMC" and len(p) >= 7:
            d["fix_valid"] = p[2] == "A"
            if d["fix_valid"]:
                d["lat"] = dm_to_dd(p[3], p[4]) if p[3] and p[4] else None
                d["lon"] = dm_to_dd(p[5], p[6]) if p[5] and p[6] else None
                tsec = safe_time_to_sec(p[1])
                if tsec is not None:
                    d["time_s"] = tsec
    except (ValueError, IndexError):
        # Anything malformed is ignored silently; caller just gets {}.
        return {}

    return d

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi   = p2 - p1
    dlamb  = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlamb/2)**2
    return 2*R*math.asin(math.sqrt(a))

def sat_icon(fb: framebuf.FrameBuffer, x, y):
    fb.fill_rect(x+1, y+1, 3, 3, 1)
    fb.pixel(x, y+1, 1); fb.pixel(x, y+2, 1); fb.pixel(x, y+3, 1)
    fb.pixel(x+4, y+1, 1); fb.pixel(x+4, y+2, 1); fb.pixel(x+4, y+3, 1)
    fb.pixel(x+2, y, 1);   fb.pixel(x+2, y+4, 1)

def log_exception(e):
    try:
        with open(LOG_PATH, "a") as f:
            f.write("{} {}\n".format(utime.time(), repr(e)))
    except OSError:
        pass

# ── runtime state ───────────────────────────────────────────────
st = dict(lat=None, lon=None, sats=0, fix=False,
          prev_lat=None, prev_lon=None, prev_t=None,
          speed=0.0)

# ── main loop ───────────────────────────────────────────────────
while True:
    try:
        # -------- read UART non-blocking --------
        raw_bytes = uart.readline()
        if raw_bytes:
            try:
                raw = raw_bytes.decode().strip()  # no keyword args
            except UnicodeError:
                raw = ''
            upd = parse_nmea(raw)

            if 'sats' in upd and upd['sats'] is not None:
                st['sats'] = upd['sats']

            if upd.get('fix_valid'):
                lat, lon, t = upd.get('lat'), upd.get('lon'), upd.get('time_s')
                if all(v is not None for v in (lat, lon, t)):
                    if st['prev_lat'] is not None:
                        dist = haversine(st['prev_lat'], st['prev_lon'], lat, lon)
                        dt   = t - st['prev_t']
                        if dt < 0:
                            dt += 86400         # midnight wrap
                        st['speed'] = dist/dt if dt > 0 else 0.0
                    st['prev_lat'], st['prev_lon'], st['prev_t'] = lat, lon, t
                st['lat'], st['lon'], st['fix'] = lat, lon, True

        # -------- render OLED --------
        oled.fill(0)
        if not st['fix']:
            oled.text('NO FIX', 0, 12)
            oled.text(f'Sat:{st["sats"]}', 0, 22)
        else:
            # row 1: latitude + speed
            left1  = fmt_deg(st['lat'], ('N', 'S'))
            sp_txt = f'{st["speed"]:.1f}m/s'
            oled.text(left1, 0, 0)
            oled.text(sp_txt, 128 - CHAR_W*len(sp_txt), 0)
            # row 2: longitude + satellites
            left2  = fmt_deg(st['lon'], ('E', 'W'))
            sat_s  = str(st['sats'])
            icon_x = 128 - CHAR_W*len(sat_s) - 7
            sat_icon(oled, icon_x, 16)
            oled.text(sat_s, icon_x + 7, 16)
            oled.text(left2, 0, 16)
        oled.show()

        # -------- housekeeping --------
        led.toggle()      # heartbeat blink
        wdt.feed()        # reset watchdog timer
        gc.collect()
        utime.sleep_ms(100)

    except Exception as e:
        log_exception(e)  # store traceback in flash
        led.on()          # solid LED for 2 s signals error
        oled.fill(0); oled.text('ERR', 0, 12); oled.show()
        utime.sleep(2)
        led.off()

```

**Usage**

* Save `main.py` and `ssd1306.py` on the Pico; power-cycle.
* Monitor the LED: blinking = healthy loop; solid = error logged.
* Read `exception.log` over USB if any issues arise.

The project now provides a compact, battery-friendly GPS speedometer with robust self-recovery and on-device diagnostics.
