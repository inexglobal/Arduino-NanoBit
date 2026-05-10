#ifndef NANOBIT_OLED_I2C_SSD1306_H
#define NANOBIT_OLED_I2C_SSD1306_H

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef __has_include
#define __has_include(x) 0
#endif
#if __has_include(<Adafruit_GFX.h>)
#include <Adafruit_GFX.h>
#else
#error "Adafruit GFX Library not found. Install it via Library Manager."
#endif
#if __has_include(<Adafruit_SSD1306.h>)
#include <Adafruit_SSD1306.h>
#else
#error "Adafruit SSD1306 Library not found. Install it via Library Manager."
#endif

#ifndef SSD1306_EZ_DEFAULT_WIDTH
#define SSD1306_EZ_DEFAULT_WIDTH 128
#endif
#ifndef SSD1306_EZ_DEFAULT_HEIGHT
#define SSD1306_EZ_DEFAULT_HEIGHT 64
#endif
#ifndef SSD1306_EZ_DEFAULT_ADDR
#define SSD1306_EZ_DEFAULT_ADDR 0x3C
#endif
#ifndef SSD1306_EZ_DEFAULT_RESET
#define SSD1306_EZ_DEFAULT_RESET -1 // -1 means no dedicated reset pin
#endif

// --- NEW: hardware reset behavior (tweak as needed) ---
#ifndef SSD1306_EZ_HWRESET_LOW_MS
#define SSD1306_EZ_HWRESET_LOW_MS 10 // hold RESET low (ms)
#endif
#ifndef SSD1306_EZ_HWRESET_HIGH_MS
#define SSD1306_EZ_HWRESET_HIGH_MS 10 // delay after releasing RESET (ms)
#endif
#ifndef SSD1306_EZ_AUTO_HWRESET
#define SSD1306_EZ_AUTO_HWRESET 1 // 1=auto reset in begin(), 0=manual only
#endif
// ------------------------------------------------------

#define BLACK SSD1306_BLACK
#define WHITE SSD1306_WHITE

class SSD1306_EZ
{
public:
  SSD1306_EZ()
      : _d(nullptr),
        _panelW(SSD1306_EZ_DEFAULT_WIDTH),
        _panelH(SSD1306_EZ_DEFAULT_HEIGHT),
        _defAddr(SSD1306_EZ_DEFAULT_ADDR),
        _defWire(&Wire),
        _defReset(SSD1306_EZ_DEFAULT_RESET),
        _rotation(0),
        _inited(false),
        _autoHwReset(SSD1306_EZ_AUTO_HWRESET != 0), // NEW: default auto-reset
        _autoTextBg(true),
        _textBgColor(SSD1306_BLACK),
        _textFgColor(SSD1306_WHITE),
        _useCustomTextBg(false),
        _useCustomTextFg(false)
  {
  }

  // Begin: optionally pulse hardware RESET before creating/initializing driver
  bool begin(uint16_t width = SSD1306_EZ_DEFAULT_WIDTH, uint16_t height = SSD1306_EZ_DEFAULT_HEIGHT,
             uint8_t addr = SSD1306_EZ_DEFAULT_ADDR, TwoWire *wire = &Wire, int8_t resetPin = SSD1306_EZ_DEFAULT_RESET)
  {
    _panelW = width;
    _panelH = height;
    _defAddr = addr;
    _defWire = wire;
    _defReset = resetPin;

    // NEW: perform a hardware reset (active-low) if enabled
    if (_autoHwReset)
    {
      _hwReset(SSD1306_EZ_HWRESET_LOW_MS, SSD1306_EZ_HWRESET_HIGH_MS);
    }

    return _doBegin();
  }

  void config(uint16_t width, uint16_t height, uint8_t addr = SSD1306_EZ_DEFAULT_ADDR,
              TwoWire *wire = &Wire, int8_t resetPin = SSD1306_EZ_DEFAULT_RESET)
  {
    _panelW = width;
    _panelH = height;
    _defAddr = addr;
    _defWire = wire;
    _defReset = resetPin;
  }

  // --- NEW: hardware reset control API ---
  void setResetPin(int8_t resetPin) { _defReset = resetPin; }
  int8_t resetPin() const { return _defReset; }

  void setAutoHardwareReset(bool enable) { _autoHwReset = enable; }
  bool autoHardwareReset() const { return _autoHwReset; }

  // Manually toggle the reset line (active-low). Returns false if no pin.
  bool hardwareReset(uint16_t low_ms = SSD1306_EZ_HWRESET_LOW_MS,
                     uint16_t high_ms = SSD1306_EZ_HWRESET_HIGH_MS)
  {
    return _hwReset(low_ms, high_ms);
  }
  void reset() { (void)hardwareReset(); } // convenience alias
  // --------------------------------------

  // ------- Text -------
  void setTextSize(uint8_t s)
  {
    if (_ensureBegin())
      _d->setTextSize(s);
  }
  void textSize(uint8_t n) { setTextSize(n); }
  void textBackgroundAuto(bool enable = true) { _autoTextBg = enable; }
  void setTextForegroundColor(uint16_t color)
  {
    _textFgColor = color;
    _useCustomTextFg = true;
    if (_d)
      _d->setTextColor(color);
  }
  void textColor(uint16_t color) { setTextForegroundColor(color); }
  void setTextBackgroundColor(uint16_t color)
  {
    _textBgColor = color;
    _useCustomTextBg = true;
    _autoTextBg = true;
  }
  void textBackgroundColor(uint16_t color) { setTextBackgroundColor(color); }
  void textColor(uint16_t fg, uint16_t bg)
  {
    setTextForegroundColor(fg);
    setTextBackgroundColor(bg);
  }
  void resetTextColorsToAuto()
  {
    _useCustomTextBg = false;
    _useCustomTextFg = false;
  }

  void _formatToBuffer(char *out, size_t outcap, const char *fmt, va_list ap_in)
  {
    size_t oi = 0;
    auto push = [&](char c)
    { if (oi < outcap-1) out[oi++] = c; };
    auto pushStr = [&](const char *s)
    { while (*s && oi < outcap-1) out[oi++] = *s++; };
    auto u32_to_bin = [&](uint32_t v)
    {
      bool started = false;
      for (int i = 31; i >= 0; i--)
      {
        bool bit = (v >> i) & 1;
        if (bit)
          started = true;
        if (started)
          push(bit ? '1' : '0');
      }
      if (!started)
        push('0');
    };
    auto u32_to_hex = [&](uint32_t v, bool upper)
    {
      const char *digs = upper ? "0123456789ABCDEF" : "0123456789abcdef";
      bool started = false;
      for (int i = 7; i >= 0; i--)
      {
        uint8_t nib = (v >> (i * 4)) & 0xF;
        if (nib)
          started = true;
        if (started)
          push(digs[nib]);
      }
      if (!started)
        push('0');
    };

    va_list ap;
    va_copy(ap, ap_in);
    for (const char *p = fmt; *p; ++p)
    {
      if (*p != '%')
      {
        push(*p);
        continue;
      }
      if (*(p + 1) == '%')
      {
        push('%');
        ++p;
        continue;
      }

      int precision = 3; // default fractional digits
      const char *q = p + 1;
      if (*q == '.')
      {
        ++q;
        int val = 0;
        bool any = false;
        while (*q >= '0' && *q <= '9')
        {
          any = true;
          val = val * 10 + (*q - '0');
          ++q;
        }
        if (any)
        {
          if (val < 0)
            val = 0;
          if (val > 6)
            val = 6; // cap (adjust if needed)
          precision = val;
        }
      }
      char spec = *q;
      if (spec == 0)
        break;
      p = q;

      switch (spec)
      {
      case 'd':
      {
        long v = va_arg(ap, long);
        char tmp[22];
        snprintf(tmp, sizeof(tmp), "%ld", v);
        pushStr(tmp);
      }
      break;

      case 'u':
      {
        unsigned long v = va_arg(ap, unsigned long);
        char tmp[22];
        snprintf(tmp, sizeof(tmp), "%lu", v);
        pushStr(tmp);
      }
      break;

      case 'x':
      case 'X':
      {
        unsigned long v = va_arg(ap, unsigned long);
        const bool upper = (spec == 'X');
        const char *digs = upper ? "0123456789ABCDEF" : "0123456789abcdef";
        bool started = false;
        for (int i = (int)(sizeof(unsigned long) * 8 - 4); i >= 0; i -= 4)
        {
          uint8_t nib = (v >> i) & 0xF;
          if (nib)
            started = true;
          if (started)
            push(digs[nib]);
        }
        if (!started)
          push('0');
      }
      break;

      case 'h':
      {
        unsigned long v = va_arg(ap, unsigned long);
        pushStr("0x");
        u32_to_hex((uint32_t)v, false);
      }
      break;

      case 'b':
      {
        unsigned long v = va_arg(ap, unsigned long);
        u32_to_bin((uint32_t)v);
      }
      break;

      case 'f':
      {
        double v = va_arg(ap, double);
        if (isnan(v))
        {
          pushStr("nan");
          break;
        }
        if (isinf(v))
        {
          pushStr((v < 0) ? "-inf" : "inf");
          break;
        }

        bool neg = (v < 0);
        if (neg)
          v = -v;
        unsigned long ip = (unsigned long)v;
        double frac = v - (double)ip;

        unsigned long scale = 1;
        for (int i = 0; i < precision; i++)
          scale *= 10;

        unsigned long fp = (precision > 0) ? (unsigned long)(frac * (double)scale + 0.5) : 0UL;
        if (precision > 0 && fp >= scale)
        {
          ip += 1;
          fp -= scale;
        }

        if (neg)
          push('-');

        char ib[22];
        ib[21] = 0;
        int ii = 21;
        unsigned long t = ip;
        do
        {
          ib[--ii] = '0' + (t % 10);
          t /= 10;
        } while (t && ii > 0);
        for (; ib[ii]; ++ii)
          push(ib[ii]);

        if (precision > 0)
        {
          push('.');
          unsigned long div = scale / 10;
          while (div > 0 && fp < div)
          {
            push('0');
            div /= 10;
          }
          char fb[22];
          fb[21] = 0;
          int fi = 21;
          unsigned long tf = fp;
          do
          {
            fb[--fi] = '0' + (tf % 10);
            tf /= 10;
          } while (tf && fi > 0);
          for (; fb[fi]; ++fi)
            push(fb[fi]);
        }
      }
      break;

      case 's':
      {
        const char *s = va_arg(ap, const char *);
        if (!s)
          s = "(null)";
        pushStr(s);
      }
      break;

      case 'c':
      {
        int ch = va_arg(ap, int);
        push((char)ch);
      }
      break;

      default:
        push('%');
        push(spec);
        break;
      }
    }
    va_end(ap);
    out[oi] = 0;
  }

  void text(int16_t x, int16_t y, const char *fmt, ...)
  {
    if (!_ensureBegin())
      return;
    char out[256];
    va_list ap;
    va_start(ap, fmt);
    _formatToBuffer(out, sizeof(out), fmt, ap);
    va_end(ap);

    if (_autoTextBg)
    {
      int16_t bx, by;
      uint16_t bw, bh;
      _d->getTextBounds(out, x, y, &bx, &by, &bw, &bh);
      uint16_t bg = _useCustomTextBg ? _textBgColor : SSD1306_BLACK;
      uint16_t fg = _useCustomTextFg ? _textFgColor : SSD1306_WHITE;
      if (bw > 0 && bh > 0)
        _d->fillRect(bx, by, bw, bh, bg);
      _d->setTextColor(fg);
    }
    _d->setCursor(x, y);
    _d->print(out);
  }

  void text(int16_t x, int16_t y, const String &s)
  {
    if (!_ensureBegin())
      return;
    if (_autoTextBg)
    {
      int16_t bx, by;
      uint16_t bw, bh;
      _d->getTextBounds(s.c_str(), x, y, &bx, &by, &bw, &bh);
      uint16_t bg = _useCustomTextBg ? _textBgColor : SSD1306_BLACK;
      uint16_t fg = _useCustomTextFg ? _textFgColor : SSD1306_WHITE;
      if (bw > 0 && bh > 0)
        _d->fillRect(bx, by, bw, bh, bg);
      _d->setTextColor(fg);
    }
    _d->setCursor(x, y);
    _d->print(s);
  }

  // Back-compat wrappers
  void drawText(int16_t x, int16_t y, const String &s) { text(x, y, s); }
  void textfmt(int16_t x, int16_t y, const char *fmt, ...)
  {
    va_list ap;
    va_start(ap, fmt);
    char out[256];
    _formatToBuffer(out, sizeof(out), fmt, ap);
    va_end(ap);
    text(x, y, out);
  }
  void textf(int16_t x, int16_t y, const char *fmt, ...)
  {
    va_list ap;
    va_start(ap, fmt);
    char out[256];
    _formatToBuffer(out, sizeof(out), fmt, ap);
    va_end(ap);
    text(x, y, out);
  }

  // ------- Geometry & Bitmaps -------
  void setTextColor(uint16_t c = SSD1306_WHITE)
  {
    _textFgColor = c;
    _useCustomTextFg = true;
    if (_ensureBegin())
      _d->setTextColor(c);
  }
  void drawPixel(int16_t x0, int16_t y0, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->drawPixel(x0, y0, c);
  }
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->drawLine(x0, y0, x1, y1, c);
  }
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c, bool fill = false)
  {
    if (!_ensureBegin())
      return;
    if (fill)
      _d->fillRect(x, y, w, h, c);
    else
      _d->drawRect(x, y, w, h, c);
  }
  void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t c, bool fill = false)
  {
    if (!_ensureBegin())
      return;
    if (fill)
      _d->fillRoundRect(x, y, w, h, r, c);
    else
      _d->drawRoundRect(x, y, w, h, r, c);
  }
  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t c, bool fill = false)
  {
    if (!_ensureBegin())
      return;
    if (fill)
      _d->fillTriangle(x0, y0, x1, y1, x2, y2, c);
    else
      _d->drawTriangle(x0, y0, x1, y1, x2, y2, c);
  }
  void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t c, bool fill = false)
  {
    if (!_ensureBegin())
      return;
    if (fill)
      _d->fillCircle(x, y, r, c);
    else
      _d->drawCircle(x, y, r, c);
  }

  void drawEllipse(int16_t xc, int16_t yc, int16_t rx, int16_t ry, uint16_t c, bool fill = false)
  {
    if (!_ensureBegin())
      return;
    if (rx < 0)
      rx = -rx;
    if (ry < 0)
      ry = -ry;
    int32_t x = 0, y = ry;
    int32_t rx2 = (int32_t)rx * rx, ry2 = (int32_t)ry * ry;
    int32_t tworx2 = 2 * rx2, twory2 = 2 * ry2;
    int32_t px = 0, py = tworx2 * y;
    int32_t p = (int32_t)(ry2 - rx2 * ry + rx2 / 4.0);
    auto plot = [&](int16_t x, int16_t y)
    {
      if (fill)
      {
        _d->drawFastHLine(xc - x, yc + y, 2 * x + 1, c);
        if (y != 0)
          _d->drawFastHLine(xc - x, yc - y, 2 * x + 1, c);
      }
      else
      {
        _d->drawPixel(xc + x, yc + y, c);
        _d->drawPixel(xc - x, yc + y, c);
        _d->drawPixel(xc + x, yc - y, c);
        _d->drawPixel(xc - x, yc - y, c);
      }
    };
    while (px < py)
    {
      plot(x, y);
      x++;
      px += twory2;
      if (p < 0)
        p += ry2 + px;
      else
      {
        y--;
        py -= tworx2;
        p += ry2 + px - py;
      }
    }
    p = (int32_t)(ry2 * (x + 0.5) * (x + 0.5) + rx2 * (y - 1) * (y - 1) - rx2 * ry2);
    while (y >= 0)
    {
      plot(x, y);
      y--;
      py -= tworx2;
      if (p > 0)
        p += rx2 - py;
      else
      {
        x++;
        px += twory2;
        p += rx2 - py + px;
      }
    }
  }

  void fillEllipse(int16_t xc, int16_t yc, int16_t rx, int16_t ry, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    if (rx < 0)
      rx = -rx;
    if (ry < 0)
      ry = -ry;
    int32_t x = 0, y = ry;
    int32_t rx2 = (int32_t)rx * rx, ry2 = (int32_t)ry * ry;
    int32_t tworx2 = 2 * rx2, twory2 = 2 * ry2;
    int32_t px = 0, py = tworx2 * y;
    int32_t p = (int32_t)(ry2 - rx2 * ry + rx2 / 4.0);
    auto hline = [&](int16_t x, int16_t y)
    {
      _d->drawFastHLine(xc - x, yc + y, 2 * x + 1, c);
      if (y != 0)
        _d->drawFastHLine(xc - x, yc - y, 2 * x + 1, c);
    };
    while (px < py)
    {
      hline(x, y);
      x++;
      px += twory2;
      if (p < 0)
        p += ry2 + px;
      else
      {
        y--;
        py -= tworx2;
        p += ry2 + px - py;
      }
    }
    p = (int32_t)(ry2 * (x + 0.5) * (x + 0.5) + rx2 * (y - 1) * (y - 1) - rx2 * ry2);
    while (y >= 0)
    {
      hline(x, y);
      y--;
      py -= tworx2;
      if (p > 0)
        p += rx2 - py;
      else
      {
        x++;
        px += twory2;
        p += rx2 - py + px;
      }
    }
  }

  // Filled shapes
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->fillRect(x, y, w, h, c);
  }
  void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->fillRoundRect(x, y, w, h, r, c);
  }
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->fillTriangle(x0, y0, x1, y1, x2, y2, c);
  }
  void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t c)
  {
    if (!_ensureBegin())
      return;
    _d->fillCircle(x, y, r, c);
  }
  void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color = SSD1306_WHITE)
  {
    if (!_ensureBegin())
      return;
    _d->drawBitmap(x, y, bitmap, w, h, color);
  }
  void drawBitmapBg(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg)
  {
    if (!_ensureBegin())
      return;
    _d->drawBitmap(x, y, bitmap, w, h, color, bg);
  }
  void drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color = SSD1306_WHITE)
  {
    if (!_ensureBegin())
      return;
    _d->drawXBitmap(x, y, bitmap, w, h, color);
  }

  // ------- Screen controls -------
  void rotation(uint8_t r)
  {
    _rotation = (r & 0x03);
    if (_d)
      _d->setRotation(_rotation);
  }
  void mode(uint8_t r)
  {
    _rotation = (r & 0x03);
    if (_d)
      _d->setRotation(_rotation);
  }
  uint16_t width() const
  {
    if (_d)
      return _d->width();
    return (_rotation & 1) ? _panelH : _panelW;
  }
  uint16_t height() const
  {
    if (_d)
      return _d->height();
    return (_rotation & 1) ? _panelW : _panelH;
  }
  void fillScreen(uint16_t c)
  {
    if (_ensureBegin())
      _d->fillRect(0, 0, width(), height(), c);
  }
  void clear()
  {
    if (_ensureBegin())
      _d->clearDisplay();
  }
  void display()
  {
    if (_ensureBegin())
      _d->display();
  }
  void show()
  {
    if (_ensureBegin())
      _d->display();
  }
  // Scroll
  void scroll(uint8_t mode, uint8_t startPage = 0, uint8_t endPage = 7)
  {
    if (!_ensureBegin())
      return;
    switch (mode)
    {
    case 0:
      _d->stopscroll();
      break;
    case 1:
      _d->startscrollright(startPage, endPage);
      break;
    case 2:
      _d->startscrollleft(startPage, endPage);
      break;
    case 3:
      _d->startscrolldiagright(startPage, endPage);
      break;
    case 4:
      _d->startscrolldiagleft(startPage, endPage);
      break;
    default:
      _d->stopscroll();
      break;
    }
  }
  void startScrollRight(uint8_t s = 0, uint8_t e = 7)
  {
    if (_ensureBegin())
      _d->startscrollright(s, e);
  }
  void startScrollLeft(uint8_t s = 0, uint8_t e = 7)
  {
    if (_ensureBegin())
      _d->startscrollleft(s, e);
  }
  void startScrollDiagRight(uint8_t s = 0, uint8_t e = 7)
  {
    if (_ensureBegin())
      _d->startscrolldiagright(s, e);
  }
  void startScrollDiagLeft(uint8_t s = 0, uint8_t e = 7)
  {
    if (_ensureBegin())
      _d->startscrolldiagleft(s, e);
  }
  void stopScroll()
  {
    if (_ensureBegin())
      _d->stopscroll();
  }

  Adafruit_SSD1306 *raw()
  {
    _ensureBegin();
    return _d;
  }

private:
  bool _doBegin()
  {
    if (_d)
      delete _d;
    _d = new Adafruit_SSD1306(_panelW, _panelH, _defWire, _defReset);
    if (!_d)
      return false;
    if (!_d->begin(SSD1306_SWITCHCAPVCC, _defAddr))
    {
      _inited = false;
      return false;
    }
    _d->clearDisplay();
    _d->setRotation(_rotation);
    _d->setTextSize(1);
    _d->setTextColor(SSD1306_WHITE);
    _d->setCursor(0, 0);
    _inited = true;
    return true;
  }

  bool _ensureBegin()
  {
    if (_inited)
      return true;
    return _doBegin();
  }

  // NEW: active-low hardware reset helper
  bool _hwReset(uint16_t low_ms, uint16_t high_ms)
  {
    if (_defReset < 0)
      return false; // nothing to do
    pinMode(_defReset, OUTPUT);
    digitalWrite(_defReset, HIGH); // idle high (inactive)
    delay(1);                      // settle
    digitalWrite(_defReset, LOW);  // assert reset
    delay(low_ms);
    digitalWrite(_defReset, HIGH); // release reset
    delay(high_ms);
    return true;
  }

  Adafruit_SSD1306 *_d;
  uint16_t _panelW, _panelH;
  uint8_t _defAddr;
  TwoWire *_defWire;
  int8_t _defReset;
  uint8_t _rotation;
  bool _inited;
  bool _autoHwReset; // NEW
  bool _autoTextBg;
  uint16_t _textBgColor, _textFgColor;
  bool _useCustomTextBg, _useCustomTextFg;
};

extern SSD1306_EZ oled;

#endif
