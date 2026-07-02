# ASSI LEDs — UART bridge to an Arduino Nano

The STM32 no longer drives the WS2812 strip directly (the 3.3 V-vs-5 V DIN level
problem made that unreliable). Instead it sends a tiny serial command over
**USART10** to an **Arduino Nano**, and the 5 V Nano drives the strip — its 5 V
logic output cleanly meets the WS2812's VIH, so the level problem goes away.

## Wire protocol

USART10 TX, **115200 8N1**, one ASCII command per state, newline-terminated:

| Command | Meaning |
|---------|---------|
| `a\n`   | LEDs off |
| `b\n`   | LEDs yellow |
| `c\n`   | LEDs blue |

Commands are re-sent every ASSI refresh (~2–5 Hz), so a dropped byte self-heals
on the next refresh. Single-char keeps parsing trivial and is easy to eyeball in
a serial monitor.

### Firmware API (`ws2812.h`)
`leds_off()`, `leds_blue()`, `leds_yellow()` — called by `assi_task.c`, which
maps the 5 ASSI modes onto them (OFF→off, READY→yellow, DRIVING→yellow flash,
EMERGENCY→blue flash, FINISHED→blue).

### Arduino Nano sketch (reference)
```cpp
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
void set(uint8_t r,uint8_t g,uint8_t b){ for(int i=0;i<NUM_LEDS;i++) strip.setPixelColor(i,r,g,b); strip.show(); }
void setup(){ Serial.begin(115200); strip.begin(); set(0,0,0); }
void loop(){
  if(!Serial.available()) return;
  char c = Serial.read();
  if      (c=='a') set(0,0,0);
  else if (c=='b') set(255,255,0);
  else if (c=='c') set(0,0,255);
}
```
Wiring: STM32 **PG12 (USART10_TX) → Nano RX (D0)**, and **common ground**.

## Pin
USART10 is on **PG11 (RX) / PG12 (TX)**, AF11 — matching the team's standard
`.ioc`. Only TX is used. The `MX_USART10_UART_Init` / `HAL_UART_MspInit` in
`Core/Src/usart.c` (and `uDV.ioc`) reflect this.

## Implementation notes
- USART10 is **CubeMX-generated HAL** (`huart10`, 115200 8N1). `ws2812.c` sends
  each command with `HAL_UART_Transmit` (blocking, ~0.17 ms for 2 bytes, no IRQ
  masking).
- **SPI is fully removed** — from the `.ioc` (CubeMX), the code, and the build
  (spi.c/dma.c gone, HAL SPI module disabled, HAL UART module enabled).
