# ASSI LEDs — UART bridge to an Arduino Nano

The STM32 no longer drives the WS2812 strip directly (the 3.3 V-vs-5 V DIN level
problem made that unreliable). Instead it sends a tiny serial command over
**USART10** to an **Arduino Nano**, and the 5 V Nano drives the strip — its 5 V
logic output cleanly meets the WS2812's VIH, so the level problem goes away.

## Wire protocol

USART10 TX, **115200 8N1**, one ASCII command per state, newline-terminated:

| Command | Meaning |
|---------|---------|
| `O\n`   | LEDs off |
| `B\n`   | LEDs blue |
| `Y\n`   | LEDs yellow |

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
  if      (c=='O') set(0,0,0);
  else if (c=='B') set(0,0,255);
  else if (c=='Y') set(255,255,0);
}
```
Wiring: STM32 **USART10_TX → Nano RX (D0)**, and **common ground**.

## Pin — CONFIRM against the board
`usart.c` uses **PE3 = USART10_TX (AF4)**. If the Nano's RX is wired to a
different MCU pin, change `USART10_TX_PORT/PIN/AF` in `Core/Src/usart.c`.
USART10_TX is only available on the AF4/AF11 pins in the datasheet AF table.

## Implementation notes
- The HAL UART driver is **not** in this project, so USART10 is driven at
  register level (`Core/Src/usart.c`). No `HAL_UART_MODULE` needed.
- **SPI is fully removed** from the code and build (spi.c/dma.c deleted, HAL SPI
  module disabled, IRQ handlers and build-list entries dropped).
- **`uDV.ioc` still lists SPI1** — it was NOT hand-edited (blind CubeMX pin/IP
  renumbering is risky). Before regenerating from CubeMX: disable SPI1 and add
  USART10 on the correct TX pin there, otherwise a regen will resurrect SPI1.
