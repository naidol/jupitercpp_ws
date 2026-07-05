// Jupiter Dock IR Beacon — Arduino Nano
// 38kHz carrier on D9 (OC1A), emitted as SHORT-BURST PACKETS for a TSOP/VS1838B
// -class receiver. Two TSAL6400 LEDs driven via BC547 transistors from D9.
// Logan Naidoo <naidoo.logan@gmail.com>, 2026
// SPDX-License-Identifier: Apache-2.0
//
// WHY PACKETS (not the old 90ms burst):
//   The KY-022 / TSOP38438 / VS1838B is a *remote-control data* receiver with AGC.
//   It suppresses long/continuous carriers (a 90ms burst is ~continuous to it), so
//   it only blipped at burst onset. Instead we send a PACKET of short bursts, then a
//   gap — the structure a remote receiver is built to demodulate. Each burst becomes
//   one clean output pulse; the ESP32 detects the pulse train via edge interrupts.
//
// Carrier: Timer1 CTC, toggle OC1A on compare match, no prescaler.
//   f = 16MHz / (2 * (OCR1A+1)) = 16MHz / (2*211) = 37,914 Hz
#define CARRIER_TOP        210     // OCR1A -> 37.9kHz carrier

// Envelope (micros()-based; 600us needs sub-millisecond resolution)
#define BURST_ON_US        600UL   // carrier ON  per burst  (~23 cycles, > TSOP ~10-cycle min)
#define BURST_OFF_US       600UL   // carrier OFF between bursts within a packet
#define BURSTS_PER_PACKET  10      // 10 bursts -> ~12ms packet
#define PACKET_GAP_US      40000UL // silence after each packet -> AGC resets (~19Hz packet rate)

enum Phase { BURST_ON, BURST_OFF, PACKET_GAP };
Phase         phase       = BURST_ON;
uint8_t       burst_count = 0;
unsigned long last_us     = 0;     // micros() at last phase change

static inline void carrierOn()  { TCCR1A |= _BV(COM1A0); }                            // enable 38kHz toggle on D9
static inline void carrierOff() { TCCR1A &= ~_BV(COM1A0); digitalWrite(9, LOW); }     // hold LEDs off in gaps

void setup() {
  pinMode(9, OUTPUT);
  TCCR1A = _BV(COM1A0);              // toggle OC1A on compare match
  TCCR1B = _BV(WGM12) | _BV(CS10);  // CTC mode, no prescaler
  OCR1A  = CARRIER_TOP;
  carrierOn();
  last_us = micros();
}

void loop() {
  unsigned long now = micros();
  switch (phase) {
    case BURST_ON:                                  // carrier on for one burst
      if (now - last_us >= BURST_ON_US) {
        carrierOff();
        last_us = now;
        phase = (++burst_count >= BURSTS_PER_PACKET) ? PACKET_GAP : BURST_OFF;
      }
      break;
    case BURST_OFF:                                 // gap between bursts in a packet
      if (now - last_us >= BURST_OFF_US) {
        carrierOn();
        last_us = now;
        phase = BURST_ON;
      }
      break;
    case PACKET_GAP:                                // long silence, lets the AGC reset
      if (now - last_us >= PACKET_GAP_US) {
        burst_count = 0;
        carrierOn();
        last_us = now;
        phase = BURST_ON;
      }
      break;
  }
}
