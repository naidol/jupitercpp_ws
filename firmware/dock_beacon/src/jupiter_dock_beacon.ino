// Jupiter Dock IR Beacon — Arduino Nano
// Generates 38kHz carrier bursted at 100Hz on Pin D9 (OC1A)
// Two TSAL6400 LEDs driven via BC547 transistors from D9
// Logan Naidoo <naidoo.logan@gmail.com>, 2026
// SPDX-License-Identifier: Apache-2.0
//
// Timer1 CTC mode: 16MHz / (2 * 211) = 37,914Hz carrier
// Burst envelope: 90ms ON / 10ms OFF = 10Hz burst rate
// 90% duty cycle: minimises NONE samples from ESP32 50Hz timer
// 10ms gap is enough to reset TSOP38438 AGC every 100ms
#define BURST_ON_MS  90
#define BURST_OFF_MS 10

bool burst_on = true;
unsigned long last_toggle = 0;

void setup() {
  pinMode(9, OUTPUT);

  // Timer1 CTC mode, toggle OC1A (Pin 9), no prescaler
  // f = 16MHz / (2 * (OCR1A + 1)) = 16MHz / (2 * 211) = 37,914Hz
  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A  = 210;

  last_toggle = millis();
}

void loop() {
  unsigned long now      = millis();
  unsigned long interval = burst_on ? BURST_ON_MS : BURST_OFF_MS;

  if (now - last_toggle >= interval) {
    burst_on = !burst_on;
    if (burst_on) {
      TCCR1A |= _BV(COM1A0);   // enable 38kHz toggle on Pin 9
    } else {
      TCCR1A &= ~_BV(COM1A0);  // disable toggle
      digitalWrite(9, LOW);     // ensure LEDs off during gap
    }
    last_toggle = now;
  }
}
