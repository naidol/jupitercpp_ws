// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// dock_charger — Arduino Nano on the DOCK: gates charge power via the SSR, keyed by the
// robot's 38kHz IR "charge-enable" beacon.
//
//   D2  <- TSOP/VS1838B receiver OUT   (idle HIGH; LOW during 38kHz bursts)  [INT0]
//   D5  -> SSR-25DD control (+)        (HIGH = SSR closed = charge power ON)
//   5V/GND -> receiver (RIGHT leg = Vcc via 100R + 10uF filter, MID = GND, LEFT = OUT)
//
// RULE (fail-safe by construction):
//   * SSR ON  only after SUSTAINED beacon activity  (>= ON_CONFIRM_MS of continuous presence)
//   * SSR OFF fast on silence                       (>  OFF_TIMEOUT_MS without edges)
// Silence always wins: robot full / undocking / fault / power loss -> pins dead in ~1/4 s,
// BEFORE any physical separation -> no arcing at the pogo contacts.
//
// The robot (ESP32, GPIO4 -> TSAL6400) sends gated 38kHz bursts only while it has confirmed
// DOCKED via its proximity sensors. A stray remote press can't reach ON_CONFIRM_MS sustained.

const uint8_t  PIN_IR  = 2;     // INT0
const uint8_t  PIN_SSR = 5;

const unsigned long WINDOW_MS     = 100;  // edge-counting window
const uint8_t       MIN_EDGES     = 2;    // edges per window = "active" (NEC remote REPEAT frames are sparse ~2-4 edges/110ms; healthy receiver gives ~0 noise edges)
const unsigned long ON_CONFIRM_MS = 500;  // continuous activity required to switch ON
const unsigned long OFF_TIMEOUT_MS = 300; // silence to switch OFF (covers NEC repeat spacing ~110ms and beacon gaps)

volatile unsigned int edge_count = 0;

void onEdge() { edge_count++; }

unsigned long window_start   = 0;
unsigned long active_since   = 0;   // 0 = not currently in an active streak
unsigned long last_active_ms = 0;
bool ssr_on = false;

void setSSR(bool on) {
  if (on != ssr_on) {
    ssr_on = on;
    digitalWrite(PIN_SSR, on ? HIGH : LOW);
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
    Serial.print(F("[dock_charger] SSR -> "));
    Serial.println(on ? F("ON (charging)") : F("OFF"));
  }
}

void setup() {
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);            // power-on state: DEAD dock, always
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(PIN_IR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IR), onEdge, FALLING);
  Serial.begin(115200);
  Serial.println(F("[dock_charger] up. SSR OFF. Waiting for sustained 38kHz beacon..."));
  window_start = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - window_start >= WINDOW_MS) {
    noInterrupts();
    unsigned int edges = edge_count;
    edge_count = 0;
    interrupts();
    window_start = now;

    if (edges >= MIN_EDGES) {
      last_active_ms = now;
      if (active_since == 0) active_since = now;      // streak begins
    }
    // streak broken only by real silence (not by a single sparse window)
    if (now - last_active_ms > OFF_TIMEOUT_MS) active_since = 0;

    if (!ssr_on && active_since != 0 && (now - active_since >= ON_CONFIRM_MS)) setSSR(true);
    if ( ssr_on && (now - last_active_ms > OFF_TIMEOUT_MS))                    setSSR(false);
  }
}
