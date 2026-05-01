/*
  Ball Spot Detection — Copper Contact Version
  ─────────────────────────────────────────────
  4 copper contacts on analog pins A1–A4.
  Ball present = contact closes, voltage rises above threshold.

  Serial commands:
    S — scan all 4 spots once
    M — toggle continuous monitoring (500 ms)
*/

// ── Pin Assignments ──────────────────────────────────────────────
const int PIN_SPOT1 = A1;
const int PIN_SPOT2 = A2;
const int PIN_SPOT3 = A3;
const int PIN_SPOT4 = A4;

// ── Tuning ────────────────────────────────────────────────────────
// Raise or lower this until empty reads below it and ball reads above it.
const int SPOT_PRESENT_THRESHOLD = 512;

// ════════════════════════════════════════════════════════════════════
//  SPOT DETECTION
// ════════════════════════════════════════════════════════════════════
int spotPin(int spotIndex) {
  switch (spotIndex) {
    case 0: return PIN_SPOT1;
    case 1: return PIN_SPOT2;
    case 2: return PIN_SPOT3;
    case 3: return PIN_SPOT4;
    default: return PIN_SPOT1;
  }
}

bool isBallAtSpot(int spotIndex) {
  return analogRead(spotPin(spotIndex)) > SPOT_PRESENT_THRESHOLD;
}

void scanAllSpots() {
  Serial.println(F("── Spot Scan ─────────────────────────────"));
  const char* labels[] = { "Spot 1 (UL)", "Spot 2 (UR)",
                            "Spot 3 (LL)", "Spot 4 (LR)" };
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(spotPin(i));
    Serial.print(F("  "));
    Serial.print(labels[i]);
    Serial.print(F("  raw="));
    Serial.print(raw);
    Serial.print(F("  → "));
    Serial.println(raw > SPOT_PRESENT_THRESHOLD ? F("BALL") : F("empty"));
  }
  Serial.println(F("──────────────────────────────────────────"));
}

// ════════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════════════════════════════════
bool monitorMode = false;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_SPOT1, INPUT);
  pinMode(PIN_SPOT2, INPUT);
  pinMode(PIN_SPOT3, INPUT);
  pinMode(PIN_SPOT4, INPUT);

  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║    COPPER CONTACT SPOT DETECTION TEST    ║"));
  Serial.println(F("╠══════════════════════════════════════════╣"));
  Serial.println(F("║  S — scan spots once                     ║"));
  Serial.println(F("║  M — toggle continuous monitoring        ║"));
  Serial.println(F("╚══════════════════════════════════════════╝\n"));

  Serial.println(F("Ready. Enter a command."));
}

void loop() {
  if (monitorMode) {
    scanAllSpots();
    delay(500);
  }

  if (Serial.available()) {
    char c = toupper((char)Serial.read());

    if (c == 'S') {
      scanAllSpots();

    } else if (c == 'M') {
      monitorMode = !monitorMode;
      Serial.println(monitorMode
        ? F("Monitor ON  — press M to stop.")
        : F("Monitor OFF."));
    }
  }
}