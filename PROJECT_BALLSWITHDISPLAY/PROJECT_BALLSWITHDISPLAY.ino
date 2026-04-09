/*
Ball Color Detector — Auto-Calibrating + 7-Segment Output via 595
───────────────────────────────────────────────────────────────────
Pin 2  → Red LED (OUTPUT)
Pin 4  → Green LED (OUTPUT)
Pin 6  → Blue LED (OUTPUT)
Pin A5 → Photoresistor (voltage divider to GND, 10kΩ pull-down)

595 Shift Register (7-segment display, common cathode):
  DS   (pin 14) → Arduino pin 8
  SH_CP(pin 11) → Arduino pin 10
  ST_CP(pin 12) → Arduino pin 11
  MR   (pin 10) → Arduino pin 12
  OE   (pin 13) → GND (always enabled)

7-segment digits:
  Red    → e, g
  Green  → a, b, c, d, f, g
  Blue   → c, d, e, f, g
  Yellow → b, c, f, g
  Unknown→ a, b, e, g  (interrogation mark)
  Off    → all segments off
*/

// Pin Definitions ─
const int PIN_PHOTO = A5;

const int LED_PINS[]     = {2, 4, 6};
const String LED_NAMES[] = {"Red", "Green", "Blue"};
const int LED_COUNT      = 3;

// 595 pins
const int PIN_DS    = 12;   // Data
const int PIN_SHCP  = 10;  // Shift clock
const int PIN_STCP  = 11;  // Latch clock


// Tuning Constants
const int CALIBRATION_MS    = 2000;
const int CALIBRATION_READS = 50;
const float DROP_FRACTION   = 0.20;
const int MIN_DROP          = 40;

const int SETTLE_MS         = 200;
const int SAMPLE_COUNT      = 32;
const int LED_PAUSE_MS      = 300;

const int MIN_SIGNAL        = 60;

const byte SEG_OFF     = 0b00000000; // all off
const byte SEG_RED     = 0b10100000; // e, g         → bits 5,7
const byte SEG_GREEN   = 0b11011110; // a,b,c,d,f,g → bits 1-7
const byte SEG_BLUE    = 0b11111000; // c,d,e,f,g    → bits 3,4,5,6,7
const byte SEG_YELLOW  = 0b11001100; // b,c,f,g      → bits 2,3,6,7
const byte SEG_UNKNOWN = 0b10100110; // a,b,e,g      → bits 1,2,5,7

// ── Globals ────────────────────────────────────────────────────────────────
int ambientBaseline    = 0;
int detectionThreshold = 0;
bool ballPresent       = false;

// ── 595 Helper ─────────────────────────────────────────────────────────────
void showSegment(byte pattern) {
  digitalWrite(PIN_STCP, LOW);
  shiftOut(PIN_DS, PIN_SHCP, MSBFIRST, pattern);
  digitalWrite(PIN_STCP, HIGH);
}

// ── Helpers ────────────────────────────────────────────────────────────────
void allLedsOff() {
  for (int i = 0; i < LED_COUNT; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
}

int averagedRead(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return (int)(sum / samples);
}

void calibrateAmbient() {
  Serial.println("Calibrating ambient light — keep sensor clear...");

  long sum = 0;
  int interval = CALIBRATION_MS / CALIBRATION_READS;

  for (int i = 0; i < CALIBRATION_READS; i++) {
    sum += analogRead(PIN_PHOTO);
    delay(interval);
    if (i % 10 == 9) Serial.print(".");
  }
  Serial.println();

  ambientBaseline = (int)(sum / CALIBRATION_READS);

  int fractionDrop = (int)(ambientBaseline * DROP_FRACTION);
  int margin = max(fractionDrop, MIN_DROP);
  detectionThreshold = ambientBaseline - margin;

  Serial.print("Ambient baseline: ");    Serial.println(ambientBaseline);
  Serial.print("Drop margin used: ");    Serial.println(margin);
  Serial.print("Detection threshold: "); Serial.println(detectionThreshold);
  Serial.println("Calibration complete. Ready.");
  Serial.println("─────────────────────────────");
}

int measureReflection(int ledPin) {
  allLedsOff();
  delay(300);
  int baseline = averagedRead(PIN_PHOTO, SAMPLE_COUNT);

  digitalWrite(ledPin, HIGH);
  delay(SETTLE_MS);
  int lit = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
  digitalWrite(ledPin, LOW);
  delay(LED_PAUSE_MS);

  int delta = lit - baseline;
  return (delta > 0) ? delta : 0;
}

String classifyColor(int deltas[]) {
  int r = deltas[0], g = deltas[1], b = deltas[2];
  int total = r + g + b;

  if (total < MIN_SIGNAL * 3) {
    return "UNKNOWN";
  }

  float rN = (float)r / total;
  float gN = (float)g / total;
  float bN = (float)b / total;

  Serial.print("  Normalized R: "); Serial.println(rN);
  Serial.print("  Normalized G: "); Serial.println(gN);
  Serial.print("  Normalized B: "); Serial.println(bN);

  // Yellow: red suppressed, green and blue close together
  if (rN < 0.33 && gN > rN && bN > rN && abs(gN - bN) < 0.05) return "YELLOW";

  if (r >= g && r >= b) return "RED";
  if (g >= r && g >= b) return "GREEN";
  return "BLUE";
}

void displayColor(String color) {
  if      (color == "RED")    showSegment(SEG_RED);
  else if (color == "GREEN")  showSegment(SEG_GREEN);
  else if (color == "BLUE")   showSegment(SEG_BLUE);
  else if (color == "YELLOW") showSegment(SEG_YELLOW);
  else                        showSegment(SEG_UNKNOWN);
}

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  pinMode(PIN_PHOTO, INPUT);
  pinMode(PIN_DS,   OUTPUT);
  pinMode(PIN_SHCP, OUTPUT);
  pinMode(PIN_STCP, OUTPUT);

  allLedsOff();
  showSegment(SEG_UNKNOWN);

  Serial.println("Ball Color Detector — Auto-Calibrating");
  Serial.println("=======================================");
  calibrateAmbient();
}

// ── Main Loop ──────────────────────────────────────────────────────────────
void loop() {
  int current = averagedRead(PIN_PHOTO, SAMPLE_COUNT);

  // ── No ball ──────────────────────────────────────────────────────────────
  if (current >= detectionThreshold) {
    if (ballPresent) {
      Serial.println("Ball removed. Waiting...\n");
      ballPresent = false;
      showSegment(SEG_OFF);
    }
    delay(50);
    return;
  }

  // ── Ball detected ─────────────────────────────────────────────────────────
  if (!ballPresent) {
    ballPresent = true;

    Serial.println("Ball detected!");
    Serial.print("  Ambient baseline: "); Serial.println(ambientBaseline);
    Serial.print("  Current reading:  "); Serial.println(current);
    Serial.print("  Drop amount:      "); Serial.println(ambientBaseline - current);
    Serial.println("Measuring color...");

    int deltas[LED_COUNT];
    for (int i = 0; i < LED_COUNT; i++) {
      deltas[i] = measureReflection(LED_PINS[i]);
      Serial.print("  "); Serial.print(LED_NAMES[i]);
      Serial.print(" delta: "); Serial.println(deltas[i]);
    }

    String color = classifyColor(deltas);
    Serial.print(">>> Color: ");
    Serial.println(color);
    Serial.println("─────────────────────────────");

    displayColor(color);
  }

  delay(100);
}