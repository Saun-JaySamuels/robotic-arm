/*
  Ball Sorting System — with Replacement Test
  ─────────────────────────────────────────────────────────────────────────────
  After the full sort + verify sequence, the system enters REPLACEMENT TEST
  mode. Remove any ball from any holder spot; the arm will grab the next
  available ball from the dispenser and place it into the vacated spot.
  Color doesn't matter — this is a live "fetch & place" functional test.

  Serial commands (active only while waiting in replacement-test mode):
    S — print current holder state (raw sensor reads + ball/empty status)
    Q — quit replacement-test mode and go home

  Spot sensors:  A1–A4, INPUT_PULLUP, calibrated at startup.
  A ball present = value DROPS below (ambient − SPOT_THRESHOLD_OFFSET).
  A spot empty   = value is near ambient.

  ─────────────────────────────────────────────────────────────────────────────
*/

#include <Servo.h>

// ════════════════════════════════════════════════════════════════════
//  ENUMS & HELPERS
// ════════════════════════════════════════════════════════════════════

enum Color { UNKNOWN = 0, RED, GREEN, BLUE, YELLOW, EMPTY };

const char* colorName(Color c) {
  switch (c) {
    case RED:     return "RED";
    case GREEN:   return "GREEN";
    case BLUE:    return "BLUE";
    case YELLOW:  return "YELLOW";
    case EMPTY:   return "EMPTY";
    default:      return "UNKNOWN";
  }
}

// ════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════

// Servos
Servo servo1, servo2, servo3, servo4;
const int SERVO_PIN1 = 10;  // base
const int SERVO_PIN2 =  9;  // shoulder
const int SERVO_PIN3 = 12;  // elbow
const int SERVO_PIN4 = 13;  // claw

// Color detector
const int PIN_PHOTO   = A5;
const int LED_RED     = 2;
const int LED_GREEN   = 3;
const int LED_BLUE    = 4;
const int LED_PINS[]  = {LED_RED, LED_GREEN, LED_BLUE};
const int LED_COUNT   = 3;

// 74HC595 → 7-segment display
const int PIN_DS   =  6;
const int PIN_SHCP =  5;
const int PIN_STCP =  7;

// Holder presence sensors (copper contact, INPUT_PULLUP)
// Ball present  → contact closes, voltage pulled LOW  → analogRead drops
// Ball absent   → contact open,   voltage near 5 V    → analogRead near 1023
const int SPOT_PINS[]          = {A1, A2, A3, A4};
const int NUM_SPOTS            = 4;
const int SPOT_THRESHOLD_OFFSET = 50;   // how far below ambient = "ball present"

// ════════════════════════════════════════════════════════════════════
//  TUNING CONSTANTS
// ════════════════════════════════════════════════════════════════════

// Ambient (photo) calibration
const int   CALIBRATION_MS    = 2000;
const int   CALIBRATION_READS = 50;
const float DROP_FRACTION     = 0.20;
const int   MIN_DROP          = 40;

// Color measurement
const int SETTLE_MS    = 200;
const int SAMPLE_COUNT = 32;
const int LED_PAUSE_MS = 300;
const int MIN_SIGNAL   = 60;

// Replacement-test: how often to poll the spot sensors (ms)
const int POLL_INTERVAL_MS = 300;

// ════════════════════════════════════════════════════════════════════
//  7-SEGMENT BYTE CONSTANTS
//  Bit mapping: bit0=dp  bit1=a  bit2=b  bit3=c  bit4=d  bit5=e  bit6=f  bit7=g
// ════════════════════════════════════════════════════════════════════

const byte SEG_OFF     = 0b00000000;
const byte SEG_RED     = 0b10100000;  // segments e, g
const byte SEG_GREEN   = 0b11011110;  // segments a,b,c,d,f,g
const byte SEG_BLUE    = 0b11111000;  // segments c,d,e,f,g
const byte SEG_YELLOW  = 0b11011100;  // segments b,c,f,g
const byte SEG_UNKNOWN = 0b10100110;  // segments a,b,e,g

// ════════════════════════════════════════════════════════════════════
//  SYSTEM STATE
// ════════════════════════════════════════════════════════════════════

Color holderState[NUM_SPOTS];
Color targetState[NUM_SPOTS];

int ambientBaseline    = 0;
int detectionThreshold = 0;
int spotAmbient[NUM_SPOTS];

// ════════════════════════════════════════════════════════════════════
//  DISPENSER FIFO QUEUE
// ════════════════════════════════════════════════════════════════════

struct DispenserQueue {
  Color slots[4];
  int   count;

  void init(Color c0, Color c1, Color c2, Color c3) {
    slots[0] = c0; slots[1] = c1; slots[2] = c2; slots[3] = c3;
    count = 4;
  }
  Color peekFront() const { return (count > 0) ? slots[0] : UNKNOWN; }
  Color popFront() {
    if (count == 0) return UNKNOWN;
    Color c = slots[0];
    for (int i = 0; i < count - 1; i++) slots[i] = slots[i + 1];
    count--;
    return c;
  }
  bool pushBack(Color c) {
    if (count >= 4) return false;
    slots[count++] = c;
    return true;
  }
  bool contains(Color c) const {
    for (int i = 0; i < count; i++) if (slots[i] == c) return true;
    return false;
  }
  int indexOf(Color c) const {
    for (int i = 0; i < count; i++) if (slots[i] == c) return i;
    return -1;
  }
  bool frontIs(Color c) const { return (count > 0 && slots[0] == c); }
  bool isFull()  const { return count >= 4; }
  bool isEmpty() const { return count == 0; }

  void printState() const {
    Serial.print(F("Dispenser ["));
    for (int i = 0; i < count; i++) {
      switch (slots[i]) {
        case RED:    Serial.print('R'); break;
        case GREEN:  Serial.print('G'); break;
        case BLUE:   Serial.print('B'); break;
        case YELLOW: Serial.print('Y'); break;
        default:     Serial.print('?'); break;
      }
      if (i < count - 1) Serial.print(',');
    }
    Serial.print(F("] front→back  count="));
    Serial.println(count);
  }
} dispenser;

// ════════════════════════════════════════════════════════════════════
//  ARM POSITIONS
// ════════════════════════════════════════════════════════════════════

const int CLAW_OPEN  = 90;
const int CLAW_CLOSE = 15;

const int TRANSIT_S2 = 135;
const int TRANSIT_S3 = 170;

const int HOME_S1 = 140;
const int HOME_S2 = TRANSIT_S2;
const int HOME_S3 = TRANSIT_S3;
const int HOME_S4 = CLAW_OPEN;

// Holder spots — per-spot S1, S2-down, S3-down
// Layout (looking down from above):
//      arm base
//         |
//      [1] [2]   ← near row
//      [3] [4]   ← far  row
const int HOLDER_S1[NUM_SPOTS]      = {  85,  70,  85,  70 };
const int HOLDER_S2_DOWN[NUM_SPOTS] = {  50,  50,  40,  40 };
const int HOLDER_S3_DOWN[NUM_SPOTS] = { 175, 170, 140, 135 };

// Dispenser
const int DISP_S1           = 178;
const int DISP_PICK_S2_DOWN =  39;
const int DISP_PICK_S3_DOWN = 165;

// Detector tube
const int DET_S1       =  15;
const int DET_S2_DOWN  =  65;
const int DET_S3_DOWN  = 240;

// Motion speed
const int ARM_SPEED_MS = 15;

// ════════════════════════════════════════════════════════════════════
//  ARM MOTION PRIMITIVES
// ════════════════════════════════════════════════════════════════════

void moveServoSlow(Servo &s, int target, int speedDelay = ARM_SPEED_MS) {
  int current = s.read();
  while (current != target) {
    current += (current < target) ? 1 : -1;
    s.write(current);
    delay(speedDelay);
  }
}

void liftToTransit() {
  // Waypoints: {servo3_angle, servo2_angle}
  int waypoints[][2] = {
    {170,  60},
    {180,  65},
    {185,  68},
    {190,  75},
    {195, 135},
  };
  int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);
  const int stepDelay = 12;

  for (int i = 0; i < numWaypoints - 1; i++) {
    int s3Start = waypoints[i][0],   s3End = waypoints[i + 1][0];
    int s2Start = waypoints[i][1],   s2End = waypoints[i + 1][1];
    int steps = max(abs(s3End - s3Start), abs(s2End - s2Start));
    if (steps == 0) continue;
    for (int s = 0; s <= steps; s++) {
      float t = (float)s / steps;
      servo3.write(s3Start + t * (s3End - s3Start));
      servo2.write(s2Start + t * (s2End - s2Start));
      delay(stepDelay);
    }
  }
}

void rotateTo(int s1)                   { moveServoSlow(servo1, s1); }
void descendTo(int s2_down, int s3_down){ moveServoSlow(servo2, s2_down); moveServoSlow(servo3, s3_down); }

void goHome() {
  liftToTransit();
  rotateTo(HOME_S1);
}

// ════════════════════════════════════════════════════════════════════
//  DISPLAY
// ════════════════════════════════════════════════════════════════════

void showSegment(byte pattern) {
  digitalWrite(PIN_STCP, LOW);
  shiftOut(PIN_DS, PIN_SHCP, MSBFIRST, pattern);
  digitalWrite(PIN_STCP, HIGH);
}

void displayColor(Color c) {
  switch (c) {
    case RED:    showSegment(SEG_RED);     break;
    case GREEN:  showSegment(SEG_GREEN);   break;
    case BLUE:   showSegment(SEG_BLUE);    break;
    case YELLOW: showSegment(SEG_YELLOW);  break;
    default:     showSegment(SEG_UNKNOWN); break;
  }
}

// ════════════════════════════════════════════════════════════════════
//  LED / PHOTORESISTOR HELPERS
// ════════════════════════════════════════════════════════════════════

void allLedsOff() {
  for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], LOW);
}

int averagedRead(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) { sum += analogRead(pin); delay(5); }
  return (int)(sum / samples);
}

// ════════════════════════════════════════════════════════════════════
//  AMBIENT CALIBRATION (photoresistor)
// ════════════════════════════════════════════════════════════════════

void calibrateAmbient() {
  Serial.println(F("Calibrating ambient light — keep sensor clear..."));
  long sum = 0;
  int interval = CALIBRATION_MS / CALIBRATION_READS;
  for (int i = 0; i < CALIBRATION_READS; i++) {
    sum += analogRead(PIN_PHOTO);
    delay(interval);
    if (i % 10 == 9) Serial.print('.');
  }
  Serial.println();
  ambientBaseline    = (int)(sum / CALIBRATION_READS);
  int margin         = max((int)(ambientBaseline * DROP_FRACTION), MIN_DROP);
  detectionThreshold = ambientBaseline - margin;
  Serial.print(F("Baseline: "));    Serial.print(ambientBaseline);
  Serial.print(F("  Threshold: ")); Serial.println(detectionThreshold);
}

// ════════════════════════════════════════════════════════════════════
//  COLOR DETECTION
// ════════════════════════════════════════════════════════════════════

int measureReflection(int ledPin) {
  allLedsOff();
  delay(300);
  int baseline = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
  digitalWrite(ledPin, HIGH);
  delay(SETTLE_MS);
  int lit   = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
  digitalWrite(ledPin, LOW);
  delay(LED_PAUSE_MS);
  int delta = lit - baseline;
  return (delta > 0) ? delta : 0;
}

Color classifyColor(int deltas[]) {
  int r = deltas[0], g = deltas[1], b = deltas[2];
  int total = r + g + b;
  if (total < MIN_SIGNAL * 3) return UNKNOWN;

  float rN = (float)r / total;
  float gN = (float)g / total;
  float bN = (float)b / total;

  Serial.print(F("  rN=")); Serial.print(rN);
  Serial.print(F(" gN=")); Serial.print(gN);
  Serial.print(F(" bN=")); Serial.println(bN);

  if (rN > 0.33 && gN >= 0.33 && bN < 0.20 && abs(gN - rN) < 0.10) return YELLOW;
  if (r > g && r > b) return RED;
  if (g > r && g > b) return GREEN;
  return BLUE;
}

Color detectBallInTube() {
  Serial.println(F("  Waiting for ball in detector..."));
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (averagedRead(PIN_PHOTO, SAMPLE_COUNT) < detectionThreshold) break;
    delay(50);
  }
  int current = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
  if (current >= detectionThreshold) {
    Serial.println(F("  Timeout — no ball detected in tube."));
    return UNKNOWN;
  }
  Serial.println(F("  Ball detected. Measuring..."));
  int deltas[LED_COUNT];
  for (int i = 0; i < LED_COUNT; i++) {
    deltas[i] = measureReflection(LED_PINS[i]);
    Serial.print(F("  "));
    Serial.print(i == 0 ? 'R' : i == 1 ? 'G' : 'B');
    Serial.print(F(" delta: ")); Serial.println(deltas[i]);
  }
  Color c = classifyColor(deltas);
  Serial.print(F("  >>> ")); Serial.println(colorName(c));
  displayColor(c);
  return c;
}

// ════════════════════════════════════════════════════════════════════
//  HOLDER SPOT SENSORS (copper contact, INPUT_PULLUP)
//
//  A ball pressing the contact pulls the analog line LOW.
//  "Ball present" = analogRead(pin) < (spotAmbient[i] - SPOT_THRESHOLD_OFFSET)
//  "Spot empty"   = analogRead(pin) is near spotAmbient[i]
//
//  calibrateSpotSensors() records the open-circuit ambient for each pin.
//  isBallAtSpot() compares live readings against that baseline.
// ════════════════════════════════════════════════════════════════════

void calibrateSpotSensors() {
  Serial.println(F("Calibrating holder spot sensors — ensure spots are EMPTY..."));
  for (int i = 0; i < NUM_SPOTS; i++) {
    spotAmbient[i] = analogRead(SPOT_PINS[i]);
    Serial.print(F("  Spot ")); Serial.print(i + 1);
    Serial.print(F(" ambient: ")); Serial.println(spotAmbient[i]);
  }
}

bool isBallAtSpot(int spotIndex) {
  int val = analogRead(SPOT_PINS[spotIndex]);
  return val < (spotAmbient[spotIndex] - SPOT_THRESHOLD_OFFSET);
}

// Convenience: print a one-line summary of all 4 spots.
void printSpotStatus() {
  Serial.println(F("── Spot Status ─────────────────────────────"));
  const char* labels[] = { "Spot 1 (UL)", "Spot 2 (UR)",
                            "Spot 3 (LL)", "Spot 4 (LR)" };
  for (int i = 0; i < NUM_SPOTS; i++) {
    int raw = analogRead(SPOT_PINS[i]);
    Serial.print(F("  "));
    Serial.print(labels[i]);
    Serial.print(F("  raw="));
    Serial.print(raw);
    Serial.print(F("  ambient="));
    Serial.print(spotAmbient[i]);
    Serial.print(F("  → "));
    Serial.println(isBallAtSpot(i) ? F("BALL") : F("empty"));
  }
  Serial.println(F("────────────────────────────────────────────"));
}

// ════════════════════════════════════════════════════════════════════
//  SERIAL: read target sequence
// ════════════════════════════════════════════════════════════════════

Color charToColor(char c) {
  switch (c) {
    case 'R': case 'r': return RED;
    case 'G': case 'g': return GREEN;
    case 'B': case 'b': return BLUE;
    case 'Y': case 'y': return YELLOW;
    default:            return UNKNOWN;
  }
}

void readTargetFromSerial() {
  Serial.println(F("READY: send target sequence as 4 chars (R/G/B/Y), e.g. RGBY"));
  int received = 0;
  while (received < NUM_SPOTS) {
    if (Serial.available()) {
      char c = Serial.read();
      Color col = charToColor(c);
      if (col != UNKNOWN) {
        targetState[received] = col;
        Serial.print(F("  Spot ")); Serial.print(received + 1);
        Serial.print(F(" → "));    Serial.println(colorName(col));
        received++;
      }
    }
  }
  Serial.println(F("Target loaded."));
}

// ════════════════════════════════════════════════════════════════════
//  HIGH-LEVEL ARM ACTIONS
//  Invariant: arm enters and exits each action at TRANSIT pose.
// ════════════════════════════════════════════════════════════════════

void pickFromDispenser() {
  Serial.print(F("[ARM] Pick from dispenser front: "));
  Serial.println(colorName(dispenser.peekFront()));

  rotateTo(DISP_S1);
  moveServoSlow(servo4, CLAW_OPEN);
  descendTo(DISP_PICK_S2_DOWN, DISP_PICK_S3_DOWN);
  delay(300);
  moveServoSlow(servo4, 10, 15);
  delay(400);

  moveServoSlow(servo3, 180, 15);
  moveServoSlow(servo2, 50, 15);
  delay(300);
  moveServoSlow(servo1, 140, 15);
  delay(300);

  liftToTransit();
  dispenser.popFront();
}

void placeToDispenser(Color c) {
  Serial.print(F("[ARM] Place to dispenser back: ")); Serial.println(colorName(c));

  moveServoSlow(servo1, 178, 15);
  delay(500);
  moveServoSlow(servo3, 40, 15);
  moveServoSlow(servo2, 50, 15);
  delay(500);
  moveServoSlow(servo4, 90, 15);
  delay(500);

  dispenser.pushBack(c);
}

void dropToDetector() {
  Serial.println(F("[ARM] Drop to detector"));
  rotateTo(DET_S1);
  descendTo(DET_S2_DOWN, DET_S3_DOWN);
  delay(300);
  moveServoSlow(servo4, CLAW_OPEN);
  delay(400);
  liftToTransit();
}

void pickFromDetector() {
  Serial.println(F("[ARM] Pick from detector"));
  rotateTo(DET_S1);
  moveServoSlow(servo4, CLAW_OPEN);
  descendTo(DET_S2_DOWN, DET_S3_DOWN);
  delay(200);
  moveServoSlow(servo4, CLAW_CLOSE);
  delay(300);
  liftToTransit();
}

void pickFromHolder(int i) {
  Serial.print(F("[ARM] Pick from holder spot ")); Serial.println(i + 1);
  rotateTo(HOLDER_S1[i]);
  moveServoSlow(servo4, CLAW_OPEN);

  if (i < 2) {
    descendTo(HOLDER_S2_DOWN[i], HOLDER_S3_DOWN[i]);
  } else {
    moveServoSlow(servo3, HOLDER_S3_DOWN[i]);
    moveServoSlow(servo2, HOLDER_S2_DOWN[i]);
  }

  delay(200);
  moveServoSlow(servo4, CLAW_CLOSE);
  delay(300);
  liftToTransit();
  delay(500);
}

void placeToHolder(int i) {
  Serial.print(F("[ARM] Place to holder spot ")); Serial.println(i + 1);
  rotateTo(HOLDER_S1[i]);

  if (i < 2) {
    descendTo(HOLDER_S2_DOWN[i], HOLDER_S3_DOWN[i]);
  } else {
    moveServoSlow(servo3, HOLDER_S3_DOWN[i]);
    moveServoSlow(servo2, HOLDER_S2_DOWN[i]);
  }

  delay(200);
  moveServoSlow(servo4, CLAW_OPEN);
  delay(300);
  liftToTransit();
}

// ════════════════════════════════════════════════════════════════════
//  SCAN / RESOLVE / VERIFY
// ════════════════════════════════════════════════════════════════════

Color scanHolderSpot(int i) {
  Serial.print(F("[SCAN] Spot ")); Serial.println(i + 1);
  pickFromHolder(i);
  dropToDetector();
  Color c = detectBallInTube();
  pickFromDetector();
  placeToHolder(i);
  holderState[i] = c;
  Serial.print(F("  Spot ")); Serial.print(i + 1);
  Serial.print(F(" = ")); Serial.println(colorName(c));
  return c;
}

void scanAllHolderSpots() {
  Serial.println(F("\n=== PHASE 1: Scanning holder ==="));
  for (int i = 0; i < NUM_SPOTS; i++) {
    holderState[i] = UNKNOWN;
    scanHolderSpot(i);
    displayColor(holderState[i]);
    delay(500);
  }
  Serial.println(F("Scan complete. Holder state:"));
  for (int i = 0; i < NUM_SPOTS; i++) {
    Serial.print(F("  Spot ")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(colorName(holderState[i]));
    Serial.print(F("  (target: ")); Serial.print(colorName(targetState[i])); Serial.println(')');
  }
}

int findInHolder(Color c, int exclude = -1) {
  for (int i = 0; i < NUM_SPOTS; i++) {
    if (i == exclude) continue;
    if (holderState[i] == c) return i;
  }
  return -1;
}

bool allCorrect() {
  for (int i = 0; i < NUM_SPOTS; i++) {
    if (holderState[i] != targetState[i]) return false;
  }
  return true;
}

void doDispenserSwap(int i, Color wantedColor, bool verifyAfter) {
  Color wrongBall = holderState[i];

  pickFromHolder(i);
  holderState[i] = UNKNOWN;

  dropToDetector();

  pickFromDispenser();

  placeToHolder(i);
  holderState[i] = wantedColor;

  pickFromDetector();
  placeToDispenser(wrongBall);

  if (verifyAfter) {
    Color seen = scanHolderSpot(i);
    if (seen != wantedColor) {
      Serial.print(F("  [SWAP] WARNING: post-swap scan saw "));
      Serial.print(colorName(seen));
      Serial.print(F(", expected "));
      Serial.println(colorName(wantedColor));
      holderState[i] = seen;
    }
  }
}

void resolveAll() {
  Serial.println(F("\n=== PHASE 2: Resolving positions ==="));
  int maxPasses = 20;
  int pass = 0;

  while (!allCorrect() && pass < maxPasses) {
    pass++;
    bool madeProgress = false;

    for (int i = 0; i < NUM_SPOTS; i++) {
      if (holderState[i] == targetState[i]) continue;

      Color needed = targetState[i];
      Serial.print(F("\n[Resolve] Spot ")); Serial.print(i + 1);
      Serial.print(F(" has ")); Serial.print(colorName(holderState[i]));
      Serial.print(F(", needs ")); Serial.println(colorName(needed));

      // Case A: needed color is in another holder spot
      int srcSpot = findInHolder(needed, i);
      if (srcSpot != -1) {
        Serial.print(F("  Case A: swap with holder spot ")); Serial.println(srcSpot + 1);
        Color wrongBall = holderState[i];

        pickFromHolder(i);
        dropToDetector();
        holderState[i] = UNKNOWN;

        pickFromHolder(srcSpot);
        placeToHolder(i);
        holderState[i]       = needed;
        holderState[srcSpot] = UNKNOWN;

        pickFromDetector();
        placeToHolder(srcSpot);
        holderState[srcSpot] = wrongBall;

        madeProgress = true;
        break;
      }

      // Case B: needed color is in the dispenser
      if (dispenser.contains(needed)) {
        Serial.print(F("  Case B: dispenser swap. Queue: ")); dispenser.printState();

        int neededPos = dispenser.indexOf(needed);
        for (int cyc = 0; cyc < neededPos; cyc++) {
          Color frontBall = dispenser.peekFront();
          Serial.print(F("  Cycle ")); Serial.print(cyc + 1);
          Serial.print(F(": re-queuing ")); Serial.println(colorName(frontBall));

          pickFromDispenser();
          dropToDetector();
          pickFromDetector();
          placeToDispenser(frontBall);
        }

        Serial.print(F("  Cycle complete. Queue: ")); dispenser.printState();
        doDispenserSwap(i, needed, /*verifyAfter=*/true);
        madeProgress = true;
        break;
      }

      Serial.println(F("  Deferring — needed color not yet accessible."));
    }

    if (!madeProgress) {
      Serial.println(F("No progress this pass — re-scanning holder to update state."));
      for (int i = 0; i < NUM_SPOTS; i++) {
        if (holderState[i] != targetState[i]) scanHolderSpot(i);
      }
    }
  }
}

void verifyAndReport() {
  Serial.println(F("\n=== VERIFICATION ==="));
  bool ok = true;
  for (int i = 0; i < NUM_SPOTS; i++) {
    scanHolderSpot(i);
    bool match = (holderState[i] == targetState[i]);
    Serial.print(F("  Spot ")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(colorName(holderState[i]));
    Serial.print(match ? F("  OK") : F("  MISMATCH (expected "));
    if (!match) { Serial.print(colorName(targetState[i])); Serial.print(')'); }
    Serial.println();
    if (!match) ok = false;
  }
  if (ok) {
    Serial.println(F("\nSorting complete. All positions correct."));
    showSegment(SEG_GREEN);
  } else {
    Serial.println(F("\nWARNING: Some positions still incorrect. Check sensors / tuning."));
    showSegment(SEG_UNKNOWN);
  }
}

// ════════════════════════════════════════════════════════════════════
//  REPLACEMENT TEST
// ────────────────────────────────────────────────────────────────────
//  After sorting is done, sit in a loop and watch the 4 spot sensors.
//  When any spot transitions from BALL → empty, immediately fetch the
//  next available ball from the dispenser and place it into that spot.
//  Color doesn't matter — this is purely a "detect removal, fetch,
//  place" functional test.
//
//  The sensor poll uses isBallAtSpot() which was already calibrated.
//  We track the last-known state to detect the falling edge (removal).
//
//  Serial commands while waiting:
//    S — print current raw sensor readings for all spots
//    Q — quit replacement-test mode (arm goes home)
//
//  The function returns when the user sends 'Q' or the dispenser runs
//  out of balls.
// ════════════════════════════════════════════════════════════════════

void replacementTestLoop() {
  Serial.println(F("\n╔══════════════════════════════════════════════╗"));
  Serial.println(F("║        REPLACEMENT TEST MODE ACTIVE          ║"));
  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.println(F("║  Remove any ball from the holder.            ║"));
  Serial.println(F("║  The arm will fetch a replacement from the   ║"));
  Serial.println(F("║  dispenser and place it in the vacant spot.  ║"));
  Serial.println(F("║                                              ║"));
  Serial.println(F("║  S — print spot sensor readings              ║"));
  Serial.println(F("║  Q — quit and go home                        ║"));
  Serial.println(F("╚══════════════════════════════════════════════╝\n"));

  // Snapshot current occupancy so we only react to *changes*.
  bool prevOccupied[NUM_SPOTS];
  for (int i = 0; i < NUM_SPOTS; i++) {
    prevOccupied[i] = isBallAtSpot(i);
  }

  while (true) {

    // ── Serial command handler ─────────────────────────────────────
    if (Serial.available()) {
      char cmd = toupper((char)Serial.read());
      if (cmd == 'S') {
        printSpotStatus();
      } else if (cmd == 'Q') {
        Serial.println(F("\nReplacement test ended by user."));
        return;
      }
    }

    // ── Check dispenser ───────────────────────────────────────────
    if (dispenser.isEmpty()) {
      Serial.println(F("[REPLACEMENT] Dispenser is empty — test complete."));
      return;
    }

    // ── Poll each spot for a ball-removal event ───────────────────
    for (int i = 0; i < NUM_SPOTS; i++) {
      bool nowOccupied = isBallAtSpot(i);

      // Detect falling edge: spot was occupied, now it's empty.
      if (prevOccupied[i] && !nowOccupied) {

        // Debounce: wait a short time and confirm it's still empty.
        delay(200);
        if (isBallAtSpot(i)) {
          // Momentary noise — spot refilled itself, ignore.
          prevOccupied[i] = true;
          continue;
        }

        Serial.print(F("\n[REPLACEMENT] Ball removed from spot "));
        Serial.println(i + 1);
        Serial.print(F("  Fetching replacement from dispenser front: "));
        Serial.println(colorName(dispenser.peekFront()));

        // Fetch the front ball from the dispenser.
        pickFromDispenser();                // arm now holds a ball; dispenser count−1

        // Place it into the vacant spot.
        placeToHolder(i);                   // arm releases; claw open; back at TRANSIT
        holderState[i] = UNKNOWN;          // color unknown (not scanned); mark as placed

        Serial.print(F("[REPLACEMENT] Spot "));
        Serial.print(i + 1);
        Serial.println(F(" refilled. Returning to watch mode."));
        Serial.println(F("  (Remove another ball or send Q to quit)\n"));

        // Update the 7-segment to show a dash while in test mode.
        showSegment(SEG_UNKNOWN);

        // Update snapshot: this spot now has a ball again.
        prevOccupied[i] = true;

        // After placing, re-snapshot ALL spots (arm movement may have
        // jostled adjacent sensors temporarily).
        delay(500);
        for (int j = 0; j < NUM_SPOTS; j++) {
          prevOccupied[j] = isBallAtSpot(j);
        }

        // Only handle one removal per poll cycle to stay predictable.
        break;
      }

      // Update snapshot for stable "still empty" or "still occupied" case.
      prevOccupied[i] = nowOccupied;
    }

    delay(POLL_INTERVAL_MS);
  }
}

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);

  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);
  servo4.attach(SERVO_PIN4);

  servo4.write(HOME_S4);
  servo3.write(HOME_S3);
  servo2.write(HOME_S2);
  servo1.write(HOME_S1);

  for (int i = 0; i < LED_COUNT; i++) pinMode(LED_PINS[i], OUTPUT);
  pinMode(PIN_PHOTO, INPUT);
  pinMode(PIN_DS,    OUTPUT);
  pinMode(PIN_SHCP,  OUTPUT);
  pinMode(PIN_STCP,  OUTPUT);
  for (int i = 0; i < NUM_SPOTS; i++) pinMode(SPOT_PINS[i], INPUT_PULLUP);
  delay(100);

  allLedsOff();
  showSegment(SEG_UNKNOWN);

  Serial.println(F("======================================"));
  Serial.println(F("  BALL SORTING SYSTEM — initialising"));
  Serial.println(F("======================================"));

  dispenser.init(RED, GREEN, BLUE, YELLOW);

  calibrateAmbient();
  calibrateSpotSensors();   // <── spots must be EMPTY at this moment
  readTargetFromSerial();

  scanAllHolderSpots();
  resolveAll();
  verifyAndReport();

  goHome();
  Serial.println(F("\nSystem idle. Starting replacement test..."));
  delay(1000);

  // ── REPLACEMENT TEST (runs until dispenser empty or user sends Q) ──
  replacementTestLoop();

  goHome();
  Serial.println(F("\nAll done. System idle."));
  showSegment(SEG_OFF);
}

// ════════════════════════════════════════════════════════════════════
//  LOOP — nothing to do; all logic is driven from setup()
// ════════════════════════════════════════════════════════════════════

void loop() {
  delay(1000);
}
