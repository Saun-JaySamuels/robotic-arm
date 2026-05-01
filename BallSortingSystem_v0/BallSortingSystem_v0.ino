/*
================================================================================
  ROBOTIC BALL SORTING SYSTEM — Full Integration
================================================================================

HARDWARE SUMMARY
  Servo 1  (pin 10) — base rotation
  Servo 2  (pin  9) — shoulder
  Servo 3  (pin 12) — elbow
  Servo 4  (pin 13) — claw (90 = open, 15 = closed)

  Photoresistor       → A5 (voltage divider, 10 kΩ pull-down)
  RGB LEDs            → pins 2 (Red), 3 (Green), 4 (Blue)

  74HC595 shift register driving common-cathode 7-segment display:
    DS   (pin 14 of 595) → Arduino pin  8
    SH_CP(pin 11 of 595) → Arduino pin 10
    ST_CP(pin 12 of 595) → Arduino pin 11
    MR   (pin 10 of 595) → 5 V directly on breadboard
    OE   (pin 13 of 595) → GND

  Ball-holder presence sensors (copper-mesh, INPUT_PULLUP):
    Spot 1 → A1,  Spot 2 → A2,  Spot 3 → A3,  Spot 4 → A4

SERIAL PROTOCOL
  At startup the sketch sends:
      READY: send target as 4 chars, e.g. RGBY
  You type exactly 4 characters (no newline needed):
      R = Red  G = Green  B = Blue  Y = Yellow
  The system then runs autonomously and reports progress.

SYSTEM INVARIANTS
  · Dispenser holds exactly 4 balls at start (R, G, B, Y — one each).
  · Holder holds exactly 4 balls at start (any combination, duplicates allowed).
  · Total balls: 8.  No ball is ever destroyed or created.
  · Dispenser count NEVER exceeds 4.
  · Detector (tube) holds at most 1 ball at a time.

STRATEGY (Scan-First, Directed-Swap)
  Phase 1: Scan all 4 holder positions → build color map.
  Phase 2: Iterate; for each wrong position:
    Case A — needed color found in another holder spot:
              use detector as a 1-ball temp buffer, do holder↔holder swap.
    Case B — needed color in dispenser:
              push wrong ball to dispenser, pull correct ball out.
    Defer   — neither available yet (resolved by a subsequent iteration).
  Loop until all 4 positions match target.

================================================================================
*/

#include <Servo.h>

// ─── Color enum ───────────────────────────────────────────────────────────────
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

// ─── Pin definitions ──────────────────────────────────────────────────────────

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
const int PIN_DS   =  6;  // Data (DS / pin 14 of 595)
const int PIN_SHCP = 5;  // Shift clock (SH_CP / pin 11 of 595)  ← NOTE: same number as servo1 pin
const int PIN_STCP = 7;  // Latch clock (ST_CP / pin 12 of 595)

//     and update the constant above accordingly.

// Holder presence sensors (copper-mesh, INPUT_PULLUP)
const int SPOT_PINS[]   = {A1, A2, A3, A4};
const int NUM_SPOTS     = 4;
const int SPOT_THRESHOLD_OFFSET = 50; // drop from ambient that signals a ball

// ─── Tuning constants ─────────────────────────────────────────────────────────

// Ambient calibration
const int   CALIBRATION_MS    = 2000;
const int   CALIBRATION_READS = 50;
const float DROP_FRACTION     = 0.20;
const int   MIN_DROP          = 40;

// Color measurement
const int SETTLE_MS    = 200;
const int SAMPLE_COUNT = 32;
const int LED_PAUSE_MS = 300;
const int MIN_SIGNAL   = 60;   // total delta below this → UNKNOWN

// ─── 7-segment byte constants ─────────────────────────────────────────────────
// Bit mapping (verified empirically via diagnostic sketch):
//   bit 0 = dp, bit 1 = a, bit 2 = b, bit 3 = c,
//   bit 4 = d, bit 5 = e, bit 6 = f, bit 7 = g
const byte SEG_OFF     = 0b00000000;
const byte SEG_RED     = 0b10100000; // segments e, g
const byte SEG_GREEN   = 0b11011110; // segments a,b,c,d,f,g
const byte SEG_BLUE    = 0b11111000; // segments c,d,e,f,g
const byte SEG_YELLOW  = 0b11011100; // segments b,c,f,g
const byte SEG_UNKNOWN = 0b10100110; // segments a,b,e,g  (question mark)

// ─── System state ─────────────────────────────────────────────────────────────
Color holderState[NUM_SPOTS];       // what is currently in each holder spot
Color targetState[NUM_SPOTS];       // what should be in each holder spot

// Dispenser: we know its initial contents (R,G,B,Y) and track changes
Color dispenserContents[4] = {RED, GREEN, BLUE, YELLOW};
int   dispenserCount       = 4;

// Ambient calibration results
int ambientBaseline    = 0;
int detectionThreshold = 0;

// Spot ambient readings for presence detection
int spotAmbient[NUM_SPOTS];

// ─── Arm positions (servo angles) ────────────────────────────────────────────
// These are starting points from your existing sketches.
// Fine-tune per physical setup.

// Home / safe transit position
const int HOME_S1 = 0, HOME_S2 = 135, HOME_S3 = 170, HOME_S4 = 90;

// Dispenser pickup position
const int DISP_S1 = 180, DISP_S2 = 40, DISP_S3 = 180, DISP_S4 = 15;
// After gripping at dispenser, lift elbow:
const int DISP_LIFT_S2 = 50, DISP_LIFT_S1 = 120;

// Detector drop-off position (ball released into tube)
const int DET_DROP_S1 = 15, DET_DROP_S2 = 65, DET_DROP_S3 = 240, DET_DROP_S4 = 90;

// Detector pickup position (gripper descends into tube to retrieve ball)
const int DET_PICK_S4 = 15; // close claw; other joints stay at DET_DROP angles

// Holder spot positions — S1 angle per spot, other joints shared
// Adjust HOLDER_S1 values for your physical layout:
const int HOLDER_S1[NUM_SPOTS] = {45, 90, 135, 170};
const int HOLDER_S2_DOWN = 50;  // elbow down to reach holder level
const int HOLDER_S3_DOWN = 175;
const int HOLDER_S2_UP   = 135; // lift after pick/place
const int HOLDER_S3_UP   = 170;
const int HOLDER_S4_OPEN  = 90; // claw open (place / release)
const int HOLDER_S4_CLOSE = 15; // claw closed (pick)

const int ARM_SPEED_MS = 15; // ms per degree — from your existing code

// ─── Utility: slow servo move ─────────────────────────────────────────────────
void moveServoSlow(Servo &s, int target, int speedDelay = ARM_SPEED_MS) {
  int current = s.read();
  while (current != target) {
    current += (current < target) ? 1 : -1;
    s.write(current);
    delay(speedDelay);
  }
}

// ─── 7-segment display ────────────────────────────────────────────────────────
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

// ─── LED helpers ──────────────────────────────────────────────────────────────
void allLedsOff() {
  for (int i = 0; i < LED_COUNT; i++) digitalWrite(LED_PINS[i], LOW);
}

int averagedRead(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) { sum += analogRead(pin); delay(5); }
  return (int)(sum / samples);
}

// ─── Ambient calibration ──────────────────────────────────────────────────────
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
  int margin = max((int)(ambientBaseline * DROP_FRACTION), MIN_DROP);
  detectionThreshold = ambientBaseline - margin;
  Serial.print("Baseline: ");    Serial.print(ambientBaseline);
  Serial.print("  Threshold: "); Serial.println(detectionThreshold);
}

// ─── Color detection ──────────────────────────────────────────────────────────
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

Color classifyColor(int deltas[]) {
  int r = deltas[0], g = deltas[1], b = deltas[2];
  int total = r + g + b;
  if (total < MIN_SIGNAL * 3) return UNKNOWN;

  float rN = (float)r / total;
  float gN = (float)g / total;
  float bN = (float)b / total;

  Serial.print("  rN="); Serial.print(rN);
  Serial.print(" gN="); Serial.print(gN);
  Serial.print(" bN="); Serial.println(bN);

  // Yellow: red and green both present, blue suppressed
  if (rN > 0.33 && gN >= 0.33 && bN < 0.20 && abs(gN - rN) < 0.10) return YELLOW;
  if (r > g && r > b) return RED;
  if (g > r && g > b) return GREEN;
  return BLUE;
}

// Wait for ball to be placed in the detector tube, then classify it.
// Returns the detected color. Times out after ~5 s and returns UNKNOWN.
Color detectBallInTube() {
  Serial.println("  Waiting for ball in detector...");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (averagedRead(PIN_PHOTO, SAMPLE_COUNT) < detectionThreshold) break;
    delay(50);
  }
  int current = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
  if (current >= detectionThreshold) {
    Serial.println("  Timeout — no ball detected in tube.");
    return UNKNOWN;
  }
  Serial.println("  Ball detected. Measuring...");
  int deltas[LED_COUNT];
  for (int i = 0; i < LED_COUNT; i++) {
    deltas[i] = measureReflection(LED_PINS[i]);
    Serial.print("  ");
    Serial.print(i == 0 ? "R" : i == 1 ? "G" : "B");
    Serial.print(" delta: "); Serial.println(deltas[i]);
  }
  Color c = classifyColor(deltas);
  Serial.print("  >>> "); Serial.println(colorName(c));
  displayColor(c);
  return c;
}

// ─── Holder presence detection ────────────────────────────────────────────────
void calibrateSpotSensors() {
  Serial.println("Calibrating holder spot sensors...");
  for (int i = 0; i < NUM_SPOTS; i++) {
    spotAmbient[i] = analogRead(SPOT_PINS[i]);
    Serial.print("  Spot "); Serial.print(i + 1);
    Serial.print(" ambient: "); Serial.println(spotAmbient[i]);
  }
}

bool isBallAtSpot(int spotIndex) {
  int val = analogRead(SPOT_PINS[spotIndex]);
  return val < (spotAmbient[spotIndex] - SPOT_THRESHOLD_OFFSET);
}

// ─── Serial input: read target sequence ───────────────────────────────────────
Color charToColor(char c) {
  switch (c) {
    case 'R': case 'r': return RED;
    case 'G': case 'g': return GREEN;
    case 'B': case 'b': return BLUE;
    case 'Y': case 'y': return YELLOW;
    default: return UNKNOWN;
  }
}

void readTargetFromSerial() {
  Serial.println("READY: send target sequence as 4 chars (R/G/B/Y), e.g. RGBY");
  int received = 0;
  while (received < NUM_SPOTS) {
    if (Serial.available()) {
      char c = Serial.read();
      Color col = charToColor(c);
      if (col != UNKNOWN) {
        targetState[received] = col;
        Serial.print("  Spot "); Serial.print(received + 1);
        Serial.print(" → "); Serial.println(colorName(col));
        received++;
      }
    }
  }
  Serial.println("Target loaded.");
}

// ─── Arm motion primitives ────────────────────────────────────────────────────
void goHome() {
  moveServoSlow(servo3, HOME_S3);
  moveServoSlow(servo2, HOME_S2);
  moveServoSlow(servo1, HOME_S1);
  moveServoSlow(servo4, HOME_S4);
}

// Pick ball from dispenser (claw grips, arm lifts to transit)
void pickFromDispenser() {
  Serial.println("[ARM] Pick from dispenser");
  moveServoSlow(servo1, DISP_S1);
  delay(300);
  moveServoSlow(servo2, DISP_S2);
  moveServoSlow(servo4, DISP_S4); // open claw before descend — already closed, re-open briefly
  delay(200);
  moveServoSlow(servo3, DISP_S3);
  moveServoSlow(servo2, DISP_LIFT_S2);
  delay(300);
  moveServoSlow(servo4, DISP_S4); // close claw (grip)
  delay(300);
  moveServoSlow(servo1, DISP_LIFT_S1);
  delay(300);
  // Lift to transit
  moveServoSlow(servo3, HOME_S3);
  moveServoSlow(servo2, HOME_S2);
  dispenserCount--;
}

// Place ball into dispenser (claw releases)
void placeToDispenser(Color c) {
  Serial.print("[ARM] Place to dispenser: "); Serial.println(colorName(c));
  moveServoSlow(servo1, DISP_S1);
  delay(300);
  moveServoSlow(servo2, DISP_LIFT_S2);
  moveServoSlow(servo3, DISP_S3);
  delay(300);
  moveServoSlow(servo4, HOLDER_S4_OPEN); // release
  delay(300);
  moveServoSlow(servo3, HOME_S3);
  moveServoSlow(servo2, HOME_S2);
  dispenserContents[dispenserCount] = c;
  dispenserCount++;
}

// Drop ball into detector tube (claw opens mid-air above tube)
void dropToDetector() {
  Serial.println("[ARM] Drop to detector");
  moveServoSlow(servo1, DET_DROP_S1);
  moveServoSlow(servo2, DET_DROP_S2);
  moveServoSlow(servo3, DET_DROP_S3);
  delay(300);
  moveServoSlow(servo4, HOLDER_S4_OPEN); // release
  delay(400);
  moveServoSlow(servo3, HOME_S3);
  moveServoSlow(servo2, HOME_S2);
}

// Retrieve ball from detector tube (claw descends, grips, lifts)
void pickFromDetector() {
  Serial.println("[ARM] Pick from detector");
  moveServoSlow(servo1, DET_DROP_S1);
  moveServoSlow(servo2, DET_DROP_S2);
  moveServoSlow(servo3, DET_DROP_S3);
  delay(200);
  moveServoSlow(servo4, DET_PICK_S4); // close claw
  delay(300);
  moveServoSlow(servo3, HOME_S3);
  moveServoSlow(servo2, HOME_S2);
}

// Pick ball from holder position i
void pickFromHolder(int i) {
  Serial.print("[ARM] Pick from holder spot "); Serial.println(i + 1);
  moveServoSlow(servo1, HOLDER_S1[i]);
  delay(200);
  moveServoSlow(servo2, HOLDER_S2_DOWN);
  moveServoSlow(servo3, HOLDER_S3_DOWN);
  delay(200);
  moveServoSlow(servo4, HOLDER_S4_CLOSE);
  delay(300);
  moveServoSlow(servo3, HOLDER_S3_UP);
  moveServoSlow(servo2, HOLDER_S2_UP);
}

// Place ball to holder position i (claw opens)
void placeToHolder(int i) {
  Serial.print("[ARM] Place to holder spot "); Serial.println(i + 1);
  moveServoSlow(servo1, HOLDER_S1[i]);
  delay(200);
  moveServoSlow(servo2, HOLDER_S2_DOWN);
  moveServoSlow(servo3, HOLDER_S3_DOWN);
  delay(200);
  moveServoSlow(servo4, HOLDER_S4_OPEN);
  delay(300);
  moveServoSlow(servo3, HOLDER_S3_UP);
  moveServoSlow(servo2, HOLDER_S2_UP);
}

// ─── Scan a single holder spot via detector tube ──────────────────────────────
Color scanHolderSpot(int i) {
  Serial.print("[SCAN] Spot "); Serial.println(i + 1);
  pickFromHolder(i);
  dropToDetector();
  Color c = detectBallInTube();
  pickFromDetector();
  placeToHolder(i);      // return ball to same spot
  holderState[i] = c;
  Serial.print("  Spot "); Serial.print(i + 1);
  Serial.print(" = "); Serial.println(colorName(c));
  return c;
}

// ─── Phase 1: Scan all 4 holder positions ────────────────────────────────────
void scanAllHolderSpots() {
  Serial.println("\n=== PHASE 1: Scanning holder ===");
  for (int i = 0; i < NUM_SPOTS; i++) {
    holderState[i] = UNKNOWN;
    scanHolderSpot(i);
    displayColor(holderState[i]);
    delay(500);
  }
  Serial.println("Scan complete. Holder state:");
  for (int i = 0; i < NUM_SPOTS; i++) {
    Serial.print("  Spot "); Serial.print(i + 1);
    Serial.print(": "); Serial.print(colorName(holderState[i]));
    Serial.print("  (target: "); Serial.print(colorName(targetState[i])); Serial.println(")");
  }
}

// ─── State helpers ────────────────────────────────────────────────────────────

// Return index of the first holder spot containing color c (other than 'exclude'),
// or -1 if not found.
int findInHolder(Color c, int exclude = -1) {
  for (int i = 0; i < NUM_SPOTS; i++) {
    if (i == exclude) continue;
    if (holderState[i] == c) return i;
  }
  return -1;
}

// Return index within dispenserContents[] of color c, or -1.
int findInDispenser(Color c) {
  for (int i = 0; i < dispenserCount; i++) {
    if (dispenserContents[i] == c) return i;
  }
  return -1;
}

// Remove one entry from dispenserContents[] by index.
void removeFromDispenserByIndex(int idx) {
  for (int i = idx; i < dispenserCount - 1; i++) {
    dispenserContents[i] = dispenserContents[i + 1];
  }
  dispenserCount--;
}

bool allCorrect() {
  for (int i = 0; i < NUM_SPOTS; i++) {
    if (holderState[i] != targetState[i]) return false;
  }
  return true;
}

// ─── Phase 2: Resolve all incorrect positions ─────────────────────────────────
//
// Each call to resolveAll() iterates over positions and tries to fix them.
// It restarts from position 0 after any state change so updated state is
// reflected immediately.  The outer while-loop guarantees termination:
//   - Each iteration either places at least one correct ball (making progress)
//     or defers (because the needed ball isn't yet accessible).
//   - Deferral can only happen when a Case-A swap in this or a previous
//     iteration will eventually free the needed color.
// ─────────────────────────────────────────────────────────────────────────────
void resolveAll() {
  Serial.println("\n=== PHASE 2: Resolving positions ===");
  int maxPasses = 20; // safety guard against infinite loop on sensor error
  int pass = 0;

  while (!allCorrect() && pass < maxPasses) {
    pass++;
    bool madeProgress = false;

    for (int i = 0; i < NUM_SPOTS; i++) {
      if (holderState[i] == targetState[i]) continue; // already correct

      Color needed = targetState[i];
      Serial.print("\n[Resolve] Spot "); Serial.print(i + 1);
      Serial.print(" has "); Serial.print(colorName(holderState[i]));
      Serial.print(", needs "); Serial.println(colorName(needed));

      // ── Case A: needed color is in another holder spot ──────────────────
      int srcSpot = findInHolder(needed, i);
      if (srcSpot != -1) {
        Serial.print("  Case A: swap with holder spot "); Serial.println(srcSpot + 1);

        // Step 1: move holder[i] (wrong ball) into detector as temp buffer
        pickFromHolder(i);
        dropToDetector();                    // detector now holds wrong ball
        Color wrongBall = holderState[i];
        holderState[i] = UNKNOWN;

        // Step 2: move holder[srcSpot] (correct ball) to position i
        pickFromHolder(srcSpot);
        placeToHolder(i);
        holderState[i]       = needed;
        holderState[srcSpot] = UNKNOWN;

        // Step 3: retrieve wrong ball from detector, place to srcSpot
        pickFromDetector();
        placeToHolder(srcSpot);
        holderState[srcSpot] = wrongBall;

        madeProgress = true;
        break; // restart outer loop after any state change
      }

      // ── Case B: needed color is in the dispenser ─────────────────────────
      int dispIdx = findInDispenser(needed);
      if (dispIdx != -1) {
        if (dispenserCount >= 4) {
          // Dispenser is full — can't push wrong ball in.
          // This should not happen with correct sequencing but guard anyway.
          Serial.println("  Case B skipped: dispenser full. Will defer.");
          continue;
        }

        Serial.println("  Case B: dispenser swap");

        // Push wrong ball to dispenser
        Color wrongBall = holderState[i];
        pickFromHolder(i);
        placeToDispenser(wrongBall);         // dispenserCount incremented inside
        holderState[i] = UNKNOWN;

        // Pull correct ball from dispenser — re-find index since count changed
        dispIdx = findInDispenser(needed);   // re-locate after push
        // To pull from dispenser we need to pick it. Since dispenser is a
        // physical tube the arm always picks the top/accessible ball.
        // We assume the ball we just pushed goes on top and that the ball
        // we want is at dispIdx.  If your dispenser is LIFO, push order
        // matters — this code tracks contents but not physical ordering.
        // Adjust pickFromDispenser() if needed for your physical layout.
        pickFromDispenser();                 // dispenserCount decremented inside
        placeToHolder(i);
        holderState[i] = needed;

        // Update dispenser contents tracking
        // pickFromDispenser takes the last accessible ball; we approximate
        // here that it took the one we wanted.  If physical order differs,
        // add a scanDetector step after pickup to verify.
        removeFromDispenserByIndex(dispIdx);

        madeProgress = true;
        break;
      }

      // ── Defer: needed ball not in holder or dispenser yet ────────────────
      Serial.println("  Deferring — needed color not yet accessible.");
    }

    if (!madeProgress) {
      Serial.println("No progress this pass — re-scanning holder to update state.");
      // Re-scan to recover from any detection mismatch
      for (int i = 0; i < NUM_SPOTS; i++) {
        if (holderState[i] != targetState[i]) scanHolderSpot(i);
      }
    }
  }
}

// ─── Final verification ───────────────────────────────────────────────────────
void verifyAndReport() {
  Serial.println("\n=== VERIFICATION ===");
  bool ok = true;
  for (int i = 0; i < NUM_SPOTS; i++) {
    // Re-scan for confirmation
    scanHolderSpot(i);
    bool match = (holderState[i] == targetState[i]);
    Serial.print("  Spot "); Serial.print(i + 1);
    Serial.print(": "); Serial.print(colorName(holderState[i]));
    Serial.print(match ? "  OK" : "  MISMATCH (expected ");
    if (!match) { Serial.print(colorName(targetState[i])); Serial.print(")"); }
    Serial.println();
    if (!match) ok = false;
  }
  if (ok) {
    Serial.println("\nSorting complete. All positions correct.");
    showSegment(SEG_GREEN); // flash green to signal success
  } else {
    Serial.println("\nWARNING: Some positions still incorrect. Check sensors / tuning.");
    showSegment(SEG_UNKNOWN);
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Servo setup
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);
  servo4.attach(SERVO_PIN4);

  // Initial safe positions (from RoboticarmAutomatedpickupcodev5.ino)
  servo4.write(HOME_S4);
  servo3.write(HOME_S3);
  servo2.write(HOME_S2);
  servo1.write(HOME_S1);

  // LED + photo + display pin modes
  for (int i = 0; i < LED_COUNT; i++) pinMode(LED_PINS[i], OUTPUT);
  pinMode(PIN_PHOTO, INPUT);
  pinMode(PIN_DS,    OUTPUT);
  pinMode(PIN_SHCP,  OUTPUT);
  pinMode(PIN_STCP,  OUTPUT);

  // Holder spot sensors
  for (int i = 0; i < NUM_SPOTS; i++) {
    pinMode(SPOT_PINS[i], INPUT_PULLUP);
  }
  delay(100);

  allLedsOff();
  showSegment(SEG_UNKNOWN);

  Serial.println("======================================");
  Serial.println("  BALL SORTING SYSTEM — initialising");
  Serial.println("======================================");

  // Calibrate photoresistor
  calibrateAmbient();

  // Calibrate holder spot sensors
  calibrateSpotSensors();

  // Get target from operator
  readTargetFromSerial();

  // ── Phase 1 ──────────────────────────────────────────────────────────────
  scanAllHolderSpots();

  // ── Phase 2 ──────────────────────────────────────────────────────────────
  resolveAll();

  // ── Verify ───────────────────────────────────────────────────────────────
  verifyAndReport();

  goHome();
  Serial.println("System idle.");
}

// loop() is intentionally empty — task runs once in setup(), then idles.
void loop() {
  delay(1000);
}
