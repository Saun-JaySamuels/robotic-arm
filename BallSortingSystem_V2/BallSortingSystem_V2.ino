/*
================================================================================
  ROBOTIC BALL SORTING SYSTEM — Full Integration (rev: per-spot + lift-then-move)
================================================================================

CHANGES IN THIS REVISION
  · Holder spots are now a 2x2 grid:
            [1] [2]   ← near row (closer to arm base)
            [3] [4]   ← far  row (further from arm base)
      Each spot has its own (S1, S2, S3) target so the elbow/shoulder reach
      adapts per row. Tune these per-spot constants on the hardware.

  · Pick/place motion is now strictly:
            (1) ascend to TRANSIT pose with S2 + S3 only (vertical lift)
            (2) rotate base S1 to next target
            (3) descend to that spot with S2 + S3 only (vertical drop)
        This avoids dragging the ball sideways through other spots.

  · Dispenser swap sequence reordered so the dispenser tube is never
    overfilled (max 4 balls at a time):
            1. pick wrong ball from holder
            2. drop wrong ball into detector tube (parking)
            3. pick needed color from dispenser front
            4. place needed color into the holder spot
            5. pick wrong ball back out of detector
            6. place wrong ball into dispenser back
            7. (optional) re-scan new ball in holder via detector

================================================================================

HARDWARE SUMMARY
  Servo 1  (pin 10) — base rotation
  Servo 2  (pin  9) — shoulder
  Servo 3  (pin 12) — elbow
  Servo 4  (pin 13) — claw (90 = open, 15 = closed)

  Photoresistor       → A5 (voltage divider, 10 kΩ pull-down)
  RGB LEDs            → pins 2 (Red), 3 (Green), 4 (Blue)

  74HC595 shift register driving common-cathode 7-segment display:
    DS   (pin 14 of 595) → Arduino pin  6
    SH_CP(pin 11 of 595) → Arduino pin  5
    ST_CP(pin 12 of 595) → Arduino pin  7
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
const int PIN_DS   =  6;
const int PIN_SHCP =  5;
const int PIN_STCP =  7;

// Holder presence sensors (copper-mesh, INPUT_PULLUP)
const int SPOT_PINS[]   = {A1, A2, A3, A4};
const int NUM_SPOTS     = 4;
const int SPOT_THRESHOLD_OFFSET = 50;

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
const int MIN_SIGNAL   = 60;

// ─── 7-segment byte constants ─────────────────────────────────────────────────
// Bit mapping: bit 0=dp, 1=a, 2=b, 3=c, 4=d, 5=e, 6=f, 7=g
const byte SEG_OFF     = 0b00000000;
const byte SEG_RED     = 0b10100000;
const byte SEG_GREEN   = 0b11011110;
const byte SEG_BLUE    = 0b11111000;
const byte SEG_YELLOW  = 0b11011100;
const byte SEG_UNKNOWN = 0b10100110;

// ─── System state ─────────────────────────────────────────────────────────────
Color holderState[NUM_SPOTS];
Color targetState[NUM_SPOTS];

// ─── Dispenser FIFO queue ─────────────────────────────────────────────────────
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
    Serial.print("Dispenser [");
    for (int i = 0; i < count; i++) {
      switch (slots[i]) {
        case RED:    Serial.print("R"); break;
        case GREEN:  Serial.print("G"); break;
        case BLUE:   Serial.print("B"); break;
        case YELLOW: Serial.print("Y"); break;
        default:     Serial.print("?"); break;
      }
      if (i < count - 1) Serial.print(",");
    }
    Serial.print("] front→back  count="); Serial.println(count);
  }
} dispenser;

// Ambient calibration results
int ambientBaseline    = 0;
int detectionThreshold = 0;
int spotAmbient[NUM_SPOTS];

// ═══ ARM POSITIONS ════════════════════════════════════════════════════════════
//
// All arm motion uses the convention:
//   TRANSIT pose = arm tucked high, safe to rotate base.
//   "Down" pose  = elbow/shoulder extended to reach target.
//
// To pick or place anywhere, the arm always:
//   1. lifts (S3 up, then S2 up) to TRANSIT
//   2. rotates base S1 to target
//   3. descends (S2 down, then S3 down) to target
//
// Tune the per-spot constants below by hand on the real hardware.

// ── Claw ──
const int CLAW_OPEN  = 90;
const int CLAW_CLOSE = 15;

// ── TRANSIT pose (high, base-rotation safe) ──
const int TRANSIT_S2 = 135;
const int TRANSIT_S3 = 170;

// ── HOME pose (idle) ──
const int HOME_S1 = 0;
const int HOME_S2 = TRANSIT_S2;
const int HOME_S3 = TRANSIT_S3;
const int HOME_S4 = CLAW_OPEN;

// ── Holder spots — per-spot S1, S2-down, S3-down ──
//
// Layout (looking down):
//      arm base
//         |
//      [1] [2]   ← near row
//      [3] [4]   ← far  row
//
// Spot 1 is the known-good reference: S1=85, S2_DOWN=50, S3_DOWN=175.
// Other spots are starting estimates — you will need to tune these.
// As a starting point: spot 2 mirrors spot 1's reach but with smaller S1
// (rotation toward right side), and far row uses lower S2 + higher S3 for
// extra reach.
const int HOLDER_S1[NUM_SPOTS]      = {  85,  70,  85,  70 };
const int HOLDER_S2_DOWN[NUM_SPOTS] = {  50,  50,  60,  60 };
const int HOLDER_S3_DOWN[NUM_SPOTS] = { 175, 175, 195, 200 };

// ── Dispenser pickup / drop-off ──
// Reach to dispenser front (FIFO eject) and dispenser back (push deepest).
// Same base S1 for both — just different elbow/shoulder for front vs back.
const int DISP_S1            = 180;
const int DISP_PICK_S2_DOWN  =  40;
const int DISP_PICK_S3_DOWN  = 180;
const int DISP_PUSH_S2_DOWN  =  50;   // back of tube — slightly different geometry
const int DISP_PUSH_S3_DOWN  = 180;

// ── Detector tube — drop a ball in, or pick a ball back out ──
const int DET_S1       =  15;
const int DET_S2_DOWN  =  65;
const int DET_S3_DOWN  = 240;

// ── Motion speed ──
const int ARM_SPEED_MS = 15; // ms per degree

// ─── Utility: slow servo move ─────────────────────────────────────────────────
void moveServoSlow(Servo &s, int target, int speedDelay = ARM_SPEED_MS) {
  int current = s.read();
  while (current != target) {
    current += (current < target) ? 1 : -1;
    s.write(current);
    delay(speedDelay);
  }
}

// ─── Composite arm motion primitives ──────────────────────────────────────────
//
// liftToTransit():
//   Bring the arm to TRANSIT pose (high) WITHOUT moving the base.
//   Order: elbow first (S3 up), then shoulder (S2 up), so the gripper
//   tucks in before rising — avoids dragging the ball sideways.
void liftToTransit() {
  moveServoSlow(servo2, TRANSIT_S2);
  delay(300);
  moveServoSlow(servo3, TRANSIT_S3);
  
}

// rotateTo(s1):
//   Rotate the base. Caller is responsible for being at TRANSIT pose first.
void rotateTo(int s1) {
  moveServoSlow(servo1, s1);
}

// descendTo(s2_down, s3_down):
//   From TRANSIT, lower the arm to a pick/place pose. Shoulder first,
//   then elbow — so the gripper drops vertically over the target.
void descendTo(int s2_down, int s3_down) {
  moveServoSlow(servo2, s2_down);
  moveServoSlow(servo3, s3_down);
}

// goHome():
//   Lift to TRANSIT, rotate base back to 0, leave arm idle.
void goHome() {
  liftToTransit();
  rotateTo(HOME_S1);
  // claw stays in whatever state caller left it
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

  if (rN > 0.33 && gN >= 0.33 && bN < 0.20 && abs(gN - rN) < 0.10) return YELLOW;
  if (r > g && r > b) return RED;
  if (g > r && g > b) return GREEN;
  return BLUE;
}

// Wait for ball to be placed in the detector tube, then classify it.
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

// ═══ HIGH-LEVEL ARM ACTIONS ═══════════════════════════════════════════════════
//
// Each action assumes the arm STARTS at TRANSIT pose (S2/S3 high) and
// LEAVES it at TRANSIT pose. This is the key invariant that lets us chain
// actions without horizontal collisions.
//
// Claw state is left as-is on exit:
//   pickFrom*  → claw closed (holding a ball)
//   placeTo*   → claw open  (just released)

// Pick the front ball from the dispenser (FIFO eject) and grip it.
void pickFromDispenser() {
  Serial.print("[ARM] Pick from dispenser front: ");
  Serial.println(colorName(dispenser.peekFront()));
  rotateTo(DISP_S1);
  moveServoSlow(servo4, CLAW_OPEN);          // ensure claw open before descend
  descendTo(DISP_PICK_S2_DOWN, DISP_PICK_S3_DOWN);
  delay(200);
  moveServoSlow(servo4, CLAW_CLOSE);         // grip
  delay(300);
  liftToTransit();
  dispenser.popFront();
}

// Place the ball currently in the claw into the back of the dispenser tube.
void placeToDispenser(Color c) {
  Serial.print("[ARM] Place to dispenser back: "); Serial.println(colorName(c));
  rotateTo(DISP_S1);
  descendTo(DISP_PUSH_S2_DOWN, DISP_PUSH_S3_DOWN);
  delay(300);
  moveServoSlow(servo4, CLAW_OPEN);          // release
  delay(300);
  liftToTransit();
  dispenser.pushBack(c);
}

// Drop ball into detector tube
void dropToDetector() {
  Serial.println("[ARM] Drop to detector");
  rotateTo(DET_S1);
  descendTo(DET_S2_DOWN, DET_S3_DOWN);
  delay(300);
  moveServoSlow(servo4, CLAW_OPEN);          // release
  delay(400);
  liftToTransit();
}

// Retrieve ball from detector tube
void pickFromDetector() {
  Serial.println("[ARM] Pick from detector");
  rotateTo(DET_S1);
  moveServoSlow(servo4, CLAW_OPEN);          // ensure claw open before descend
  descendTo(DET_S2_DOWN, DET_S3_DOWN);
  delay(200);
  moveServoSlow(servo4, CLAW_CLOSE);         // grip
  delay(300);
  liftToTransit();
}

// Pick ball from holder position i
void pickFromHolder(int i) {
  Serial.print("[ARM] Pick from holder spot "); Serial.println(i + 1);
  rotateTo(HOLDER_S1[i]);
  moveServoSlow(servo4, CLAW_OPEN);          // ensure claw open before descend
  descendTo(HOLDER_S2_DOWN[i], HOLDER_S3_DOWN[i]);
  delay(200);
  moveServoSlow(servo4, CLAW_CLOSE);         // grip
  delay(300);
  liftToTransit();
}

// Place ball to holder position i
void placeToHolder(int i) {
  Serial.print("[ARM] Place to holder spot "); Serial.println(i + 1);
  rotateTo(HOLDER_S1[i]);
  descendTo(HOLDER_S2_DOWN[i], HOLDER_S3_DOWN[i]);
  delay(200);
  moveServoSlow(servo4, CLAW_OPEN);          // release
  delay(300);
  liftToTransit();
}

// ─── Scan a single holder spot via detector tube ──────────────────────────────
Color scanHolderSpot(int i) {
  Serial.print("[SCAN] Spot "); Serial.println(i + 1);
  pickFromHolder(i);
  dropToDetector();
  Color c = detectBallInTube();
  pickFromDetector();
  placeToHolder(i);
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

// ─── Dispenser swap (Case B) — single helper ──────────────────────────────────
//
// Sequence (verified safe re: 4-ball dispenser cap):
//   PRE: dispenser has wantedColor at front (caller cycles first if needed).
//        Holder spot `i` has wrong color, claw is empty, arm at TRANSIT.
//
//   1. pick wrong ball from holder[i]                 (claw: wrong)
//   2. drop wrong ball into detector tube             (claw: empty, dispenser: 4)
//   3. pick wanted ball from dispenser front          (claw: wanted, dispenser: 3)
//   4. place wanted ball into holder[i]               (claw: empty, holder[i] updated)
//   5. pick wrong ball back out of detector           (claw: wrong)
//   6. place wrong ball into dispenser back           (claw: empty, dispenser: 4)
//   7. (verify) re-scan holder[i] via detector        (optional)
//
//   POST: dispenser still has 4 balls. Holder[i] holds wanted color.
//
// At step 2 the dispenser still has 4 balls AND the detector has 1 — this is
// fine because the detector is a separate buffer. The dispenser count never
// exceeds 4 because we always pop (step 3) before we push (step 6).
//
void doDispenserSwap(int i, Color wantedColor, bool verifyAfter) {
  Serial.print("  [SWAP] holder["); Serial.print(i + 1);
  Serial.print("] wrong → detector, dispenser → holder, wrong → dispenser back");
  if (verifyAfter) Serial.println(", then verify"); else Serial.println();

  Color wrongBall = holderState[i];

  // 1. pick wrong from holder
  pickFromHolder(i);
  holderState[i] = UNKNOWN;

  // 2. drop wrong into detector
  dropToDetector();

  // 3. pick wanted from dispenser front
  pickFromDispenser();

  // 4. place wanted into holder[i]
  placeToHolder(i);
  holderState[i] = wantedColor;

  // 5. pick wrong back from detector
  pickFromDetector();

  // 6. place wrong into dispenser back
  placeToDispenser(wrongBall);

  // 7. optional: scan the newly-placed ball to confirm color
  if (verifyAfter) {
    Color seen = scanHolderSpot(i);
    if (seen != wantedColor) {
      Serial.print("  [SWAP] WARNING: post-swap scan saw ");
      Serial.print(colorName(seen));
      Serial.print(", expected ");
      Serial.println(colorName(wantedColor));
      // Trust the scan — sensor mismatch should be reflected in state.
      holderState[i] = seen;
    }
  }
}

// ─── Phase 2: Resolve all incorrect positions ─────────────────────────────────
void resolveAll() {
  Serial.println("\n=== PHASE 2: Resolving positions ===");
  int maxPasses = 20;
  int pass = 0;

  while (!allCorrect() && pass < maxPasses) {
    pass++;
    bool madeProgress = false;

    for (int i = 0; i < NUM_SPOTS; i++) {
      if (holderState[i] == targetState[i]) continue;

      Color needed = targetState[i];
      Serial.print("\n[Resolve] Spot "); Serial.print(i + 1);
      Serial.print(" has "); Serial.print(colorName(holderState[i]));
      Serial.print(", needs "); Serial.println(colorName(needed));

      // ── Case A: needed color is in another holder spot ──────────────────
      int srcSpot = findInHolder(needed, i);
      if (srcSpot != -1) {
        Serial.print("  Case A: swap with holder spot "); Serial.println(srcSpot + 1);

        Color wrongBall = holderState[i];

        // Step 1: park wrong ball in detector
        pickFromHolder(i);
        dropToDetector();
        holderState[i] = UNKNOWN;

        // Step 2: move correct ball from srcSpot into spot i
        pickFromHolder(srcSpot);
        placeToHolder(i);
        holderState[i]       = needed;
        holderState[srcSpot] = UNKNOWN;

        // Step 3: retrieve wrong ball from detector, place to srcSpot
        pickFromDetector();
        placeToHolder(srcSpot);
        holderState[srcSpot] = wrongBall;

        madeProgress = true;
        break;
      }

      // ── Case B: needed color is in the dispenser ─────────────────────────
      if (dispenser.contains(needed)) {
        Serial.print("  Case B: dispenser swap. Queue: "); dispenser.printState();

        int neededPos = dispenser.indexOf(needed);
        Serial.print("  Needed color at dispenser index "); Serial.print(neededPos);
        Serial.println(" (0=front)");

        // ── If needed isn't at the front, cycle it forward.
        // FIFO cycling rule: pop front → push back.
        // We must avoid both "dispenser >4" and "detector >1 ball" violations.
        // Strategy:
        //   For each position the needed ball is buried, do a one-ball cycle:
        //     pickFromDispenser (count 4→3, claw holds front ball)
        //     dropToDetector    (claw empty, detector holds it)
        //     pickFromDetector  (claw holds it again)
        //     placeToDispenser  (count 3→4, ball now at back)
        //
        // After neededPos cycles, the originally-buried ball is at front.
        for (int cyc = 0; cyc < neededPos; cyc++) {
          Color frontBall = dispenser.peekFront();
          Serial.print("  Cycle "); Serial.print(cyc + 1);
          Serial.print(": re-queuing "); Serial.println(colorName(frontBall));

          pickFromDispenser();         // 4 → 3
          dropToDetector();            // park in detector
          pickFromDetector();          // back into claw
          placeToDispenser(frontBall); // 3 → 4
        }

        Serial.print("  Cycle complete. Queue: "); dispenser.printState();

        // ── Now run the actual swap with the new sequence (verify after).
        doDispenserSwap(i, needed, /*verifyAfter=*/true);

        madeProgress = true;
        break;
      }

      // ── Defer ────────────────────────────────────────────────────────────
      Serial.println("  Deferring — needed color not yet accessible.");
    }

    if (!madeProgress) {
      Serial.println("No progress this pass — re-scanning holder to update state.");
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
    showSegment(SEG_GREEN);
  } else {
    Serial.println("\nWARNING: Some positions still incorrect. Check sensors / tuning.");
    showSegment(SEG_UNKNOWN);
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);
  servo3.attach(SERVO_PIN3);
  servo4.attach(SERVO_PIN4);

  // Initial safe positions — write directly so .read() returns sane values.
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

  Serial.println("======================================");
  Serial.println("  BALL SORTING SYSTEM — initialising");
  Serial.println("======================================");

  dispenser.init(RED, GREEN, BLUE, YELLOW);

  calibrateAmbient();
  calibrateSpotSensors();
  readTargetFromSerial();

  scanAllHolderSpots();
  resolveAll();
  verifyAndReport();

  goHome();
  Serial.println("System idle.");
}

void loop() {
  delay(1000);
}