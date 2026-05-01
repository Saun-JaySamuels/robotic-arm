#include <Servo.h>

/*
Ball Color Detector — Auto-Calibrating + 7-Segment Output via 595
───────────────────────────────────────────────────────────────────
Pin 2  → Red LED (OUTPUT)
Pin 3  → Green LED (OUTPUT)
Pin 4  → Blue LED (OUTPUT)
Pin A5 → Photoresistor (voltage divider to GND, 10kΩ pull-down)

595 Shift Register (7-segment display, common cathode):
  DS   (pin 14) → Arduino pin 6
  SH_CP(pin 11) → Arduino pin 5
  ST_CP(pin 12) → Arduino pin 7
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

const int LED_PINS[]     = {2, 3, 4};
const String LED_NAMES[] = {"Red", "Green", "Blue"};
const int LED_COUNT      = 3;

// 595 pins
const int PIN_DS    = 6;   // Data
const int PIN_SHCP  = 5;  // Shift clock
const int PIN_STCP  = 7;  // Latch clock


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
const byte SEG_YELLOW  = 0b11011100; // b,c,f,g      → bits 2,3,6,7
const byte SEG_UNKNOWN = 0b10100110; // a,b,e,g      → bits 1,2,5,7

// ── Globals ────────────────────────────────────────────────────────────────
int ambientBaseline    = 0;
int detectionThreshold = 0;
bool ballPresent       = false;

// Servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Servo pins
const int servoPin1 = 10;
const int servoPin2 = 9;
const int servoPin3 = 12;
const int servoPin4 = 13;

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

  // Yellow: blue suppressed, green and red close together
  if (rN > 0.33 && gN >= 0.33 && bN < 0.20 && abs((gN - rN) < 0.05)) return "YELLOW";

  if (r > g && r > b) return "RED";
  if (g > r && g > b) return "GREEN";
  return "BLUE";
}

void displayColor(String color) {
  if      (color == "RED")    showSegment(SEG_RED);
  else if (color == "GREEN")  showSegment(SEG_GREEN);
  else if (color == "BLUE")   showSegment(SEG_BLUE);
  else if (color == "YELLOW") showSegment(SEG_YELLOW);
  else                        showSegment(SEG_UNKNOWN);
}

void setup() {
  Serial.begin(9600);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  // Initial positions
  servo4.write(90);
  servo3.write(170);
  servo2.write(135);
  servo1.write(140);

  // pinMode setup (was previously outside the function)
  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  pinMode(PIN_PHOTO, INPUT);
  pinMode(PIN_DS,    OUTPUT);
  pinMode(PIN_SHCP,  OUTPUT);
  pinMode(PIN_STCP,  OUTPUT);

  allLedsOff();
  showSegment(SEG_UNKNOWN);

  Serial.println("Ball Color Detector — Auto-Calibrating");
  Serial.println("=======================================");
  calibrateAmbient();
}

void loop() {

  // Initial pickup sequence
  delay(500);
  moveServoSlow(servo1, 178, 15);
  delay(500);
  moveServoSlow(servo2, 42, 15);
  moveServoSlow(servo4, 15, 15);
  delay(500);
  moveServoSlow(servo3, 180, 15);
  moveServoSlow(servo2, 50, 15);
  delay(500);
  moveServoSlow(servo1, 120, 15);
  delay(500);

  // Move to color detector
  moveServoSlow(servo3, 170, 15);
  moveServoSlow(servo2, 135, 15);
  delay(500);
  moveServoSlow(servo1, 15, 15);
  moveServoSlow(servo2, 65, 15);
  moveServoSlow(servo3, 240, 15);
  delay(500);
  moveServoSlow(servo4, 90, 15);  // release ball into detector
  delay(500);

  // ── Wait here until ball is detected ─────────────────────────────────────
  Serial.println("Waiting for ball in detector...");
  int current;
  do {
    current = averagedRead(PIN_PHOTO, SAMPLE_COUNT);
    delay(50);
  } while (current >= detectionThreshold);

  // ── Ball detected ─────────────────────────────────────────────────────────
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
  delay(500);

  // Pick ball back up from color detector
  moveServoSlow(servo4, 15, 15);
  delay(500);
  moveServoSlow(servo3, 170, 15);
  moveServoSlow(servo2, 135, 15);
  delay(500);

  // Put ball back into dispenser
  moveServoSlow(servo1, 180, 15);
  delay(500);
  moveServoSlow(servo2, 85, 15);
  moveServoSlow(servo4, 90, 15);
  delay(500);
  moveServoSlow(servo3, 160, 15);
  moveServoSlow(servo2, 135, 15);
  delay(500);
}

void moveServoSlow(Servo &s, int target, int speedDelay) {
  int current = s.read();

  while (current != target) {
    if (current < target) current++;
    else if (current > target) current--;

    s.write(current);
    delay(speedDelay);
  }
}