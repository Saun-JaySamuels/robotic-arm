const int SPOTS[] = {A1, A2, A3, A4};
const int NUM_SPOTS = 4;
const int THRESHOLD_OFFSET = 50;  // tune this: how much above ambient = ball present

int ambient[NUM_SPOTS];

void setup() {
  Serial.begin(9600);

  // External pulldown to GND — use plain INPUT, not INPUT_PULLUP
  for (int i = 0; i < NUM_SPOTS; i++) {
    pinMode(SPOTS[i], INPUT);
  }

  delay(200);  // let pins settle

  Serial.println("Capturing ambient (no ball)...");
  for (int i = 0; i < NUM_SPOTS; i++) {
    ambient[i] = analogRead(SPOTS[i]);
    Serial.print("Spot "); Serial.print(i + 1);
    Serial.print(" ambient: "); Serial.println(ambient[i]);
  }
  Serial.println("Ready.");
  Serial.println("----------");
}

bool isBallPresent(int spotIndex) {
  int val = analogRead(SPOTS[spotIndex]);
  // Ball bridges mesh to 5V → val rises ABOVE ambient
  return val > (ambient[spotIndex] + THRESHOLD_OFFSET);
}

void loop() {
  for (int i = 0; i < NUM_SPOTS; i++) {
    int raw = analogRead(SPOTS[i]);
    bool ball = isBallPresent(i);
    Serial.print("Spot "); Serial.print(i + 1);
    Serial.print(" ["); Serial.print(raw); Serial.print("]: ");
    Serial.println(ball ? "BALL" : "empty");
  }
  Serial.println("----------");
  delay(500);
}