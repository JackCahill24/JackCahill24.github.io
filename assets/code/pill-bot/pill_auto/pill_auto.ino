// === Pin Assignments ===
const int DIRbucket        = 2;
const int PULbucket        = 3;
const int BEAMbucket       = 13;

const int DIRtrack         = 4;
const int PULtrack         = 5;
const int BEAMconveyor     = 12;

const int DIRconveyor      = 6;
const int PULconveyor      = 7;

const int DIRpill          = 37;
const int PULpill          = 39;
const int PILLsensor       = 41;

const int FLAGBEAM         = 26;
const int STOPBEAM         = 34;

const int ACTUATOR_EXTEND  = 48;
const int ACTUATOR_RETRACT = 44;

const int RESTART_BUTTON   = 50;

// === Timing (µs) ===
const unsigned int stepDelayBucket   = 3500;
const unsigned int stepDelayTrack    = 2000;
const unsigned int stepDelayConveyor = 300;
const unsigned int stepDelayPill     = 700;

const unsigned long trackStartDelay    = 1000; // ms
const unsigned long conveyorStartDelay = 1000; // ms

// === Pill-counting globals ===
const int breaksPerPillCycle = 30;
int breakCount               = 0;
bool beamState               = HIGH;

// State flag
bool upright = false;

void setup() {
  Serial.begin(9600);

  // bucket

  pinMode(DIRbucket, OUTPUT);
  pinMode(PULbucket, OUTPUT);
  pinMode(BEAMbucket, INPUT_PULLUP);

  // track
  pinMode(DIRtrack, OUTPUT);
  pinMode(PULtrack, OUTPUT);
  pinMode(BEAMconveyor, INPUT_PULLUP);

  // conveyor
  pinMode(DIRconveyor, OUTPUT);
  pinMode(PULconveyor, OUTPUT);

  // pill motor + sensor
  pinMode(DIRpill, OUTPUT);
  pinMode(PULpill, OUTPUT);
  pinMode(PILLsensor, INPUT_PULLUP);

  // flags
  pinMode(FLAGBEAM, INPUT_PULLUP);
  pinMode(STOPBEAM, INPUT_PULLUP);

  // actuator

  pinMode(ACTUATOR_EXTEND, OUTPUT);
  pinMode(ACTUATOR_RETRACT, OUTPUT);

  // restart/start button
  pinMode(RESTART_BUTTON, INPUT_PULLUP);

  // initial states
  digitalWrite(DIRbucket, LOW);
  digitalWrite(DIRtrack,  LOW);
  digitalWrite(DIRconveyor, LOW);
  digitalWrite(DIRpill,   LOW);

  digitalWrite(ACTUATOR_EXTEND, LOW);
  digitalWrite(ACTUATOR_RETRACT, LOW);

  Serial.println("System powered on. Waiting for start button press...");

  // === Wait for button press to start ===
  while (digitalRead(RESTART_BUTTON) == HIGH) {
    delay(10);
  }
  while (digitalRead(RESTART_BUTTON) == LOW) {
    delay(10);

  }

  Serial.println("Start button pressed — beginning operation...");
}

void stepOnce(int pulPin, unsigned int delayTime) {
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(pulPin, LOW);
  delayMicroseconds(delayTime);
}

bool checkRestart() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(RESTART_BUTTON);

  if (lastState == HIGH && currentState == LOW) {
    Serial.println("Restart button pressed — restarting system...");
    delay(200);
    return true;
  }

  lastState = currentState;

  return false;
}

void runProcess() {
  // Reset state
  upright = false;
  breakCount = 0;
  beamState = HIGH;

  // === Stage 1: Bucket motor until bucket beam trips ===
  Serial.println("Stage 1: Bucket running...");
  unsigned long bucketStartTime = millis();
  bool reversed = false;

  digitalWrite(DIRbucket, LOW); // Forward

  while (digitalRead(BEAMbucket) == HIGH) {
    if (checkRestart()) return;

    if (digitalRead(FLAGBEAM) == LOW && !upright) {
      upright = true;
      Serial.println("FLAGBEAM tripped — upright = true");
    }

    // Reverse after 10s if not already reversed
    if (!reversed && millis() - bucketStartTime >= 10000) {
      Serial.println("Bucket has been running for 10s — reversing for 2s...");
      digitalWrite(DIRbucket, HIGH); // Reverse direction

      unsigned long reverseStart = millis();
      while (millis() - reverseStart < 2000) {
        if (checkRestart()) return;
        stepOnce(PULbucket, stepDelayBucket);
      }

      Serial.println("Reversing complete — resuming forward direction");
      digitalWrite(DIRbucket, LOW); // Forward again
      reversed = true; // Only reverse once
    }

    stepOnce(PULbucket, stepDelayBucket);
  }
  Serial.println("  ↳ Bucket beam tripped — stopping bucket");
  delay(trackStartDelay);

  // === Stage 2: Track motor until conveyor beam trips ===

  Serial.println("Stage 2: Track running...");
  while (digitalRead(BEAMconveyor) == HIGH) {
    if (checkRestart()) return;
    if (digitalRead(FLAGBEAM) == LOW && !upright) {
      upright = true;
      Serial.println("FLAGBEAM tripped — upright = true");
    }
    stepOnce(PULtrack, stepDelayTrack);
  }
  Serial.println("  ↳ Conveyor beam tripped — stopping track");
  delay(conveyorStartDelay);

  // === Stage 3: Conveyor motor until STOPBEAM trips ===
  Serial.println("Stage 3: Conveyor running...");
  unsigned long lastConveyorStep = micros();
  bool conveyorHigh = false;
  bool conveyorActive = true;

  while (conveyorActive) {
    if (checkRestart()) return;
    if (digitalRead(FLAGBEAM) == LOW && !upright) {
      upright = true;
      Serial.println("FLAGBEAM tripped — upright = true");

    }

    if (digitalRead(STOPBEAM) == LOW) {
      conveyorActive = false;
      digitalWrite(PULconveyor, LOW);
      Serial.println("STOPBEAM tripped — stopping conveyor");

      Serial.println("Running conveyor for 170 ms...");
      unsigned long postStopStart = millis();
      unsigned long lastStepAfterStop = micros();
      bool convHighAfterStop = false;
      while (millis() - postStopStart < 170) {
        if (checkRestart()) return;
        unsigned long nowAS = micros();
        if (nowAS - lastStepAfterStop >= stepDelayConveyor) {
          lastStepAfterStop = nowAS;
          convHighAfterStop = !convHighAfterStop;
          digitalWrite(PULconveyor, convHighAfterStop ? HIGH : LOW);
        }
      }
      digitalWrite(PULconveyor, LOW);
      Serial.println("250 ms conveyor run complete.");
      break;

    }

    unsigned long now = micros();
    if (now - lastConveyorStep >= stepDelayConveyor) {
      lastConveyorStep = now;
      conveyorHigh = !conveyorHigh;
      digitalWrite(PULconveyor, conveyorHigh ? HIGH : LOW);
    }
  }

  digitalWrite(PULconveyor, LOW);
  Serial.println("↳ Conveyor stage completed");
  delay(100);

  // === Stage 4a or 4b based on upright ===
  if (upright) {
    Serial.println("Stage 4: Upright is true — Starting pill fill stage");
    breakCount = 0;
    beamState  = HIGH;
    unsigned long lastPillStep = micros();
    bool pillHigh = false;

    while (breakCount < breaksPerPillCycle) {

      if (checkRestart()) return;
      if (digitalRead(FLAGBEAM) == LOW && !upright) {
        upright = true;
        Serial.println("FLAGBEAM tripped — upright = true");
      }

      bool currentState = digitalRead(PILLsensor);
      if (currentState == LOW && beamState == HIGH) {
        breakCount++;
        Serial.print("Pill count: ");
        Serial.println(breakCount);
      }
      beamState = currentState;

      unsigned long now = micros();
      if (now - lastPillStep >= stepDelayPill) {
        lastPillStep = now;
        pillHigh = !pillHigh;
        digitalWrite(PULpill, pillHigh ? HIGH : LOW);
      }
    }

    digitalWrite(PULpill, LOW);

    Serial.println("Pill count complete.");

    delay(700); // <<< NEW: Delay after 30th pill is counted

    Serial.println("Running conveyor for 5 seconds...");
    unsigned long conveyorStartTime = millis();
    unsigned long lastConveyorPulse = micros();
    conveyorHigh = false;

    while (millis() - conveyorStartTime < 5000) {
      if (checkRestart()) return;
      unsigned long now2 = micros();
      if (now2 - lastConveyorPulse >= stepDelayConveyor) {
        lastConveyorPulse = now2;
        conveyorHigh = !conveyorHigh;
        digitalWrite(PULconveyor, conveyorHigh ? HIGH : LOW);
      }
    }

    digitalWrite(PULconveyor, LOW);
    Serial.println("Conveyor run after pill motor complete.");

  } else {

    Serial.println("Stage 4: Upright is false — Running actuator path");

    Serial.println("Extending actuator...");
    digitalWrite(ACTUATOR_EXTEND, HIGH);
    for (int i = 0; i < 30; i++) {
      if (checkRestart()) return;
      delay(100);
    }
    digitalWrite(ACTUATOR_EXTEND, LOW);
    Serial.println("Actuator extended.");

    delay(500);
    if (checkRestart()) return;

    Serial.println("Retracting actuator...");
    digitalWrite(ACTUATOR_RETRACT, HIGH);
    for (int i = 0; i < 30; i++) {
      if (checkRestart()) return;
      delay(100);
    }
    digitalWrite(ACTUATOR_RETRACT, LOW);
    Serial.println("Actuator retracted.");
  }

}

void loop() {
  runProcess();
}