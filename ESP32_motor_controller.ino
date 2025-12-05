#include <AccelStepper.h>

#define HOMING false

// PIN DEFINITIONS
#define PIN_nENABLE     22
#define PIN_DIR          4
#define PIN_STEP        21
#define PIN_LIMIT_MIN   19
#define PIN_LIMIT_MAX   18

bool loopActive = false;
int loopTargetCount = 0;
int loopCount = 0;
long loopDistance = 0;
bool goingOut = true; 

// STEPPER SETUP
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// positions
long targetPos = 0;
long maxPos = 0;

// settings
int maxSpeed = 400;
int accel = 400;

// serial buffer
String input = "";
String last_valid_command = "";

// status print interval
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 100;  // ms

// SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("nStepper Controller Booting");

  pinMode(PIN_nENABLE, OUTPUT);
  digitalWrite(PIN_nENABLE, LOW);

  pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);

  if (HOMING) {
    Serial.println("Homing sequence starting...");
    homeAxis();
    Serial.println("Homing done.");
  } else {
    maxPos = 400000;
    stepper.setCurrentPosition(0);
  }
  digitalWrite(RGB_BUILTIN, HIGH);

  Serial.println("\nCommands:");
  Serial.println("  MOVE <steps>");
  Serial.println("  GOTO <position>");
  Serial.println("  SPEED <steps_per_sec>");
  Serial.println("  ZERO");
  Serial.println("======================================");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      Serial.printf("Received command: '%s'\n", input.c_str());
      processCommand(input);
      input = "";
    } else {
      input += c;
    }
  }

  //  enforce limits: MIN 
  if (digitalRead(PIN_LIMIT_MIN) == LOW) {
    if (stepper.currentPosition() != 0) {
      Serial.println("** MIN switch hit: position set to 0");
    }
    stepper.setCurrentPosition(0);

    if (stepper.targetPosition() < 0) {
      Serial.println("Clamped target to MIN");
      stepper.moveTo(0);
    }
  }

  //  enforce limits: MAX 
  if (digitalRead(PIN_LIMIT_MAX) == LOW) {
    if (stepper.currentPosition() != maxPos) {
      Serial.printf("** MAX switch hit: position set to %ld\n", maxPos);
    }
    stepper.setCurrentPosition(maxPos);

    if (stepper.targetPosition() > maxPos) {
      Serial.println("Clamped target to MAX");
      stepper.moveTo(maxPos);
    }
  }

  if (loopActive) {
      if (stepper.distanceToGo() == 0) {

          if (goingOut) {
              stepper.moveTo(loopDistance);
              goingOut = false;
              Serial.printf("Loop %d: Moving OUT → %ld\n", loopCount + 1, loopDistance);
          }

          // finished back to 0, loop finished
          else {
              stepper.moveTo(0);
              goingOut = true;
              loopCount++;

              Serial.printf("Loop %d: Returning BACK → 0\n", loopCount);

              if (loopCount >= loopTargetCount) {
                  loopActive = false;
                  Serial.println("LOOP COMPLETE.");
              }
          }
      }
  }

  // run stepper motion
  stepper.run();

  // print status
  unsigned long now = millis();
  if (now - lastStatusPrint >= STATUS_INTERVAL) {
    lastStatusPrint = now;

    Serial.printf(
      "[STATUS] Pos: %ld | Target: %ld | Speed: %.2f | Min: 0 | Max: %ld\n",
      stepper.currentPosition(),
      stepper.targetPosition(),
      stepper.speed(),
      maxPos
    );
  }
}




// homing
void homeAxis() {
  Serial.println("Homing: Searching for MIN...");
  rgbLedWrite(RGB_BUILTIN, 0, 0, 63);
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(300);
  stepper.moveTo(-20000);

  while (digitalRead(PIN_LIMIT_MIN) != LOW) {
    stepper.run();
  }

  Serial.println("Homing: MIN switch triggered.");
  stepper.stop();
  delay(200);

  stepper.setCurrentPosition(0);  // set zero

  Serial.println("Homing: Searching for MAX...");
  rgbLedWrite(RGB_BUILTIN, 63, 63, 0);
  stepper.moveTo(20000);

  while (digitalRead(PIN_LIMIT_MAX) != LOW) {
    stepper.run();
  }

  Serial.println("Homing: MAX switch triggered.");
  stepper.stop();
  delay(200);

  maxPos = stepper.currentPosition();
  Serial.printf("Homing: Max travel = %ld steps\n", maxPos);

  long mid = maxPos / 2;
  Serial.printf("Homing: Moving to mid point: %ld\n", mid);
  stepper.moveTo(mid);

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  targetPos = mid;
  Serial.println("Homing: Midpoint reached");
}




// commands
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // MOVE <steps>
  if (cmd.startsWith("MOVE")) {
    rgbLedWrite(RGB_BUILTIN, 0, 63, 0);
    long delta = cmd.substring(5).toInt();
    last_valid_command = "MOVE command: delta = " + String(delta);
    Serial.println(last_valid_command);

    long newTarget = targetPos + delta;
    if (newTarget < 0) newTarget = 0;
    if (newTarget > maxPos) newTarget = maxPos;

    targetPos = newTarget;
    Serial.printf("New target position: %ld\n", targetPos);
    stepper.moveTo(targetPos);
    return;
  }

  // GOTO <position>
  if (cmd.startsWith("GOTO")) {
    rgbLedWrite(RGB_BUILTIN, 63, 0, 63);
    long pos = cmd.substring(5).toInt();
    last_valid_command = "GOTO command: pos = " + String(pos);
    Serial.println(last_valid_command);

    if (pos < 0) pos = 0;
    if (pos > maxPos) pos = maxPos;

    targetPos = pos;
    Serial.printf("Clamped GOTO target: %ld\n", targetPos);
    stepper.moveTo(targetPos);
    return;
  }

  // SPEED <steps/sec>
  if (cmd.startsWith("SPEED")) {
    rgbLedWrite(RGB_BUILTIN, 20, 0, 0);
    maxSpeed = cmd.substring(6).toInt();
    stepper.setMaxSpeed(maxSpeed);

    last_valid_command = "Speed updated: " + String(maxSpeed);
    Serial.println(last_valid_command);
    return;
  }

  // ZERO
  if (cmd == "ZERO") {
    rgbLedWrite(RGB_BUILTIN, 0, 63, 63);
    stepper.setCurrentPosition(0);
    targetPos = 0;

    last_valid_command = "ZERO command: Position set to 0";
    Serial.println(last_valid_command);
    return;
  }

  if (cmd.startsWith("LOOP")) {
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);

      if (firstSpace < 0 || secondSpace < 0) {
        Serial.println("Usage: LOOP <count> <distance>");
        return;
      }

      loopTargetCount = cmd.substring(firstSpace + 1, secondSpace).toInt();
      loopDistance = cmd.substring(secondSpace + 1).toInt();

      loopCount = 0;
      goingOut = true;
      loopActive = true;

      Serial.printf("Starting LOOP: %d cycles, distance=%ld\n", loopTargetCount, loopDistance);

      // Start first outbound move
      stepper.moveTo(loopDistance);
      return;
  }


  // UNKNOWN
  Serial.println("Unknown command.");
  Serial.print("Last valid command: ");
  Serial.println(last_valid_command);
}

