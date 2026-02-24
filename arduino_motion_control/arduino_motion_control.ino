#include <Servo.h>

Servo myServo;
const int servoPin = 7; // defining servo pins
const int stepperPins[] = {11, 10, 9, 8}; // defining stepper pins
const int offsetServo = 13;
const int measurementDelay = 5000; // Delay between measurements in ms
int currentStepperPositionSteps = 0; // defining stepper starting position
int currentStepperStep = 0;
int currentServoPosition = 0;
float offsetsStepper[] = {0.5, 1.0, 2.0, 3.0};
const int rollOffset = 6.723;
#define interruptSTM32Pin 4
#define interuptPin 2

const int STEPS_PER_REV = 4096; // Confirmed for full-step mode
const int stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

void stepperStep(int direction, int stepDelay);
void stepperMoves(float targetAngle, float rate);
void calibrateStepper();
void moveServo(float angle);

void setup() {
  myServo.attach(servoPin);
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    pinMode(stepperPins[i], OUTPUT);
    digitalWrite(stepperPins[i], LOW);
  }
  pinMode(interuptPin, INPUT_PULLUP);
  pinMode(interruptSTM32Pin, OUTPUT);
  digitalWrite(interruptSTM32Pin, LOW);
  myServo.writeMicroseconds(int(trunc(687.919)));

  Serial.println("Enter command (e.g., calibrate, run test case 1, stepper to 45, or servo to 90):");
}

void calibrateStepper() {
  Serial.println("Calibrating...");
  stepperMoves(-360.0, 5.0);
  currentStepperPositionSteps = 0;
  Serial.println("Done.");
}

void stepperStep(int direction, int stepDelay) {
  currentStepperStep += direction;
  if (currentStepperStep < 0) currentStepperStep = 7;
  else if (currentStepperStep > 7) currentStepperStep = 0;

  for (int j = 0; j < 4; j++) {
    digitalWrite(stepperPins[j], stepSequence[currentStepperStep][j]);
  }

  delay(stepDelay);
}

void stepperMoves(float targetAngle, float rate = 5.0) {
  //OLD: int steps = round(targetAngle * (STEPS_PER_REV / 360.0));
  // int stepDelay = (rate == 5.0) ? 18 : 879;
  targetAngle = targetAngle - rollOffset;
  int steps = round(targetAngle * (STEPS_PER_REV / 360.0)) - currentStepperPositionSteps;
  const float slope = -175.71;
  const float offset = 896.571;
  float stepDelay = slope * rate + offset;
  if (stepDelay < 1) {
    stepDelay = 1;
  }
  int direction = (steps > 0) ? 1 : -1;

  for (int i = 0; i < abs(steps); i++) {
    if(direction < 0 && !digitalRead(interuptPin)){
      Serial.println("Endstop activated!");
      return;
    }
    stepperStep(-direction, int(trunc(stepDelay)));
    currentStepperPositionSteps += direction;
    //Serial.println(!digitalRead(interuptPin));
  }
} 

void moveServo(float angle) {
  angle += offsetServo;
  if (angle < 0) angle = 0;
  if (angle > 170) angle = 170;
  const float slope = 10.763;
  const float offset = 548;
  int microseconds = int(trunc(angle * slope + offset));
  int microsecondsPrevious = int(trunc(currentServoPosition * slope + offset));
  for(int i = 0; i < abs(microseconds - microsecondsPrevious); i++){
    myServo.writeMicroseconds(microsecondsPrevious + (i + 1)*((microseconds - microsecondsPrevious > 0) ? 1 : -1)); // Truncate
    delay(2);
  }
  currentServoPosition = angle;
  Serial.print("Current servo angle: ");
  Serial.println(angle - offsetServo);
}

void stepMoves(int offset) {
  int rollAngles[] = {30, 45, 60, 75, 90, 105};
  // digitalWrite(interruptSTM32Pin, HIGH);
  // delay(measurementDelay);
  // digitalWrite(interruptSTM32Pin, LOW);
  
  for (int i = 0; i < (sizeof(rollAngles)/sizeof(*rollAngles)); i++) {
    stepperMoves(rollAngles[i] + offset);
    Serial.print("Current stepper angle: ");
    Serial.println(rollAngles[i]);
    digitalWrite(interruptSTM32Pin, HIGH);
    delay(measurementDelay);
    digitalWrite(interruptSTM32Pin, LOW);
  }
}

void runTestCase(int testCase) {
  if (testCase >= 1 && testCase <= 12) {
    calibrateStepper();
    delay(1000);

    int servoAngles[] = {0, 30, 60, 90, 0, 30, 60, 90, 0, 30, 60, 90};
    int servoTarget = servoAngles[testCase - 1];
    moveServo(servoTarget);

    stepperMoves(offsetsStepper[(testCase - 1)%4]);
    Serial.print("Stepper offset by ");
    Serial.print(offsetsStepper[(testCase - 1)%4]);
    Serial.println(" deg to compensate for servo error");
    
    delay(1000);

    if (testCase >= 1 && testCase <= 4) {
      stepMoves(offsetsStepper[(testCase - 1)%4]);
    } else if (testCase >= 5 && testCase <= 8) {
      stepperMoves(30 + offsetsStepper[(testCase - 1)%4], 5.0);
      delay(1000);
      digitalWrite(interruptSTM32Pin, HIGH);
      stepperMoves(105 + offsetsStepper[(testCase - 1)%4], 5.0);
      digitalWrite(interruptSTM32Pin, LOW);
    } else if (testCase >= 9 && testCase <= 12) {
      stepperMoves(30 + offsetsStepper[(testCase - 1)%4], 5.0);
      delay(1000);
      digitalWrite(interruptSTM32Pin, HIGH);
      stepperMoves(105 + offsetsStepper[(testCase - 1)%4], 0.1);
      digitalWrite(interruptSTM32Pin, LOW);
    }
  }

  else if (testCase == 13) { //staticGroup
    Serial.println("Starting static test group ...");
    calibrateStepper();

    // Serial.println("Starting timer ...");
    // unsigned long staticTime = millis();
    // unsigned long staticTestTime = 33000 + 5*measurementDelay; //this depends on how much measurementDelays are in the loop as well as on other delays + callibration worst case time (30s assumed)

    for (int i = 1; i <= 4; i++) {
      Serial.println();
      Serial.print("Running test case ");
      Serial.println(i);
      runTestCase(i);
      // while(millis() - staticTime < staticTestTime){} //waits to have the same test duration of tests that is a independent from the speed of the Arduino
      // staticTime = millis();
    }

    Serial.println("Static test group complete!");
  }

  else if (testCase == 14) {
    Serial.println("Starting dynamic test group ...");
    calibrateStepper();

    // Serial.println("Starting timer ...");

    // unsigned long dynamicTime = millis();
    // unsigned long dynamicTestTime01 = 1200000;
    // unsigned long dynamicTestTime50 = 63000;
    
    for (int i = 5; i <= 12; i++) {
      Serial.println();
      Serial.print("Running test case ");
      Serial.println(i);
      runTestCase(i);
      // while(millis() - dynamicTime < (i < 9 ? dynamicTestTime01 : dynamicTestTime50)){} //waits to have the same test duration of tests that is a independent from the speed of the Arduino
      // dynamicTime = millis();
    }

    Serial.println("Dynamic test group complete!");
  }

  else {
    Serial.println("Invalid test case number.");
  }
}

void loop() {
  //Serial.println("Enter command (e.g., calibrate, run test case 1, stepper to 45, or servo to 90):");
  static String command = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      command.trim();

      if (command == "calibrate") {
        calibrateStepper();
      }
      else if (command.startsWith("run test case ")) {
        int testCase = command.substring(14).toInt();
        Serial.println();
        Serial.print("Running test case ");
        Serial.println(testCase);
        runTestCase(testCase);
      }
      else if (command.startsWith("stepper to ")) {
        float angle = command.substring(11).toFloat();
        stepperMoves(angle);
      }
      else if (command.startsWith("servo to ")) {
        float angle = command.substring(9).toFloat();
        moveServo(angle);
      }
      else {
        Serial.println("Invalid command.");
      }

      Serial.println();
      Serial.println("Enter command (e.g., calibrate, run test case 1, stepper to 45, or servo to 90):");
      command = "";
    }
    else {
      command += c;
    }
  }
}
