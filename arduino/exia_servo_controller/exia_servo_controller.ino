// JAN 3 2026: THIS CODEE IS WORKING, PLS DONT TOUCH


#include <Servo.h>

// define ports
#define STEERING_PIN  9
#define THROTTLE_PIN  10
#define BRAKE_PIN     11
#define LED_PIN       13

// initial starting points
#define STEER_CENTER     90
#define ESC_NEUTRAL      90
#define BRAKE_FULL       180

const unsigned long TIMEOUT_MS = 2000;

Servo steeringServo;
Servo throttleServo;
Servo brakeServo;

bool armed = false;
unsigned long lastCommandTime = 0;

const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// attach servos and move them to starting pos
void setup() {
  Serial.begin(115200);
  steeringServo.attach(STEERING_PIN);
  throttleServo.attach(THROTTLE_PIN);
  brakeServo.attach(BRAKE_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  steeringServo.write(STEER_CENTER);
  throttleServo.write(ESC_NEUTRAL);
  brakeServo.write(BRAKE_FULL);
  delay(1000);
  Serial.println("READY");
  lastCommandTime = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read(); // read the command

    // if end of command, terminate it and reset buffer
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        buffer[bufferIndex] = '\0';
        processCommand(buffer);
        bufferIndex = 0;
      }
    } // if not the end of the command, add command to buffer
    else if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;
    }
  }

  // emergeency timeout
  if (armed && millis() - lastCommandTime > TIMEOUT_MS) {
    emergencyStop();
    Serial.println("TIMEOUT");
  }

  digitalWrite(LED_PIN, armed);
}

void processCommand(char* cmd) {

  // if armed, reset brake
  if (strcasecmp(cmd, "ARM") == 0) {
    armed = true;
    lastCommandTime = millis();
    steeringServo.write(STEER_CENTER);
    throttleServo.write(ESC_NEUTRAL);
    brakeServo.write(0);
    Serial.println("ARMED");
    return;
  }

  // if disarmeed, emergeency stop
  if (strcasecmp(cmd, "DISARM") == 0) {
    emergencyStop();
    Serial.println("DISARMED");
    return;
  }

  if (!armed) return;


  // commands are in thee form S90, T120, etc
  char cmdCopy[BUFFER_SIZE];
  strncpy(cmdCopy, cmd, BUFFER_SIZE);

  int steer = -1, throttle = -1, brake = -1;

  char* token = strtok(cmdCopy, ",");
  while (token != NULL) { // parse through the entire command to geet thee target values
    if (token[0] == 'S' || token[0] == 's') steer = atoi(token + 1);
    else if (token[0] == 'T' || token[0] == 't') throttle = atoi(token + 1);
    else if (token[0] == 'B' || token[0] == 'b') brake = atoi(token + 1);
    token = strtok(NULL, ",");
  }

  // seet the target value
  if (steer != -1) steeringServo.write(constrain(steer, 0, 180));
  if (throttle != -1) throttleServo.write(constrain(throttle, 0, 180));
  if (brake != -1) brakeServo.write(constrain(brake, 0, 180));

  lastCommandTime = millis();
}

// emergency stop
void emergencyStop() {
  armed = false;
  throttleServo.write(ESC_NEUTRAL);
  brakeServo.write(BRAKE_FULL);
  digitalWrite(LED_PIN, LOW);
}
