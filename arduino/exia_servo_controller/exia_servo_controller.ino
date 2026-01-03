 /*
   * Exia Ground Robot - Arduino Mega Servo Controller
   *
   * Serial Protocol:
   *   S<steer>,T<throttle>,B<brake>\n   (only when ARMED)
   *   ARM\n
   *   DISARM\n
   *
   * Example:
   *   ARM
   *   S90,T110,B0
   *
   * Author: Exia Labs
   * Date: January 2026
   */

  #include <Servo.h>

  // ===================== PIN DEFINITIONS =====================
  #define STEERING_PIN  9
  #define THROTTLE_PIN  10
  #define BRAKE_PIN     11
  #define LED_PIN       13

  // ===================== SERVO CONSTANTS =====================
  #define STEER_CENTER     90
  #define ESC_NEUTRAL      90
  #define THROTTLE_MIN     90
  #define THROTTLE_MAX     180
  #define BRAKE_RELEASE    0
  #define BRAKE_FULL       180

  // ===================== SAFETY =====================
  const unsigned long TIMEOUT_MS = 2000;  // <-- CHANGED: was 500, now 2000

  // ===================== OBJECTS =====================
  Servo steeringServo;
  Servo throttleServo;
  Servo brakeServo;

  // ===================== STATE =====================
  int currentSteering = STEER_CENTER;
  int currentThrottle = ESC_NEUTRAL;
  int currentBrake    = BRAKE_FULL;

  bool armed = false;
  bool safetyStop = true;
  unsigned long lastCommandTime = 0;

  // ===================== SERIAL BUFFER =====================
  const int BUFFER_SIZE = 64;
  char buffer[BUFFER_SIZE];
  int bufferIndex = 0;

  // ==========================================================
  // ===================== SETUP ==============================
  // ==========================================================
  void setup() {
    Serial.begin(115200);

    steeringServo.attach(STEERING_PIN);
    throttleServo.attach(THROTTLE_PIN);
    brakeServo.attach(BRAKE_PIN);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // SAFE STARTUP
    steeringServo.write(STEER_CENTER);
    throttleServo.write(ESC_NEUTRAL);
    brakeServo.write(BRAKE_FULL);

    delay(1000);

    Serial.println("Exia Servo Controller READY");
    Serial.println("Commands: ARM | DISARM | S<T>,T<T>,B<T>");

    lastCommandTime = millis();
  }

  // ==========================================================
  // ===================== LOOP ===============================
  // ==========================================================
  void loop() {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (bufferIndex > 0) {
          buffer[bufferIndex] = '\0';
          processCommand(buffer);
          bufferIndex = 0;
        }
      }
      else if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = c;
      }
    }

    if (armed && millis() - lastCommandTime > TIMEOUT_MS) {
      emergencyStop();
      Serial.println("SAFETY TIMEOUT");
    }

    digitalWrite(LED_PIN, armed && !safetyStop);
  }

  // ==========================================================
  // ===================== COMMAND PARSER =====================
  // ==========================================================
  void processCommand(char* cmd) {
    if (strcasecmp(cmd, "ARM") == 0) {
      armed = true;
      safetyStop = false;
      lastCommandTime = millis();
      brakeServo.write(BRAKE_RELEASE);
      throttleServo.write(ESC_NEUTRAL);
      Serial.println("ARMED");
      return;
    }

    if (strcasecmp(cmd, "DISARM") == 0) {
      emergencyStop();
      Serial.println("DISARMED");
      return;
    }

    if (!armed || safetyStop) {
      Serial.println("IGNORED (NOT ARMED)");
      return;
    }

    char cmdCopy[BUFFER_SIZE];
    strncpy(cmdCopy, cmd, BUFFER_SIZE);

    int steer = -1, throttle = -1, brake = -1;

    char* token = strtok(cmdCopy, ",");
    while (token != NULL) {
      if (token[0] == 'S' || token[0] == 's') {
        steer = atoi(token + 1);
      }
      else if (token[0] == 'T' || token[0] == 't') {
        throttle = atoi(token + 1);
      }
      else if (token[0] == 'B' || token[0] == 'b') {
        brake = atoi(token + 1);
      }
      token = strtok(NULL, ",");
    }

    if (steer != -1) {
      currentSteering = constrain(steer, 0, 180);
      steeringServo.write(currentSteering);
    }

    if (throttle != -1) {
      currentThrottle = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);
      throttleServo.write(currentThrottle);
    }

    if (brake != -1) {
      currentBrake = constrain(brake, 0, 180);
      brakeServo.write(currentBrake);
    }

    lastCommandTime = millis();

    Serial.print("OK S");
    Serial.print(currentSteering);
    Serial.print(" T");
    Serial.print(currentThrottle);
    Serial.print(" B");
    Serial.println(currentBrake);
  }

  // ==========================================================
  // ===================== EMERGENCY STOP =====================
  // ==========================================================
  void emergencyStop() {
    armed = false;
    safetyStop = true;
    throttleServo.write(ESC_NEUTRAL);
    brakeServo.write(BRAKE_FULL);
    digitalWrite(LED_PIN, LOW);
  }
