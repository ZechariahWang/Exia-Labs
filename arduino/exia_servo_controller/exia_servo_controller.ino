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

  // ===================== SOFT-START / RATE LIMITING =====================
  // Maximum degrees the servo can change per 10ms update cycle
  // At 100Hz: rate * 100 = degrees per second
  const int MAX_STEERING_RATE = 15;  // 1500 deg/sec - responsive steering
  const int MAX_THROTTLE_RATE = 8;   // 800 deg/sec - smooth throttle ramp
  const int MAX_BRAKE_RATE = 20;     // 2000 deg/sec - fast brake response for safety

  // ===================== OBJECTS =====================
  Servo steeringServo;
  Servo throttleServo;
  Servo brakeServo;

  // ===================== STATE =====================
  int currentSteering = STEER_CENTER;
  int currentThrottle = ESC_NEUTRAL;
  int currentBrake    = BRAKE_FULL;

  // Smoothed servo positions (exponential filter)
  float smoothSteering = STEER_CENTER;
  float smoothThrottle = ESC_NEUTRAL;
  float smoothBrake    = BRAKE_FULL;

  // Rate-limited output positions (after both smoothing AND rate limiting)
  // These are the actual values written to servos - prevents sudden jumps
  float rateLimitedSteering = STEER_CENTER;
  float rateLimitedThrottle = ESC_NEUTRAL;
  float rateLimitedBrake    = BRAKE_FULL;

  // Smoothing factor: 0.1 = very smooth, 0.5 = responsive
  const float SMOOTH_ALPHA = 0.25;

  bool armed = false;
  bool safetyStop = true;
  unsigned long lastCommandTime = 0;
  unsigned long lastServoUpdate = 0;
  const unsigned long SERVO_UPDATE_INTERVAL = 10;  // Update servos every 10ms (100Hz)

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

    // Update servos with smoothing at 100Hz
    if (millis() - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
      lastServoUpdate = millis();
      updateServos();
    }

    if (armed && millis() - lastCommandTime > TIMEOUT_MS) {
      emergencyStop();
      Serial.println("SAFETY TIMEOUT");
    }

    digitalWrite(LED_PIN, armed && !safetyStop);
  }

  // ==========================================================
  // ===================== RATE LIMITING =======================
  // ==========================================================
  // Apply rate limiting to prevent sudden large servo movements
  // Returns a value that moves toward target but limited by maxRate per cycle
  float applyRateLimit(float current, float target, int maxRate) {
    float diff = target - current;
    if (abs(diff) <= (float)maxRate) {
      return target;
    }
    return current + (diff > 0 ? (float)maxRate : -(float)maxRate);
  }

  // ==========================================================
  // ===================== SERVO SMOOTHING ====================
  // ==========================================================
  void updateServos() {
    if (!armed || safetyStop) {
      return;
    }

    // Step 1: Exponential moving average for smooth transitions (reduces jitter)
    smoothSteering = SMOOTH_ALPHA * currentSteering + (1.0 - SMOOTH_ALPHA) * smoothSteering;
    smoothThrottle = SMOOTH_ALPHA * currentThrottle + (1.0 - SMOOTH_ALPHA) * smoothThrottle;
    smoothBrake    = SMOOTH_ALPHA * currentBrake    + (1.0 - SMOOTH_ALPHA) * smoothBrake;

    // Step 2: Apply rate limiting ON TOP of smoothing (soft-start protection)
    // This prevents sudden large jumps that could damage motors/gears
    rateLimitedSteering = applyRateLimit(rateLimitedSteering, smoothSteering, MAX_STEERING_RATE);
    rateLimitedThrottle = applyRateLimit(rateLimitedThrottle, smoothThrottle, MAX_THROTTLE_RATE);
    rateLimitedBrake    = applyRateLimit(rateLimitedBrake, smoothBrake, MAX_BRAKE_RATE);

    // Write rate-limited values to servos (not just smoothed values)
    steeringServo.write((int)(rateLimitedSteering + 0.5));
    throttleServo.write((int)(rateLimitedThrottle + 0.5));
    brakeServo.write((int)(rateLimitedBrake + 0.5));
  }

  // ==========================================================
  // ===================== COMMAND PARSER =====================
  // ==========================================================
  void processCommand(char* cmd) {
    if (strcasecmp(cmd, "ARM") == 0) {
      armed = true;
      safetyStop = false;
      lastCommandTime = millis();
      // Initialize all values to safe positions
      currentSteering = STEER_CENTER;
      currentThrottle = ESC_NEUTRAL;
      currentBrake = BRAKE_RELEASE;
      // Initialize smooth values (exponential filter state)
      smoothSteering = STEER_CENTER;
      smoothThrottle = ESC_NEUTRAL;
      smoothBrake = BRAKE_RELEASE;
      // Initialize rate-limited values (prevents jumps on ARM)
      rateLimitedSteering = STEER_CENTER;
      rateLimitedThrottle = ESC_NEUTRAL;
      rateLimitedBrake = BRAKE_RELEASE;
      // Write initial positions to servos
      brakeServo.write(BRAKE_RELEASE);
      throttleServo.write(ESC_NEUTRAL);
      steeringServo.write(STEER_CENTER);
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

    // Update target values - smoothing in updateServos() handles actual writes
    if (steer != -1) {
      currentSteering = constrain(steer, 0, 180);
    }

    if (throttle != -1) {
      currentThrottle = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);
    }

    if (brake != -1) {
      currentBrake = constrain(brake, 0, 180);
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
