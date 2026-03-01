#include <LSS.h>

#define CH1_PIN 2
#define CH2_PIN 3
#define LED_PIN 21

#define ENABLE_GEARSHIFT 1

#if ENABLE_GEARSHIFT
#define CH3_PIN 4

#define CH3_UP_THRESHOLD 1750
#define CH3_DOWN_THRESHOLD 1250

#define GEAR_HIGH_POS 110
#define GEAR_NEUTRAL_POS 80
#define GEAR_REVERSE_POS 57

#define MAX_GEAR_RATE 100.0f
#define NUM_GEARS 3

const int GEAR_SERVO_POSITIONS[NUM_GEARS] = {
    GEAR_REVERSE_POS, GEAR_NEUTRAL_POS, GEAR_HIGH_POS
};
#endif

#define RC_CENTER 1500
#define RC_DEADZONE 60
#define RC_MIN 900
#define RC_MAX 2100

#define THROTTLE_NEUTRAL 0
#define THROTTLE_MAX 75
#define BRAKE_NEUTRAL 180
#define BRAKE_MAX 80
#define BRAKE_THRESHOLD_FOR_SHIFT 140

#define HEARTBEAT_MS 100
#define RC_TIMEOUT_MS 500
#define SERIAL_TIMEOUT_MS 500
#define JETSON_TIMEOUT_MS 300
#define STEERING_CHANGE_THRESHOLD 10
#define AUTONOMOUS_CMD_TIMEOUT_MS 500

#define OUTPUT_HYSTERESIS 1
#define MAX_THROTTLE_RATE 2000.0f
#define MAX_BRAKE_RATE 2000.0f
#define MEDIAN_FILTER_SIZE 3

#define LSS_THROTTLE_ID 1
#define LSS_BRAKE_ID 2
#define LSS_GEAR_ID 3

#define LSS_SERVO_TIMEOUT_MS 500

LSS throttleServo(LSS_THROTTLE_ID);
LSS brakeServo(LSS_BRAKE_ID);

#if ENABLE_GEARSHIFT
enum GearState { GEAR_REVERSE = 0, GEAR_NEUTRAL = 1, GEAR_HIGH = 2 };
enum Ch3Position { CH3_POS_UP, CH3_POS_CENTER, CH3_POS_DOWN };
enum SwitchLatchState { LATCH_READY, LATCH_WAITING };

LSS gearServoLSS(LSS_GEAR_ID);
GearState currentGear = GEAR_NEUTRAL;
SwitchLatchState switchLatch = LATCH_READY;
float currentGearPos = GEAR_NEUTRAL_POS;
int lastGearOutput = GEAR_NEUTRAL_POS;
int ch3Buffer[MEDIAN_FILTER_SIZE];
GearState jetsonTargetGear = GEAR_NEUTRAL;
#endif

unsigned long lastHeartbeatTime = 0;
unsigned long lastValidRCTime = 0;
unsigned long lastSerialRxTime = 0;
unsigned long lastLoopTime = 0;
int lastSteeringValue = 0;
bool rcLost = false;
bool jetsonConnected = false;
bool autonomousMode = false;
unsigned long lastJetsonCmdTime = 0;
float jetsonTargetThrottle = THROTTLE_NEUTRAL;
float jetsonTargetBrake = BRAKE_NEUTRAL;

int currentState = 0;
int odriveError = 0;

float currentThrottlePos = THROTTLE_NEUTRAL;
float currentBrakePos = BRAKE_NEUTRAL;
int lastThrottleOutput = THROTTLE_NEUTRAL;
int lastBrakeOutput = BRAKE_NEUTRAL;

int ch1Buffer[MEDIAN_FILTER_SIZE];
int ch2Buffer[MEDIAN_FILTER_SIZE];
int bufferIdx = 0;
bool bufferReady = false;

char serialBuffer[64];
int serialBufIndex = 0;

int servoDegreesToLSS(int degrees) {
    return (degrees - 90) * 10;
}

bool queryServoPresent(LSS& servo) {
    servo.getPosition();
    return (LSS::getLastCommStatus() == LSS_CommStatus_ReadSuccess);
}

bool verifyServosAtStartup() {
    unsigned long startTime;

    startTime = millis();
    while (millis() - startTime < LSS_SERVO_TIMEOUT_MS) {
        if (queryServoPresent(throttleServo)) break;
        delay(50);
    }
    if (millis() - startTime >= LSS_SERVO_TIMEOUT_MS) return false;

    startTime = millis();
    while (millis() - startTime < LSS_SERVO_TIMEOUT_MS) {
        if (queryServoPresent(brakeServo)) break;
        delay(50);
    }
    if (millis() - startTime >= LSS_SERVO_TIMEOUT_MS) return false;

#if ENABLE_GEARSHIFT
    startTime = millis();
    while (millis() - startTime < LSS_SERVO_TIMEOUT_MS) {
        if (queryServoPresent(gearServoLSS)) break;
        delay(50);
    }
    if (millis() - startTime >= LSS_SERVO_TIMEOUT_MS) return false;
#endif

    return true;
}

void setup() {
    pinMode(CH1_PIN, INPUT);
    pinMode(CH2_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
#if ENABLE_GEARSHIFT
    pinMode(CH3_PIN, INPUT);
#endif

    LSS::initBus(Serial1, LSS_DefaultBaud);

    delay(100);

    throttleServo.move(servoDegreesToLSS(THROTTLE_NEUTRAL));
    brakeServo.move(servoDegreesToLSS(BRAKE_NEUTRAL));
#if ENABLE_GEARSHIFT
    gearServoLSS.move(servoDegreesToLSS(GEAR_NEUTRAL_POS));
    currentGear = GEAR_NEUTRAL;
    currentGearPos = GEAR_NEUTRAL_POS;
    lastGearOutput = GEAR_NEUTRAL_POS;
    switchLatch = LATCH_READY;
#endif

    Serial.begin(115200);
    delay(1000);

    if (!verifyServosAtStartup()) {
        while (true) {
            Serial.println("SERVO_ERR");
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    }

    Serial.println("READY");
    while (true) {
        if (Serial.available() && Serial.read() == 'C') {
            break;
        }
        delay(100);
    }

    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
        ch1Buffer[i] = RC_CENTER;
        ch2Buffer[i] = RC_CENTER;
#if ENABLE_GEARSHIFT
        ch3Buffer[i] = RC_CENTER;
#endif
    }

    lastValidRCTime = millis();
    lastHeartbeatTime = millis();
    lastSerialRxTime = millis();
    lastLoopTime = millis();

    digitalWrite(LED_PIN, LOW);
    Serial.println("READY");
}

void sendSteering(int value) {
    value = constrain(value, -1000, 1000);
    Serial.print("S");
    Serial.println(value);
    lastHeartbeatTime = millis();
}

void sendHeartbeat() {
    Serial.println("H");
    lastHeartbeatTime = millis();
}

void sendRCLost() {
    Serial.println("L");
}

void sendRCRecovered() {
    Serial.println("R");
}

void sendEmergencyStop() {
    Serial.println("E");
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        lastSerialRxTime = millis();

        if (c == '\n' || c == '\r') {
            if (serialBufIndex > 0) {
                serialBuffer[serialBufIndex] = '\0';
                parseJetsonMessage(serialBuffer);
                serialBufIndex = 0;
            }
        } else if (serialBufIndex < 63) {
            serialBuffer[serialBufIndex++] = c;
        }
    }
}

void parseJetsonMessage(const char* msg) {
    if (strcmp(msg, "AUTO") == 0) {
        autonomousMode = true;
        lastJetsonCmdTime = millis();
        jetsonTargetThrottle = THROTTLE_NEUTRAL;
        jetsonTargetBrake = BRAKE_NEUTRAL;
        Serial.println("AUTONOMOUS");
        jetsonConnected = true;
    }
    else if (strcmp(msg, "MANUAL") == 0) {
        autonomousMode = false;
        jetsonTargetThrottle = THROTTLE_NEUTRAL;
        jetsonTargetBrake = BRAKE_NEUTRAL;
#if ENABLE_GEARSHIFT
        jetsonTargetGear = GEAR_NEUTRAL;
#endif
        Serial.println("MANUAL");
        jetsonConnected = true;
    }
    else if (msg[0] == 'J' && autonomousMode) {
        int t, b;
        if (sscanf(msg + 1, "%d,%d", &t, &b) == 2) {
            jetsonTargetThrottle = constrain(t, THROTTLE_NEUTRAL, THROTTLE_MAX);
            jetsonTargetBrake = constrain(b, BRAKE_MAX, BRAKE_NEUTRAL);
            lastJetsonCmdTime = millis();
        }
        jetsonConnected = true;
    }
    else if (msg[0] == 'K' && autonomousMode) {
        int g = atoi(msg + 1);
        if (g >= 0 && g <= 2) {
#if ENABLE_GEARSHIFT
            jetsonTargetGear = (GearState)g;
#endif
            lastJetsonCmdTime = millis();
        }
        jetsonConnected = true;
    }
    else if (msg[0] == 'F') {
        jetsonConnected = true;
    }
    else if (msg[0] == 'Z') {
        currentState = atoi(msg + 1);
        jetsonConnected = true;
        if (currentState == 3) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
    else if (msg[0] == 'A') {
        jetsonConnected = true;
    }
}

int readRCChannel(int pin) {
    int pulse = pulseIn(pin, HIGH, 25000);
    if (pulse == 0) {
        return -1;
    }
    return pulse;
}

bool isValidRC(int pulse) {
    return (pulse >= RC_MIN && pulse <= RC_MAX);
}

int getMedian(int* buf, int size) {
    int sorted[MEDIAN_FILTER_SIZE];
    for (int i = 0; i < size; i++) {
        sorted[i] = buf[i];
    }
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (sorted[j] < sorted[i]) {
                int temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }
    return sorted[size / 2];
}

void updateInputBuffer(int ch1Raw, int ch2Raw, int ch3Raw) {
    if (isValidRC(ch1Raw)) {
        ch1Buffer[bufferIdx] = ch1Raw;
    }
    if (isValidRC(ch2Raw)) {
        ch2Buffer[bufferIdx] = ch2Raw;
    }
#if ENABLE_GEARSHIFT
    if (isValidRC(ch3Raw)) {
        ch3Buffer[bufferIdx] = ch3Raw;
    }
#endif
    bufferIdx = (bufferIdx + 1) % MEDIAN_FILTER_SIZE;
    if (bufferIdx == 0) {
        bufferReady = true;
    }
}

int getFilteredCh1() {
    if (!bufferReady) {
        return ch1Buffer[0];
    }
    return getMedian(ch1Buffer, MEDIAN_FILTER_SIZE);
}

int getFilteredCh2() {
    if (!bufferReady) {
        return ch2Buffer[0];
    }
    return getMedian(ch2Buffer, MEDIAN_FILTER_SIZE);
}

#if ENABLE_GEARSHIFT
int getFilteredCh3() {
    if (!bufferReady) return ch3Buffer[0];
    return getMedian(ch3Buffer, MEDIAN_FILTER_SIZE);
}

Ch3Position getCh3Position(int pwmValue) {
    if (pwmValue < CH3_DOWN_THRESHOLD) return CH3_POS_UP;
    if (pwmValue > CH3_UP_THRESHOLD) return CH3_POS_DOWN;
    return CH3_POS_CENTER;
}

void sendGearChange(GearState gear) {
    Serial.print("G");
    Serial.println((int)gear);
    lastHeartbeatTime = millis();
}

void shiftGearForward() {
    if (currentGear < GEAR_HIGH) {
        currentGear = (GearState)(currentGear + 1);
        sendGearChange(currentGear);
    }
}

void shiftGearBackward() {
    if (currentGear > GEAR_REVERSE) {
        currentGear = (GearState)(currentGear - 1);
        sendGearChange(currentGear);
    }
}

bool isBrakeEngaged() {
    return currentBrakePos < BRAKE_THRESHOLD_FOR_SHIFT;
}

void processGearShift(int ch3Filtered, bool rcLostFlag) {
    if (rcLostFlag) return;

    String val = String(ch3Filtered);
    Serial.println(val);

    Ch3Position pos = getCh3Position(ch3Filtered);

    if (switchLatch == LATCH_READY) {
        if (pos == CH3_POS_UP && isBrakeEngaged()) {
            shiftGearForward();
            switchLatch = LATCH_WAITING;
        } else if (pos == CH3_POS_DOWN && isBrakeEngaged()) {
            shiftGearBackward();
            switchLatch = LATCH_WAITING;
        }
    } else {
        if (pos == CH3_POS_CENTER) {
            switchLatch = LATCH_READY;
        }
    }
}

void updateGearServo(float dt) {
    float targetPos = (float)GEAR_SERVO_POSITIONS[currentGear];
    currentGearPos = applyRateLimit(currentGearPos, targetPos, MAX_GEAR_RATE, dt);
    writeServoWithHysteresis(gearServoLSS, currentGearPos, lastGearOutput);
}
#endif

float applyRateLimit(float current, float target, float maxRate, float dt) {
    float maxChange = maxRate * dt;
    float diff = target - current;
    if (diff > maxChange) {
        return current + maxChange;
    } else if (diff < -maxChange) {
        return current - maxChange;
    }
    return target;
}

void writeServoWithHysteresis(LSS& servo, float position, int& lastOutput) {
    int newOutput = (int)(position + 0.5f);
    if (abs(newOutput - lastOutput) >= OUTPUT_HYSTERESIS) {
        servo.move(servoDegreesToLSS(newOutput));
        lastOutput = newOutput;
    }
}

int rcToSteering(float smoothedPulse) {
    int pulse = (int)(smoothedPulse + 0.5f);
    int centered = pulse - RC_CENTER;
    if (abs(centered) < RC_DEADZONE) {
        return 0;
    }
    int value = map(pulse, 1000, 2000, -1000, 1000);
    return constrain(value, -1000, 1000);
}

void handleThrottleBrake(float smoothedCh2Val, float dt) {
    int ch2 = (int)(smoothedCh2Val + 0.5f);

    float targetThrottle = THROTTLE_NEUTRAL;
    float targetBrake = BRAKE_NEUTRAL;

    if (ch2 < RC_MIN || ch2 > RC_MAX) {
        targetThrottle = THROTTLE_NEUTRAL;
        targetBrake = BRAKE_NEUTRAL;
    } else {
        int delta = ch2 - RC_CENTER;

        if (abs(delta) < RC_DEADZONE) {
            targetThrottle = THROTTLE_NEUTRAL;
            targetBrake = BRAKE_NEUTRAL;
        }
        else if (delta > 0) {
            targetThrottle = map(delta, RC_DEADZONE, 500, THROTTLE_NEUTRAL, THROTTLE_MAX);
            targetThrottle = constrain(targetThrottle, THROTTLE_NEUTRAL, THROTTLE_MAX);
            targetBrake = BRAKE_NEUTRAL;
        }
        else {
            targetBrake = map(-delta, RC_DEADZONE, 500, BRAKE_NEUTRAL, BRAKE_MAX);
            targetBrake = constrain(targetBrake, BRAKE_MAX, BRAKE_NEUTRAL);
            targetThrottle = THROTTLE_NEUTRAL;
        }
    }

    currentThrottlePos = applyRateLimit(currentThrottlePos, targetThrottle, MAX_THROTTLE_RATE, dt);
    currentBrakePos = applyRateLimit(currentBrakePos, targetBrake, MAX_BRAKE_RATE, dt);

    writeServoWithHysteresis(throttleServo, currentThrottlePos, lastThrottleOutput);
    writeServoWithHysteresis(brakeServo, currentBrakePos, lastBrakeOutput);
}

void safeState() {
    autonomousMode = false;
    jetsonTargetThrottle = THROTTLE_NEUTRAL;
    jetsonTargetBrake = BRAKE_NEUTRAL;
    currentThrottlePos = THROTTLE_NEUTRAL;
    currentBrakePos = BRAKE_MAX;
    throttleServo.move(servoDegreesToLSS(THROTTLE_NEUTRAL));
    brakeServo.move(servoDegreesToLSS(BRAKE_MAX));
    brakeServo.hold();
    lastThrottleOutput = THROTTLE_NEUTRAL;
    lastBrakeOutput = BRAKE_MAX;
#if ENABLE_GEARSHIFT
    currentGear = GEAR_NEUTRAL;
    currentGearPos = GEAR_NEUTRAL_POS;
    gearServoLSS.move(servoDegreesToLSS(GEAR_NEUTRAL_POS));
    lastGearOutput = GEAR_NEUTRAL_POS;
    switchLatch = LATCH_READY;
#endif
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    lastLoopTime = now;

    processSerialInput();

    if (jetsonConnected && (now - lastSerialRxTime > JETSON_TIMEOUT_MS)) {
        jetsonConnected = false;
        sendEmergencyStop();
        safeState();
        return;
    }

    if (autonomousMode && (now - lastJetsonCmdTime > AUTONOMOUS_CMD_TIMEOUT_MS)) {
        autonomousMode = false;
        safeState();
        Serial.println("MANUAL");
        return;
    }

    int ch1Raw = readRCChannel(CH1_PIN);
    int ch2Raw = readRCChannel(CH2_PIN);
#if ENABLE_GEARSHIFT
    int ch3Raw = readRCChannel(CH3_PIN);
#else
    int ch3Raw = -1;
#endif

    updateInputBuffer(ch1Raw, ch2Raw, ch3Raw);

    int ch1Filtered = getFilteredCh1();
    int ch2Filtered = getFilteredCh2();
#if ENABLE_GEARSHIFT
    int ch3Filtered = getFilteredCh3();
#endif

    bool ch1Valid = isValidRC(ch1Raw);
    bool ch2Valid = isValidRC(ch2Raw);
#if ENABLE_GEARSHIFT
    bool ch3Valid = isValidRC(ch3Raw);

    Serial.print("CH3:");
    Serial.print(ch3Raw);
    Serial.print(" F:");
    Serial.print(ch3Filtered);
    Serial.print(" G:");
    Serial.print(currentGear);
    Serial.print(" L:");
    Serial.println(switchLatch);
#endif

#if ENABLE_GEARSHIFT
    if (ch1Valid || ch2Valid || ch3Valid) {
#else
    if (ch1Valid || ch2Valid) {
#endif
        lastValidRCTime = now;

        if (rcLost) {
            rcLost = false;
            sendRCRecovered();
        }
    }

    if (!autonomousMode && (now - lastValidRCTime > RC_TIMEOUT_MS)) {
        if (!rcLost) {
            rcLost = true;
            sendRCLost();
            safeState();
        }
        sendSteering(0);
        delay(20);
        return;
    }

    if (ch1Valid) {
        int steeringValue = rcToSteering((float)ch1Filtered);

        if (abs(steeringValue - lastSteeringValue) > STEERING_CHANGE_THRESHOLD) {
            sendSteering(steeringValue);
            lastSteeringValue = steeringValue;
        }
    }

    if (now - lastHeartbeatTime > HEARTBEAT_MS) {
        sendHeartbeat();
    }

    if (autonomousMode) {
        currentThrottlePos = applyRateLimit(currentThrottlePos, jetsonTargetThrottle, MAX_THROTTLE_RATE, dt);
        currentBrakePos = applyRateLimit(currentBrakePos, jetsonTargetBrake, MAX_BRAKE_RATE, dt);
        writeServoWithHysteresis(throttleServo, currentThrottlePos, lastThrottleOutput);
        writeServoWithHysteresis(brakeServo, currentBrakePos, lastBrakeOutput);
    } else {
        handleThrottleBrake((float)ch2Filtered, dt);
    }

#if ENABLE_GEARSHIFT
    if (autonomousMode) {
        currentGear = jetsonTargetGear;
    } else if (!rcLost) {
        processGearShift(ch3Filtered, rcLost);
    }
    updateGearServo(dt);
#endif

    delay(10);
}
