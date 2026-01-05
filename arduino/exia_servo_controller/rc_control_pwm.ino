#include <Servo.h>

// define channel pins
#define CH1_PIN 2
#define CH2_PIN 3
#define THROTTLE_PIN 44
#define BRAKE_PIN 45
#define LED_PIN 21

#define ENABLE_GEARSHIFT 1

#if ENABLE_GEARSHIFT
#define CH3_PIN 4
#define GEAR_SERVO_PIN 46

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

// standard pwm signal vals
#define RC_CENTER 1500
#define RC_DEADZONE 60
#define RC_MIN 900
#define RC_MAX 2100

// DONT TOUCH PLS, these have been set on hardware, motor movement positions
#define THROTTLE_NEUTRAL 0
#define THROTTLE_MAX 75
#define BRAKE_NEUTRAL 180
#define BRAKE_MAX 80
#define BRAKE_THRESHOLD_FOR_SHIFT 140

// timing constants
#define HEARTBEAT_MS 100
#define RC_TIMEOUT_MS 500
#define SERIAL_TIMEOUT_MS 500
#define JETSON_TIMEOUT_MS 300
#define STEERING_CHANGE_THRESHOLD 10

// median filter constants
#define OUTPUT_HYSTERESIS 1
#define MAX_THROTTLE_RATE 2000.0f
#define MAX_BRAKE_RATE 2000.0f
#define MEDIAN_FILTER_SIZE 3

Servo throttle;
Servo brake;

#if ENABLE_GEARSHIFT
enum GearState { GEAR_REVERSE = 0, GEAR_NEUTRAL = 1, GEAR_HIGH = 2 };
enum Ch3Position { CH3_POS_UP, CH3_POS_CENTER, CH3_POS_DOWN };
enum SwitchLatchState { LATCH_READY, LATCH_WAITING };

Servo gearServo;
GearState currentGear = GEAR_NEUTRAL;
SwitchLatchState switchLatch = LATCH_READY;
float currentGearPos = GEAR_NEUTRAL_POS;
int lastGearOutput = GEAR_NEUTRAL_POS;
int ch3Buffer[MEDIAN_FILTER_SIZE];
#endif

unsigned long lastHeartbeatTime = 0;
unsigned long lastValidRCTime = 0;
unsigned long lastSerialRxTime = 0;
unsigned long lastLoopTime = 0;
int lastSteeringValue = 0;
bool rcLost = false;
bool jetsonConnected = false;

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

void setup() {

    // config pin modes
    pinMode(CH1_PIN, INPUT);
    pinMode(CH2_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
#if ENABLE_GEARSHIFT
    pinMode(CH3_PIN, INPUT);
#endif

    // attach servos to their pins, write init vals
    throttle.attach(THROTTLE_PIN, 1000, 2000);
    brake.attach(BRAKE_PIN, 1000, 2000);
#if ENABLE_GEARSHIFT
    gearServo.attach(GEAR_SERVO_PIN, 1000, 2000);
#endif

    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_NEUTRAL);
#if ENABLE_GEARSHIFT
    gearServo.write(GEAR_NEUTRAL_POS);
    currentGear = GEAR_NEUTRAL;
    currentGearPos = GEAR_NEUTRAL_POS;
    lastGearOutput = GEAR_NEUTRAL_POS;
    switchLatch = LATCH_READY;
#endif

    Serial.begin(115200);
    delay(1000);
    Serial.println("READY");
    // Handshake bypassed for testing
    // while (true) {
    //     if (Serial.available() && Serial.read() == 'C') {
    //         break;
    //     }
    //     delay(100);
    // }

    // prefill buffer values
    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
        ch1Buffer[i] = RC_CENTER;
        ch2Buffer[i] = RC_CENTER;
#if ENABLE_GEARSHIFT
        ch3Buffer[i] = RC_CENTER;
#endif
    }

    // timestamp is curr time
    lastValidRCTime = millis();
    lastHeartbeatTime = millis();
    lastSerialRxTime = millis();
    lastLoopTime = millis();

    digitalWrite(LED_PIN, LOW);
    Serial.println("READY");
}

// send steering command as "S500 example"
void sendSteering(int value) {
    value = constrain(value, -1000, 1000);
    Serial.print("S");
    Serial.println(value);
    lastHeartbeatTime = millis();
}

// send h, this will show arduino is still alvie
void sendHeartbeat() {
    Serial.println("H");
    lastHeartbeatTime = millis();
}

// if this happens, uh oh signalk was lost
void sendRCLost() {
    Serial.println("L");
}

// if recieved, rc signal is back
void sendRCRecovered() {
    Serial.println("R");
}

// send emerecny stop noti
void sendEmergencyStop() {
    Serial.println("E");
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        lastSerialRxTime = millis();

        if (c == '\n' || c == '\r') { // on new line, reset buffer and parse new message
            if (serialBufIndex > 0) {
                serialBuffer[serialBufIndex] = '\0';
                parseJetsonMessage(serialBuffer);
                serialBufIndex = 0;
            }
        } else if (serialBufIndex < 63) { // otherwise, add add characher to current buffer
            serialBuffer[serialBufIndex++] = c;
        }
    }
}

void parseJetsonMessage(const char* msg) {
    if (msg[0] == 'F') {
        jetsonConnected = true;
    }
    else if (msg[0] == 'Z') {
        currentState = atoi(msg + 1); // turns string into integer (ascii to int)
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

// read rc value, wait up to 25ms
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
    writeServoWithHysteresis(gearServo, currentGearPos, lastGearOutput);
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

void writeServoWithHysteresis(Servo& servo, float position, int& lastOutput) {
    int newOutput = (int)(position + 0.5f);
    if (abs(newOutput - lastOutput) >= OUTPUT_HYSTERESIS) {
        servo.write(newOutput);
        lastOutput = newOutput;
    }
}

// map 1000-2000 us to -1000 to +1000
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

    writeServoWithHysteresis(throttle, currentThrottlePos, lastThrottleOutput);
    writeServoWithHysteresis(brake, currentBrakePos, lastBrakeOutput);
}

void safeState() {
    currentThrottlePos = THROTTLE_NEUTRAL;
    currentBrakePos = BRAKE_MAX;
    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_MAX);
    lastThrottleOutput = THROTTLE_NEUTRAL;
    lastBrakeOutput = BRAKE_MAX;
#if ENABLE_GEARSHIFT
    currentGear = GEAR_NEUTRAL;
    currentGearPos = GEAR_NEUTRAL_POS;
    gearServo.write(GEAR_NEUTRAL_POS);
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

    // unsure abt this 
    // if (jetsonConnected && (now - lastSerialRxTime > JETSON_TIMEOUT_MS)) {
    //     jetsonConnected = false;
    //     sendEmergencyStop();
    //     safeState();
    //     return;
    // }

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

    if (now - lastValidRCTime > RC_TIMEOUT_MS) {
        if (!rcLost) {
            rcLost = true;
            sendRCLost();
            safeState();
        }
        sendSteering(0);
        delay(20);
        return;
    }

    if (ch1Valid) { // send steering val to ros2
        int steeringValue = rcToSteering((float)ch1Filtered);

        if (abs(steeringValue - lastSteeringValue) > STEERING_CHANGE_THRESHOLD) {
            sendSteering(steeringValue);
            lastSteeringValue = steeringValue;
        }
    }

    if (now - lastHeartbeatTime > HEARTBEAT_MS) {
        sendHeartbeat();
    }

    handleThrottleBrake((float)ch2Filtered, dt);

#if ENABLE_GEARSHIFT
    if (!rcLost) {
        processGearShift(ch3Filtered, rcLost);
    }
    updateGearServo(dt);
#endif

    delay(10);
}