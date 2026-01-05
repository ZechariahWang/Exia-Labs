#include <Servo.h>

// define channel pins
#define CH1_PIN 2
#define CH2_PIN 3
#define THROTTLE_PIN 44
#define BRAKE_PIN 45
#define LED_PIN 4

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
#define MEDIAN_FILTER_SIZE 5

Servo throttle;
Servo brake;

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

    // attach servos to their pins, write init vals
    throttle.attach(THROTTLE_PIN, 1000, 2000);
    brake.attach(BRAKE_PIN, 1000, 2000);

    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_NEUTRAL);

    Serial.begin(115200);
    while (true) {
        if (Serial.available() && Serial.read() == 'C') {
            break;
        }
        delay(100);
    }

    // prefill buffer values
    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++) {
        ch1Buffer[i] = RC_CENTER;
        ch2Buffer[i] = RC_CENTER;
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

void updateInputBuffer(int ch1Raw, int ch2Raw) {
    if (isValidRC(ch1Raw)) {
        ch1Buffer[bufferIdx] = ch1Raw;
    }
    if (isValidRC(ch2Raw)) {
        ch2Buffer[bufferIdx] = ch2Raw;
    }
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

    updateInputBuffer(ch1Raw, ch2Raw);

    int ch1Filtered = getFilteredCh1();
    int ch2Filtered = getFilteredCh2();

    bool ch1Valid = isValidRC(ch1Raw);
    bool ch2Valid = isValidRC(ch2Raw);

    if (ch1Valid || ch2Valid) {
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

    delay(10);
}