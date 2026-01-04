#include <Servo.h>

// Receiver pins
#define CH1_PIN 8     // Steering
#define CH2_PIN 9     // Throttle / Brake

// Servo pins
#define STEERING_PIN 11
#define THROTTLE_PIN 12
#define BRAKE_PIN 13

// RC tuning
#define RC_CENTER 1344
#define RC_DEADZONE 60

// Servo positions (CALIBRATE THESE)
#define STEERING_CENTER 60
#define THROTTLE_NEUTRAL 60
#define THROTTLE_MAX 120
#define BRAKE_NEUTRAL 60
#define BRAKE_MAX 120

Servo steering;
Servo throttle;
Servo brake;

void setup() {
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);

  steering.attach(STEERING_PIN);
  throttle.attach(THROTTLE_PIN);
  brake.attach(BRAKE_PIN);

  steering.write(STEERING_CENTER);
  throttle.write(THROTTLE_NEUTRAL);
  brake.write(BRAKE_NEUTRAL);

  Serial.begin(115200);
}

void loop() {
  int ch1 = pulseIn(CH1_PIN, HIGH, 25000);
  int ch2 = pulseIn(CH2_PIN, HIGH, 25000);

  if (ch1 < 900 || ch1 > 2100 || ch2 < 900 || ch2 > 2100) {
    steering.write(STEERING_CENTER);
    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_NEUTRAL);
    return;
  }

  int steer_angle;
  if (abs(ch1 - RC_CENTER) < RC_DEADZONE) {
    steer_angle = STEERING_CENTER;
  } else {
    steer_angle = map(ch1, 1000, 2000, 20, 160);
  }
  steering.write(steer_angle);


  int delta = ch2 - RC_CENTER;
  if (abs(delta) < RC_DEADZONE) {
    // Neutral
    throttle.write(THROTTLE_NEUTRAL);
    brake.write(BRAKE_NEUTRAL);
  }
  else if (delta > 0) {
    int t = map(delta, RC_DEADZONE, 500, THROTTLE_NEUTRAL, THROTTLE_MAX);
    throttle.write(constrain(t, THROTTLE_NEUTRAL, THROTTLE_MAX));
    brake.write(BRAKE_NEUTRAL);
  }
  else {
    int b = map(-delta, RC_DEADZONE, 500, BRAKE_NEUTRAL, BRAKE_MAX);
    brake.write(constrain(b, BRAKE_NEUTRAL, BRAKE_MAX));
    throttle.write(THROTTLE_NEUTRAL);
  }

  delay(20);
}
