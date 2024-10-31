#include <PS4Controller.h>

// Motor pins

constexpr uint8_t motor_left_forwards = 25;
constexpr uint8_t motor_left_backwards = 26;
constexpr uint8_t motor_right_forwards = 33;
constexpr uint8_t motor_right_backwards = 32;

// LED pin
const int led_pin = LED_BUILTIN;

void setup() {
  Serial.begin(115200);

  // Set up motor pins as outputs
  pinMode(motor_left_forwards, OUTPUT);
  pinMode(motor_left_backwards, OUTPUT);
  pinMode(motor_right_forwards, OUTPUT);
  pinMode(motor_right_backwards, OUTPUT);

  // Set up led pin as outputs
  pinMode(led_pin, OUTPUT);

  PS4.setLed(255, 0, 100);

  // Begin PS4 bluetooth connection
  PS4.begin("e0:5a:1b:d0:5c:2a");

  PS4.setLed(255, 0, 100);

  Serial.println("Conectando al controlador PS4...");
}

template<typename T>
T betterMap(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<typename T, typename O>
O linearInterpolation(T x, T* x_values, O* y_values, size_t len) {
  size_t start_index = len - 2;
  size_t end_index = len - 1;

  for (size_t i = 0; i < len - 1; i++) {
    if (input >= x_values[i]) {
      start_index = i;
      end_index = i;
    }
  }

  return betterMap(x, x_values[start_index], x_values[end_index], y_values[start_index], y_values[end_index]);
}

struct MotorSpeeds {
  uint8_t left_speed;
  bool left_forward;
  uint8_t right_speed;
  bool right_forward;
};

MotorSpeeds calculateMotorSpeeds(int8_t x, int8_t y) {
  constexpr float left_x_values = [0, PI/2, PI, 3*PI/2, 2*PI];
  constexpr float left_y_values = [1, 1, 0, -1, 1];
  constexpr float right_x_values = [0, PI/2, PI, 3*PI/2, 2*PI];
  constexpr float right_y_values = [0, 1, 1, -1, 1];

  float angle = atan2(y, x);
  float magnitude = constrain(sqrt(x * x + y * y) / 128.0, 0.0, 128.0);

  float left_speed = linearInterpolation(angle, left_x_values, left_y_values, sizeof(left_x_values) / sizeof(float));
  bool left_forward = true;
  if (left_speed < 0) {
    left_speed = -left_speed;
    left_forward = false;
  }

  float right_speed = linearInterpolation(angle, right_x_values, right_y_values, sizeof(right_x_values) / sizeof(float));
  bool right_forward = true;
  if (right_speed < 0) {
    right_speed = -right_speed;
    right_forward = false;
  }

  struct MotorSpeeds result = {
    left_speed,
    left_forward,
    right_speed,
    right_forward,
  };
  return result;
}

void loop() {
  if (PS4.isConnected() == false) {
    return;
  }

  // Serial.println("Joystick PS4 conectado.");

  int left_x = PS4.LStickX();
  int left_y = PS4.LStickY();
  int right_x = PS4.RStickX();
  int right_y = PS4.RStickY();

  // Calcular angulo del stick

  struct MotorSpeeds motor_speeds = calculateMotorSpeeds(left_x, left_y);

  if (motor_speeds.left_forward) {
    analogWrite(motor_left_forwards, motor_speeds.left_speed);
    analogWrite(motor_left_backwards, 0);
  } else {
    analogWrite(motor_left_forwards, 0);
    analogWrite(motor_left_backwards, motor_speeds.left_speed);
  }
  
  if (motor_speeds.right_forward) {
    analogWrite(motor_right_forwards, motor_speeds.right_speed);
    analogWrite(motor_right_backwards, 0);
  } else {
    analogWrite(motor_right_forwards, 0);
    analogWrite(motor_right_backwards, motor_speeds.right_speed);
  }
}