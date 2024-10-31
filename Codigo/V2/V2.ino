#include <PS4Controller.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pines de conexión al controlador L298
const int motorLeftPin1 = 25;   // IN1 del L298
const int motorLeftPin2 = 26;   // IN2 del L298
const int motorRightPin1 = 33;  // IN3 del L298
const int motorRightPin2 = 32;  // IN4 del L298
const int enableLeftPin = 0;    // ENA del L298
const int enableRightPin = 0;   // ENB del L298

// Pin para el LED (cambiar si es necesario)
const int ledPin = 23;  // Pin para el LED

void setup() {
  Serial.begin(115200);

  // Configurar pines de motores como salida
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(enableLeftPin, OUTPUT);
  pinMode(enableRightPin, OUTPUT);

  // Configurar el pin del LED
  pinMode(ledPin, OUTPUT);

  PS4.setLed(255, 0, 100);  // Establecer color rosa (RGB)

  // Iniciar la conexión Bluetooth en Core 0
  PS4.begin("e0:5a:1b:d0:5c:2a");  // Reemplaza con la dirección MAC de tu ESP32
  Serial.println("Conectando al controlador PS4...");
}

float angle = 0;
float magnitude = 0;

float mapFloat(long x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void loop() {
  if (PS4.isConnected() == false) {
    return;
  }

  // Serial.println("Joystick PS4 conectado.");

  int leftY = PS4.LStickY();
  int leftX = PS4.LStickX();
  int rightX = PS4.RStickX();
  int rightY = PS4.RStickY();

  // Calcular angulo del stick
  angle = degrees(atan2f((float)leftY, (float)leftX));

  int16_t motorL = map(leftY, -128, 127, -255, 255) * (float)constrain(1.0 - angle, 0.0, 1.0);
  int16_t motorR = map(leftY, -128, 127, -255, 255);

  Serial.printf("%d\t%d\n", motorL, motorR);
}