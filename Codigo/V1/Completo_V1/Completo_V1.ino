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

// Variables para el control de motores y temporización
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
unsigned long actionStartTime = 0;  // Tiempo de inicio para acciones temporizadas

// Definición de estados
enum State {
  IDLE,
  FORWARD,
  BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
  TURN_45_RIGHT,
  TURN_45_LEFT,
  STOP,
  SPIN_RIGHT,
  SPIN_LEFT,
  CRAZY_LEFT_SPIN
};
State currentState = IDLE;

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

  // Iniciar la conexión Bluetooth en Core 0
  PS4.begin("e0:5a:1b:d0:5c:2a");  // Reemplaza con la dirección MAC de tu ESP32
  Serial.println("Conectando al controlador PS4...");

  // Crear tareas en diferentes núcleos
  xTaskCreatePinnedToCore(readBluetoothCommands, "Bluetooth Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motor Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // El loop principal no hace nada en este caso
}

// Tarea para manejar la conexión Bluetooth y leer comandos
void readBluetoothCommands(void* parameter) {
  while (true) {
    if (PS4.isConnected()) {
      // Encender el LED en rosa al conectarse
      PS4.setLed(255, 20, 147);  // Establecer color rosa (RGB)
      Serial.println("Joystick PS4 conectado.");

      // Verificar si se ha presionado el botón R1 para detener los motores
      if (PS4.R1()) {
        currentState = STOP;
      } else if (PS4.Circle()) {
        currentState = SPIN_RIGHT;
      } else if (PS4.Square()) {
        currentState = CRAZY_LEFT_SPIN;
      } else if (PS4.Triangle() && PS4.Right()) {
        currentState = TURN_45_RIGHT;
        actionStartTime = millis();
      } else if (PS4.Triangle() && PS4.Left()) {
        currentState = TURN_45_LEFT;
        actionStartTime = millis();
      } else if (currentState == IDLE) {
        if (PS4.Up()) {
          currentState = FORWARD;
          actionStartTime = millis();
        } else if (PS4.Down()) {
          currentState = BACKWARD;
          actionStartTime = millis();
        } else if (PS4.Right()) {
          currentState = TURN_RIGHT;
          actionStartTime = millis();
        } else if (PS4.Left()) {
          currentState = TURN_LEFT;
          actionStartTime = millis();
        } else {
          // Lectura de sticks con margen de error
          int leftY = PS4.LStickY();
          int leftX = PS4.LStickX();
          int rightX = PS4.RStickX();
          int rightY = PS4.RStickY();

          // float angle = tanf((float)leftY / (float)leftX);
          // Serial.println(angle);

          // Aplicar el margen de error de -19 a 19
          if (abs(leftY) < 20) leftY = 0;
          if (abs(rightX) < 20) rightX = 0;
          if (abs(rightY) < 20) rightY = 0;
          if (abs(leftX) < 20) leftX = 0;
          // Mapear los valores de los sticks a velocidades de los motores
          leftMotorSpeed = constrain(map(leftY + rightX, -128, 128, -255, 255), -255, 255);
          rightMotorSpeed = constrain(map(leftY - rightX, -128, 128, -255, 255), -255, 255);
        }
      }

      // Si el botón Círculo se ha soltado y estamos en SPIN_RIGHT, detener el giro
      if (!PS4.Circle() && currentState == SPIN_RIGHT) {
        currentState = STOP;
      }

      // Si el botón Cuadrado se ha soltado y estamos en CRAZY_LEFT_SPIN, detener el giro
      if (!PS4.Square() && currentState == CRAZY_LEFT_SPIN) {
        currentState = STOP;
      }
    }
    vTaskDelay(1);  // Pausa de 10 ms para no saturar
  }
}

// Tarea para controlar los motores
void motorControlTask(void* parameter) {
  while (true) {
    switch (currentState) {
      case IDLE:
        // Control normal de los motores en el estado IDLE
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, leftMotorSpeed);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, rightMotorSpeed);
        // Serial.print("Estado: IDLE, Velocidad izquierda: ");
        // Serial.print(leftMotorSpeed);
        // Serial.print(", Velocidad derecha: ");
        // Serial.println(rightMotorSpeed);
        break;

      case FORWARD:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200);
        // Serial.println("Estado: FORWARD");

        if (millis() - actionStartTime >= 5000) {
          currentState = STOP;
        }
        break;

      case BACKWARD:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200);
        // Serial.println("Estado: BACKWARD");

        if (millis() - actionStartTime >= 5000) {
          currentState = STOP;
        }
        break;

      case TURN_RIGHT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200);
        // Serial.println("Estado: TURN_RIGHT");

        if (millis() - actionStartTime >= 1000) {
          currentState = STOP;
        }
        break;

      case TURN_LEFT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200);
        // Serial.println("Estado: TURN_LEFT");

        if (millis() - actionStartTime >= 1000) {
          currentState = STOP;
        }
        break;

      case TURN_45_RIGHT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200);
        // Serial.println("Estado: TURN_45_RIGHT");

        if (millis() - actionStartTime >= 500) {
          currentState = STOP;
        }
        break;

      case TURN_45_LEFT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200);
        // Serial.println("Estado: TURN_45_LEFT");

        if (millis() - actionStartTime >= 500) {
          currentState = STOP;
        }
        break;

      case SPIN_RIGHT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200);
        // Serial.println("Estado: SPIN_RIGHT");
        break;

      case SPIN_LEFT:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200);
        // Serial.println("Estado: SPIN_LEFT");
        break;

      case CRAZY_LEFT_SPIN:
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -255);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 255);
        // Serial.println("Estado: CRAZY_LEFT_SPIN");
        break;

      case STOP:
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        stopMotors();
        Serial.println("Estado: STOP");
        currentState = IDLE;
        break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Pausa de 10 ms para no saturar
  }
}

// Función para controlar un motor
void controlMotor(int pin1, int pin2, int enablePin, int speed) {
  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else if (speed < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }

  // Ajustar velocidad PWM
  analogWrite(enablePin, abs(speed));
}

// Función para detener todos los motores
void stopMotors() {
  controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 0);
  controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 0);
}
