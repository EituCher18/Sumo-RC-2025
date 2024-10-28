#include <PS4Controller.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// Pines de conexión al controlador L298
const int motorLeftPin1 = 15;  // IN1 del L298
const int motorLeftPin2 = 2;   // IN2 del L298
const int motorRightPin1 = 4;  // IN3 del L298
const int motorRightPin2 = 16; // IN4 del L298
const int enableLeftPin = 13;  // ENA del L298
const int enableRightPin = 12; // ENB del L298


////Intervalos
#define Intervalo_Adelante 5000
#define Intervalo_Atras 5000
#define Intervalo_45_Grados_Derecha 500
#define Intervalo_45_Grados_Izquierda 500
#define Intervalo_Derecha 1000
#define Intervalo_Izquierda 1000

// Pin para el LED (cambiar si es necesario)
const int ledPin = 23; // Pin para el LED

// Variables para el control de motores y temporización
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
unsigned long actionStartTime = 0; // Tiempo de inicio para acciones temporizadas

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
  SPIN_LEFT
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
  setColor(255, 20, 147); // Establecer color rosa (RGB)

  // Iniciar la conexión Bluetooth en Core 0
  PS4.begin("XX:XX:XX:XX:XX:XX");  // Reemplaza con la dirección MAC de tu ESP32
  Serial.println("Conectando al controlador PS4...");

  // Crear tareas en diferentes núcleos
  xTaskCreatePinnedToCore(readBluetoothCommands, "Bluetooth Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorControlTask, "Motor Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // El loop principal no hace nada en este caso
}

// Tarea para manejar la conexión Bluetooth y leer comandos
void readBluetoothCommands(void * parameter) {
  while (true) {
    if (PS4.isConnected()) {
      // Encender el LED en rosa al conectarse
      setColor(255, 20, 147); // Establecer color rosa (RGB)
      
      // Verificar si se ha presionado el botón R1 para detener los motores
      if (PS4.R1()) {
        currentState = STOP; // Cambiar al estado STOP inmediatamente
      } else if (PS4.Circle()) {
        currentState = SPIN_RIGHT; // Cambiar al estado SPIN_RIGHT si se presiona Círculo
      } else if (PS4.Square()) {
        currentState = SPIN_LEFT; // Cambiar al estado SPIN_LEFT si se presiona Cuadrado
      } else if (PS4.Triangle() && PS4.Right()) {
        currentState = TURN_45_RIGHT; // Cambiar al estado TURN_45_RIGHT si se mantiene Triángulo y se toca Derecha
        actionStartTime = millis();   // Guardar el tiempo de inicio para el giro de 45 grados a la derecha
      } else if (PS4.Triangle() && PS4.Left()) {
        currentState = TURN_45_LEFT; // Cambiar al estado TURN_45_LEFT si se mantiene Triángulo y se toca Izquierda
        actionStartTime = millis();   // Guardar el tiempo de inicio para el giro de 45 grados a la izquierda
      } else if (currentState == IDLE) {
        if (PS4.Up()) {
          currentState = FORWARD;          // Cambiar al estado FORWARD
          actionStartTime = millis();       // Guardar el tiempo de inicio del movimiento hacia adelante
        } else if (PS4.Down()) {
          currentState = BACKWARD;          // Cambiar al estado BACKWARD
          actionStartTime = millis();       // Guardar el tiempo de inicio del movimiento hacia atrás
        } else if (PS4.Right()) {
          currentState = TURN_RIGHT;        // Cambiar al estado TURN_RIGHT
          actionStartTime = millis();       // Guardar el tiempo de inicio del giro a la derecha
        } else if (PS4.Left()) {
          currentState = TURN_LEFT;         // Cambiar al estado TURN_LEFT
          actionStartTime = millis();       // Guardar el tiempo de inicio del giro a la izquierda
        } else {
          // Control normal con joystick en el estado IDLE
          int leftY = PS4.LStickY();        // Joystick izquierdo en el eje Y
          int rightX = PS4.RStickX();       // Joystick derecho en el eje X

          // Calcular velocidad de motores
          leftMotorSpeed = constrain(map(leftY + rightX, -128, 128, -255, 255), -255, 255);
          rightMotorSpeed = constrain(map(leftY - rightX, -128, 128, -255, 255), -255, 255);
        }
      }
      
      // Si el botón Círculo se ha soltado y estamos en SPIN_RIGHT, detener el giro
      if (!PS4.Circle() && currentState == SPIN_RIGHT) {
        currentState = STOP;
      }

      // Si el botón Cuadrado se ha soltado y estamos en SPIN_LEFT, detener el giro
      if (!PS4.Square() && currentState == SPIN_LEFT) {
        currentState = STOP;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Pausa de 10 ms para no saturar
  }
}

// Tarea para controlar los motores
void motorControlTask(void * parameter) {
  while (true) {
    switch (currentState) {
      case IDLE:
        // Control normal de los motores en el estado IDLE
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, leftMotorSpeed);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, rightMotorSpeed);
        break;

      case FORWARD:
        // Configurar los motores para moverse hacia adelante en el estado FORWARD
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200);

        // Cambiar al estado STOP después de 5 segundos
        if (millis() - actionStartTime >= Intervalo_Adelante) {
          currentState = STOP;
        }
        break;

      case BACKWARD:
        // Configurar los motores para moverse hacia atrás en el estado BACKWARD
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200);
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200);

        // Cambiar al estado STOP después de 5 segundos
        if (millis() - actionStartTime >= Intervalo_Atras) {
          currentState = STOP;
        }
        break;

      case TURN_RIGHT:
        // Configurar los motores para girar a la derecha
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);  // Motor izquierdo hacia adelante
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200); // Motor derecho hacia atrás

        // Cambiar al estado STOP después de 1 segundo (ajusta el tiempo según pruebas)
        if (millis() - actionStartTime >= Intervalo_Derecha) {
          currentState = STOP;
        }
        break;

      case TURN_LEFT:
        // Configurar los motores para girar a la izquierda
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200); // Motor izquierdo hacia atrás
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200); // Motor derecho hacia adelante

        // Cambiar al estado STOP después de 1 segundo (ajusta el tiempo según pruebas)
        if (millis() - actionStartTime >= Intervalo_Izquierda) {
          currentState = STOP;
        }
        break;

      case TURN_45_RIGHT:
        // Configurar los motores para girar 45 grados a la derecha
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);  // Motor izquierdo hacia adelante
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200); // Motor derecho hacia atrás

        // Cambiar al estado STOP después de 0.5 segundos para un giro de 45 grados
        if (millis() - actionStartTime >= Intervalo_45_Grados_Derecha) {
          currentState = STOP;
        }
        break;

      case TURN_45_LEFT:
        // Configurar los motores para girar 45 grados a la izquierda
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200); // Motor izquierdo hacia atrás
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200); // Motor derecho hacia adelante

        // Cambiar al estado STOP después de 0.5 segundos para un giro de 45 grados
        if (millis() - actionStartTime >= Intervalo_45_Grados_Izquierda) {
          currentState = STOP;
        }
        break;

      case SPIN_RIGHT:
        // Configurar los motores para giro continuo a la derecha
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, 200);  // Motor izquierdo hacia adelante
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, -200); // Motor derecho hacia atrás
        break;

      case SPIN_LEFT:
        // Configurar los motores para giro continuo a la izquierda
        controlMotor(motorLeftPin1, motorLeftPin2, enableLeftPin, -200); // Motor izquierdo hacia atrás
        controlMotor(motorRightPin1, motorRightPin2, enableRightPin, 200); // Motor derecho hacia adelante
        break;

      case STOP:
        // Detener los motores y regresar al estado IDLE
        stopMotors();
        currentState = IDLE;
        break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Pausa de 10 ms para no saturar
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

// Función para establecer el color del LED
void setColor(int red, int green, int blue) {
  analogWrite(ledPin, red);   // Rojo
  delay(1);
  analogWrite(ledPin + 1, green); // Verde
  delay(1);
  analogWrite(ledPin + 2, blue); // Azul
}
