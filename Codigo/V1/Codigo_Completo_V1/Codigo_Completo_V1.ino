//FALTA CAMBIAR TODOS LOS PINES

#include "arduino.h"
#include <PS4Controller.h>

//Pines
#define LED_Bluetooth 1

// Motor Izquierdo
#define AIN_2 3
#define AIN_1 4
#define PWM_A 6

// Motor Derecho
#define BIN_2 2
#define BIN_1 7
#define PWM_B 5


//Nucleos
TaskHandle_t Core0Task;
TaskHandle_t Core1Task;

void Setup_Nucleos();
void Setup_Ps4();



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Setup_Nucleos();
  Setup_Ps4();
  pinMode(LED_Bluetooth, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}



void Nucleo_Lectura_Ps(void* parameter) {
  uint8_t R = 229;
  uint8_t G = 13;
  uint8_t B = 13;

  for (;;) {
    if (PS4.isConnected()) {
      PS4.setLed(R, G, B);
      PS4.sendToController();
      digitalWrite(LED_Bluetooth, HIGH);
    } else {
      digitalWrite(LED_Bluetooth, LOW);
    }
  }
}

void Nucleo_Codigo(void* parameter) {
  uint8_t Maquina_P;
  enum Estados_P {
    Principal,
    ATRAS_Emergencia,
  };

  for (;;) {
    switch (Maquina_P) {
      case Principal:
        if (PS4.DownLeft() == LOW) {

          digitalWrite(AIN_1, HIGH);
          digitalWrite(AIN_2, LOW);

          digitalWrite(BIN_1, LOW);
          digitalWrite(BIN_2, HIGH);
          Maquina_P = ATRAS_Emergencia;
        }
        break;

      case ATRAS_Emergencia:
    }
  }
}

void Setup_Nucleos() {
  xTaskCreatePinnedToCore(
    Nucleo_Lectura_Ps,
    "Core 0 task",
    10000,
    NULL,
    1,
    &Core0Task,
    0);
  xTaskCreatePinnedToCore(
    Nucleo_Codigo,
    "Core 1 task",
    10000,
    NULL,
    1,
    &Core1Task,
    1);
}

void Setup_Ps4() {
  PS4.begin();
  Serial.println("Ready.");
}