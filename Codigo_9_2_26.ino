#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Variables para rastrear la posición actual de cada servo
int pos1 = 90, pos2 = 90, pos3 = 90, pos4 = 90, pos5 = 90;

// Tiempo total para cada movimiento de articulación (2 segundos)
int tiempoMov = 2000;
int espera    = 1500;

// Función personalizada basada en la lógica de "8 pasos" proporcionada por el usuario
void moverSuave(ServoEasing &servo, int inicio, int fin, int tiempoTotal) {
  int pasos = 8;
  float delta = (float)(fin - inicio) / pasos;
  int tiempoPaso = tiempoTotal / pasos;
  float pos = (float)inicio;

  for (int i = 0; i < pasos; i++) {
    pos += delta;
    // Usamos startEaseToD para asegurar que tiempoPaso se trate como milisegundos
    servo.startEaseToD(pos, tiempoPaso);
    synchronizeAllServosStartAndWaitForAllServosToStop();
  }
}

void setup() {
  // Pines ESP32 e inicialización
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- INICIO: ASEGURAR POSICIÓN 90 -----
  moverSuave(SERVO1, pos1, 90, tiempoMov); pos1 = 90;
  moverSuave(SERVO2, pos2, 90, tiempoMov); pos2 = 90;
  moverSuave(SERVO3, pos3, 90, tiempoMov); pos3 = 90;
  moverSuave(SERVO4, pos4, 90, tiempoMov); pos4 = 90;
  moverSuave(SERVO5, pos5, 90, tiempoMov); pos5 = 90;

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  moverSuave(SERVO1, pos1, 90, tiempoMov); pos1 = 90;
  moverSuave(SERVO2, pos2, 90, tiempoMov); pos2 = 90;
  moverSuave(SERVO3, pos3, 90, tiempoMov); pos3 = 90;
  moverSuave(SERVO4, pos4, 90, tiempoMov); pos4 = 90;
  moverSuave(SERVO5, pos5, 90, tiempoMov); pos5 = 90;

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  moverSuave(SERVO1, pos1, 40, tiempoMov); pos1 = 40;
  moverSuave(SERVO2, pos2, 40, tiempoMov); pos2 = 40;
  moverSuave(SERVO3, pos3, 95, tiempoMov); pos3 = 95;
  moverSuave(SERVO4, pos4, 54, tiempoMov); pos4 = 54;
  moverSuave(SERVO5, pos5, 5, tiempoMov);  pos5 = 5;

  delay(espera);
}
