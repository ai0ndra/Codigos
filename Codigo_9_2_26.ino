#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// Variables para recordar la posición y asegurar movimientos fluidos
float posActual1 = 90, posActual2 = 90, posActual3 = 90, posActual4 = 90, posActual5 = 90;

// Tiempo total para cada movimiento (2000ms = 2 segundos)
int tiempoTotal = 2000;
int espera      = 1500;

// Función personalizada de 8 pasos que te dio el resultado que buscabas
void moverSuave(ServoEasing &servo, float inicio, float fin, int tiempo) {
  int pasos = 8;
  float delta = (fin - inicio) / pasos;
  int tiempoPaso = tiempo / pasos;
  float pos = inicio;

  for (int i = 0; i < pasos; i++) {
    pos += delta;
    // Usamos startEaseToD para garantizar que el tiempo sea en milisegundos
    servo.startEaseToD(pos, tiempoPaso);
    synchronizeAllServosStartAndWaitForAllServosToStop();
  }
}

void setup() {
  // Pines ESP32 e inicialización
  // Al hacer attach(pin, 90), el servo saltará a 90°. Es un salto de hardware inevitable.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  moverSuave(SERVO1, posActual1, 90, tiempoTotal); posActual1 = 90;
  moverSuave(SERVO2, posActual2, 90, tiempoTotal); posActual2 = 90;
  moverSuave(SERVO3, posActual3, 90, tiempoTotal); posActual3 = 90;
  moverSuave(SERVO4, posActual4, 90, tiempoTotal); posActual4 = 90;
  moverSuave(SERVO5, posActual5, 90, tiempoTotal); posActual5 = 90;

  delay(espera);

  // -------- SECUENCIA 2: MOVER UNO POR UNO A POSICIÓN DE TRABAJO --------
  moverSuave(SERVO1, posActual1, 40, tiempoTotal); posActual1 = 40;
  moverSuave(SERVO2, posActual2, 40, tiempoTotal); posActual2 = 40;
  moverSuave(SERVO3, posActual3, 95, tiempoTotal); posActual3 = 95;
  moverSuave(SERVO4, posActual4, 54, tiempoTotal); posActual4 = 54;
  moverSuave(SERVO5, posActual5, 5, tiempoTotal);  posActual5 = 5;

  delay(espera);
}
