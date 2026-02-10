#include <ServoEasing.hpp>

ServoEasing SERVO1;
ServoEasing SERVO2;
ServoEasing SERVO3;
ServoEasing SERVO4;
ServoEasing SERVO5;

// VELOCIDAD (Grados por segundo)
// 20 es muy lento y suave.
// 60 es una velocidad media.
// ¡NO uses números grandes como 2000 aquí!
int velocidadLenta = 25;
int espera         = 1500; // Pausa entre posiciones

void setup() {
  // Pines ESP32 e inicialización en 90 grados.
  // attach(pin, inicial) ayuda a evitar saltos bruscos.
  SERVO1.attach(23, 90);
  SERVO2.attach(33, 90);
  SERVO3.attach(32, 90);
  SERVO4.attach(25, 90);
  SERVO5.attach(22, 90);

  // Configuración de suavizado (Easing)
  // CUBIC ofrece el arranque y parada más suaves.
  SERVO1.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO2.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO3.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO4.setEasingType(EASE_CUBIC_IN_OUT);
  SERVO5.setEasingType(EASE_CUBIC_IN_OUT);

  // ----- INICIO: ASEGURAR POSICIÓN 90 -----
  // easeTo(angulo, velocidad) es una función bloqueante:
  // Espera a que el servo termine antes de pasar al siguiente.
  SERVO1.easeTo(90, velocidadLenta);
  SERVO2.easeTo(90, velocidadLenta);
  SERVO3.easeTo(90, velocidadLenta);
  SERVO4.easeTo(90, velocidadLenta);
  SERVO5.easeTo(90, velocidadLenta);

  delay(espera);
}

void loop() {

  // -------- SECUENCIA 1: IR A HOME (90) UNO POR UNO --------
  SERVO1.easeTo(90, velocidadLenta);
  SERVO2.easeTo(90, velocidadLenta);
  SERVO3.easeTo(90, velocidadLenta);
  SERVO4.easeTo(90, velocidadLenta);
  SERVO5.easeTo(90, velocidadLenta);

  delay(espera);

  // -------- SECUENCIA 2: IR A TRABAJO UNO POR UNO --------
  SERVO1.easeTo(40, velocidadLenta);
  SERVO2.easeTo(40, velocidadLenta);
  SERVO3.easeTo(95, velocidadLenta);
  SERVO4.easeTo(54, velocidadLenta);
  SERVO5.easeTo(5, velocidadLenta);

  delay(espera);
}
