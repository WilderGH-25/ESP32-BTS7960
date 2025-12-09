/*
    Programa principal para controlar un motor con el driver BTS7960
    usando un ESP32 y un encoder en cuadratura.

    Este programa NO usa delay(), sino millis(), para que el ESP32
    siga funcionando sin pausas.
 
*/

#include <Arduino.h>       // Importa funciones básicas de Arduino
#include "BTS7960.h"       // Importa la librería que controla el driver y el encoder

// --------------------------- Definición de pines ---------------------------

// Define el pin de habilitación del lado derecho del driver BTS7960
#define PIN_R_EN 5

// Define el pin de habilitación del lado izquierdo
#define PIN_L_EN 4

// Define el pin PWM para mover el motor hacia adelante (sentido horario)
#define PIN_RPWM 23

// Define el pin PWM para mover el motor hacia atrás (sentido antihorario)
#define PIN_LPWM 22

// Pines del encoder (detectan el movimiento y sentido del motor)
#define ENC_A 14      // Canal A del encoder
#define ENC_B 27      // Canal B del encoder

// Configuración del PWM
#define PWM_FREQ 20000    // Frecuencia del PWM: 20 kHz (evita ruidos)
#define PWM_RES 8         // Resolución del PWM: 8 bits (0–255)

// Parámetros del motor + encoder
const int PPR_MOTOR = 11;     // Pulsos por vuelta del motor
const int GEAR_RATIO = 34;    // Relación de reducción (34:1)

// Se crea un objeto llamado "motor" que controlará todo
BTS7960 motor(
  PIN_R_EN, PIN_L_EN,        // Pines de habilitación
  PIN_RPWM, PIN_LPWM,        // Pines PWM
  PWM_FREQ, PWM_RES,         // Configuración PWM
  ENC_A, ENC_B,              // Pines del encoder
  PPR_MOTOR, GEAR_RATIO      // Datos del motor
);

// Variable que guarda el último tiempo registrado
unsigned long tiempoAnterior = 0;

// Variable para controlar los pasos del programa (máquina de estados)
int estado = 0;

void setup() {
  Serial.begin(115200);     // Inicia comunicación serial
  delay(50);                // Pequeña pausa para estabilizar la comunicación

  motor.iniciar();          // Inicializa el driver BTS7960
  motor.iniciarEncoder();   // Inicializa el encoder

  Serial.println("Sistema BTS7960 listo.");
  Serial.println("------------------------------------------");

  tiempoAnterior = millis();   // Guarda el tiempo inicial
}

void loop() {

  unsigned long ahora = millis();   // Guarda el tiempo actual

  switch (estado) {

    case 0:  // Estado 0: motor hacia adelante
      motor.avanzar(255);               // Velocidad máxima
      if (ahora - tiempoAnterior >= 1500) {   // Espera 1.5 segundos sin delay()
        estado = 1;                          // Cambia al siguiente estado
        tiempoAnterior = ahora;              // Actualiza el tiempo
      }
      break;

    case 1:  // Estado 1: detener motor
      motor.detenerMotor();                 // Apaga ambos PWM
      if (ahora - tiempoAnterior >= 1500) { // Espera 1.5 s
        estado = 2;                         // Siguiente estado
        tiempoAnterior = ahora;
      }
      break;

    case 2:  // Estado 2: motor hacia atrás
      motor.retroceder(200);                // Velocidad 200/255
      if (ahora - tiempoAnterior >= 3000) { // Espera 3 s
        estado = 3;                         // Siguiente estado
        tiempoAnterior = ahora;
      }
      break;

    case 3:  // Estado 3: frenado activo
      motor.frenar();                       // Aplica fuerza de frenado
      if (ahora - tiempoAnterior >= 500) {  // Espera 0.5 s
        estado = 4;
        tiempoAnterior = ahora;
      }
      break;

    case 4:  // Estado 4: motor totalmente detenido
      motor.detenerMotor();
      if (ahora - tiempoAnterior >= 1000) {  // Espera 1 s
        estado = 0;                          // Reinicia ciclo
        tiempoAnterior = ahora;
      }
      break;
  }

  // --------------------------
  // IMPRESIÓN CADA 1 SEGUNDO
  // --------------------------

  static unsigned long tPrint = 0;       // Guarda el tiempo de la última impresión

  if (millis() - tPrint >= 1000) {       // Si pasó 1 segundo
    tPrint = millis();                   // Actualiza el tiempo

    int PPS = motor.getPPS();            // Pulsos por segundo
    float RPM = motor.getRPM();          // Revoluciones por minuto
    int sentido = motor.getSentido();    // Sentido del giro

    Serial.println("PPS      RPM      SENTIDO");
    Serial.println("----------------------------------------");

    Serial.print(PPS);
    Serial.print("      ");
    Serial.print(RPM, 2);
    Serial.print("      ");

    if (sentido == 1) Serial.println("Horario");
    else if (sentido == -1) Serial.println("Antihorario");
    else Serial.println("Detenido");

    Serial.println();                    // Línea en blanco
  }
}

