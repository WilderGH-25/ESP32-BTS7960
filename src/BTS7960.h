#ifndef BTS7960_H      // Evita que el archivo se incluya dos veces
#define BTS7960_H

#include <Arduino.h>   // Funciones básicas del ESP32

/*
   Esta clase controla el driver BTS7960 y también lee un encoder en cuadratura.

   Permite obtener:
   - PPS (pulsos por segundo)
   - RPM (velocidad)
   - Sentido de giro
*/

class BTS7960 {

  private:

    // ---------------- Pines del driver ----------------
    int pinR_EN;       // Pin para habilitar lado derecho del driver
    int pinL_EN;       // Pin para habilitar lado izquierdo
    int pinRPWM;       // PWM para girar en sentido horario
    int pinLPWM;       // PWM para girar en sentido antihorario

    // ---------------- Pines del encoder ----------------
    int pinEncoderA;   // Canal A del encoder
    int pinEncoderB;   // Canal B del encoder

    // ---------------- PWM configuraciones --------------
    uint32_t freq;      // Frecuencia del PWM
    uint8_t resolution; // Resolución del PWM (bits)

    // ---------------- Datos del motor -------------------
    int PPR_motor;      // Pulsos por vuelta del motor
    int gearRatio;      // Relación de reducción
    long countsPerEje;  // Pulsos por vuelta de la rueda final

    // -------- Variables actualizadas con interrupciones --------
    volatile long pulsos;  // Cuenta total de pulsos
    volatile int sentido;  // Sentido del giro (1, -1, 0)

    // -------- Variables para calcular PPS y RPM --------
    unsigned long t_lastPPS;       // Último tiempo calculado
    long pulsos_prev_for_pps;      // Pulsos guardados del cálculo anterior
    int lastPPS;                   // Último PPS calculado
    float lastRPM;                 // Último RPM calculado

    // Anti rebote (si se quiere usar)
    unsigned int debounceMicros;

  public:

    // -------- Constructor --------
    BTS7960(
      int R_EN, int L_EN,          // Pines de habilitación
      int RPWM, int LPWM,          // Pines PWM
      uint32_t freqPWM,            // Frecuencia PWM
      uint8_t resPWM,              // Resolución PWM
      int encA, int encB,          // Pines del encoder
      int pprMotor,                // Pulsos por vuelta
      int gearR                    // Relación de engranajes
    );

    // -------- Inicialización --------
    void iniciar();           // Configura pines del driver
    void iniciarEncoder();    // Configura pines + interrupciones del encoder

    // -------- Movimientos del motor --------
    void avanzar(int vel);       // Motor hacia adelante
    void retroceder(int vel);    // Motor hacia atrás
    void frenar();               // Frenado activo
    void detenerMotor();         // Apaga el motor

    // -------- Interrupciones --------
    static void IRAM_ATTR isrEncoderA(void *arg);
    static void IRAM_ATTR isrEncoderB(void *arg);

    // -------- Lecturas --------
    int getPPS();        // Obtiene pulsos por segundo
    float getRPM();      // Calcula RPM
    int getSentido();    // Devuelve sentido del giro
};

#endif

