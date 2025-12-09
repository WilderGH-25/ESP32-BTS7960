#include "BTS7960.h"   // Importa la definición de la clase

/*
   Constructor de la clase BTS7960
   Recibe pines del driver, del encoder y parámetros del PWM.
*/

BTS7960::BTS7960(
  int R_EN, int L_EN, int RPWM, int LPWM,
  uint32_t freqPWM, uint8_t resPWM,
  int encA, int encB,
  int pprMotor, int gearR
) {
  // Guarda los pines del driver
  pinR_EN = R_EN;
  pinL_EN = L_EN;
  pinRPWM = RPWM;
  pinLPWM = LPWM;

  // Guarda los pines del encoder
  pinEncoderA = encA;
  pinEncoderB = encB;

  // Guarda la configuración del PWM
  freq = freqPWM;
  resolution = resPWM;

  // Guarda datos del motor
  PPR_motor = pprMotor;
  gearRatio = gearR;

  // Calcula cuántos pulsos genera una vuelta completa final
  countsPerEje = (long)PPR_motor * (long)gearRatio;

  // Inicializa contador de pulsos
  pulsos = 0;

  // Sentido del motor (inicialmente detenido)
  sentido = 0;

  // Inicializa variables para el cálculo de PPS
  t_lastPPS = millis();
  pulsos_prev_for_pps = 0;
  lastPPS = 0;

  // Inicializa RPM
  lastRPM = 0;

  // Tiempo mínimo entre interrupciones (anti rebote)
  debounceMicros = 120;
}

/*
   Inicializa el driver BTS7960
*/
void BTS7960::iniciar() {

  // Pines como salida
  pinMode(pinR_EN, OUTPUT);
  pinMode(pinL_EN, OUTPUT);

  // Activa el driver para permitir el movimiento del motor
  digitalWrite(pinR_EN, HIGH);
  digitalWrite(pinL_EN, HIGH);

  // Adjunta el PWM a los pines usando el método moderno de ESP32
  ledcAttach(pinRPWM, freq, resolution);   // PWM del lado derecho
  ledcAttach(pinLPWM, freq, resolution);   // PWM del lado izquierdo

  // Detiene el motor al iniciar
  detenerMotor();
}

/*
   Inicializa el encoder y activa las interrupciones
*/
void BTS7960::iniciarEncoder() {

  // Configura pines del encoder como entradas con pull-up
  pinMode(pinEncoderA, INPUT_PULLUP);
  pinMode(pinEncoderB, INPUT_PULLUP);

  // Interrupciones para ambos canales en flanco ascendente
  attachInterruptArg(pinEncoderA, isrEncoderA, this, RISING);
  attachInterruptArg(pinEncoderB, isrEncoderB, this, RISING);

  // Reinicia variables de cálculo
  t_lastPPS = millis();
  pulsos_prev_for_pps = pulsos;
  lastPPS = 0;
  lastRPM = 0;
}

/*
   Interrupción del canal A del encoder
*/
void IRAM_ATTR BTS7960::isrEncoderA(void *arg) {

  BTS7960 *m = (BTS7960*)arg;   // Convierte nuevamente a clase

  int A = digitalRead(m->pinEncoderA);   // Lee canal A
  int B = digitalRead(m->pinEncoderB);   // Lee canal B

  // Si A = B entonces va hacia adelante
  if (A == B) {
    m->pulsos++;                // Suma 1 pulso
    m->sentido = 1;             // Giro horario
  } else {                      // Si A != B
    m->pulsos--;                // Resta 1 pulso
    m->sentido = -1;            // Giro antihorario
  }
}

/*
   Interrupción del canal B del encoder
*/
void IRAM_ATTR BTS7960::isrEncoderB(void *arg) {

  BTS7960 *m = (BTS7960*)arg;   // Recupera puntero a la clase

  int A = digitalRead(m->pinEncoderA);
  int B = digitalRead(m->pinEncoderB);

  if (A != B) {     // Lógica del cuadrante
    m->pulsos++;
    m->sentido = 1;
  } else {
    m->pulsos--;
    m->sentido = -1;
  }
}

/*
   Movimiento hacia adelante (sentido horario)
*/
void BTS7960::avanzar(int vel) {

  int maxDuty = (1 << resolution) - 1;   // Calcula valor máximo del PWM

  // Asegura que vel esté dentro del rango permitido
  if (vel < 0) vel = 0;
  if (vel > maxDuty) vel = maxDuty;

  ledcWrite(pinLPWM, 0);   // Apaga el lado contrario
  ledcWrite(pinRPWM, vel); // Aplica velocidad hacia adelante
}

/*
   Movimiento hacia atrás (sentido antihorario)
*/
void BTS7960::retroceder(int vel) {

  int maxDuty = (1 << resolution) - 1;

  if (vel < 0) vel = 0;
  if (vel > maxDuty) vel = maxDuty;

  ledcWrite(pinRPWM, 0);   // Apaga el lado contrario
  ledcWrite(pinLPWM, vel); // Aplica velocidad hacia atrás
}

/*
   Frenado activo
*/
void BTS7960::frenar() {
  int maxDuty = (1 << resolution) - 1;

  ledcWrite(pinRPWM, maxDuty);   // Activa ambos lados
  ledcWrite(pinLPWM, maxDuty);   // Esto produce frenado
  sentido = 0;
}

/*
   Motor detenido (sin fuerza)
*/
void BTS7960::detenerMotor() {
  ledcWrite(pinRPWM, 0);
  ledcWrite(pinLPWM, 0);
  sentido = 0;
}

/*
   Calcula PPS (pulsos por segundo)
*/
int BTS7960::getPPS() {

  unsigned long now = millis();   // Tiempo actual

  // Si ha pasado 1 segundo
  if (now - t_lastPPS >= 1000) {

    noInterrupts();               // Protege la lectura de pulsos
    long current = pulsos;        // Captura actual
    interrupts();                 // Libera interrupciones

    long delta = current - pulsos_prev_for_pps;   // Diferencia de pulsos

    pulsos_prev_for_pps = current;                // Guarda nuevo valor
    t_lastPPS = now;                              // Actualiza tiempo

    if (delta < 0) delta = -delta;                // Solo magnitud

    lastPPS = delta;                              // Actualiza PPS
  }

  return lastPPS;                 // Devuelve PPS
}

/*
   Convierte PPS a RPM
*/
float BTS7960::getRPM() {

  int pps = getPPS();              // Obtiene pulsos por segundo

  if (countsPerEje <= 0) return 0; // Evita division por cero

  // Fórmula: RPM = (PPS / pulsos_por_vuelta) * 60
  lastRPM = ((float)pps / (float)countsPerEje) * 60.0f;

  return lastRPM;
}

/*
   Devuelve sentido del motor:
   1 = horario
   -1 = antihorario
   0 = detenido
*/
int BTS7960::getSentido() {
  return sentido;
}

