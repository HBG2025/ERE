#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int C1 = 32; // Entrada de la señal A del encoder.
const int C2 = 33; // Entrada de la señal B del encoder.
const int PWM1 = 16;
const int PWM2 = 17;
const int MOTOR0 = 18; // IZQUIERDA (propulsión, sin encoder)
const int MOTOR1 = 19; // DERECHA (propulsión, sin encoder)
int led = 2;

volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

unsigned long lastTime = 0;      // Tiempo anterior (cálculo de P)
unsigned long sampleTime = 10;   // ms para P (posición)

double P = 0;        // Posición (°) del motor con encoder → se envía como >delta:
double R = 35020;    // PPR*reductor*4
double d = 0.0;      // Referencia de posición (°) por serial (double soporta decimales)
double pwm = 0;
int pwm0 = 0;

// --- Telemetría no intrusiva ---
static uint32_t last_tx = 0;     // marca de tiempo para envío periódico
volatile float imu_x_deg = 0.0f; // BNO055 X (°) → >imu:

// --- NUEVO: bandera de “ya hay referencia” ---
static bool has_ref = false;

// --- Parser de un número con signo y decimales ---
static bool readNumberFromSerial(double &outValue) {
  static char buf[24];
  static uint8_t idx = 0;
  bool has_dot = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      if (idx == 0) return false;
      char *endptr = nullptr;
      double v = strtod(buf, &endptr);
      idx = 0;
      if (endptr != buf) { outValue = v; return true; }
      else return false;
    }

    if (idx < sizeof(buf) - 1) {
      if ((c >= '0' && c <= '9') ||
          ((c == '-' || c == '+') && idx == 0) ||
          (c == '.' && !has_dot)) {
        if (c == '.') has_dot = true;
        buf[idx++] = c;
      } else {
        // ignora caracteres no válidos
      }
    } else {
      idx = 0; // overflow: resetea
    }
  }
  return false;
}

// Encoder precisión cuádruple.
void encoderReading(void)
{
  ant = act;

  if (digitalRead(C1)) bitSet(act,1); else bitClear(act,1);
  if (digitalRead(C2)) bitSet(act,0); else bitClear(act,0);

  if (n < R) {
    if (ant == 2 && act == 0) n++;
    if (ant == 0 && act == 1) n++;
    if (ant == 3 && act == 2) n++;
    if (ant == 1 && act == 3) n++;
  } else { n = 0; }

  if (n > -R) {
    if (ant == 1 && act == 0) n--;
    if (ant == 3 && act == 1) n--;
    if (ant == 0 && act == 2) n--;
    if (ant == 2 && act == 3) n--;
  } else { n = 0; }
}

void setup(void) 
{
  Serial.begin(921600);
  Serial.setTimeout(10); // evitar bloqueos en timeouts de lectura

  if(!bno.begin())
  {
    Serial.println("No BNO055 detected!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(led, OUTPUT);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(MOTOR0, OUTPUT);
  pinMode(MOTOR1, OUTPUT);

  // --- NUEVO: motores en cero al arrancar ---
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(MOTOR0, 0);
  analogWrite(MOTOR1, 0);

  attachInterrupt(digitalPinToInterrupt(C1), encoderReading, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoderReading, CHANGE);

  Serial.println(">status:WAITING_FIRST_REF"); // info de arranque
}

void loop(void) 
{
  // --- Leer número por serial y actualizar d ---
  double val;
  if (readNumberFromSerial(val)) {
    d = val;
    if (!has_ref) {
      has_ref = true;                       // --- NUEVO ---
      Serial.println(">status:REF_ACQUIRED");
    }
    Serial.print(">ref:"); Serial.println(d, 2);
  }

  // 1) IMU (no bloqueante) → guardar X para telemetría
  sensors_event_t event; 
  if (bno.getEvent(&event)) {
    imu_x_deg = event.orientation.x;   // Se enviará como >imu:
  }

  // 2) LED “alive”
  digitalWrite(led, HIGH);

  // 3) Actualizar P (cada sampleTime ms), SIN imprimir aquí
  if (millis() - lastTime >= sampleTime || lastTime == 0)
  {
    lastTime = millis();
    P = (n * 360.0) / R;   // Se enviará como >delta:
  }

  // 4) Control SOLO si ya hubo primera referencia
  if (has_ref) {
    // *** TU LÓGICA DE DIRECCIÓN Y PROPULSIÓN SE MANTIENE ***
    //    VALORES NEGATIVOS GIRA DERECHA, VALORES POSITIVOS GIRA IZQUIERDA
    if (d <= 50.0 && d >= -45.0) {
      if (P > d){
        analogWrite(PWM1, 0); // retrocede o derecha
        analogWrite(PWM2, 35);
        delay(40);
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);

        if (P >= -2 && P <= 2) { 
          analogWrite(MOTOR1, 64);
          analogWrite(MOTOR0, 64);
        }
        if (P < -2) { 
          analogWrite(MOTOR1, 32);
          analogWrite(MOTOR0, 64);
        }
        if (P > 2) { 
          analogWrite(MOTOR1, 64);
          analogWrite(MOTOR0, 32);
        }
      }
      else if (P <= d){
        if (P > d - 0.7 ){
          analogWrite(PWM1, 0); // para
          analogWrite(PWM2, 0);
        } else {
          analogWrite(PWM1, 35);  // avance o izquierda
          analogWrite(PWM2, 0);
          delay(40);
          analogWrite(PWM1, 0);
          analogWrite(PWM2, 0);
        }

        if (P >= -2 && P <= 2) { 
          analogWrite(MOTOR1, 64);
          analogWrite(MOTOR0, 64);
        }
        if (P < -2) { 
          analogWrite(MOTOR1, 32);
          analogWrite(MOTOR0, 64);
        }
        if (P > 2) { 
          analogWrite(MOTOR1, 64);
          analogWrite(MOTOR0, 32);
        }
      }
      else {
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);
      }
    } else {
      // fuera de rango permitido → seguridad: todo a cero
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);
      analogWrite(MOTOR0, 0);
      analogWrite(MOTOR1, 0);
    }
  } else {
    // --- NUEVO: sin referencia aún → mantener todo quieto
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    analogWrite(MOTOR0, 0);
    analogWrite(MOTOR1, 0);
  }

  // 5) Telemetría ligera cada ~200 ms
  static uint32_t last_dbg = 0;
  uint32_t now = millis();
  if (now - last_dbg >= 200) {
    last_dbg = now;
    Serial.print(">omega1:"); Serial.println(P, 3);
    Serial.print(">imu:");    Serial.println(imu_x_deg, 3);
    if (!has_ref) Serial.println(">status:WAITING");
  }
}
