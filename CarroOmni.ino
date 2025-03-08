#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>

class SimplePID {
  private:
    float kp, kd, ki, umax, vmin; 
    float eprev, eintegral; 

  public:
    // Constructor
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), vmin(15.0){}
    // Configurar parámetros
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn, float vminIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; vmin = vminIn;
    }
    // Calcular la señal de control
    void evalu(int value, int target, float deltaT, int &pwm) {
      float e = (target - value) * ((float) fabs(target) > vmin);
      float dedt = (e - eprev) / deltaT * ((float) fabs(target) > vmin);
      eintegral = (eintegral + e * deltaT) * ((float) fabs(target) > vmin);
      if (umax / ki < eintegral)
        eintegral = umax / ki;
      if (-umax / ki > eintegral)
        eintegral = -umax / ki;
      float u = kp * e + kd * dedt + ki * eintegral;
      pwm = (int) fabs(u);
      if (pwm > umax)
        pwm = umax;
      if (pwm < 0)
        pwm = 0;
      eprev = e;
    }
};

#define NMOTORS 4
// Pines de cada motor
const int encPin[] = {4, 5, 8, 13};
const int dirPin[] = {2, 7, 9, 12}; 
const int pwmPin[] = {3, 6, 10, 11}; 

// Variables globales
float vel[]   = {0.0, 0.0, 0.0, 0.0};
float vt[]    = {0.0, 0.0, 0.0, 0.0}; 
int dir[]     = {0, 0, 0, 0};        
long prevT    = 0;   
const double ppr[] = {1390, 1390, 1390, 1390};

SimplePID pid[NMOTORS];

// Variables dinámicas
float velAng[]     = {0, 0, 0, 0};
int velEnc[]       = {0, 0, 0, 0};
int velEncSlack[]  = {0, 0, 0, 0};
float sampleT      = 0.1;

double pose[] = {0.0, 0.0, 0.0}; 
double posePrev[] = {0.0, 0.0, 0.0};
double velAngPrev[4] = {0.0, 0.0, 0.0, 0.0};

double vminLim = 15.0;
int sgn[]     = {1, 1, 1, 1}; 
double t0 = 0.0, t1 = 0.0;
double v[] = {0.0, 0.0, 0.0}; // {vx, vy, vz}
double w[] = {0.0, 0.0, 0.0, 0.0};
int pwm = 0;

bool corrigiendo = false;
double tiempo_entra = 0.0;  // Tiempo de inicio de corrección (en segundos)

// Secuencias
const int nseq = 47; // Puede ser 25 en el ejemplo real
int seq = 0;
float T[] = {0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80};
float vSeq[3][nseq]  = {
    {0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, },
    {0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, 0.00, 0.00, -0.25, -0.25, -0.25, -0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.25, 0.25, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, },
    {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, },
};

const double a_b =  0.195/2 + 0.21/2; 
const double R = 0.04;
int index = 0;

const int UF = A2; 
const int ULI = A1; 
const int ULD = A0;  

void setup() {
  Serial.begin(9600);
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(encPin[k], INPUT);
    pinMode(pwmPin[k], OUTPUT);
    pinMode(dirPin[k], OUTPUT);
  }
  pid[0].setParams(1.2587012, 12, 0.0400112, 255, vminLim);
  pid[1].setParams(1.2587012, 12, 0.0400112, 255, vminLim);
  pid[2].setParams(1.2587012, 12, 0.0400112, 255, vminLim);
  pid[3].setParams(1.2587012, 12, 0.0400112, 255, vminLim);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encPin[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encPin[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encPin[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encPin[3]), readEncoder<3>, CHANGE);
  delay(2000);
  prevT = micros();
  t0 = ((float)prevT) / 1.0e6;
}

void loop() {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  t1 = ((float)currT) / 1.0e6;
  
  if (sampleT <= deltaT) {
    prevT = currT;
    
    noInterrupts(); 
    for (int k = 0; k < NMOTORS; k++) {
      velEncSlack[k] = velEnc[k];
      velEnc[k] = 0;
    }
    interrupts();
    
    for (int k = 0; k < NMOTORS; k++) {
      vel[k] = velEncSlack[k] / deltaT / ppr[k] * 60.0;
      velAng[k] = sgn[k] * vel[k] * PI / 30;
    }
    
    // Lectura de sensores
    double dist_izq = leerDistancia(ULI);
    double dist_der = leerDistancia(ULD);
    double dist_front = leerDistancia(UF);
    
    // Si se detecta obstáculo (o umbral violado), se entra en corrección
    if (dist_izq < 6.0 || dist_der < 6.0 || dist_front < 8.0) {
      if (!corrigiendo) {
        // Se inicia la corrección y se guarda el tiempo actual (en segundos)
        tiempo_entra = millis() / 1000.0;
        corrigiendo = true;
        StopMotors();
      }
      corregirPosicion(dist_izq, dist_der, dist_front);
      // Durante corrección no se actualiza la secuencia principal.
    }
    else {
      // Si se terminó la corrección, se actualiza la base de tiempo (t0)
      if (corrigiendo) {
        double correctionTime = (millis() / 1000.0) - tiempo_entra;
        t0 += correctionTime;  // Se "descarta" el tiempo usado en corrección
        corrigiendo = false;
      }
      
      // Procesamiento normal de la secuencia
      if (seq < nseq) {
        for (int i = 0; i < 3; i++) {
          v[i] = vSeq[i][seq];
        }
        CalculateVelAng(v);
        for (int k = 0; k < NMOTORS; k++) {
          pid[k].evalu(vel[k], vt[k], deltaT, pwm);
          setMotor(dir[k], pwm, pwmPin[k], dirPin[k]);
        }
        // Si se cumplió el tiempo programado para la secuencia (excluyendo correcciones)
        if (T[seq] <= (t1 - t0)) {
          t0 = t1;
          seq++;
        }
      }
      else {
        StopMotors();
      }
    }
  }
}

void CalculateVelAng(double vm[]) { 
  double vx = vm[0];
  double vy = vm[1];
  double wz = vm[2];
  
  w[0] = (vx - vy - (a_b)*wz) / R;
  w[1] = (vx + vy + (a_b)*wz) / R;
  w[2] = (vx + vy - (a_b)*wz) / R;
  w[3] = (vx - vy + (a_b)*wz) / R;  

  for (int i = 0; i < NMOTORS; i++) {
    sgn[i] = w[i] / fabs(w[i]);
    dir[i] = (1 + sgn[i]) / 2;
    vt[i] = fabs(w[i] * 30 / PI);
  }
}

void setMotor(int dir, int pwmVal, int pwmch, int dirch) {
  analogWrite(pwmch, pwmVal);
  if (dirch == 12 || dirch == 7) {
    digitalWrite(dirch, (dir == 1) ? LOW : HIGH);
  } else {
    digitalWrite(dirch, (dir == 1) ? HIGH : LOW);
  }
}

template <int j>
void readEncoder() {
  velEnc[j]++;
}

void StopMotors() {
  for (int k = 0; k < NMOTORS; k++) {
    setMotor(dir[k], 0, pwmPin[k], dirPin[k]);
  }
}

float leerDistancia(int sensorPin) {
  int lecturaADC = analogRead(sensorPin);
  float distancia = lecturaADC * 0.3 + 0.04;
  return distancia;
}

void corregirPosicion(double dist_izq, double dist_der, double dist_front) {
  if (dist_izq < 7) {
    v[0] = 0.0;    // Avanza
    v[1] = -0.1;   // Corrección a la derecha
  } 
  else if (dist_der < 7) {
    v[0] = 0.0;    // Avanza
    v[1] = 0.1;    // Corrección a la izquierda
  } 
  else if (dist_front < 8.0) {
    v[0] = -0.1;   // Retrocede
    v[1] = 0.0;
  }
  CalculateVelAng(v);
  for (int k = 0; k < NMOTORS; k++) {
    // Se usa un deltaT fijo (0.1) para la corrección, pero puedes ajustar según convenga
    pid[k].evalu(vel[k], vt[k], 0.1, pwm);
    setMotor(dir[k], pwm, pwmPin[k], dirPin[k]);
  }
}
