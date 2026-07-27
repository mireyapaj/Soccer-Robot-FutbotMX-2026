#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HUSKYLENS.h"

const int M1_IN1 = 24, M1_IN2 = 22, M1_PWM = 2;
const int M2_IN1 = 26, M2_IN2 = 28, M2_PWM = 3;
const int M3_IN1 = 32, M3_IN2 = 30, M3_PWM = 4;
const int STBY = 34;

const float RPM_MAX = 300.0;
const float R = 0.0275;
const float L = 0.085;

const int PWM_MAX = 255;

float vmax = 0.0;
float kv = 0.0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float yawRef = 0.0;
float Kp = 0.10;
float maxThetaDot = 2.0;

HUSKYLENS huskylens;

const int CENTRO_X = 160;
const int MARGEN = 35;

float KP_CAM = 0.004;

enum EstadoRobot {
    BUSCANDO_PELOTA,
    CENTRANDO_PELOTA
};

EstadoRobot estado = BUSCANDO_PELOTA;

const uint8_t NUM_ADC = 10;
const uint16_t UMBRAL_CAMBIO = 70;
const float ALPHA_EMA = 0.001f;

volatile uint16_t adcValues[NUM_ADC];
volatile uint8_t currentChannel = 0;
volatile bool discardSample = true;

float adc_Values_mean[NUM_ADC];
bool sensores_piso[NUM_ADC] = {false};

float direccionLateral = 0.25;
float direccionFrontal = 0.0;

ISR(ADC_vect)
{
  uint16_t value = ADC;

  if (discardSample)
  {
    discardSample = false;
    return;
  }

  adcValues[currentChannel] = value;

  currentChannel++;

  if (currentChannel >= NUM_ADC)
    currentChannel = 0;

  ADMUX = (1 << REFS0) | (currentChannel & 0x07);
  ADCSRB = (currentChannel & 0x08) ? (1 << MUX5) : 0;

  discardSample = true;
}

void setMotor(int in1, int in2, int pwmPin, float velLineal) {
  int pwm = (int)(fabs(velLineal) * kv);
  pwm = constrain(pwm, 0, 255);

  if (velLineal > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (velLineal < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwm = 0;
  }

  analogWrite(pwmPin, pwm);
}

void calcularVelocidadesRueda(float xdot, float ydot, float thetadot,
                              float &v1, float &v2, float &v3) {
  float theta = 0.0;

  v1 = xdot * sin(theta) - ydot * cos(theta) + thetadot * L;
  v2 = xdot * cos(theta + PI / 6.0) + ydot * sin(theta + PI / 6.0) + thetadot * L;
  v3 = -xdot * sin(theta + PI / 3.0) + ydot * cos(theta + PI / 3.0) + thetadot * L;
}

void aplicarMovimiento(float xdot, float ydot, float thetadot) {
  float v1, v2, v3;
  calcularVelocidadesRueda(xdot, ydot, thetadot, v1, v2, v3);

  setMotor(M1_IN1, M1_IN2, M1_PWM, v1);
  setMotor(M2_IN1, M2_IN2, M2_PWM, v2);
  setMotor(M3_IN1, M3_IN2, M3_PWM, v3);
}

void detenerRobot() {
  aplicarMovimiento(0.0, 0.0, 0.0);
}

float normalizar360(float ang) {
  while (ang >= 360.0) ang -= 360.0;
  while (ang < 0.0) ang += 360.0;
  return ang;
}

float errorAngular(float actual, float referencia) {
  float e = referencia - actual;
  while (e > 180.0) e -= 360.0;
  while (e < -180.0) e += 360.0;
  return e;
}

float leerYaw() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return normalizar360(euler.x());
}

void calibrarFrente() {
  Serial.println("Calibrando frente...");
  delay(1500);

  float suma = 0, N = 70.0;
  for (int i = 0; i < N; i++) {
    suma += leerYaw();
    delay(20);
  }

  yawRef = suma / N;

  Serial.print("Yaw referencia: ");
  Serial.println(yawRef);
}


float obtenerCorreccionIMU(bool camaraActiva)
{
  float error = errorAngular(leerYaw(), yawRef);

  float thetadot = -Kp * error;

  if (fabs(error) < 4.0) return 0.0;

  if (camaraActiva) {
    thetadot *= 0.3;   
  }

  return constrain(thetadot, -maxThetaDot, maxThetaDot);
}

bool leerPelota(int &x) {
  if (!huskylens.request()) return false;

  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();
    if (r.ID == 1) {
      x = r.xCenter;
      return true;
    }
  }
  return false;
}

void estadoCentrandoPelota()
{
    int xBall;

    if(!leerPelota(xBall))
    {
        estado = BUSCANDO_PELOTA;
        return;
    }

    int error = xBall - CENTRO_X;

    float imu = obtenerCorreccionIMU(true);

    if(abs(error) < MARGEN)
    {
        detenerRobot();
        return;
    }

    float ydot = -KP_CAM * error;
    ydot = constrain(ydot, -0.4, 0.4);

    aplicarMovimiento(0.0, ydot, imu);
}

void actualizarSensoresPiso()
{
    uint16_t copias_adc[NUM_ADC];

    noInterrupts();

    for(uint8_t i=0;i<NUM_ADC;i++)
    {
        copias_adc[i] = adcValues[i];
    }

    interrupts();

    for(uint8_t i=0;i<NUM_ADC;i++)
    {
        float cambio =
            fabs(adc_Values_mean[i] - (float)copias_adc[i]);

        if(cambio > UMBRAL_CAMBIO)
        {
            sensores_piso[i] = true;
        }
        else
        {
            sensores_piso[i] = false;

            adc_Values_mean[i] +=
                ALPHA_EMA *
                ((float)copias_adc[i] - adc_Values_mean[i]);
        }
    }

    bool ladoIzquierdo =
        //sensores_piso[3] ||
        sensores_piso[4] ||
        sensores_piso[8];

    bool ladoDerecho =
        sensores_piso[1] ||
        //sensores_piso[2] ||
        sensores_piso[7];

    bool atras =
    sensores_piso[0] ||
    sensores_piso[5] ||
    sensores_piso[6] ||
    sensores_piso[9];

    if(atras && !ladoIzquierdo && !ladoDerecho)
    {
        direccionFrontal = 0.25; // Mover hacia adelante
    }
    else
    {
        direccionFrontal = 0.0;  // Detener el avance frontal si ya no detecta
    }
    // ----------------------------

    if(ladoIzquierdo)
    {
        direccionLateral = -0.25;
    }

    if(ladoDerecho)
    {
        direccionLateral = 0.25;
    }
}

void estadoBuscandoPelota()
{
    int xBall;

    if(leerPelota(xBall))
    {
        estado = CENTRANDO_PELOTA;
        return;
    }

    actualizarSensoresPiso();

    float imu = obtenerCorreccionIMU(false);

    aplicarMovimiento(direccionFrontal, direccionLateral, imu);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  vmax = RPM_MAX * (2.0 * PI / 60.0) * R;
  kv = PWM_MAX / vmax;

  ADMUX = (1 << REFS0);
  ADCSRB = 0;

  ADCSRA =
      (1 << ADEN) |
      (1 << ADATE) |
      (1 << ADIE) |
      (1 << ADPS2) |
      (1 << ADPS1) |
      (1 << ADPS0);

  sei();
  ADCSRA |= (1 << ADSC);

  delay(100);

  noInterrupts();

  for (uint8_t i = 0; i < NUM_ADC; i++)
  {
      adc_Values_mean[i] = adcValues[i];
  }

  interrupts();

  // IMU
  if (!bno.begin()) {
    Serial.println("Error BNO055");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // HUSKYLENS
  Serial1.begin(9600);
  while (!huskylens.begin(Serial1)) {
    Serial.println("Buscando HuskyLens...");
    delay(500);
  }


  calibrarFrente();

  detenerRobot();
  Serial.println("Listo");
}

void loop()
{

    
    switch(estado)
    {
        case BUSCANDO_PELOTA:
            estadoBuscandoPelota();
            break;

        case CENTRANDO_PELOTA:
            estadoCentrandoPelota();
            break;
    }

    delay(20);
}