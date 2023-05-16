#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

#define SERVO_PWM_PIN 11
#define SERVO_CLOSED 0
#define SERVO_OPEN 0xff

#define BUZZER_PIN 13
#define DT 10
#define BUZZER_T 75
#define HISTORY_SIZE 100

#define ACC_THRUST_MIN 10 //to be tuned
#define ALT_FALLING_THRESHOLD_MIN 1 //to be tuned

uint8_t state = 0; //0 = static, 1 = pushed, 2 = free fall climbing, 3 free fall falling
uint8_t buzzer_counter = 0;

//Altimeter
Adafruit_BME280 bme;
float reset_pressure = 1013.25;
float alt_hist[HISTORY_SIZE];
uint8_t alt_i = 0;

//IMU
Adafruit_MPU6050 mpu;
float acc_hist[HISTORY_SIZE];
uint8_t acc_i = 0;

float mean(float *start, uint8_t size) {
  float sum = 0.0;
  for(uint8_t i=0; i < size; i++)
    sum += start[i];
  return sum / float(size);
}

void setup() {
  pinMode(SERVO_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(SERVO_PWM_PIN, SERVO_CLOSED);

  mpu.begin(0x69);
  bme.begin(118);
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  while(acc_i < HISTORY_SIZE) {
    acc_hist[acc_i] = 0.0;
    acc_i++;
  }
  acc_i = 0;

  buzzer_counter = 0;
  for(uint16_t i=0; i < HISTORY_SIZE; i++) {
    alt_hist[i] = bme.readAltitude(reset_pressure);
    digitalWrite(BUZZER_PIN, 1);  
    delay(i);
    digitalWrite(BUZZER_PIN, 0);
    delay(i);
  }
  reset_pressure = mean(alt_hist, HISTORY_SIZE);
  state = 0;
}

void loop() {
  // Buzzer
  buzzer_counter++;
  if(buzzer_counter == BUZZER_T / DT)
    digitalWrite(BUZZER_PIN, 1);  
  else if (buzzer_counter == 2 * BUZZER_T / DT) {
    digitalWrite(BUZZER_PIN, LOW);  
    buzzer_counter = 0;
  }

  //acceleration
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float acc = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
  acc_hist[acc_i] = acc;
  if(acc_i < HISTORY_SIZE)
    acc_i++;
  else
    acc_i = 0;
  
  //altitude
  float alt = bme.readAltitude(reset_pressure);
  acc_hist[alt_i] = alt;
  if(alt_i < HISTORY_SIZE)
    alt_i++;
  else
    alt_i = 0;
  
  switch(state) {
    case 0:
      if((acc - mean(acc_hist, HISTORY_SIZE)) > ACC_THRUST_MIN)
        state = 1;
      break;
    case 1:
      if((acc - mean(acc_hist, HISTORY_SIZE)) < ACC_THRUST_MIN)
        state = 2;
      break;
    case 2:
      if(((acc - mean(acc_hist, HISTORY_SIZE)) < ACC_THRUST_MIN) && ((mean(alt_hist, 10) - alt) > ALT_FALLING_THRESHOLD_MIN))
        state = 3;
      break;
    case 3: //release chute!!
      analogWrite(SERVO_PWM_PIN, SERVO_OPEN);
      break;
    }
  delay(DT);
}
