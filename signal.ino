
#include "TinyWireM.h"
#include <GyverPower.h>

#define IR_PIN  10
#define PHOTO_PIN 9
#define SMALL_BUZZ 11
#define BIG_BUZZ  12
#define MPU_VCC 8


#define BIG_FREQ      1000   //сток 1000
#define LOW_BIG_FREQ  500     //сток 500
#define SMALL_FREQ    1000    //сток 1000

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t Temp;
long ACC, GYR;
long maxACC, maxGYR;
long trACC = 270, trGYR = 120;
char mpu = 0x68;

boolean pos = 1;
boolean mode = 0;
boolean alertMode = 0;
boolean firstFlag = 0;

void setup() {
  pinMode(IR_PIN, OUTPUT);
  pinMode(PHOTO_PIN, INPUT);
  pinMode(SMALL_BUZZ, OUTPUT);
  pinMode(BIG_BUZZ, OUTPUT);
  pinMode(MPU_VCC, OUTPUT);

  attachInterrupt(1, isr, RISING);

  TinyWireM.begin();
  power.autoCalibrate();
  power.setSleepMode(POWERDOWN_SLEEP);
}

void isr() {
  alertMode = 0;
}

void loop() {
  power.sleepDelay(8000);
  power.setSystemPrescaler(PRESCALER_2);

  digitalWrite(IR_PIN, true);

  if (digitalRead(PHOTO_PIN) == 0 && pos == 1) {
    pos = 0;
    delay(1000);
  }


  delay(100);

  if (digitalRead(PHOTO_PIN) == 1 && pos == 0) {
    pos = 1;
    mode = 0;
    firstFlag = 0;
    digitalWrite(IR_PIN, false);
    digitalWrite(MPU_VCC, false);
    power.setSystemPrescaler(PRESCALER_8);
  }

  if (pos == 0) {
    if (firstFlag == 0) {
      startSignal();
      firstFlag = 1;
    }

    start_MPU();
    get_MPU();
    if (mode == 0) {
      calib();
      mode = 1;
    }
    watch();
  }
  loop();
}

void get_MPU() {
  TinyWireM.beginTransmission(mpu);
  TinyWireM.send(0x3B);                               // Starting with register 0x3B (ACCEL_XOUT_H)
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(mpu, 14);                // Request a total of 14 registers
  ax = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = TinyWireM.receive() << 8 | TinyWireM.receive(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void start_MPU() {
  digitalWrite(MPU_VCC, true);
  TinyWireM.begin();
  TinyWireM.beginTransmission(mpu);
  TinyWireM.send(0x6B); //  установка регистра питания
  TinyWireM.send(0b00000000); // отключаем спящий режим
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu);
  TinyWireM.send(0x1B); // конфигурация регистров гироскопа
  TinyWireM.send(0x00000000); // 250° per second range (default)
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); //адрес I2C гироскопа
  TinyWireM.send(0x1C); // конфигурация регистров акселерометра
  TinyWireM.send(0b00000000); // 2g range +/- (default)
  TinyWireM.endTransmission();
  delay(2000);
}

void calib() {
  // "калибровка" максимальных значений
  for (int i = 0; i < 30; i++) {
    ACC = abs(ax) + abs(ay) + abs(az); GYR = abs(gx) + abs(gy) + abs(gz);
    if (ACC > maxACC) maxACC = ACC;
    if (GYR > maxGYR) maxGYR = GYR;
    delay(10);
  }
  maxACC = maxACC + trACC;
  maxGYR = maxGYR + trGYR;
}


void watch() {
  // сумма модулей
  ACC = abs(ax) + abs(ay) + abs(az);
  GYR = abs(gx) + abs(gy) + abs(gz);
  if (ACC > maxACC || GYR > maxGYR) {
    alert();
  }
  else
  { digitalWrite(IR_PIN, false);
    TinyWireM.beginTransmission(mpu);
    TinyWireM.send(0x6B);
    TinyWireM.send(0b01000000);
    TinyWireM.endTransmission();
  }
}

void alert() {
  if (alertMode == 0) {
    tone(BIG_BUZZ, LOW_BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, LOW_BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, LOW_BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    mode = 0;
    alertMode = 1;
    delay(2000);

    for (byte i = 0; i < 120; i++) {
      get_MPU();
      if (mode == 0) {
        calib();
        mode = 1;
      }

      // сумма модулей
      ACC = abs(ax) + abs(ay) + abs(az);
      GYR = abs(gx) + abs(gy) + abs(gz);
      if (ACC > maxACC || GYR > maxGYR) {
        alert();
      }
      delay(250);
    }
    alertMode = 0;
  }


  while (alertMode) {
    back();
    digitalWrite(MPU_VCC, false);

    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(200);
    noTone(BIG_BUZZ);
    delay(200);

    back();

    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ + 1000);
    delay(100);
    tone(BIG_BUZZ, BIG_FREQ - 200);
    delay(100);
    noTone(BIG_BUZZ);
    delay(200);

    back();

    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(75);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(75);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(75);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(75);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(75);
    tone(BIG_BUZZ, BIG_FREQ);
    delay(75);
    noTone(BIG_BUZZ);
    delay(200);

    back();
  }

  loop();
}

void startSignal() {
  tone(SMALL_BUZZ, SMALL_FREQ);
  delay(150);
  noTone(SMALL_BUZZ);
  delay(150);
  tone(SMALL_BUZZ, SMALL_FREQ);
  delay(150);
  noTone(SMALL_BUZZ);
  delay(150);
  tone(SMALL_BUZZ, SMALL_FREQ);
  delay(150);
  noTone(SMALL_BUZZ);
}

void back() {
  if (digitalRead(PHOTO_PIN) == 1 && pos == 0) {
    pos = 1;
    alertMode = 0;
    digitalWrite(IR_PIN, false);
    delay(50);
    loop();
  }
}
