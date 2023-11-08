#include <Servo.h>
//PID tanpa library, perhitungan manual

Servo myServo;
const int inputFlexPin = A0;
const int outputFlexPin = A1;
const int servoPin = 9;

float Kp = 0.1, Ki = 0, Kd = 0; // Kp=1 berarti 1DerajatPerubahanServo/(1ADCcountError)

float servoState = 0;

int desiredPosition = 90; // posisi yang diinginkan untuk servo
int currentPosition = 90; // posisi saat ini dari servo
int previousError = 0; // error dari iterasi sebelumnya dari loop kontrol
int integralError = 0; // jumlah semua error selama ini

void setup() {
  myServo.attach(servoPin); // menghubungkan servo ke pin 9
  pinMode(inputFlexPin, INPUT); // mengatur pin analog sebagai input untuk potensiometer
  pinMode(outputFlexPin, INPUT); // mengatur pin analog sebagai input untuk potensiometer
  myServo.write(servoState);
  Serial.begin(115200);
}

int SetPoint = 0;
int OutputValue = 0;
float controlSignal = 0;

void loop() {
  unsigned long interval = 100;
  static unsigned long lastPID = 0;
  unsigned long now = millis();
  if (now - lastPID >= interval) {
    lastPID += interval;
    SetPoint = analogRead(inputFlexPin); // membaca nilai dari potensiometer
    OutputValue = analogRead(outputFlexPin); // membaca nilai dari potensiometer

    desiredPosition = map(SetPoint, 0, 1023, 0, 180); // memetakan nilai potensiometer ke posisi servo

    int error = SetPoint - OutputValue; // menghitung error antara posisi yang diinginkan dan posisi saat ini
    integralError += error; // menambahkan error ke istilah error integral
    int derivativeError = error - previousError; // menghitung turunan dari error

    // menghitung sinyal kontrol (dalam hal ini, posisi servo) menggunakan pengontrol PID
    controlSignal = Kp * error + Ki * integralError + Kd * derivativeError;

    previousError = error; // menyimpan error saat ini untuk iterasi berikutnya dari loop kontrol

    servoState = constrain(servoState + controlSignal, 0, 180);
    //Serial.print('p');
    // memperbarui posisi servo
    myServo.write(servoState);
    currentPosition = controlSignal;
  }

  delay(20); // menunggu selama 20 milidetik sebelum menjalankan loop kontrol lagi

  report();
}

void report(void) {
  unsigned long interval = 250;
  static unsigned long last = -interval;
  if (millis() - last < interval) return;
  last += interval;
  Serial.print("SP:");
  Serial.print( SetPoint);
  Serial.print(" PV:");
  Serial.print( OutputValue);
  Serial.print(" CV:");
  Serial.print( controlSignal,2);
  Serial.print( " Servo°:");
  Serial.print(servoState);
  Serial.println();
}
