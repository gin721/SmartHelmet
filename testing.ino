#include <Arduino.h>
#include <PulseSensorPlayground.h>
#include <Wire.h>
#include <MPU6050.h>

PulseSensorPlayground pulseSensor;
MPU6050 mpu;

const int Buzzer_INPUT = 4;
const int MQ3_INPUT = 5;
const int IR1_INPUT = 18;
const int IR2_INPUT = 19;
const int PULSE_INPUT = A0;
const int THRESHOLD = 685;   
boolean sendPulseSignal = false;
const float fallThreshold = 200000; // Adjust this value to suit your needs (in m/s^3)
const int sampleInterval = 10;   // Interval in milliseconds between readings
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(IR1_INPUT, INPUT);
  pinMode(IR2_INPUT, INPUT);
  pinMode(MQ3_INPUT, INPUT);
  pinMode(Buzzer_INPUT, OUTPUT);
  analogReadResolution(10);
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.setSerial(Serial);
  pulseSensor.setThreshold(THRESHOLD);
  pulseSensor.begin();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert accelerometer readings from m/s^2 to mg (1 g = 9.8 m/s^2)
  float acceleration_mg_x = ax / 9.8;
  float acceleration_mg_y = ay / 9.8;
  float acceleration_mg_z = az / 9.8;

  // Calculate jerk by taking the derivative of acceleration with respect to time
  float jerkX = (acceleration_mg_x - prevAccX) / (sampleInterval / 1000.0);
  float jerkY = (acceleration_mg_y - prevAccY) / (sampleInterval / 1000.0);
  float jerkZ = (acceleration_mg_z - prevAccZ) / (sampleInterval / 1000.0);

  // Update previous acceleration values for the next iteration
  prevAccX = acceleration_mg_x;
  prevAccY = acceleration_mg_y;
  prevAccZ = acceleration_mg_z;

  // Calculate the magnitude of jerk
  float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);
  // Check for a fall
  if (jerkMagnitude > fallThreshold)
  {
    // Fall detected
    Serial.println("Fall detected!");
    buzzer();
  }

  //Record Start of Beat
  if (pulseSensor.sawStartOfBeat()) {
    if(!sendPulseSignal){
      //Get BPM
      int pulse = pulseSensor.getBeatsPerMinute();
      if(pulse < 40 || pulse > 150){
        Serial.println("Pulse off limits");
        buzzer();
      }
    }
  }

  int ir1Value = digitalRead(IR1_INPUT);
  int ir2Value = digitalRead(IR2_INPUT);
  int MQ3Value = digitalRead(MQ3_INPUT);

  if(ir1Value == LOW || ir2Value == LOW){
    Serial.println("Helmet not worn properly");
    buzzer();
  }
  if(ir1Value == LOW && ir2Value == LOW){
    Serial.println("Helmet worn");
  }

  if(MQ3Value == LOW){
    Serial.println("Drunk");
    buzzer();
  }
}

void buzzer(){
  digitalWrite(Buzzer_INPUT, HIGH);
  delay(500);
  digitalWrite(Buzzer_INPUT, LOW);
}


