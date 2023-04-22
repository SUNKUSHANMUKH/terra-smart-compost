#include <Servo.h> 
#define Servo_PWM 6 
Servo MG995_Servo;  

int redLed = 12;
int buzzer = 10;
int smokeA0 = A5;
int sensorThres = 400;
int relayPin = 8;
int sensor_pin = A0; 
int output_value ;
int val;
int tempPin = 1;


void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  Serial.println("Reading From the Sensor ...");
  delay(2000);
  MG995_Servo.attach(Servo_PWM);
}

void loop() {
  int analogSensor = analogRead(smokeA0);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
    if (analogSensor > sensorThres)
  {
    digitalWrite(redLed, HIGH);
    tone(buzzer, 1000, 200);
  }
  else
  {
    digitalWrite(redLed, LOW);
    noTone(buzzer);
  }
  delay(100);

  val = analogRead(tempPin);
  float mv = ( val/1024.0)*5000;
  float cel = mv/10;
  float farh = (cel*9)/5 + 32;
  Serial.print("TEMPRATURE = ");
  Serial.print(cel);
  Serial.print("*C");
  Serial.println();
  delay(1000);

 output_value= analogRead(sensor_pin);
 output_value = map(output_value,550,10,0,100);
 Serial.print("Mositure : ");
 Serial.print(output_value);
 Serial.println("%");
 if(output_value<20){
  digitalWrite(relayPin, LOW);
 }
 else
 {
  digitalWrite(relayPin, HIGH);       
 }
 delay(1000);


  Serial.println("0");
  MG995_Servo.write(0);
  delay(3000);
  MG995_Servo.detach();
  delay(2000);
  MG995_Servo.attach(Servo_PWM);
  Serial.println("0");
  MG995_Servo.write(180);
  delay(3000);
  MG995_Servo.detach();
  delay(2000);
  MG995_Servo.attach(Servo_PWM);   
}
