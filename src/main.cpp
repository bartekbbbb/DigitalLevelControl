#include <Arduino.h>
#include <Wire.h>
 
//adres MPU
#define MPU 0x68

double gyro_raw = 0;
double accel_raw = 0;

double deviation = 0;
double prev_deviation = 0;
double noise = 0.02;
double prev_noise = 0.02;

double deviation_raw = 0;
double noise_raw = 0;

double P[2][2] = {{0, 0},{0, 0}};
double P_raw[2][2] = {{0, 0},{0, 0}};
double K1 = 0,K2 = 0;

double q = 0.05, r = 0.5, dt = 0.01;

int AcX, AcY, GyX, GyY;

void kalman(double gyroraw, double accelraw) {
  	prev_deviation = deviation;
  	prev_noise = noise;

	//Faza predykcji
  	deviation_raw = prev_deviation + (gyroraw - prev_noise) * dt;
  	noise_raw = prev_noise;

  	P_raw[0][0] = P[0][0] - P[1][0] * dt + P[1][1] * dt * dt - P[0][1] * dt + q;
  	P_raw[0][1] = P[0][1] - P[1][1] * dt;
  	P_raw[1][0] = P[1][0] - P[1][1] * dt;
  	P_raw[1][1] = P[1][1] + q;

	//Faza korekcji
  	K1 = P_raw[0][0] * (1/(P_raw[0][0] + r));
  	K2 = P_raw[1][0] * (1/(P_raw[0][0] + r));

 	deviation = deviation_raw + K1 * (accelraw - deviation_raw);
  	noise = noise_raw + K2 * (accelraw - deviation_raw);

  	P[0][0] = P_raw[0][0] * (1 - K1);
  	P[0][1] = P_raw[0][1] * (1 - K1);
  	P[1][0] = P_raw[1][0] - (P_raw[0][0] * K2);
  	P[1][1] = P_raw[1][1] - (K2 * P_raw[0][1]);
}

void set_LED(double deviation_){
	deviation_ = deviation_ - 90;
	if (deviation_ <= 45 && deviation_ >= -45) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 6 || i == 7)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ <= 90 && deviation_ > 45) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 6 || i == 5)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ <= 135 && deviation_ > 90) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 5 || i == 4)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ <= 180 && deviation_ > 135) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 4 || i == 3)
			 	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ > 180) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 2)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ < -45 && deviation_ >= -90) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 7 || i == 8)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ < -90 && deviation_ >= -135) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 8 || i == 9)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ < -135 && deviation_ >= -180) {
	   	for (int i = 2; i < 12; i++) {
		   	if (i == 10 || i == 9)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
   	if (deviation_ < -180) {
	   for (int i = 2; i < 12; i++) {
		   	if (i == 11)
			   	digitalWrite(i, HIGH);
		   	else
			   	digitalWrite(i, LOW);
	   	}
   	}
}

void setup()
{ 
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B);   
	Wire.write(0);
	Wire.endTransmission(true);
  	//Konfiguracja żyroskopu
  	Wire.beginTransmission(MPU);
  	Wire.write(0x1B);
  	Wire.write(0x10);
  	Wire.endTransmission(true);
  	//Konfiguracja akcelerometru
  	Wire.beginTransmission(MPU);
  	Wire.write(0x1C);
  	Wire.write(0x10);
  	Wire.endTransmission(true); 
	for (int i = 2; i < 12; i++) 
		pinMode(i, OUTPUT);
	
	for (int i = 2; i < 12; i++) {
		digitalWrite(i, HIGH);
		delay(50);
		digitalWrite(i, LOW);
	}
	
}

void loop()
{
	//Odczyt z akcelerometru
	Wire.beginTransmission(MPU);
  	Wire.write(0x3B);
  	Wire.endTransmission(false);
  	Wire.requestFrom(MPU,4,true);
  	AcX=Wire.read()<<8|Wire.read();
  	AcY=Wire.read()<<8|Wire.read();
  	Wire.endTransmission(true);
	//Odczyt z żyroskopu
  	Wire.beginTransmission(MPU);
  	Wire.write(0x43);
  	Wire.endTransmission(false);
  	Wire.requestFrom(MPU,4,true);
  	GyX=Wire.read()<<8|Wire.read();
  	GyY=Wire.read()<<8|Wire.read();
  	Wire.endTransmission(true);
	
	kalman (GyY, AcY);
	set_LED(deviation);

   	delay(200);
}