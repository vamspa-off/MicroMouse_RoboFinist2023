#include <Robot_L298P.h>
#include <FastLED.h>

#define DATA_PIN A3
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

float Kp = 0.41, Kd = 0.97, Ki = -0.02;
int BASIC_SPEED = 35;
int E_old = 0;
long int I = 0;

int flag = 0;
float K_l = 0.8;

int cou = 0;

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  Robot.setup();
  Serial.begin(9600);
  FastLED.setBrightness(20);
  leds[0] = CRGB(0, 0, 255);
  leds[1] = CRGB(0, 0, 255);
  leds[8] = CRGB(0, 0, 255);
  leds[9] = CRGB(0, 0, 255);
  leds[3] = CRGB(255, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  leds[6] = CRGB(255, 0, 0);
  leds[7] = CRGB(255, 0, 0);
  leds[4] = CRGB(255, 255, 0);
  leds[5] = CRGB(255, 255, 0);
  FastLED.show();
  FastLED.delay(33);

}

void pid(){
  int E = analogRead(A0)-analogRead(A2);
  I = E + I*0.95;
  int PID = E*Kp + (E-E_old)*Kd + I*Ki;
  E_old = E;
  int M1 = BASIC_SPEED + PID;
  int M2 = BASIC_SPEED - PID; 
  M2 = constrain(M2, -100, 100);
  M1 = constrain(M1, -100, 100);
  Robot.motor_A(M1);
  Robot.motor_B(M2);
}

bool f_wall(){
  return digitalRead(A1);
}
bool l_wall(){
  if(analogRead(A2) < 800){
    return true;
  } else {
    return false;
  }
}

void rot_l(){
  if (flag == 0){
    Robot.enc_A = -130;
    Robot.enc_B = 130;
    flag = 1;
    Robot.motors(-30, -30);
    delay(60);
  }
  while(Robot.enc_A != 0 && Robot.enc_B != 0){
    Robot.motors(Robot.enc_A * K_l, Robot.enc_B * K_l);
  }
  if(f_wall()){
    Robot.motors(30, 30);
    delay(60);
    Robot.motors(0, 0);
  }
}

void rot_r(){
  if (flag == 0){
    Robot.enc_A = 130;
    Robot.enc_B = -130;
    flag = 1;
    Robot.motors(-30, -30);
    delay(60);
  }
  while(Robot.enc_A != 0 && Robot.enc_B != 0){
    Robot.motors(Robot.enc_A * K_l, Robot.enc_B * K_l);
  }
  if(f_wall()){
    Robot.motors(30, 30);
    delay(60);
    Robot.motors(0, 0);
  }
}
void loop() {
  uint8_t hue;
  if(f_wall()){
    pid();
  } else {
    if(!l_wall()){
      flag = 0;
      rot_l();
    } else {
      flag = 0;
      rot_r();
    }
  }
}
