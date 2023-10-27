#include <Robot_L298P.h>

float Kp = 0.41, Kd = 0.97, Ki = -0.02;
int BASIC_SPEED = 60;
int E_old = 0;
long int I = 0;

int flag = 0;
float K_l = 0.8;

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  Robot.setup();
  Serial.begin(9600);
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
  if(analogRead(A1) < 500){
    return true;
  } else {
    return false;
  }
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
  }
  Robot.motors(Robot.enc_A * K_l, Robot.enc_B * K_l);
  Serial.println(Robot.enc_A);
}

void rot_r(){
  if (flag == 0){
    Robot.enc_A = 130;
    Robot.enc_B = -130;
    flag = 1;
  }
  Robot.motors(Robot.enc_A * K_l, Robot.enc_B * K_l);
  Serial.println(Robot.enc_A);
}

void loop() {
  BASIC_SPEED = map(analogRead(A1), 0, 980, 0, 50);
  Serial.println(analogRead(A1));
  if(!f_wall()){
    pid();
  } else {
    if(!l_wall()){
      flag = 0;
      rot_r();
    } else {
      flag = 0;
      rot_l();
    }
  }
}
