/*
  ROS2 Differential Drive Robot Controller
  Hardware:
  - TB6612FNG motor driver
  - 6V 120:1 motors
  - 65mm wheels
  - 1940 ticks/rev encoders

  Serial protocol (matches arduino_bridge.py)

  RX:
  $CMD,linear,angular

  TX:
  $ODOM,x,y,yaw,linear,angular
*/

#include <Arduino.h>

//////////////////////////////////////////////////////////////
// Robot geometry
//////////////////////////////////////////////////////////////

const float WHEEL_DIAMETER = 0.090;
const float WHEEL_BASE = 0.20;
const int TICKS_PER_REV = 1600;

const float WHEEL_CIRC = PI * WHEEL_DIAMETER;
const float TICK_DIST = WHEEL_CIRC / TICKS_PER_REV;

//////////////////////////////////////////////////////////////
// Motor driver pins
//////////////////////////////////////////////////////////////

#define PWMA 5
#define AIN1 7
#define AIN2 8

#define PWMB 6
#define BIN1 9
#define BIN2 10

#define STBY 4

//////////////////////////////////////////////////////////////
// Encoders
//////////////////////////////////////////////////////////////

#define L_ENC_A 2
#define L_ENC_B 11

#define R_ENC_A 3
#define R_ENC_B 12

volatile long left_ticks = 0;
volatile long right_ticks = 0;

//////////////////////////////////////////////////////////////
// Robot state
//////////////////////////////////////////////////////////////

float x = 0;
float y = 0;
float yaw = 0;

//////////////////////////////////////////////////////////////
// Control timing
//////////////////////////////////////////////////////////////

const float CONTROL_HZ = 100.0;
const float DT = 1.0 / CONTROL_HZ;

unsigned long last_loop = 0;

//////////////////////////////////////////////////////////////
// Encoder velocity
//////////////////////////////////////////////////////////////

long prev_left_ticks = 0;
long prev_right_ticks = 0;

float left_vel = 0;
float right_vel = 0;

float left_vel_f = 0;
float right_vel_f = 0;

const float FILTER = 0.7;

//////////////////////////////////////////////////////////////
// Commands
//////////////////////////////////////////////////////////////

float linear_cmd = 0;
float angular_cmd = 0;

float linear_target = 0;
float angular_target = 0;

float target_left = 0;
float target_right = 0;

unsigned long last_cmd_time = 0;

//////////////////////////////////////////////////////////////
// Ramp limiting
//////////////////////////////////////////////////////////////

const float MAX_ACCEL = 0.9;
const float MAX_ANG_ACCEL = 5.0;

//////////////////////////////////////////////////////////////
// PID
//////////////////////////////////////////////////////////////

float KP = 15;
float KI = 10;
float KD = 0;

float left_i = 0;
float right_i = 0;


//for tuning the motors
const float LEFT_GAIN = 1.0;
const float RIGHT_GAIN = 1.0;

//////////////////////////////////////////////////////////////
// Feedforward
//////////////////////////////////////////////////////////////

const float FF_GAIN = 220.0;

//////////////////////////////////////////////////////////////
// Encoder interrupts
//////////////////////////////////////////////////////////////

void leftISR()
{
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B))
    left_ticks++;
  else
    left_ticks--;
}

void rightISR()
{
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B))
    right_ticks--;
  else
    right_ticks++;
}

//////////////////////////////////////////////////////////////
// Motor control
//////////////////////////////////////////////////////////////

void setMotorA(int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (pwm < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    // brake
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMA, abs(pwm));
}

void setMotorB(int pwm)
{
  pwm = constrain(pwm, -255, 255);

  // reversed motor direction
  if (pwm > 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else if (pwm < 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMB, abs(pwm));
}

//////////////////////////////////////////////////////////////
// Serial command parser
//////////////////////////////////////////////////////////////

String buffer;

void parseCommand(String line)
{
  if (!line.startsWith("$CMD")) return;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);

  if (c1 < 0 || c2 < 0) return;

  linear_cmd = line.substring(c1 + 1, c2).toFloat();
  angular_cmd = line.substring(c2 + 1).toFloat();

  last_cmd_time = millis();
}

//////////////////////////////////////////////////////////////
// Safe encoder read
//////////////////////////////////////////////////////////////

void readEncoders(long &l, long &r)
{
  noInterrupts();
  l = left_ticks;
  r = right_ticks;
  interrupts();
}

//////////////////////////////////////////////////////////////
// Control loop
//////////////////////////////////////////////////////////////

void controlLoop()
{

  //////////////////////////////////////////////////////////
  // watchdog
  //////////////////////////////////////////////////////////

  if (millis() - last_cmd_time > 150)
  {
    linear_cmd = 0;
    angular_cmd = 0;
  }

  //////////////////////////////////////////////////////////
  // velocity ramp limiting
  //////////////////////////////////////////////////////////

  float d_lin = linear_cmd - linear_target;

  float accel = MAX_ACCEL;
  float decel = MAX_ACCEL * 2.0;  // brake twice as fast

  float max_step;

  if (d_lin > 0)
    max_step = accel * DT;
  else
    max_step = decel * DT;

  if (d_lin > max_step) d_lin = max_step;
  if (d_lin < -max_step) d_lin = -max_step;

  linear_target += d_lin;

  float d_ang = angular_cmd - angular_target;
  float max_ang = MAX_ANG_ACCEL * DT;

  if (d_ang > max_ang) d_ang = max_ang;
  if (d_ang < -max_ang) d_ang = -max_ang;

  angular_target += d_ang;

  //////////////////////////////////////////////////////////
  // hard stop when command is zero
  //////////////////////////////////////////////////////////

  if (abs(linear_target) < 0.001 && abs(angular_target) < 0.001)
  {
    setMotorA(0);
    setMotorB(0);

    left_i = 0;
    right_i = 0;

    return;
  }

  //////////////////////////////////////////////////////////
  // convert to wheel velocities
  //////////////////////////////////////////////////////////

  target_left =
      linear_target - angular_target * WHEEL_BASE / 2.0;

  target_right =
      linear_target + angular_target * WHEEL_BASE / 2.0;

  //////////////////////////////////////////////////////////
  // encoder update
  //////////////////////////////////////////////////////////

  long l, r;
  readEncoders(l, r);

  long dl = l - prev_left_ticks;
  long dr = r - prev_right_ticks;

  prev_left_ticks = l;
  prev_right_ticks = r;

  float left_dist  = -dl * TICK_DIST;
  float right_dist = -dr * TICK_DIST;

  left_vel = left_dist / DT;
  right_vel = right_dist / DT;

  //////////////////////////////////////////////////////////
  // velocity filtering
  //////////////////////////////////////////////////////////

  left_vel_f = FILTER * left_vel_f + (1 - FILTER) * left_vel;
  right_vel_f = FILTER * right_vel_f + (1 - FILTER) * right_vel;

  //////////////////////////////////////////////////////////
  // PID + feedforward
  //////////////////////////////////////////////////////////

  float err_l = target_left - left_vel_f;
  float err_r = target_right - right_vel_f;

  left_i += err_l * DT;
  right_i += err_r * DT;

  left_i = constrain(left_i, -1, 1);
  right_i = constrain(right_i, -1, 1);

  float u_l =
    (FF_GAIN * target_left +
     KP * err_l +
     KI * left_i) * LEFT_GAIN;

  float u_r =
      (FF_GAIN * target_right +
      KP * err_r +
      KI * right_i) * RIGHT_GAIN;

  setMotorA(u_l);
  setMotorB(u_r);

  //////////////////////////////////////////////////////////
  // odometry
  //////////////////////////////////////////////////////////

  float d = (left_dist + right_dist) / 2.0;
  float dtheta = (right_dist - left_dist) / WHEEL_BASE;

  float mid = yaw + dtheta * 0.5;

  x += d * cos(mid);
  y += d * sin(mid);
  yaw += dtheta;

  //////////////////////////////////////////////////////////
  // publish odometry
  //////////////////////////////////////////////////////////

  Serial.print("$ODOM,");
  Serial.print(x,4);
  Serial.print(",");
  Serial.print(y,4);
  Serial.print(",");
  Serial.print(yaw,4);
  Serial.print(",");
  Serial.print(d / DT,4);
  Serial.print(",");
  Serial.println(dtheta / DT,4);
}

//////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);

  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, CHANGE);

  last_loop = millis();
  last_cmd_time = millis();

  Serial.println("$DEBUG,READY");
}

//////////////////////////////////////////////////////////////
// Main loop
//////////////////////////////////////////////////////////////

void loop()
{

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      parseCommand(buffer);
      buffer = "";
    }
    else
    {
      buffer += c;
    }
  }

  if (millis() - last_loop >= 10)
  {
    last_loop += 10;
    controlLoop();
  }
}
