#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
MPU6050 mpu6050(Wire);
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
volatile long int count1 = 0, pr_count1 = 0;
volatile long int count2 = 0, pr_count2 = 0;
unsigned long int timer = 0, timeNow = 0;
unsigned long int deltaTime = 1, timer_debug = 0;
float speedset = 0; //rpm
#define ENCODER1_PIN_A 2 
#define ENCODER1_PIN_B 4
#define ENCODER2_PIN_A 3 
#define ENCODER2_PIN_B 7

#define kp1 35
#define ki1 10
#define kd1 10

#define i_error_max1 125

#define velmax 95

#define pwm_max 255

#define Angle_i_error_max 40

void encoder1(void);
float velocityEncoder1(void);
float i_error1 = 0, pr_error1 = 0;
float PIDControlVel1(float error);
// float AngleMeasure = 0;
#define kp2 25
#define ki2 4
#define kd2 4

void encoder2(void);
float velocityEncoder2(void);
float i_error2 = 0, pr_error2 = 0;
float PIDControlVel2(float error);

int motorSpeed1 = 5,
    motorSpeed2 = 6,
    INA1 = 8,
    INB1 = 9,
    INA2 = 10,
    INB2 = 11;

void MOVE_UP(void){
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW); 
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, HIGH); 
}
void MOVE_DOWN(void){
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW); 
  digitalWrite(INB1, 1);
  digitalWrite(INB2, 1); 
}


float AngleOffset = 1;
float AngleMeasure = 0;//read from MPU
// #define kpA 18
// #define kiA 3
// #define kdA 2
#define kpA 93.312
#define kiA 0
#define kdA 0
float Angle_i_error = 0, Angle_pr_error = 0;
float PIDAngleControl(float error){
  Angle_i_error += error * (deltaTime * 0.001);
  if (Angle_i_error > Angle_i_error_max) Angle_i_error = Angle_i_error_max;
  else if(Angle_i_error < -Angle_i_error_max) Angle_i_error = -Angle_i_error_max;
  float delta = error - Angle_pr_error;
  float out = kpA * error + kiA * Angle_i_error + kdA * (delta /(deltaTime* 0.001));
  Angle_pr_error = error;
  if (out > pwm_max) out = pwm_max;
  else if(out < - pwm_max) out = -pwm_max;
  return out; //Velocity
}

void setup() {
  Serial.begin(9600);
  
  //I2C + config MPU
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);


      // Serial.print(mpu6050.getAngleX());
      // Serial.print("\t ");
      // Serial.print(mpu6050.getAngleY());
      // Serial.print("\t ");
      // Serial.println(mpu6050.getAngleZ());


  pinMode(ENCODER1_PIN_A, INPUT);
  pinMode(ENCODER1_PIN_B, INPUT);
  pinMode(ENCODER2_PIN_A, INPUT);
  pinMode(ENCODER2_PIN_B, INPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(motorSpeed1, OUTPUT);
  pinMode(motorSpeed2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoder2, RISING);
}
int MotorControl1(int A);


int X = 0, X2 = 0, PWM_RUN = 0;
float CalVel1 = 0, CalVel2 = 0, AnglePID = 0;



//RUN
void loop() {
  digitalWrite(13, 1);
  timeNow = millis();
  
  speedset = AnglePID;
  if (AngleMeasure - AngleOffset > 0 && AngleMeasure - AngleOffset < 40 && AngleMeasure - AngleOffset > -40){
    MOVE_UP();
  }else if(AngleMeasure - AngleOffset < 0 && AngleMeasure - AngleOffset < 40 && AngleMeasure - AngleOffset > -40){
    MOVE_DOWN();
  }else {
  digitalWrite(INB1, 0);
  digitalWrite(INB2, 0); 
  digitalWrite(INA1, 0);
  digitalWrite(INA2, 0); 
  Angle_i_error = 0;
  // speedset = 0;
 }
 PWM_RUN = abs((int)AnglePID);
 if ( PWM_RUN > 0 && PWM_RUN < 50) PWM_RUN = 50;
  analogWrite(motorSpeed2, PWM_RUN);
  analogWrite(motorSpeed1, PWM_RUN);



  if (timeNow - timer >= 5){
    mpu6050.update();
    deltaTime = timeNow - timer;
    AngleMeasure = mpu6050.getAngleY();
    //AngleMeasure = simpleKalmanFilter.updateEstimate(AngleMeasure);
    AnglePID = PIDAngleControl(AngleMeasure - AngleOffset);
    // CalVel1 = velocityEncoder1();
    // X = (int)PIDControlVel1(abs(speedset) - CalVel1);
    // CalVel2 = velocityEncoder2();
    // X2 = (int)PIDControlVel2(abs(speedset) - CalVel2);
    timer = timeNow;
  }
  // if(timeNow - timer_debug >= 10){
  //   Serial.print(AngleMeasure); Serial.print("\t"); 
  //   Serial.println(AnglePID);
  //   timer_debug = timeNow;
  // }
}



void encoder1(void){
  if (digitalRead(ENCODER1_PIN_B) == 0){
    count1++;
  }else{
    count1--;
  }
}
float velocityEncoder1(void){
  //rpm
  //speedset = AnglePID /60 * 990 * 1000 * deltaTime;
  float vel = ((float)(count1 - pr_count1)/990) / (deltaTime * 0.001) * 60;
  pr_count1 = count1;
  if (deltaTime == 0) return 0;
  return abs(vel);//pulse per second
}
float PIDControlVel1(float error){
  i_error1 += error*deltaTime*0.001;
  float delta = error - pr_error1;
  float D = kd1 *(delta/ deltaTime*0.001);
  if (deltaTime == 0) D = 0;
  if(i_error1 > i_error_max1) i_error1 = i_error_max1;
  else if (i_error1 < -i_error_max1) i_error1 = -i_error_max1; 
  float out = kp1 * error + ki1 * i_error1 + D;
  pr_error1 = error;
  if(out > 255) return 255;
  else if (out < 0) return 0;
  else return out;// PWM
}



void encoder2(void){
  if (digitalRead(ENCODER2_PIN_B) == 0){
    count2++;
    }else{
    count2--;
  }
}
float velocityEncoder2(void){
  float vel = ((float)count2 - pr_count2)/990/ (deltaTime * 0.001) * 60;
  pr_count2 = count2;
  if (deltaTime == 0) return 0;
  return abs(vel);//pulse  per second
}
float PIDControlVel2(float error){
  i_error2 += error* deltaTime*0.001;
  float delta = error - pr_error2;
   if(i_error2 > i_error_max1) i_error2 = i_error_max1;
  else if (i_error2 < -i_error_max1) i_error2 = -i_error_max1; 
  float D = kd2 *(delta/ deltaTime*0.001);
  if (deltaTime == 0) D = 0;
  float out = kp2 * error + ki2 * (i_error2 ) + D;
  pr_error2 = error;
  if(out > 255) return 255;
  else if (out < 0) return 0;
  else return out; // PWM
}