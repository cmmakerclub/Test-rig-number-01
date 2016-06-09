#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>
#include <SPI.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;



#define VR_1 0
#define VR_2 1
#define VR_3 2
#define VR_4 6
#define VR_5 3


#define Gry_offset 0.00
#define Gyr_Gain 0.00763358
#define Angle_offset -2.0f
#define offset_motor1  1275
#define offset_motor2  1275
#define pi 3.14159

void myPID(void);
void PWMControl(int set_motor1, int set_motor2);
float limit(float input, int min_limit, int max_limit);
float smooth(float alfa, float new_data, float old_data);

float P_CompCoeff = 0.95;

long data;
int x, y;
float kp = 1, ki = 0, kd = 0;
float r_angle, f_angle, output_pid;
float Turn_Speed = 0, Turn_Speed_K = 0;
float Run_Speed = 0, Run_Speed_K = 0, Run_Speed_T = 0;
float LOutput, ROutput;

unsigned long preTime = 0;
float SampleTime = 0.08;
unsigned long lastTime;
float Input, Output;
float errSum, dErr, error, lastErr;
int timeChange;

uint8_t buf[32];
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 10;
int8_t motor_a_m, motor_b_m, angle_m, error_m, error_dot_m, kp_m, ki_m, kd_m;
int Output_m;
uint32_t time_now, time_prev, time_prev2;


Servo myservo1;
Servo myservo2;

float vr1, vr2, vr3, vr4, vr5, vr6;


void setup()
{
  pinMode(VR_1, INPUT);
  pinMode(VR_2, INPUT);
  pinMode(VR_3, INPUT);
  pinMode(VR_4, INPUT);
  pinMode(VR_5, INPUT);

  Wire.begin();
  delay(300);
  accelgyro.initialize();
  myservo1.attach(10);
  myservo2.attach(9);
  myservo1.writeMicroseconds(1000);
  myservo2.writeMicroseconds(1000);
  delay(3000);

  Serial.begin(115200);
}

void loop()
{
  time_now = millis();

  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    ///////////////////////////////////////////////////
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //  ay  az gx

    kp = vr1 * .005f ;
    ki = vr2 * .01f ;
    kd = vr3 * .01f ;
    Run_Speed_T = vr4 * 0.25f ;
    r_angle = (-vr5 + 512) * 0.04f ;

    //    Serial.print("data : ");
    //    Serial.print(kp);         Serial.print("  ");
    //    Serial.print(ki);        Serial.print("  ");
    //    Serial.print(kd);        Serial.print("  ");
    //    Serial.print(Run_Speed_T);        Serial.print("  ");
    //    Serial.println(r_angle);

    f_angle += ((gx / 32.8f) * (0.01f));
    float pitchAcc = atan2(ay, az) * RAD_TO_DEG;
    f_angle = P_CompCoeff * f_angle + (1.0f - P_CompCoeff) * pitchAcc;

    // Serial.print(pitchAcc); Serial.print("\t");
    Serial.println(f_angle);

    if (vr2 < 20)  errSum = 0;

    if (Run_Speed_T > 50)
    {
      myPID();

      PWMControl( LOutput,  ROutput);
    } else {
      errSum = 0;
      PWMControl( 0,  0);
    }
  }

  if (time_now - time_prev2 >= 20)
  {
    static int8_t motor_a_m, motor_b_m;
    time_prev2 = time_now;
    motor_a_m = (uint8_t)smooth(0.6f, ((((int)limit(LOutput, 1050, 1500)) - 1000) / 5), motor_a_m);
    motor_b_m = (uint8_t)smooth(0.6f, ((((int)limit(ROutput, 1050, 1500)) - 1000) / 5), motor_a_m);
    int8_t  angle_m = (int8_t)f_angle;
    int8_t   error_m = (int8_t)r_angle;
    int8_t   kp_m = (int8_t)(kp * 20.0f);
    int8_t   ki_m = (int8_t)(ki * 10.0f);
    int8_t   kd_m = (int8_t)(kd * 10.0f);
    Serial.print("&");
    Serial.print(",");
    Serial.print((int8_t)(motor_a_m)); //ความแรงเป็น%มอเตอร์ซ้าย
    Serial.print(",");
    Serial.print((int8_t)(motor_b_m)); //ความแรงเป็น%มอเตอร์ขวา
    Serial.print(",");
    Serial.print(angle_m);  // มุมจากใจโร
    Serial.print(",");
    Serial.print(error_m); //มุมที่ต้องการ
    Serial.print(",");
    Serial.print(kp_m); //  kp
    Serial.print(",");
    Serial.print(ki_m); //  ki
    Serial.print(",");
    Serial.print(kd_m); //  kd
    Serial.println(",");
  }
  vr1 = smooth(0.01f, (analogRead(VR_1) ), vr1);
  vr2 = smooth(0.01f, (analogRead(VR_2) ), vr2);
  vr3 = smooth(0.01f, (analogRead(VR_3) ), vr3);
  vr4 = smooth(0.01f, (analogRead(VR_4) ), vr4);
  vr5 = smooth(0.01f, (analogRead(VR_5) ), vr5);

}


void myPID()
{
  lastErr = error;
  error = r_angle - f_angle;
  dErr = (error - lastErr) * 15.0f;
  errSum = limit(errSum + (error * 0.01f), -50, 50);

  output_pid = (kp * error) + (ki * errSum) + (kd * dErr);

  LOutput = Run_Speed_T - output_pid + 1000;
  ROutput = Run_Speed_T + output_pid + 1000;

}

void PWMControl(int set_motor1, int set_motor2)
{
  myservo1.writeMicroseconds(limit(set_motor1, 1000, 1500));
  myservo2.writeMicroseconds(limit(set_motor2, 1000, 16500));
}

float limit(float input, int min_limit, int max_limit)
{
  if (input > max_limit)input = max_limit;
  if (input < min_limit)input = min_limit;
  return input;
}

float smooth(float alfa, float new_data, float old_data)
{
  return (old_data + alfa * (new_data - old_data));
}
