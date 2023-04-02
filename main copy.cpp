#include <Arduino.h>
// // // #include <Servo.h>
// // #include <Wire.h>
// // #include <Adafruit_PWMServoDriver.h>

// #define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
// // #define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
// // #define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
// // #define SERVO_FREQ 1600 // Analog servos run at ~50 Hz updates

// // #define LEG_A 55
// // #define LEG_B 95
// // #define LEG_E 23
// #define RADIUS 25
// #define SHITY 95

// // Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);

// // double rad(double degree){
// //   return (degree * (double)71) / (double)4068;
// // }

// // void serv(int servoNum, int angle){
// //   pwm.setPWM(servoNum, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
// // }

// // double calc_angle_1(double x, double y){
// //   double sum = x * x + y * y;
// //   double sq = sqrt(sum);
// //   return acos(-x / sq) - acos((sum + LEG_A + LEG_B) / (2 * LEG_A * sq));
// // }

// // double calc_angle_2(double x_, double y){
// //   double x = x_ - LEG_E;
// //   double sum = x * x + y * y;
// //   double sq = sqrt(sum);
// //   return acos(x / sq) - acos((sum + LEG_A + LEG_B) / (2 * LEG_A * sq));
// // }

// // void move(){
// //   // static double angle = 0;
// //   // double rads = rad(angle);
// //   // double x = cos(rads);
// //   // double y = sin(rads);

// //   // double angle1 = calc_angle_1(angle)

// //   // angle += 1;
// // }

// // void setup() {
// //   Serial.begin(9600);
// //   // pwm.setOscillatorFrequency(27000000);
// //   pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
// //   delay(100);

// // }

// // void loop() {
// //   serv(0, 0);
// //   serv(1, 0);
// //   delay(100);
// //   Serial.println("sus");
// //   // move();
// //   // delay(10);
// // }

// /***************************************************
//   This is an example for our Adafruit 16-channel PWM & Servo driver
//   PWM test - this will drive 16 PWMs in a 'wave'
//   Pick one up today in the adafruit shop!
//   ------> http://www.adafruit.com/products/815
//   These drivers use I2C to communicate, 2 pins are required to
//   interface.
//   Adafruit invests time and resources providing this open source code,
//   please support Adafruit and open-source hardware by purchasing
//   products from Adafruit!
//   Written by Limor Fried/Ladyada for Adafruit Industries.
//   BSD license, all text above must be included in any redistribution
//  ****************************************************/

// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// // called this way, it uses the default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// // you can also call it with a different address you want
// //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// // you can also call it with a different address and I2C interface
// //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// void setup() {
//   Serial.begin(9600);
//   Serial.println("16 channel PWM test!");

//   pwm.begin();
//   /*
//    * In theory the internal oscillator (clock) is 25MHz but it really isn't
//    * that precise. You can 'calibrate' this by tweaking this number until
//    * you get the PWM update frequency you're expecting!
//    * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
//    * is used for calculating things like writeMicroseconds()
//    * Analog servos run at ~50 Hz updates, It is importaint to use an
//    * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
//    * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
//    *    the I2C PCA9685 chip you are setting the value for.
//    * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
//    *    expected value (50Hz for most ESCs)
//    * Setting the value here is specific to each individual I2C PCA9685 chip and
//    * affects the calculations for the PWM update frequency.
//    * Failure to correctly set the int.osc value will cause unexpected PWM results
//    */
//   pwm.setOscillatorFrequency(27000000);
//   pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

//   // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
//   // some i2c devices dont like this so much so if you're sharing the bus, watch
//   // out for this!
//   Wire.setClock(400000);
// }

// void loop() {
//   pwm.writeMicroseconds(0, 1500);
//   pwm.writeMicroseconds(1, 1500);
//   delay(100);
// }

// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#include <Servo.h>
#include <math.h>

#define BUTTON 11

#define LEG_A 55
#define LEG_B 95
#define LEG_E 23

#define SHITY 40
#define SHITX LEG_E / 2

#define SHITFLL -7
#define SHITFLR 7

#define STEP_SIZE 4

double legAnB = LEG_A * LEG_A - LEG_B * LEG_B;
double doubleLegA = 2 * LEG_A;

int precompLeft[360];
int precompRight[360];

#define SPEED_tic 1400
double dotsX[] = {1.875, 0 , -1.875, -3.75, 0, 3.75};//16 SEc 750 mis
double dotsY[] = {0, 0, 0, 1.65 ,3.5, 0};


// double dotsX[] = {-2.5, 0, 2.5};
// double dotsY[] = {0, 4, 0};

// double dotsX[] = {-2,2,2.25, 1,-1,-2.25};
// double dotsY[] = {3,3,0, 0,0,0};
int dotsCount = 6;
double rad(double degree)
{
  return degree * 0.0174533;
}




double deg(double radian)
{
  return radian * 57.2958;
}

float Q_rsqrt(float number)
{
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y = number;
  i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y));

  return 1.0F / y;
}

float fastacos(float x)
{
  float negate = float(x < 0);
  float ret = -0.0187293;
  x = abs(x);
  ret = x * -0.0187293;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret *= Q_rsqrt(1.0 - x);
  ret = ret - 2.0 * negate * ret;
  return negate * 3.14159265358979 + ret;
}

class Leg
{
public:
  Leg(int pinLeftNum, int pinRightNum, double shiftLeft, double shiftRight, double servoDistance, int direction, int directionX)
  {
    servoLPin = pinLeftNum;
    servoRPin = pinRightNum;
    shiftL = shiftLeft;
    shiftR = shiftRight;
    distance = servoDistance;
    dir = direction;
    dirX = directionX;
  };

  void attach()
  {
    servoL.attach(servoLPin);
    servoR.attach(servoRPin);
  }

  double calc_angle_1(double x, double y)
  {
    double sum = x * x + y * y;
    double sq = Q_rsqrt(sum);
    return fastacos(-x / sq) - fastacos((sum + legAnB) / (doubleLegA * sq));
  }

  double calc_angle_2(double x_, double y)
  {
    double x = x_ - distance;
    double sum = x * x + y * y;
    double sq = Q_rsqrt(sum);
    return fastacos(x / sq) - fastacos((sum + legAnB) / (doubleLegA * sq));
  }

  void move(float angle, int & out1, int & out2)
  {
    static double lastX = 0, lastY = 0, prevX = 0, prevY = 0;
    static int coord = 0;
    static unsigned long nextTimer = millis() + 1000;
    // double rads = rad((double)angle);
    double x;
    double y;

    // x = 50 + SHITX;
    // y = 0 + SHITY;
    // if ((angle >= 0 && angle <= 90) || (angle > 270))
    // {
    //   x = sin(rads) * STEP_SIZE * dirX + SHITX;
    //   y = SHITY * 1;
    //   Serial.println("Line");
    // }
    // else
    // {

    //   x = sin(rads) * STEP_SIZE * dirX + SHITX;
    //   y = cos(rads) * STEP_SIZE * 2 + SHITY;
    //   Serial.println("Circle");
    // }

    // if(angle<=100){

    //   y = 0;
    //   x = (-STEP_SIZE*(100-angle)/100)+((STEP_SIZE*angle)/100)*1.5;
    // }
    // else if(angle<=200){
    //   x = STEP_SIZE * (200 - angle)/100*1.5;
    //   y = STEP_SIZE * (angle-100)/100;
    // }
    // else if(angle<=300)
    // {
    //   x = STEP_SIZE * (300 - angle)/100*1.5;
    //   y = STEP_SIZE * (300 - angle)/100;
    // }

    // Serial.print(angle);
    // Serial.print("a|");

    // Serial.print(x);
    // Serial.print("x|");

    // Serial.print(y);
    // Serial.print("y|");

    // x = ((prevX - dotsX[coord]) * millis() * STEP_SIZE) / nextTimer + SHITX + prevX;
    // y = ((prevY - dotsY[coord]) * millis() * STEP_SIZE) / nextTimer + SHITY + prevY;

    // Serial.println(x);
    int indexL = floor(angle);
    int index_ = (((int)floor(angle) + 1) % dotsCount) ;
    float time_ = angle - indexL;
    x = dotsX[indexL] * (1 - time_)*STEP_SIZE + dotsX[index_] * (time_)*STEP_SIZE;
    y = dotsY[indexL] * (1 - time_)*STEP_SIZE + dotsY[index_] * (time_)*STEP_SIZE;

    // Serial.print(angle);
    // Serial.print("a|");
    // Serial.print(x);
    // Serial.print("x|");
    // Serial.print(y);
    // Serial.print("y|");
    //  Serial.print(index_);
    // Serial.print("i|");
    // Serial.print(indexL);
    // Serial.println("iL|");

    x += SHITX;
    y += SHITY;
    double angle1 = deg(calc_angle_1(x, y));
    double angle2 = -deg(calc_angle_2(x, y));
    angle1 *= dir;
    angle2 *= dir;
    angle1 += 90;
    angle2 += 90;
    Serial.print(angle1);
    Serial.print(" ");
    Serial.println(angle2);

    out1 = (int)angle1;
    out2 = (int)angle2;

    // servoL.write(angle1);
    // servoR.write(angle2);

    // if (millis() >= nextTimer){
    //   Serial.println("O");
    //   prevX = x;
    //   prevY = y;
    //   nextTimer = millis() + 1000;
    //   coord = (coord + 1) % 3;
    // }
  };

  void write(int angle)
  {
    servoL.write(angle + shiftL);
    servoR.write(angle + shiftR);
  }

  void write(int angle1, int angle2){
    servoL.write(((angle1-90) * dir)+90 + shiftL);
    servoR.write(((angle2-90) * dir)+90 + shiftR);
  }

private:
  double shiftL;
  double shiftR;
  double distance;

  int servoLPin;
  int servoRPin;

  int dir;
  int dirX;

  Servo servoL;
  Servo servoR;
};

Leg legBackLeft(3, 2, 9, 4, LEG_E, 1, -1);   // 1
Leg legBackRight(4, 5, -2, 10, LEG_E, -1, -1); // 2

Leg legFrontLeft(7, 6, 8, 0, LEG_E, 1, 1);    // 3
Leg legFrontRight(8, 9, 9, -2, LEG_E, -1, 1); // 4

// Servo servoL;
// Servo servoR;

// void move()
// {
//   for (float i = 0; i < dotsCount; i += 0.05)
//   {
//     legFrontLeft.move(fmod(i+1.5,dotsCount));
//     legBackRight.move(fmod(i+1.5,dotsCount));
//     legBackLeft.move(i);
//     legFrontRight.move(i);
//   }
//   }
  // while(digitalRead(BUTTON))
  //   ;
  // delay(1000);


// void move1()
// {
//   int offset = 285;

//   for (int i = 0; i < 180; i += 15)
//   {
//     legFrontLeft.move((i + offset) % 360);
//   }
//   for (int i = 0; i < 180; i += 15)
//   {
//     legFrontRight.move((i + offset) % 360);
//   }
//   for (int i = 0; i < 180; i += 15)
//   {
//     legBackLeft.move((i + offset) % 360);
//   }
//   for (int i = 0; i < 180; i += 15)
//   {
//     legBackRight.move((i + offset) % 360);
//   }
//   for (int i = 180; i < 360; i += 15)
//   {
//     legFrontLeft.move((i + offset) % 360);
//     legFrontRight.move((i + offset) % 360);
//     legBackLeft.move((i + offset) % 360);
//     legBackRight.move((i + offset) % 360);
//     delay(200);
//   }

  // while(digitalRead(BUTTON))
  //   ;
  // delay(1000);
// }

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  Serial.begin(9600);
  delay(100);
  legFrontLeft.attach();
  legFrontRight.attach();

  legBackLeft.attach();
  legBackRight.attach();
  legFrontLeft.write(90);
  legFrontRight.write(90);
  legBackLeft.write(90);
  legBackRight.write(90);

  // servoL.attach(2);
  // servoR.attach(3);

  // servoL.write(0);
  // servoR.write(0);
  // move();

  // legFrontLeft.write(0);
  const float step__ = (float)dotsCount / 360.0F;
  for (float i = 0; i < dotsCount; i += step__){
    int index = (i * 360) / dotsCount;
    legBackLeft.move(i, precompLeft[index], precompRight[index]);
  }
  digitalWrite(13, HIGH);
  // for (int i = 0; i < 360; i++){
  //   Serial.println(precompLeft[i]);
  // }
    while (digitalRead(BUTTON))
      ;
}

void move(){
  static int index = 0;
  int otherIndex = (index + 180) % 360;
  legBackLeft.write(precompLeft[index], precompRight[index]);
  legFrontRight.write(precompLeft[index], precompRight[index]);

  legBackRight.write(precompLeft[otherIndex], precompRight[otherIndex]);
  legFrontLeft.write(precompLeft[otherIndex], precompRight[otherIndex]);

  index = (index + 1) % 360;
}

void loop()
{
  // legFrontLeft.write(0);
  // servoL.write(90);
  // servoR.write(90);
  move();
  delayMicroseconds(SPEED_tic);
  // while(true)
  //   ;
  // legFrontLeft.write(90);
  // legFrontRight.write(90);

  // legBackLeft.write(90);
  // legBackRight.write(90);
  // unsigned long start = millis();
  // while(millis() - start > 50)
  //   ;
}