#include <Servo.h>

#include <INA226.h>

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

INA226 ina;
Servo servo;

//TODO Bluetooth transmission
//TODO rightleft/right ultrasonic
//TODO charging data feedback

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

int count = 0;//begin delay

//first distance
int outDistance = 0;
int inDistance = 0;

//forward distance measurement
int averageDistance = 0;

//Distance to the station
int firstDistance = 0;
int newDistance = 0;

//Ultrasonic settings
#define leftEchoPin 33
#define leftTriggerPin 32
#define rightEchoPin 29
#define rightTriggerPin 28
#define fineLeftEchoPin 48
#define fineLeftTriggerPin 47
#define fineRightEchoPin 45
#define fineRightTriggerPin 44

long fineLeftDistance;
long fineRightDistance;
long leftDistance;
long rightDistance;

unsigned long time;
unsigned long timem;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

// Servo definition
#define servoPin 41

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int upperLimit = 100;
int fineThreshold = 100;

int Motor_PWM = 1900;
int A_PWM = 950; //left front
int B_PWM = 1000; // right front
int C_PWM = 950; // left back
int D_PWM = 950; // right back

// Rotation needs different duty cycles
// TODO test optimal duty cycles for rotation
int A_rotate_PWM = 950; //left front
int B_rotate_PWM = 1000; // right front
int C_rotate_PWM = 950; // left back
int D_rotate_PWM = 950; // right back

float targetShuntVoltage = 5.0;
float currentShuntVoltage = 0;

//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;     
int int_left, int_right;

// Supportive flags
bool isParallel1 = false; // Parallel check at the beginning
bool isParallel2 = false;
bool moveLeft = false;
bool moveForward = false;
bool left = false;
bool right = false;

void BACK()
{
  MOTORA_BACKOFF(A_PWM); 
  MOTORB_BACKOFF(B_PWM);
  MOTORC_BACKOFF(C_PWM); 
  MOTORD_BACKOFF(D_PWM);
}

void ADVANCE()
{
  MOTORA_FORWARD(A_PWM); 
  MOTORB_FORWARD(B_PWM);
  MOTORC_FORWARD(C_PWM); 
  MOTORD_FORWARD(D_PWM);
}

void LEFT()
{
  MOTORA_BACKOFF(A_PWM);
  MOTORB_FORWARD(B_PWM);
  MOTORC_FORWARD(C_PWM);
  MOTORD_BACKOFF(D_PWM);
}

void RIGHT() {
  MOTORA_FORWARD(A_PWM);
  MOTORB_BACKOFF(B_PWM);
  MOTORC_BACKOFF(C_PWM);
  MOTORD_FORWARD(D_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_left()
{
  MOTORA_BACKOFF(A_rotate_PWM); 
  MOTORB_FORWARD(B_rotate_PWM);
  MOTORC_BACKOFF(C_rotate_PWM); 
  MOTORD_FORWARD(D_rotate_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_right()
{
  MOTORA_FORWARD(A_rotate_PWM);
  MOTORB_BACKOFF(B_rotate_PWM);
  MOTORC_FORWARD(C_rotate_PWM);
  MOTORD_BACKOFF(D_rotate_PWM);
}

void STOP()
{
  MOTORA_STOP(A_PWM);
  MOTORB_STOP(B_PWM);
  MOTORC_STOP(C_PWM);
  MOTORD_STOP(D_PWM);
} 

void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot

  //Ultrasonic serial setup
  pinMode(leftEchoPin, INPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(fineLeftEchoPin, INPUT);
  pinMode(fineRightEchoPin, INPUT);
  pinMode(leftTriggerPin, OUTPUT);
  pinMode(rightTriggerPin, OUTPUT);
  pinMode(fineLeftTriggerPin, OUTPUT);
  pinMode(fineRightTriggerPin, OUTPUT);

  Serial.println("pinmode setup complete");

  ///INA226 setup///
  //Default INA226 address is 0x40
  ina.begin();
  // Configure INA226
  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  // Calibrate INA226. Rshunt = 0.0015 ohm, Max excepted current = 4A
  ina.calibrate(0.01, 4);

  Serial3.begin(9600); // BT serial setup
  // Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);

  Serial.println("INA226 setup complete");
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(40, 10);     // Start at top-left corner
  display.println("MIKU");
  display.display();

  Serial.println("LED setup complete");
  //Servo setup
  pinMode(A0,INPUT);
  servo.attach(servoPin);

  //Photoresistor calibration
  

  //First ultrasonic readings
  measure();
  // left or right side to start
  delay(3000);
  ledprint();
  Serial.println("GOOD");
}

void loop()
{
  measure();
  while (fineLeftDistance < fineThreshold || fineRightDistance < fineThreshold || abs(leftDistance - rightDistance) > 2) {
    measure();
    if (leftDistance > rightDistance) { //rotate left
      rotate_right();
      delay(30);
    }
    else {
      rotate_left();
      delay(30);
    }
    STOP();
    measure();  
    delay(700);
  }

  measure();
  left = fineLeftDistance < fineRightDistance;
  right = fineLeftDistance > fineRightDistance;
  directionprint();
  delay(5000);
  statusprint();

  

  //left & right shift
  inDistance = left ? rightDistance : leftDistance;
  outDistance = left ? leftDistance : rightDistance;
  firstDistance = (leftDistance + rightDistance) / 2;
  while (true) {
    while (!(abs(leftDistance - rightDistance) <= 1)){
      if (leftDistance < rightDistance && abs(leftDistance - rightDistance) <=10) { 
        rotate_left();
        delay(20);
      }
      else {
        rotate_right();
        delay(20);
      }
      STOP();
      measure();
      delay(700);
    }

    if (rightDistance >= upperLimit) {
      for (int i = 0;i < 3; i++) {
        rotate_left();
        delay(20);
      }
    }
    if (leftDistance >= upperLimit) {
      for (int i = 0;i < 3; i++) {
        rotate_right();
        delay(20);
      }
    }
    if(left) {
      RIGHT();
    }
    else if (right) {
      LEFT();
    }
    else {
      STOP();
      display.clearDisplay();
      display.println("error");
    }
    delay(50);
    STOP();
    ledprint();
    delay(500);
    measure();
    inDistance = left ? rightDistance : leftDistance;
    outDistance = left ? leftDistance : rightDistance;
    
    // determination of arrival
    if (abs(inDistance - firstDistance) >= 8) {
      newDistance = inDistance;
      targetprint();
    }

    //termination
    if (newDistance != 0 && abs(outDistance - newDistance) <= 2) {break;}
  }
  STOP();
  delay(1000);
  statusprint();

  //Move forward
  measure();
  averageDistance = (leftDistance + rightDistance) / 2;
  
  currentShuntVoltage = ina.readShuntVoltage();
  while(currentShuntVoltage < targetShuntVoltage){
    ADVANCE();
    delay(50);
    STOP();

    measure();
    while (!(abs(leftDistance - rightDistance) <= 1)){
      if (leftDistance < rightDistance && abs(leftDistance - rightDistance) <=10) { 
        rotate_left();
        delay(20);
      }
      else {
        rotate_right();
        delay(20);
      }
      STOP();
      measure();
      delay(200);
    }

    currentShuntVoltage = ina.readShuntVoltage();
    voltageprint();
    Serial3.println(currentShuntVoltage);
  }
  STOP();
  statusprint();
  delay(10000);

  // Move back and park
  accurate_measure();
  averageDistance = (leftDistance + rightDistance) / 2;
  while (averageDistance <= 8) {
    BACK();
    delay(50);
    STOP();

    measure();
    while (!(abs(leftDistance - rightDistance) <= 1)){
      if (leftDistance < rightDistance && abs(leftDistance - rightDistance) <=10) { 
        rotate_left();
        delay(20);
      }
      else {
        rotate_right();
        delay(20);
      }
      STOP();
      measure();
      delay(500);
    }

    accurate_measure();
  }

  // //final rotation
  // while (!abs(leftDistance - rightDistance) == 0){
  //   measure();
  //   if (leftDistance > rightDistance) { //rotate left
  //     rotate_right();
  //     delay(5);
  //    }
  //   else {
  //     rotate_left();
  //     delay(5);
  //   }
  //   STOP();
  //   measure();  
  //   ledprint();
  //   delay(1000);
  // }
}

int measure_left_distance(){
  unsigned long leftDuration;

  digitalWrite(leftTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTriggerPin, LOW);
  
  leftDuration = pulseIn(leftEchoPin, HIGH);
  int leftD = (leftDuration / 2.0) / 29.1;

  return leftD;
}

int measure_right_distance() {
  unsigned long rightDuration;

  digitalWrite(rightTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTriggerPin, LOW);
  
  rightDuration = pulseIn(rightEchoPin, HIGH);
  int rightD = (rightDuration/2.0) / 29.1;

  return rightD;
}

int measure_fine_left_distance() {
  unsigned long leftDuration;

  digitalWrite(fineLeftTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(fineLeftTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(fineLeftTriggerPin, LOW);
  
  leftDuration = pulseIn(fineLeftEchoPin, HIGH);
  int leftD = (leftDuration / 2.0) / 29.1;
  return leftD;
}

int measure_fine_right_distance() {
  unsigned long rightDuration;

  digitalWrite(fineRightTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(fineRightTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(fineRightTriggerPin, LOW);
  
  rightDuration = pulseIn(fineRightEchoPin, HIGH);
  int rightD = (rightDuration/2.0) / 29.1;
  return rightD;
}

void accurate_measure() { // The last measurement.
  int leftM = 0;
  int rightM = 0;

  for (int i = 0; i < 3; i++) {
    leftM = leftM + measure_left_distance();
    rightM = rightM + measure_right_distance();
    delay(100);
  }

  // leftDistance = leftM / 3;
  // rightDistance = rightM / 3;
  leftDistance = leftM / 3;
  rightDistance = rightM / 3;
}

void measure() {
  Serial.println("measure");
  int leftM = 0;
  int rightM = 0;
  int fineLeftM = 0;
  int fineRightM = 0;

  for (int i = 0; i < 3; i++) {
    leftM = leftM + measure_left_distance();
    rightM = rightM + measure_right_distance();
    fineLeftM = fineLeftM + measure_fine_left_distance();
    fineRightM = fineRightM + measure_fine_right_distance();
    delay(20);
  }

  leftDistance = leftM / 3;
  rightDistance = rightM / 3;
  fineLeftDistance = fineLeftM / 3;
  fineRightDistance = fineRightM / 3;

  // int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
  // int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m; 

  ledprint();
}

void ledprint() {
  Serial.println("ledprint");
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);     // Start at top-left corner
  display.print("left: ");
  display.print(leftDistance);
  display.println("");
  display.print("right: ");
  display.print(rightDistance);
  display.println("");
  display.print("fleft: ");
  display.print(fineLeftDistance);
  display.println("");
  display.print("fright: ");
  display.print(fineRightDistance);
  Serial.println(leftDistance);
  Serial.println(rightDistance);
  Serial.println(fineLeftDistance);
  Serial.println(fineRightDistance);
  // display.println("");
  if (left) {
    display.print(" 2 right");
  }
  else if (right) {
    display.println(" 2 left");
  }
  else {
    display.println(" wtf");
  }
  display.display();
}

void statusprint() {
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);     // Start at top-left corner
  display.println("stage finished");
  display.display();
  delay(5000);
}

void voltageprint() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);     // Start at top-left corner
  display.print("Shunt voltage: ");
  display.println(currentShuntVoltage);
}

void targetprint() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);     // Start at top-left corner
  display.print("New distance found: ");
  display.println(newDistance);
  display.println("Original distance: ");
  display.println(firstDistance);
}

void directionprint() {
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0,0);     // Start at top-left corner
  display.print("fine left: ");
  display.println(fineLeftDistance);
  display.print("fine right: ");
  display.println(fineRightDistance);
  display.println(left);
}