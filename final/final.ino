#include <Servo.h>

// FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

float analogVoltage;
int isSmoke;

// Define motor pins

#define AO 55 //ADC[1]
#define DO 54 //D54

#define PWMA 12 // Motor A PWM
#define DIRA1 34
#define DIRA2 35 // Motor A Direction
#define PWMB 8   // Motor B PWM
#define DIRB1 37
#define DIRB2 36 // Motor B Direction
#define PWMC 9   // Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42 // Motor C Direction
#define PWMD 5   // Motor D PWM
#define DIRD1 A4 // 26
#define DIRD2 A5 // 27  //Motor D Direction

#define BTSERIAL Serial2; 

// Servo definition

#define MOTORA_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRA1, LOW);  \
    digitalWrite(DIRA2, HIGH); \
    analogWrite(PWMA, pwm);    \
  } while (0)
#define MOTORA_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRA1, LOW); \
    digitalWrite(DIRA2, LOW); \
    analogWrite(PWMA, 0);     \
  } while (0)
#define MOTORA_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRA1, HIGH); \
    digitalWrite(DIRA2, LOW);  \
    analogWrite(PWMA, pwm);    \
  } while (0)

#define MOTORB_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRB1, LOW);  \
    digitalWrite(DIRB2, HIGH); \
    analogWrite(PWMB, pwm);    \
  } while (0)
#define MOTORB_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRB1, LOW); \
    digitalWrite(DIRB2, LOW); \
    analogWrite(PWMB, 0);     \
  } while (0)
#define MOTORB_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRB1, HIGH); \
    digitalWrite(DIRB2, LOW);  \
    analogWrite(PWMB, pwm);    \
  } while (0)

#define MOTORC_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRC1, LOW);  \
    digitalWrite(DIRC2, HIGH); \
    analogWrite(PWMC, pwm);    \
  } while (0)
#define MOTORC_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRC1, LOW); \
    digitalWrite(DIRC2, LOW); \
    analogWrite(PWMC, 0);     \
  } while (0)
#define MOTORC_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRC1, HIGH); \
    digitalWrite(DIRC2, LOW);  \
    analogWrite(PWMC, pwm);    \
  } while (0)

#define MOTORD_BACKOFF(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRD1, LOW);  \
    digitalWrite(DIRD2, HIGH); \
    analogWrite(PWMD, pwm);    \
  } while (0)
#define MOTORD_STOP(x)        \
  do                          \
  {                           \
    digitalWrite(DIRD1, LOW); \
    digitalWrite(DIRD2, LOW); \
    analogWrite(PWMD, 0);     \
  } while (0)
#define MOTORD_FORWARD(pwm)    \
  do                           \
  {                            \
    digitalWrite(DIRD1, HIGH); \
    digitalWrite(DIRD2, LOW);  \
    analogWrite(PWMD, pwm);    \
  } while (0)

#define SERIAL Serial

// PWM Definition
#define MAX_PWM 2000
#define MIN_PWM 300

int upperLimit = 100;
int fineThreshold = 100;

int Motor_PWM = 1900;
int A_PWM = 1200; // left front
int B_PWM = 1200; // right front
int C_PWM = 1200; // left back
int D_PWM = 1200; // right back

// Rotation needs different duty cycles
// TODO test optimal duty cycles for rotation
int A_rotate_PWM = 1200; // left front
int B_rotate_PWM = 1200; // right front
int C_rotate_PWM = 1200; // left back
int D_rotate_PWM = 1200; // right back

unsigned long time;
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
  MOTORB_FORWARD(1230);
  MOTORC_FORWARD(C_PWM);
  MOTORD_BACKOFF(D_PWM);
}

void RIGHT()
{
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

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  pinMode(AO, INPUT);
  pinMode(DO, INPUT);
  Serial.begin(115200);
  analogVoltage = analogRead(AO);
  isSmoke = digitalRead(DO);
  delay(1000);
  Serial2.begin(9600);
}

void loop() {
  
    if (Serial.available() > 0) {
        int label = Serial.parseInt();  // Read the incoming label
        // Serial.print("Received label: ");
        // Serial.println(label);
        bluetoothPrint(label);
        if (label == 1) {// thumbs up
          time = millis();
          while (millis() - time < 500) {
            rotate_left();
            ADVANCE();
          }
          STOP();
          delay(20);
          while (millis() - time < 500) {
            rotate_right();
            BACK();
          }
          STOP();
          // Serial.println(1);
        }
        else if (label == 2) { // ok
          time = millis();
          while (millis() - time < 500){
            ADVANCE();
          }
          STOP();
          delay(20);
          while (millis() - time < 1000){
            rotate_left();
          }
          STOP();
          delay(20);
          while (millis() - time < 500){
            BACK();
          }
          STOP();
          delay(20);
          while (millis() - time < 1000){
            rotate_right();
          }
          STOP();
          // Serial.println(2);
        }
        else if (label == 3) {// gimme five
          time = millis();
          while (millis() - time < 1000) {
            LEFT();
          }
          STOP();
          delay(20);
          while (millis() - time < 1000) {
            RIGHT();
          }
          STOP();
          // Serial.println(3);
        }
        else if (label == 4) {// 666
          time = millis();
          while (millis() - time < 300){
            LEFT();
          }
          STOP();
          delay(20);
          while (millis() - time < 500){
            rotate_left();
          }
          STOP();
          delay(20);
          while (millis() - time < 300){
            RIGHT();
          }
          STOP();
          delay(20);
          while (millis() - time < 500){
            rotate_right();
          }
          STOP();
          // Serial.println(4);
        }
        else if (label == 5) { // fist - 出离愤怒！！
          // Serial.print(1);
        }
        else { //illegal signal
          Serial.write("Invalid label");
        }
        analogVoltage = analogRead(AO);
        isSmoke = digitalRead(DO);
        // Serial.println(analogVoltage);
        Serial.println(isSmoke);
        delay(1000);
        delay(20);
    }
}
void bluetoothPrint(int label){
  String labels[5] = {"thumb up", "ok", "five", "six", "fist"};
  Serial2.print("Label: ");
}

