#include <Wire.h>
#include <string.h>
//dc motor
#define IN1	7
#define IN2	6
#define MAX_SPEED 150 //từ 0-255
#define MIN_SPEED 0
//cb
int cb1 = 22; int gtcb1; int demcb1 = 0;
int cb2 = 24; int gtcb2; int demcb2 = 0;
int macdinh = 1;
//python
int dem = 0;
String data = "";
String data1 = "";
char data2[20] = "";


char reply[] = {'9'};


void setup() {
  Serial.begin(115200);
  Wire.begin();
  // data1();
  pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
  pinMode (cb1, INPUT);
  pinMode (cb2, INPUT);
  
}

void loop() {
  gtcb1 = digitalRead(cb1);
  gtcb2 = digitalRead(cb2);

  while (digitalRead(cb1) == macdinh && digitalRead(cb2) == macdinh && demcb1 == 0 && demcb2 == 0)
  {
    demcb1 = 0;
    demcb2 = 0;
    motorRun1(MAX_SPEED);
  }

//////////////////////////////
  
//////////////////////////////
  while (digitalRead(cb2) != macdinh && demcb2 == 0)
  {
    motorStop();
    delay (50);
    while (Serial.available() > 0 && demcb2 == 0)
    {
      data = Serial.readStringUntil('\r');
      data1 = data;
      data = "";
      demcb2 ++;
      delay (50);
    }

  }
  if (demcb2 > 0 && demcb2 < 10)
  {
    while (digitalRead(cb1) != macdinh)
    {
      motorStop();
    }
    if (digitalRead(cb1) == macdinh)
    {
      motorRun1(MAX_SPEED);
    }
    demcb2 = 10;
    delay (50);
  }
  if (demcb2 == 10){
    int data1_len = data1.length() + 1;
    char arr[data1_len];
    data1.toCharArray(arr, data1_len);
    data2[0] = arr[0];
    demcb2 = 11;
    delay (50);
  }
  //cb 
  while (digitalRead(cb1) != macdinh && demcb2 == 11){
    motorStop();
    if (demcb1 == 0){
      Wire.beginTransmission(1);
      // Wire.write(data2[0]);
      Wire.write(data2);//
      Wire.endTransmission();
      demcb1 = 1;
    }
    delay (50);
  }
  while (digitalRead(cb1) != macdinh && demcb1 == 1){
    
  }
  if (demcb1 == 1 && demcb2 == 11) {
    demcb1 = 0;
    demcb2 = 0;
    data = "";
    delay (50);
  }
  
  //  while (Serial.available() == 0){
    
  // }
  
  // data = Serial.readStringUntil('\r');
  // Serial.print("Data : "); Serial.print(data);
  // Serial.println();

  
  //doc gt vi slaver
  // if (giatri == 1)
  // {
  //   Wire.requestFrom(1, 1);
  //   if (Wire.available() > 0 && giatri == 1) // đợi cho đến khi serial có tín hiệu
  //   {
  //     reply[0] = Wire.read(); // gán biến c đọc dữ liệu trong serial
  //     Serial.println (reply[0] -  48);
  //     if (reply[0] == '1')
  //     {
  //       giatri = 2;
  //     }
  //   }
  // }
  //DC motor
  
}
void motorStop() {
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
}
void motorRun1 (int speed) { //speed: từ 0 - MAX_SPEED
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	digitalWrite(IN1, HIGH);// chân này không có PWM
	analogWrite(IN2, 255 - speed);
}
void motorRun2(int speed) {
	speed = constrain(speed, MIN_SPEED, MAX_SPEED);//đảm báo giá trị nằm trong một khoảng từ 0 - MAX_SPEED - http://arduino.vn/reference/constrain
	digitalWrite(IN1, LOW);// chân này không có PWM
	analogWrite(IN2, speed);
}



