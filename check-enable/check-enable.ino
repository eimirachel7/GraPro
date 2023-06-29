#include <Wire.h>
#include <Servo.h> 
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>
//chan dong co
int stepPinX = 54; int dirPinX = 55; int enaPinX = 38;
int stepPinY = 60; int dirPinY = 61; int enaPinY = 56;
int stepPinZ = 46; int dirPinZ = 48; int enaPinZ = 62;
//end CTHT
#define endX 3
#define endY 14
#define endZ 18 
//dc motor
#define IN1	7
#define IN2	6
#define MAX_SPEED 125 //từ 0-255
#define MIN_SPEED 0
//nut an

//bien tro
int btX = A3; int btY = A4; int btZ = A5;
//bien doc ghi  vao EEPROM
int dcX1 = 0; int dcY1 = 1; int dcZ1 = 2;
int dcX2 = 0; int dcY2 = 1; int dcZ2 = 2;
byte gtX1; byte gtY1; byte gtZ1;
int gtX; int gtY; int gtZ;
byte gtX2; byte gtY2; byte gtZ2;

//bien doc ghi bien tro;
int gtbtX; int gtbtY; int gtbtZ;// gt hien tai
int bdX = 0; int bdY = 0; int bdZ = 0;//gt ban dau
int buocX; int buocY; int buocZ;//in gt sau khi map
//nut nhan
int cd1 = 16; int gtcd1; int demcd1 = 0;//cd1
int run1 = 17; int gtrun1; int demrun1 = 0;
int set1 = 23; int gtset1; int demset1 = 0;
int xoa1 = 25; int gtxoa1; int demxoa1 = 0;
int macdinh1 = 1; int macdinh2 = 1;
int cd2 = 32; int gtcd2; int demcd2 = 0;//cd2
#define SERVO_PIN 9
char dulieu = 0;
int s1;

unsigned long thoigian;//bien dong bo dong co thay delay
char read = '1';
char save;
char send [] = {'0'};

AccelStepper stepX(1, stepPinX, dirPinX, enaPinX);
AccelStepper stepY(1, stepPinY, dirPinY, enaPinY);
AccelStepper stepZ(1, stepPinZ, dirPinZ, enaPinZ);


Servo gServo;

void setup() {
  Serial.begin(115200);
  Wire.begin(1);
  Wire.onReceive(receiveEvent);// hàm nhận
  Wire.onRequest(requestEvent);// hàm yêu cầu
//khai bao cac chan tin hieu cua dong co  
  stepX.setEnablePin(38);
  stepX.setPinsInverted (false, false, true);
  stepY.setEnablePin(56);
  stepY.setPinsInverted (false, false, true);
  stepZ.setEnablePin(62);
  stepZ.setPinsInverted (false, false, true);
//khai bao tin hieu 3 CTHT 
  pinMode (endX, INPUT_PULLUP);
  pinMode (endZ, INPUT_PULLUP);
  pinMode (endY, INPUT_PULLUP);
//bien tro
  pinMode (btX, INPUT);
  pinMode (btY, INPUT);
  pinMode (btZ, INPUT);
//nut nhan
  pinMode (cd1, INPUT_PULLUP);
  pinMode (set1, INPUT_PULLUP); 
  pinMode (run1, INPUT_PULLUP); 
  pinMode (xoa1, INPUT_PULLUP);
  pinMode (cd2, INPUT_PULLUP);
//dau hut
  pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
//xet vi tri home
  homeYZ ();
  homeX ();
  // motorRun2 (MAX_SPEED);
//xet lai van toc gia toc cho dong co
  stepX.setMaxSpeed (200);
  stepX.setAcceleration(200);
  stepY.setMaxSpeed (150);
  stepY.setAcceleration(150);
  stepZ.setMaxSpeed (150);
  stepZ.setAcceleration(150);
  // motorRun2 (MAX_SPEED);
//
  thoigian = millis();
}

void loop() {
  //xet trang thai 
  gtbtX = analogRead (btX); 
  gtbtY = analogRead (btY); 
  gtbtZ = analogRead (btZ);
  gtcd1 = digitalRead (cd1);
  gtrun1 = digitalRead (run1); 
  gtset1 = digitalRead (set1); 
  gtxoa1 = digitalRead (xoa1);
  gtcd2 = digitalRead (cd2);
  //che do 1
  if (gtcd1 != macdinh1 && demcd2 == 0 )
  {
    if (gtcd1 == 0 && demcd2 == 0 ){
      if (demcd1 >= 1){
        demcd1 = 0;
      } else {
        demcd1 ++;
      }
    }
    macdinh1 = gtcd1;
    delay (100);
  }
  if (gtcd2 != macdinh2 )
  {
    if (gtcd2 == 0 ){
      if (demcd2 >= 1){
        demcd2 = 0;
      } else {
        demcd2 ++;
      }
    }
    macdinh2 = gtcd2;
    delay (100);
  }
  if (demcd1 == 1)
  {
    gtbtX = analogRead (btX); 
    gtbtY = analogRead (btY); 
    gtbtZ = analogRead (btZ);
    if (((gtbtX  > bdX + 5) or (gtbtX < bdX - 5)) or
      ((gtbtY  > bdY + 5) or (gtbtY < bdY - 5)) or
      ((gtbtZ  > bdZ + 5) or (gtbtZ < bdZ - 5)))
      {
        buocX = map (gtbtX, 0, 1023, 0, 1600);
        buocY = map (gtbtY, 0, 1023, 0, 600);
        buocZ = map (gtbtZ, 0, 1023, 0, 600);
        stepX.moveTo(buocX);
        stepY.moveTo(buocY);
        stepZ.moveTo(buocZ);
        while (stepX.distanceToGo() !=0 or stepY.distanceToGo() !=0 or stepZ.distanceToGo() !=0)
        {
          stepX.run();
          stepY.run();
          stepZ.run();
        }
        bdX = gtbtX;
        bdY = gtbtY;
        bdZ = gtbtZ;
      }
    if (gtrun1 != macdinh1)
    {
      if (gtrun1 == 0)
      {
        if (demrun1 >= 1){
          demrun1 = 0;
        } else {
          demrun1 ++;
        }
        macdinh1 = gtrun1;
        delay (100);
      }
    }
    if (demrun1 == 0)
    {
      if (gtset1 != macdinh1){
        if (gtset1 == 0)
        {
          demset1 ++;
          luulenh ();
        }
        macdinh1 = gtset1;
        delay (100);
      }
    }
    if (gtxoa1 != macdinh1 && demrun1 == 0){
      if (gtxoa1 == 0 && demrun1 == 0){
        demxoa1 ++;
        if (demxoa1 > 0){
          xoa ();
          Serial.print("dcX1: "); Serial.print(dcX1); Serial.print("dcX2: "); Serial.print(dcX2);
          Serial.println();
        demxoa1 = 0;
        }  
      }
        macdinh1 = gtxoa1;
        delay (100);
    }
    if (demrun1 == 1)
    {
      chaylenh ();
    }
  }
  Serial.print("cd1: "); Serial.print(demcd1); Serial.print("cd2: "); Serial.print(demcd2); Serial.print("run1: "); Serial.print(demrun1);
  Serial.println();  
  //che do 2
  if (gtcd2 != macdinh2 && demcd1 == 0 )
  {
    if (gtcd2 == 0 && demcd1 == 0 ){
      if (demcd2 >= 1){
        demcd2 = 0;
      } else {
        demcd2 ++;
      }
    }
    macdinh2 = gtcd2;
    delay (100);
  }
  if (demcd2 == 1)
  {
    run (500, 250, 260);
    if (read == 'Y')
    {
      run (500, 325, 355);
      delay (500);
      motorRun2(MAX_SPEED);
      delay (1000);
      run (500, 250, 260);
      delay (500);
      run (1100, 250, 260);
      delay (500);
      run (1100, 440, 520);
      delay (500);
      motorStop();
      delay (1000);
      read = '0';
    }
    if (read == 'D'){
      run (500, 325, 355);
      delay (500);
      motorRun2(MAX_SPEED);
      delay (1000);
      run (500, 250, 260);
      delay (500);
      run (850, 250, 260);
      delay (500);
      run (850, 440, 520);
      delay (500);
      motorStop();
      delay (1000);
      read = '0';
    }
    // if (read = 'v'){
    //   run (800, 200, 200);
    //   read = '0';
    // }
    // if (read = 'T'){
    //   run (800, 200, 200);
    //   read = '0';
    // }
    // if (read = 'V'){
    //   run (800, 200, 200);
    //   read = '0';
    // }
    // if (read = 'G'){
    //   run (800, 200, 200);
    //   read = '0';
    // }
    // if (read = '0'){

    // }
    
  }
  
}
void homeYZ(){
  int homeY = 0;
  int homeZ = 0;
  stepY.setMaxSpeed (100);
  stepY.setAcceleration(100);
  stepY.enableOutputs();
  stepZ.setMaxSpeed (100);
  stepZ.setAcceleration(100);
  stepZ.enableOutputs();
  while (digitalRead(endY) == 1 and digitalRead(endZ) == 1 ){
    stepY.moveTo(homeY);
    stepZ.moveTo(homeZ);
    homeY --;
    homeZ --;
    stepY.run();
    stepZ.run();
  }
  while (digitalRead(endY) == 1 ){
    stepY.moveTo(homeY);
    homeY --;
    stepY.run();
  }
  stepY.setCurrentPosition(0);
  
  while (digitalRead(endZ) == 1 ){
    stepZ.moveTo(homeZ);
    homeZ --;
    stepZ.run();
  }
  stepZ.setCurrentPosition(0);
  homeZ = 0; homeY = 0;
}
void homeX()
{
  int homeX = 0;
  stepX.setMaxSpeed (100);
  stepX.setAcceleration(100);
  stepX.enableOutputs();
  while (digitalRead(endX) == 1 ){
    stepX.moveTo(homeX);
    homeX --;
    stepX.run();
  }
  stepX.setCurrentPosition(0);
  homeX = 0;
}
void run (int X, int Y, int Z){
  stepX.moveTo(X);
  stepY.moveTo(Y);
  stepZ.moveTo(Z);
  while (stepX.distanceToGo() != 0 or stepY.distanceToGo() != 0 or stepZ.distanceToGo() != 0)
  {
    stepX.run();
    stepY.run();
    stepZ.run();
  }

}
void luulenh (){
  if (demset1 > 0)
    {   
      gtX1 = map (buocX, 0 , 1600, 0, 255);
      gtY1 = map (buocY, 0 , 600, 0, 255);
      gtZ1 = map (buocZ, 0 , 600, 0, 255);
      EEPROM.write(dcX1, gtX1);
      delay(50);
      EEPROM.write(dcY1, gtY1);
      delay(50);
      EEPROM.write(dcZ1, gtZ1);
      delay(50);
      Serial.print("           "); 
      Serial.print("Địa chỉ X: "); Serial.print(dcX1); Serial.print("   ");
      Serial.print("Góc X: "); Serial.println(buocX);
      Serial.print("Địa chỉ Y: "); Serial.print(dcY1); Serial.print("   ");
      Serial.print("Góc Y: "); Serial.println(buocY);
      Serial.print("Địa chỉ Z: "); Serial.print(dcZ1); Serial.print("   ");
      Serial.print("Góc Z: "); Serial.println(buocZ);
      Serial.println("  ");
      dcX1 = dcX1 + 3;
      dcY1 = dcY1 + 3;
      dcZ1 = dcZ1 + 3;
      if (dcZ1 == EEPROM.length()) 
        {
          dcX1 = 0;
          dcY1 = 1;
          dcZ1 = 2;
        } 
      demset1 = 0;   
    }
}
void chaylenh(){
   if (dcX2 < dcX1)
    {                                     
      gtX2 = EEPROM.read(dcX2);
      delay (50);
      gtY2 = EEPROM.read(dcY2);
      delay (50);
      gtZ2 = EEPROM.read(dcZ2);
      delay (50);
      gtX = map (gtX2, 0, 255, 0, 1600);
      gtY = map (gtY2, 0, 255, 0, 600);
      gtZ = map (gtZ2, 0, 255, 0, 600);
      run (gtX, gtY, gtZ);
      delay (50);   
      if ( (unsigned long) (millis() - thoigian) > 3000)
        { 
          thoigian = millis();
          Serial.print("           "); 
          Serial.print("ĐcX2: "); Serial.print(dcX2); Serial.print("   "); 
          Serial.print("ĐcY2: "); Serial.print(dcY2); Serial.print("   ");
          Serial.print("ĐcZ2: "); Serial.print(dcZ2); Serial.print("   ");
          Serial.print("Goc X2: "); Serial.println(gtX);   
          Serial.print("Goc Y2: "); Serial.println(gtY);
          Serial.print("Goc Z2: "); Serial.println(gtZ); 
          dcX2 = dcX2 + 3;  
          dcY2 = dcY2 + 3;
          dcZ2 = dcZ2 + 3;
        }

    }
      else
      {
        dcX2 = 0;  
        dcY2 = 1;
        dcZ2 = 2;
      }
}
void xoa(){
  for (int i = 0; i <= dcZ2; i ++){
    EEPROM.write(i, 0);
    delay (50);
  }
  dcX1 = 0; dcY1 = 1; dcZ1 = 2;
  dcX2 = 0; dcY2 = 1; dcZ2 = 2;
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


void receiveEvent(int howMany){
  read = Wire.read();
}

void requestEvent(){
  Wire.write(send);
}


