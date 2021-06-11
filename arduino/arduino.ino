
#include <Servo.h>

Servo kickerservo;
Servo intakeservo;
Servo holderservo;
Servo lifterservo;

int intakepin = 3;
int kickerpin = 4;
int batterypin = 5;



const int kickerActivePosition = 125;
const int kickerDefaultPosition = 90;

const int intakeActivePosition = 120;
const int intakeDefaultPosition = 90;

const int lifterLoweredPosition = 149;
const int lifterHoldingPosition = 110;
const int lifterLiftingPosition = 49;

const int holderOpenPosition = 110;
const int holderHoldingPosition = 140;
bool delayOrExitOnPin(int delay, int pin, bool state){
  unsigned long start = millis();
  while(millis() < start+delay){
    if(digitalRead(pin) == state)return true;
  }
  return false;
}
void executeBattery()
{
  lifterservo.write(lifterLoweredPosition);
  if(delayOrExitOnPin(400, batterypin,false)){
    lifterservo.write(lifterLiftingPosition);

    return;    
  }
  for(float i=0;i<1;i+=.003){
    lifterservo.write((lifterLiftingPosition-lifterLoweredPosition)*i+lifterLoweredPosition);
    delay(1);
    if(digitalRead(batterypin)==false){
      lifterservo.write(lifterLiftingPosition);
      return;
    }
  }
  while(digitalRead(batterypin)==true) delay(1);

  lifterservo.write(lifterHoldingPosition);
  delay(300);
  holderservo.write(holderHoldingPosition);

  delay(400);

  lifterservo.write(lifterLiftingPosition);
  delay(400);

  holderservo.write(holderOpenPosition);
  delay(200);

}

void setup() {

  pinMode(intakepin, INPUT);
  pinMode(kickerpin, INPUT);
  pinMode(batterypin, INPUT);

  enableHolderLifter();
  enableIntakeKicker();

}

void disableIntakeKicker(){
  kickerservo.detach();
  intakeservo.detach();
}
void enableIntakeKicker(){
  kickerservo.attach(9);
  intakeservo.attach(10);
  kickerservo.write(kickerDefaultPosition);
  intakeservo.write(intakeDefaultPosition);
}

void disableHolderLifter(){
  holderservo.detach();
  lifterservo.detach();
}
void enableHolderLifter(){
  holderservo.attach(11);
  lifterservo.attach(12);
  holderservo.write(holderOpenPosition);

  lifterservo.write(lifterLiftingPosition);
}

void loop() {

  if(digitalRead(kickerpin)){
    kickerservo.write(kickerActivePosition);
  }else{
    kickerservo.write(kickerDefaultPosition);
  }
  if(digitalRead(intakepin)){
    intakeservo.write(intakeActivePosition);
  }else{
    intakeservo.write(intakeDefaultPosition);
  }
  if(digitalRead(batterypin)){
    executeBattery();
  }
}