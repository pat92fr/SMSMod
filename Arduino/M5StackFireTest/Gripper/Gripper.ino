#include "MyServoProtocol2.h"
MyServoProtocol2 servo(500000); 
int error = 0;
int state = 0; //0:open, 1:close

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(5); 

  // enable torque servo ID=2
  error = servo.enableTorque(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // switch to "velocity-profil current based position control" operating mode
  error = servo.setMode(2,2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // set position to 50° (home), limit velocity by 50 dps, motor current by 60mA and PWM ratio do 50% maximum
  error = servo.setPositionVelocityCurrentPWM(2,30,20,50,30);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Gripper");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Open");
  delay(1000);
}

void loop() {
  if(M5.BtnC.read())
  {
    state = 1;    
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("Pick!  ");
    error = servo.setPosition(2,120.0f);
  
  }
  if(M5.BtnA.read())
  {
    state = 0;    
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf("Drop!  ");
    error = servo.setPosition(2,50.0f);
  }
  float present_current = servo.getCurrent(2,&error);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("Srv1:%3.0fmA",present_current);
  delay(10);
}
