#include "MyServoProtocol2.h"
MyServoProtocol2 servo(500000); 
int error = 0;

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(5); 

  // enable torque servo ID=1
  error = servo.enableTorque(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // disable torque servo ID=2
  error = servo.disableTorque(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // switch to "velocity-profil current based position control" operating mode
  error = servo.setMode(1,2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // read servo ID2 position
  float present_position_2 = servo.getPosition(2,&error);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // set servo ID1 position to present position, and limit velocity by 400 dps, motor current by 150mA and PWM ratio do 99% maximum
  error = servo.setPositionVelocityCurrentPWM(1,present_position_2,400,150,99);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Mirroring+");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Compliance");
}

void loop() {
  delay(2);
  float present_position_2 = servo.getPosition(2,&error);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  error = servo.setPosition(1,present_position_2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  float present_position_1 = servo.getPosition(1,&error);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  float present_current_1 = servo.getCurrent(1,&error);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("Srv2->Srv1");  
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("%3.0f*->%3.0f*",present_position_2,present_position_1);  
  M5.Lcd.setCursor(0, 200);
  M5.Lcd.printf("Srv1:%3.0fmA",present_current_1);  
}
