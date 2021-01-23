#include "MyServoProtocol2.h"
MyServoProtocol2 servo(500000); 
int error = 0;

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(5); 


  // read EEPROM
  uint16_t Kp = 0;
  servo.readWordCommand(1,0x2A,1,&Kp,0);
  Serial.println(Kp);
  // read EEPROM
  servo.readWordCommand(2,0x2A,1,&Kp,0);
  Serial.println(Kp);
  // Backup Kp = 2000 (factory)
  // lower K for extreme compliance
  Kp = 500;
// uncomment here to set Kp to 500
  //servo.writeWordCommand(1,0x2A,1,&Kp,500,0);
  //servo.writeWordCommand(2,0x2A,1,&Kp,500,0);
// uncomment here to set Kp back to 2000
  //servo.writeWordCommand(1,0x2A,1,&Kp,2000,0);
  //servo.writeWordCommand(2,0x2A,1,&Kp,2000,0);

  
  // enable torque servo ID=1
  error = servo.enableTorque(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // disable torque servo ID=2
  error = servo.enableTorque(2);
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
  // switch to "velocity-profil current based position control" operating mode
  error = servo.setMode(2,2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // read servo ID1 position
  float present_position_1 = servo.getPosition(1,&error);
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
  float goal_position = (present_position_1+present_position_2)/2;
  // set servo ID1 position to present position, and limit velocity by 400 dps, motor current by 50mA and PWM ratio do 99% maximum
  error = servo.setPositionVelocityCurrentPWM(1,goal_position,400,250,99);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // set servo ID2 position to present position, and limit velocity by 400 dps, motor current by 50mA and PWM ratio do 99% maximum
  error = servo.setPositionVelocityCurrentPWM(2,goal_position,400,250,99);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Haptic");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Compliance");
}

void loop() {
  delay(2);
  float present_position_1 = servo.getPosition(1,&error);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  float present_position_2 = servo.getPosition(2,&error);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  float goal_position = (present_position_1+present_position_2)/2;
  error = servo.setPosition(1,goal_position);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  error = servo.setPosition(2,goal_position);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("S1<->S2");  
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("%3.0f*->%3.0f*",present_position_1,present_position_2);  
  M5.Lcd.setCursor(0, 200);
  M5.Lcd.printf("Goal:%3.0fmA",goal_position);  
  //Serial.print(".");
}
