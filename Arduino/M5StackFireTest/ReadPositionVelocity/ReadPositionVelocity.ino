#include "MyServoProtocol2.h"
MyServoProtocol2 servo(500000); 

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(5); 

  // disable torque servo ID=1
  int error = servo.disableTorque(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }

   
}

void loop() {
  delay(100);
  int error = 0;
  float present_position = servo.getPosition(1, &error);
  float present_velocity = servo.getVelocity(1, &error);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Position :");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("%.1f deg  S", present_position);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("Velocity :");
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("%.1f dps  ", present_velocity);
}
