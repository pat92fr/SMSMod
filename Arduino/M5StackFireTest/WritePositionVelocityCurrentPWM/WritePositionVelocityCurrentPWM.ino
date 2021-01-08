#include "MyServoProtocol2.h"
MyServoProtocol2 servo(500000); 
int error = 0;
unsigned int counter = 0;
float velocity = 50;

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
  // switch to "velocity-profil current based position control" operating mode
  error = servo.setMode(1,2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  // set position to 90°, limit velocity by 50 dps, motor current by 250mA and PWM ratio do 50% maximum
  error = servo.setPositionVelocityCurrentPWM(1,90,50,500,99);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  delay(1000);
}

void loop() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Velocity:");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("%.0f dps  ", velocity);
  delay(1000);

  error = servo.setVelocityLimit(1,velocity);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  error = servo.setPosition(1,150);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  delay(1200);
  error = servo.setPosition(1,90);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  delay(1200);
  error = servo.setPosition(1,30);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  delay(1200);
  error = servo.setPosition(1,90);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  delay(1200);

 
  ++counter;
  velocity = 2*velocity;
  if(counter>=5)
  {
    error = servo.disableTorque(1);
    if(error !=0)
    {
      Serial.print("rx_packet_error:");
      Serial.println(error);
    }
    M5.Power.powerOFF();
  }
  delay(100);
}
