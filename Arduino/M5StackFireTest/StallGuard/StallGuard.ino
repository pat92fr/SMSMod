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
  // set position to 30° (home), limit velocity by 50 dps, motor current by 150mA and PWM ratio do 50% maximum
  error = servo.setPositionVelocityCurrentPWM(1,30,50,150,50);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  delay(1000);
}

void loop() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("StallGuard");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Still!  ");
  delay(3000);
  // go to target position while monitoring current
  error = servo.setPositionVelocity(1,150,50);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Moving..");
  unsigned long start_time = millis();
  // monitor current for 3 seconds
  while(millis()<start_time+3000)
  {
    float present_current = servo.getCurrent(1,&error);
    // current excend 23mA
    if((present_current>23) && (millis()>start_time+250))
    {
      M5.Lcd.setCursor(0, 50);
      M5.Lcd.printf("STALL!!!");
      float present_position = servo.getPosition(1,&error);
      // back up for 5°
      error = servo.setPosition(1,present_position-5.0f);
      delay(3000);
      break;    
    }
    delay(2);
  }
  // return to home
  error = servo.setPositionVelocity(1,30,50);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }  
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Return..");
  delay(3000);
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("Still!   ");
  delay(3000);
  ++counter;
  if(counter>=1)
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
