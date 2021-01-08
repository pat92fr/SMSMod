TOBECOMPLETED
TOBECOMPLETED
TOBECOMPLETED
TOBECOMPLETED
#include "MyServoProtocol2.h"

MyServoProtocol2 servo(1000000); // 1Mbps
uint16_t model_number = 0;
uint8_t firmware_version = 0;
float present_position_hip = 0.0f;
float present_velocity_hip = 0.0f;
float present_current_hip = 0.0f;
float present_pwm_hip = 0.0f;
float present_position_knee = 0.0f;
float present_velocity_knee = 0.0f;
float present_current_knee = 0.0f;
float present_pwm_knee = 0.0f;
uint8_t data[32];
uint16_t wdata[32];
int error = 0;

void setup()
{
  // Ping servo ID = 1, Return model and version
  error = servo.pingCommand(1,&model_number,&firmware_version); 
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  else
  {
    Serial.print("model_number:");
    Serial.println(model_number);

    Serial.print("firmware_version:");
    Serial.println(firmware_version);
    Serial.println("");
  }
  // Ping servo ID = 2, Return model and version
  error = servo.pingCommand(2,&model_number,&firmware_version);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  else
  {
    Serial.print("model_number:");
    Serial.println(model_number);

    Serial.print("firmware_version:");
    Serial.println(firmware_version);
    Serial.println("");
  }
  // enable torque 1
  Serial.print("Enable Torque...");
  error = servo.enableTorque(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // enable LED 1
  Serial.print("Enable Led...");
  error = servo.enableLed(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // enable torque 2
  Serial.print("Enable Torque...");
  error = servo.enableTorque(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // enable LED 2
  Serial.print("Enable Led...");
  error = servo.enableLed(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // set 90
  Serial.print("Set position...");
  error = servo.setPosition(1,90.0f);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // set 90
  Serial.print("Set position...");
  error = servo.setPosition(2,90.0f);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");

      // set 2 (is Hip)
      error = servo.setPositionVelocityCurrentPWM(2,135.0f-radiansToDegrees(servo_angle_hip_rad),800.0f,350.0f,55.0f);
      if(error !=0)
      {
        Serial.print("rx_packet_error:");
        Serial.println(error);
      }   
      
      // set 1 (is Knee)
      error = servo.setPositionVelocityCurrentPWM(1,90.0f+45.0f-radiansToDegrees(servo_angle_knee_rad),800.0f,350.0f,55.0f);
      if(error !=0)
      {
        Serial.print("rx_packet_error:");
        Serial.println(error);
      }   

      // set 1 & 2 using sunc write 
      {
        MyServoProtocol2::sync_word_data swd[2];
        swd[0].id = 2;
        swd[0].add_param_f((135.0f-radiansToDegrees(servo_angle_hip_rad))*10.0f);
        swd[0].add_param_f(1000.0f);//////////////////////////////////////////////////////////////LIMIT
        swd[0].add_param_f(350.0f);
        swd[0].add_param_f(60.0f);
        swd[1].id = 1;
        swd[1].add_param_f((90.0f+45.0f-radiansToDegrees(servo_angle_knee_rad))*10.0f);
        swd[1].add_param_f(1000.0f); //////////////////////////////////////////////////////////////LIMIT
        swd[1].add_param_f(3500.0f);
        swd[1].add_param_f(60.0f);
        error = servo.syncWriteWordCommand(0x43,2,swd,0);  
        if(error !=0)
        {
          Serial.print("rx_packet_error:");
          Serial.println(error);
        }   
      }
#endif      
      unsigned long t3 = micros();

      // get feedback
#ifdef PROTOCOL2_CALL_SHORT_FUNCTION      
//      float present_position_hip = servo.getPosition(2, &error)-45.0f;
//      float present_velocity_hip = servo.getVelocity(2, &error);
//      float present_current_hip = servo.getCurrent(2, &error);
//      float present_position_knee = -servo.getPosition(1, &error)+90.0f+45.0f;
//      float present_velocity_knee = servo.getVelocity(1, &error);
//      float present_current_knee = servo.getCurrent(1, &error);
#else
      present_position_hip = 0.0f;
      present_velocity_hip = 0.0f;
      present_current_hip = 0.0f;
      present_pwm_hip = 0.0f;
      present_position_knee = 0.0f;
      present_velocity_knee = 0.0f;
      present_current_knee = 0.0f;
      present_pwm_knee = 0.0f;
      error = servo.readWordCommand(2,0x4D,4,wdata,0);
      if(error!=0)
      {
        Serial.print("rx_packet_error:");
        Serial.println(error);
      }   
      else
      {
        present_position_hip = -(float)wdata[0]/10.0f+135.0f;
        present_velocity_hip = (float)(SIGN(wdata[1]));
        present_current_hip = (float)(SIGN(wdata[2]));
        present_pwm_hip = (float)(SIGN(wdata[3]));
      }      
      error = servo.readWordCommand(1,0x4D,4,wdata,0);
      if(error!=0)
      {
        Serial.print("rx_packet_error:");
        Serial.println(error);
      }   
      else
      {
        present_position_knee = -(float)wdata[0]/10.0f+90.0f+45.0f;
        present_velocity_knee = (float)(SIGN(wdata[1]));
        present_current_knee = (float)(SIGN(wdata[2]));
        present_pwm_knee = (float)(SIGN(wdata[3]));
      }      
#endif
      unsigned long t4 = micros();

      real_foot_pos_x_m = 0.0f;  
      real_foot_pos_z_m = 0.0f;
      k.forward(degreesToRadians(present_position_hip),degreesToRadians(present_position_knee), real_foot_pos_x_m, real_foot_pos_z_m);


      
      // data logging
//      Serial.print("Temps de calcul de la trajectoire (Bezier&Trigo): ");
//      Serial.print(t1-t0);
//      Serial.println("us");
//      
//      Serial.print("Temps de calcul de la cinématique inverse (IK) : ");
//      Serial.print(t2-t1);
//      Serial.println("us");
//
//      Serial.print("Temps transmission de la consigne de position et des limites en vitesse, courant et PWM aux servo Hip et Knee : ");
//      Serial.print(t3-t2);
//      Serial.println("us");
//      
//      Serial.print("Temps transmission du retour position, vitesse, courant et PWM des servo Hip et Knee : ");
//      Serial.print(t4-t3);
//      Serial.println("us");
//      
      Serial.print(time_s,3);
      Serial.print(",");
      Serial.print((float)(micros()-start_time)/1000000.0f,3);
      Serial.print(",");
      Serial.print(foot_pos_x_m,3);
      Serial.print(",");
      Serial.print(foot_pos_z_m,3);
      Serial.print(",");
      Serial.print(radiansToDegrees(servo_angle_hip_rad),1);
      Serial.print(",");
      Serial.print(radiansToDegrees(servo_angle_knee_rad),1);
      Serial.print(",");
      Serial.print(present_position_hip,1);
      Serial.print(",");
      Serial.print(present_velocity_hip,0);
      Serial.print(",");
      Serial.print(present_current_hip,0);
      Serial.print(",");
      Serial.print(present_position_knee,1);
      Serial.print(",");
      Serial.print(present_velocity_knee,0);
      Serial.print(",");
      Serial.print(present_current_knee,0);
      Serial.print(",");
      Serial.print(real_foot_pos_x_m,3);
      Serial.print(",");
      Serial.println(real_foot_pos_z_m,3);

      //delay(5);
    }


    // disable torque 1
  Serial.print("Disable Torque...");
  error = servo.disableTorque(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // Disable LED 1
  Serial.print("Disable Led...");
  error = servo.disableLed(1);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // disable torque 2
  Serial.print("Disable Torque...");
  error = servo.disableTorque(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
  // disable LED 2
  Serial.print("Disable Led...");
  error = servo.disableLed(2);
  if(error !=0)
  {
    Serial.print("rx_packet_error:");
    Serial.println(error);
  }
  Serial.println("");
}

void loop()
{

}
