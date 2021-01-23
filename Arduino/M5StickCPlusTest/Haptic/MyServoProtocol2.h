// MyServoProtocol is using Serial1 on MEGA
// MyServoProtocol is using Serial2 on M5Stack FIRE (through port C)

// GROVE C port is TX:GPIO17 RX:GPIO16 not usable on fire !
#include <M5Stack.h>
#define GPIO_PIN26 26
#define GPIO_PIN36 36
HardwareSerial MySerial(2);

// Buffers
#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 256

// Packet field position
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8
#define PKT_PARAMETER1          9
#define PKT_PARAMETER2          10
#define PKT_PARAMETER3          11
#define PKT_PARAMETER4          12

#define INSTR_PING            0x01
#define INSTR_READ            0x02
#define INSTR_WRITE           0x03
#define INSTR_FACTORY_RESET   0x06
#define INSTR_REBOOT          0x08
#define INSTR_STATUS          0x55
#define INSTR_SYNC_READ       0x82
#define INSTR_SYNC_WRITE      0x83

#define ERROR_NONE                0x00
#define ERROR_RESULT_FAIL         0x01
#define ERROR_INSTRUCTION_ERROR   0x02
#define ERROR_CRC_ERROR           0x03
#define ERROR_DATA_RANGE_ERROR    0x04
#define ERROR_DATA_LENGTH_ERROR   0x05
#define ERROR_DATA_LIMIT_ERROR    0x06
#define ERROR_ACCESS_ERROR        0x07

// binary tool macro
#define LOW_BYTE(x)     ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x)    ((unsigned char)(((x)>>8)&0xFF))
#define MAKE_SHORT(l,h) (((uint16_t)((h)&0xFF)<<8) | (uint16_t)((l)&0xFF))
#define SIGN(x)         (((x)&0x8000)?((x)-0x10000):(x))

class MyServoProtocol2
{
  
public:

  // MEGA 16MHz:
  //  baud = 115200 OK 1.4ms reply
  //  baud = 250000 OK 0.7ms reply
  //  baud = 500000 OK 440us reply
  //  baud = 1000000 OK 430us reply
  // M5Stack:
  //  baud = 500000 OK
  //  baud = 1000000 OK
  
  MyServoProtocol2(uint32_t baud)
  {
    MySerial.begin(baud, SERIAL_8N1,GPIO_PIN36,GPIO_PIN26);
    rx_packet_position = 0;
    rx_packet_payload_length = 0;
    tx_packet_length = 0;
    rx_packet_id = 0;
    rx_packet_instruction = 0;
    rx_packet_error = 0;
    rx_packet_crc = 0;
    rx_timeout_us = 10000; // 10ms
  }

  int pingCommand(uint8_t id, uint16_t * model_number, uint8_t * firmware_version)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with ping instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_PING;
    completePacket(id,1);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us,1);
    
    // exit when internal error 
    if(error<0)
    {
      if(model_number!=0)
        *model_number=-1;
      if(firmware_version!=0)
        *firmware_version=-1;
      return error;
    }

    // check length
    if(rx_packet_payload_length == 4+3) // INSTR + ERROR + PARAMS (model(2)+version(1)) + CRC1 + CRC2
    {
      if(model_number!=0)
        *model_number=MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER1],rx_packet_buffer[PKT_PARAMETER2]);
      if(firmware_version!=0)
        *firmware_version=rx_packet_buffer[PKT_PARAMETER3];
      return error;
    }
    else
    {
      if(model_number!=0)
        *model_number=-1;
      if(firmware_version!=0)
        *firmware_version=-1;
      return error;
    }
  }

  int readByteCommand(uint8_t id, uint16_t address, uint16_t length, uint8_t * data, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with read instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_READ;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER2]= LOW_BYTE(length);
    tx_packet_buffer[PKT_PARAMETER3]= HIGH_BYTE(length);
    completePacket(id,5);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us,verbose);
    
    // exit when internal error 
    if(error<0)
    {
      return error;
    }

    // check length
    if(rx_packet_payload_length == 4+length) // INSTR + ERROR + PARAMS (length) + CRC1 + CRC2
    {
      if(data!=0)
      {
        memcpy(data,rx_packet_buffer+PKT_PARAMETER1,length);
      }
    }
    return error;
  }

  int readWordCommand(uint8_t id, uint16_t address, uint16_t length, uint16_t * data, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with read instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_READ;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER2]= LOW_BYTE(length*2);
    tx_packet_buffer[PKT_PARAMETER3]= HIGH_BYTE(length*2);
    completePacket(id,5);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us,verbose);
    
    // exit when internal error 
    if(error<0)
    {
      return error;
    }

    // check length
    if(rx_packet_payload_length == (4+2*length)) // INSTR + ERROR + PARAMS (length*2) + CRC1 + CRC2
    {
      if(data!=0)
      {
        //for(int index=0;index<length;++index)
        //{
        //  data[index] = MAKE_SHORT(rx_packet_buffer[PKT_PARAMETER1+index*2+0],rx_packet_buffer[PKT_PARAMETER1+index*2+1]);
        //}
        memcpy(data,rx_packet_buffer+PKT_PARAMETER1,2*length);
      }
    }
    return error;
  }

  int writeByteCommand(uint8_t id, uint16_t address, uint16_t length, uint8_t const * data, unsigned long extra_timeout_us=0, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with write instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_WRITE;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    memcpy(tx_packet_buffer+PKT_PARAMETER2,data,length);
    completePacket(id,3+length);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us+extra_timeout_us,verbose);
    return error;
  }

  int writeWordCommand(uint8_t id, uint16_t address, uint16_t length, uint16_t const * data, unsigned long extra_timeout_us=0, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with write instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_WRITE;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    memcpy(tx_packet_buffer+PKT_PARAMETER2,data,length*2);
    completePacket(id,3+length*2);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us+extra_timeout_us,verbose);
    return error;
  }
  
  int factoryResetCommand(uint8_t id)
  {
    // flush serial input buffer
    serialFlush();    

    // build packed with factory reset instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_FACTORY_RESET;
    completePacket(id,1);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us,1);
    return error;
  }

  int rebootCommand(uint8_t id)
  {
    // flush serial input buffer
    serialFlush();    

    // build packed with reboot instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_REBOOT;
    completePacket(id,1);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // wait for response
    int error = receiveStatusPacket(id,rx_timeout_us,1);
    return error;
  }

  struct sync_byte_data
  {
    uint8_t id;
    uint8_t length;
    uint8_t data[16];
    // helpers
    sync_byte_data() : id(0), length(0) {}
    sync_byte_data(uint8_t const & i) : id(i), length(0) {}
    void add_param_u8(uint8_t const & param) { data[length++]=param;}
    void add_param_f(float const & param) { data[length++]=(uint8_t)(param);}
  };

  struct sync_word_data
  {
    uint8_t id;
    uint8_t length;
    uint16_t wdata[8];
    // helpers
    sync_word_data() : id(0), length(0) {}
    sync_word_data(uint8_t const & i) : id(i), length(0) {}
    void add_param_u16(uint16_t const & param) { wdata[length++]=param;}
    void add_param_f(float const & param) { wdata[length++]=(uint16_t)(param);}
  } ;

  void syncWriteByteCommand(uint16_t address, uint8_t number_of_ids, sync_byte_data const * data, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with write instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_SYNC_WRITE;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER2]= LOW_BYTE(data[0].length);
    tx_packet_buffer[PKT_PARAMETER3]= HIGH_BYTE(data[0].length);
    size_t position = PKT_PARAMETER4;
    for(int index=0;index<number_of_ids;++index)
    {
      tx_packet_buffer[position]= data[index].id;
      ++position;
      for(int index2=0;index2<data[index].length;++index2)
      {
        tx_packet_buffer[position]= data[index].data[index2];
        ++position;
      }
    }
    completePacket(0xFE ,5+(data[0].length+1)*number_of_ids);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // no response
  }

  int syncWriteWordCommand(uint16_t address, uint8_t number_of_ids, sync_word_data const * data, int verbose=0)
  {
    // flush serial input buffer
    serialFlush();

    // build packed with write instruction
    tx_packet_buffer[PKT_INSTRUCTION]= INSTR_SYNC_WRITE;
    tx_packet_buffer[PKT_PARAMETER0]= LOW_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER1]= HIGH_BYTE(address);
    tx_packet_buffer[PKT_PARAMETER2]= LOW_BYTE(data[0].length*2);
    tx_packet_buffer[PKT_PARAMETER3]= HIGH_BYTE(data[0].length*2);
    size_t position = PKT_PARAMETER4;
    for(int index=0;index<number_of_ids;++index)
    {
      tx_packet_buffer[position]= data[index].id;
      ++position;
      for(int index2=0;index2<data[index].length;++index2)
      {
        tx_packet_buffer[position]= LOW_BYTE(data[index].wdata[index2]);
        ++position;
        tx_packet_buffer[position]= HIGH_BYTE(data[index].wdata[index2]);
        ++position;
      }
    }
    completePacket(0xFE ,5+(data[0].length*2+1)*number_of_ids);

    // send packet
    MySerial.write(tx_packet_buffer,tx_packet_length);
    MySerial.flush();

    // no response

    // debug
#ifdef TRACE_PROTOCOL2
    for(size_t index=0; index<tx_packet_length; ++index)
    {
      Serial.print(tx_packet_buffer[index],HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif    
  }
  
  // short function

  int enableTorque(uint8_t id)
  {
    uint8_t data = 1;
    return writeByteCommand(id,0x40,1,&data,0,0);
  }

  int disableTorque(uint8_t id)
  {
    uint8_t data = 0;
    return writeByteCommand(id,0x40,1,&data,0,0);
  }

  int enableLed(uint8_t id)
  {
    uint8_t data = 1;
    return writeByteCommand(id,0x41,1,&data,0,0);
  }
  
  int disableLed(uint8_t id)
  {
    uint8_t data = 0;
    return writeByteCommand(id,0x41,1,&data,0,0);
  }

  int setMode(uint8_t id, uint8_t mode)
  {
    uint8_t data = mode;
    return writeByteCommand(id,0x42,1,&data,0,0);
  }

  int setPosition(uint8_t id, float angle_deg)
  {
    uint16_t wdata = (uint16_t)(angle_deg*10.0f);
    return writeWordCommand(id,0x43,1,&wdata,0,0);
  }

  int setVelocityLimit(uint8_t id, float velocity_dps)
  {
    uint16_t wdata = (uint16_t)(velocity_dps);
    return writeWordCommand(id,0x45,1,&wdata,0,0);
  }

  int setPositionVelocity(uint8_t id, float angle_deg, float velocity_dps)
  {
    uint16_t wdata[2];
    wdata[0] = (uint16_t)(angle_deg*10.0f);
    wdata[1] = (uint16_t)(velocity_dps);
    return writeWordCommand(id,0x43,2,wdata,0,0);
  }

  int setCurrentLimit(uint8_t id, uint16_t current_mA)
  {
    uint16_t wdata = (uint16_t)(current_mA);
    return writeWordCommand(id,0x47,1,&wdata,0,0);
  }

  int setPWMLimit(uint8_t id, uint8_t pwm_pc)
  {
    uint16_t wdata = (uint8_t)(pwm_pc);
    return writeWordCommand(id,0x49,1,&wdata,0,0);
  }

  int setPositionVelocityCurrentPWM(uint8_t id, float angle_deg, float velocity_dps, uint16_t current_mA, uint8_t pwm_pc)
  {
    uint16_t wdata[4];
    wdata[0] = (uint16_t)(angle_deg*10.0f);
    wdata[1] = (uint16_t)(velocity_dps);
    wdata[2] = (uint16_t)(current_mA);
    wdata[3] = (uint16_t)(pwm_pc);
    return writeWordCommand(id,0x43,4,wdata,0,0);
  }    

  float getPosition(uint8_t id, int * error)
  {
    uint16_t wdata;
    int local_error = readWordCommand(id,0x4D,1,&wdata,0);
    if(error!=0)
    *error = local_error;
    return (float)wdata/10.0f;
  }

  float getVelocity(uint8_t id, int * error)
  {
    uint16_t wdata;
    int local_error = readWordCommand(id,0x4F,1,&wdata,0);
    if(error!=0)
    *error = local_error;
    return (float)(SIGN(wdata));
  }

  float getCurrent(uint8_t id, int * error)
  {
    uint16_t wdata;
    int local_error = readWordCommand(id,0x51,1,&wdata,0);
    if(error!=0)
    *error = local_error;
    return (float)(SIGN(wdata));
  }
  
private:

  uint8_t tx_packet_buffer[TX_BUFFER_SIZE];
  uint8_t rx_packet_buffer[RX_BUFFER_SIZE];
  
  uint32_t rx_packet_position;
  uint16_t rx_packet_payload_length;
  uint32_t tx_packet_length;

  uint8_t rx_packet_id;
  uint8_t rx_packet_instruction;
  uint8_t rx_packet_error;
  uint16_t rx_packet_crc;
  unsigned long rx_timeout_us;

  uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
  {
    uint16_t i, j;
    static const uint16_t crc_table[256] = { 0x0000,
      0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
      0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
      0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
      0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
      0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
      0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
      0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
      0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
      0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
      0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
      0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
      0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
      0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
      0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
      0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
      0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
      0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
      0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
      0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
      0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
      0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
      0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
      0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
      0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
      0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
      0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
      0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
      0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
      0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
      0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
      0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
      0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
      0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
      0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
      0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
      0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
      0x820D, 0x8207, 0x0202 };
  
    for (j = 0; j < data_blk_size; j++)
    {
      i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
      crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
  
    return crc_accum;
  }
  
  void serialFlush()
  {
    while(MySerial.available() > 0)
    {
      MySerial.read();
    }
  }   

  void completePacket(uint8_t id, uint16_t payload_length) // length from instruction byte to last parameter byte
  {
    // header
    tx_packet_buffer[PKT_HEADER0]=0xFF;
    tx_packet_buffer[PKT_HEADER1]=0xFF;
    tx_packet_buffer[PKT_HEADER2]=0xFD;
    tx_packet_buffer[PKT_RESERVED]=0x00;
    tx_packet_buffer[PKT_ID]=id;
    // length
    uint16_t length = payload_length+2; // +crc
    tx_packet_buffer[PKT_LENGTH_L]= LOW_BYTE(length);
    tx_packet_buffer[PKT_LENGTH_H]= HIGH_BYTE(length);
    // crc
    uint16_t packet_crc = updateCRC(0, tx_packet_buffer, length+5);
    tx_packet_buffer[PKT_INSTRUCTION+payload_length]= LOW_BYTE(packet_crc);     // CRC 1
    tx_packet_buffer[PKT_INSTRUCTION+payload_length+1]= HIGH_BYTE(packet_crc);  // CRC 2
    tx_packet_length = PKT_INSTRUCTION+length;
  }

  int receiveStatusPacket(uint8_t id, unsigned long timeout_us, int verbose)
  {
    unsigned long start_time = micros();
    while(micros()<(timeout_us+start_time))
    {
      if(MySerial.available() > 0)
      {
        int inByte = MySerial.read();
        if(inByte>=0)
        {
          //Serial.print(inByte);
          // one byte received
          if(decodePacket(inByte))
          {
            // one packet received
            if( (rx_packet_id==id) || (id == 0xFE) )
            {
              // received packet id is the one expected, or just receive any id after broacast
              if(verbose>=1)
              {
                Serial.print("reply delay:");
                Serial.print(micros()-start_time);
                Serial.println("us");
              }
              if(verbose==2)
              {
                Serial.print("rx_packet_payload_length:");
                Serial.print(rx_packet_payload_length);
                Serial.println("bytes");
              }
              // check instruction (status return packet)
              rx_packet_instruction = rx_packet_buffer[PKT_INSTRUCTION];
              if(verbose==2)
              {
                Serial.print("rx_packet_instruction:");
                Serial.println(rx_packet_instruction);
              }
              if(rx_packet_instruction == INSTR_STATUS)
              {
                // good instruction (return)
                // check error and extract parameters
                rx_packet_error = rx_packet_buffer[PKT_ERROR];
                return rx_packet_error;
              }
              else
              {
                Serial.println("[PROTOCOL] ERROR : Received an unexpected packet!");
                return -1;
              }
            }
            else
            {
              Serial.println("[PROTOCOL] ERROR : Received packet from another ID!");
              return -1;
            }
          }
        }
      }
    }
    Serial.println("[PROTOCOL] ERROR : Time-out!");
    return -2;
  }


  bool decodePacket(int b)
  {
    if(rx_packet_position==PKT_HEADER0)
    {
      if(b==0xFF)
      {
        rx_packet_buffer[rx_packet_position]=(uint8_t)b;
        ++rx_packet_position;
      }
      return false;
    }
    else if(rx_packet_position==PKT_HEADER1)
    {
      if(b==0xFF)
      {
        rx_packet_buffer[rx_packet_position]=(uint8_t)b;
        ++rx_packet_position;
      }
      else
      {
        rx_packet_position = PKT_HEADER0;
      }
      return false;
    }
    else if(rx_packet_position==PKT_HEADER2)
    {
      if(b==0xFD)
      {
        rx_packet_buffer[rx_packet_position]=(uint8_t)b;
        ++rx_packet_position;
      }
      else
      {
        rx_packet_position = PKT_HEADER0;
      }
      return false;
    }
    else if(rx_packet_position==PKT_RESERVED)
    {
      if(b==0x00)
      {
        rx_packet_buffer[rx_packet_position]=(uint8_t)b;
        ++rx_packet_position;
      }
      else
      {
        rx_packet_position = PKT_HEADER0;
      }
      return false;
    }
    else if(rx_packet_position==PKT_ID)
    {
      if( (b<=252) || (b==254) )
      {
        rx_packet_buffer[rx_packet_position]=(uint8_t)b;
        rx_packet_id=(uint8_t)b;
        ++rx_packet_position;
      }
      else
      {
        rx_packet_position = PKT_HEADER0;
      }
      return false;
    }
    else if(rx_packet_position==PKT_LENGTH_L)
    {
      rx_packet_buffer[rx_packet_position]=(uint8_t)b;
      ++rx_packet_position;
      return false;
    }
    else if(rx_packet_position==PKT_LENGTH_H)
    {
      rx_packet_buffer[rx_packet_position]=(uint8_t)b;
      ++rx_packet_position;
      rx_packet_payload_length = MAKE_SHORT(rx_packet_buffer[PKT_LENGTH_L],rx_packet_buffer[PKT_LENGTH_H]);
      if(rx_packet_payload_length+PKT_LENGTH_H>RX_BUFFER_SIZE)
      {
        rx_packet_position = PKT_HEADER0;
        Serial.println("[PROTOCOL] ERROR : Received packet too long!");
      }
      return false;
    }
    else if(rx_packet_position<=(PKT_LENGTH_H+rx_packet_payload_length-2))
    {
      rx_packet_buffer[rx_packet_position]=(uint8_t)b;
      ++rx_packet_position;
      return false;      
    }
    else if(rx_packet_position==(PKT_LENGTH_H+rx_packet_payload_length-1)) // CRC_L
    {
      rx_packet_buffer[rx_packet_position]=(uint8_t)b;
      ++rx_packet_position;
      return false;      
    }
    else if(rx_packet_position==(PKT_LENGTH_H+rx_packet_payload_length)) // CRC_H
    {
      rx_packet_buffer[rx_packet_position]=(uint8_t)b;
      ++rx_packet_position;
      rx_packet_crc = MAKE_SHORT(rx_packet_buffer[rx_packet_position-2],rx_packet_buffer[rx_packet_position-1]);
      uint16_t calculated_crc = updateCRC(0, rx_packet_buffer, PKT_LENGTH_H+rx_packet_payload_length-1);
      rx_packet_position = PKT_HEADER0;
      if(rx_packet_crc == calculated_crc)
      {
        //Serial.println("[PROTOCOL] NOTICE : CRC check OK.");
        return true;
      }
      else
      {
        Serial.println("[PROTOCOL] ERROR : CRC check FAILED!");
        return false;
      }
    }
    else
    {
        rx_packet_position = PKT_HEADER0;
        return false;
    }
  }

      
};
