/*
 * Author: Klusjesman, supersjimmie, modified and reworked by arjenhiemstra 
 */

#pragma once

enum IthoCommand
{    
  IthoUnknown = 0,
    
  IthoJoin = 1,
  IthoLeave = 2,
        
  IthoStandby = 3,
  IthoLow = 4,
  IthoMedium = 5,
  IthoHigh = 6,
  IthoFull = 7,
  
  IthoTimer1 = 8,
  IthoTimer2 = 9,
  IthoTimer3 = 10,
  
  //  //duco c system remote
  //  DucoStandby = 11,
  //  DucoLow = 12,
  //  DucoMedium = 13,
  //  DucoHigh = 14
};

enum message_state {
  S_START,
  S_HEADER,
  S_ADDR0,
  S_ADDR1,
  S_ADDR2,
  S_PARAM0,
  S_PARAM1,
  S_OPCODE,
  S_LEN,
  S_PAYLOAD,
  S_CHECKSUM,
  S_TRAILER,
  S_COMPLETE,
  S_ERROR
};


class IthoPacket
{
  public:
    IthoCommand command;

    uint8_t dataDecoded[32];
    uint8_t dataDecodedChk[32];
    uint8_t length;
    
    uint8_t deviceType;
    uint8_t deviceId[3];
    
    uint8_t counter;    //0-255, counter is increased on every remote button press
};
