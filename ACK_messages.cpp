#include "Arduino.h"
#include "BPPMAXGNSS.h"

namespace BPPGNSS {

  ACK_ACK::ACK_ACK(void){
    _msgClass = CLASS_ACK;
		_msgID = ID_ACK_ACK;
		_dataLen = 2; // Length of data

		//Message-specific fields

    clsID = 0;
    msgID = 0;

  }

  void ACK_ACK::encodeMsg(uint8_t* buf){
    uint8_t temp[4];

    //Encode clsID
    encodeU1(temp,clsID);
    buf[NUM_HEADER_BYTES + 0] = temp[0];

    //Encode msgID
    encodeU1(temp,msgID);
    buf[NUM_HEADER_BYTES + 1] = temp[0];

  }

  void ACK_ACK::decodeMsg(uint8_t* buf){
    uint8_t temp[4];

    //Decode clsID
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		clsID = decodeU1(temp);

    //Decode msgID
		temp[0] = buf[NUM_HEADER_BYTES + 1];
		msgID = decodeU1(temp);

  }

  ACK_NACK::ACK_NACK(void){
    _msgClass = CLASS_ACK;
		_msgID = ID_ACK_NACK;
		_dataLen = 2; // Length of data

		//Message-specific fields

    clsID = 0;
    msgID = 0;

  }

  void ACK_NACK::encodeMsg(uint8_t* buf){
    uint8_t temp[4];

    //Encode clsID
    encodeU1(temp,clsID);
    buf[NUM_HEADER_BYTES + 0] = temp[0];

    //Encode msgID
    encodeU1(temp,msgID);
    buf[NUM_HEADER_BYTES + 1] = temp[0];

  }

  void ACK_NACK::decodeMsg(uint8_t* buf){
    uint8_t temp[4];

    //Decode clsID
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		clsID = decodeU1(temp);

    //Decode msgID
		temp[0] = buf[NUM_HEADER_BYTES + 1];
		msgID = decodeU1(temp);

  }






}
