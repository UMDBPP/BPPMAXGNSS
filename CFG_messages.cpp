#include "Arduino.h"
#include "BPPMAXGNSS.h"

namespace BPPGNSS {
	uint8_t UBXMsg::getDataLength(void) {
		return _dataLen;
	}
	
	CFG_MSG_Poll::CFG_MSG_Poll(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = 0x01;
		_dataLen = 2; // Length of data
		msgClass = 0;
		msgID = 0;
	}
	
	void CFG_MSG_Poll::encodeMsg(uint8_t* buf) {
		buf[0] = UBX_HEADER_1; // Header
		buf[1] = UBX_HEADER_2;
		buf[2] = _msgClass; // Class
		buf[3] = _msgID; // ID
		buf[4] = _dataLen; // Length
		buf[5] = 0x00; // Upper byte of length; always 0 in this implementation
		buf[6] = msgClass;
		buf[7] = msgID;
		buf[8] = 0x00; // Checksum
		buf[9] = 0x00;
		
		appendChecksum(buf, _dataLen + NUM_CONTROL_BYTES);
	}
	
	void CFG_MSG_Poll::decodeMsg(uint8_t* buf) {
		msgClass = buf[6];
		msgID = buf[7];
	}
	
	CFG_MSG_SetRates::CFG_MSG_SetRates(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = 0x01;
		_dataLen = 8; // Length of data
		msgClass = 0;
		msgID = 0;
		rates[0] = 0;
		rates[1] = 0;
		rates[2] = 0;
		rates[3] = 0;
		rates[4] = 0;
		rates[5] = 0;
	}
	
	void CFG_MSG_SetRates::encodeMsg(uint8_t* buf) {
		buf[0] = UBX_HEADER_1; // Header
		buf[1] = UBX_HEADER_2;
		buf[2] = _msgClass; // Class
		buf[3] = _msgID; // ID
		buf[4] = _dataLen; // Length
		buf[5] = 0x00; // Upper byte of length; always 0 in this implementation
		buf[6] = msgClass;
		buf[7] = msgID;
		buf[8] = rates[0]; // Messages rates on each port
		buf[9] = rates[1];
		buf[10] = rates[2];
		buf[11] = rates[3];
		buf[12] = rates[4];
		buf[13] = rates[5];
		buf[14] = 0x00; // Checksum
		buf[15] = 0x00;
		
		appendChecksum(buf, _dataLen + NUM_CONTROL_BYTES);
	}
	
	void CFG_MSG_SetRates::decodeMsg(uint8_t* buf) {
		msgClass = buf[6];
		msgID = buf[7];
		rates[0] = buf[8];
		rates[1] = buf[9];
		rates[2] = buf[10];
		rates[3] = buf[11];
		rates[4] = buf[12];
		rates[5] = buf[13];
	}
	
	CFG_MSG_SetRate::CFG_MSG_SetRate(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = 0x01;
		_dataLen = 3; // Length of data
		msgClass = 0;
		msgID = 0;
		rate = 0;
	}
	
	void CFG_MSG_SetRate::encodeMsg(uint8_t* buf) {
		buf[0] = UBX_HEADER_1; // Header
		buf[1] = UBX_HEADER_2;
		buf[2] = _msgClass; // Class
		buf[3] = _msgID; // ID
		buf[4] = _dataLen; // Length
		buf[5] = 0x00; // Upper byte of length; always 0 in this implementation
		buf[6] = msgClass;
		buf[7] = msgID;
		buf[8] = rate; // Messages rate on current port
		buf[9] = 0x00; // Checksum
		buf[10] = 0x00;
		
		appendChecksum(buf, _dataLen + NUM_CONTROL_BYTES);
	}
	
	void CFG_MSG_SetRate::decodeMsg(uint8_t* buf) {
		msgClass = buf[6];
		msgID = buf[7];
		rate = buf[8];
	}
	
	
}