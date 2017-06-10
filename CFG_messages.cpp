#include "Arduino.h"
#include "BPPMAXGNSS.h"

namespace BPPGNSS {
	uint8_t UBXMsg::getDataLength(void) {
		return _dataLen;
	}
	
	CFG_MSG_Poll::CFG_MSG_Poll(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_MSG;
		_dataLen = 2; // Length of data
		msgClass = 0;
		msgID = 0;
	}
	
	void CFG_MSG_Poll::encodeMsg(uint8_t* buf) {
	
		buf[NUM_HEADER_BYTES + 0] = msgClass;
		buf[NUM_HEADER_BYTES + 1] = msgID;
		
		UBXMsg::encodeMsg(buf);
	}
	
	void CFG_MSG_Poll::decodeMsg(uint8_t* buf) {
		msgClass = buf[NUM_HEADER_BYTES + 0];
		msgID = buf[NUM_HEADER_BYTES + 1];
	}
	
	CFG_MSG_SetRates::CFG_MSG_SetRates(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_MSG;
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
		buf[NUM_HEADER_BYTES + 0] = msgClass;
		buf[NUM_HEADER_BYTES + 1] = msgID;
		buf[NUM_HEADER_BYTES + 2] = rates[0]; // Messages rates on each port
		buf[NUM_HEADER_BYTES + 3] = rates[1];
		buf[NUM_HEADER_BYTES + 4] = rates[2];
		buf[NUM_HEADER_BYTES + 5] = rates[3];
		buf[NUM_HEADER_BYTES + 6] = rates[4];
		buf[NUM_HEADER_BYTES + 7] = rates[5];
		
		UBXMsg::encodeMsg(buf);
	}
	
	void CFG_MSG_SetRates::decodeMsg(uint8_t* buf) {
		msgClass = buf[NUM_HEADER_BYTES + 0];
		msgID = buf[NUM_HEADER_BYTES + 1];
		rates[0] = buf[NUM_HEADER_BYTES + 2];
		rates[1] = buf[NUM_HEADER_BYTES + 3];
		rates[2] = buf[NUM_HEADER_BYTES + 4];
		rates[3] = buf[NUM_HEADER_BYTES + 5];
		rates[4] = buf[NUM_HEADER_BYTES + 6];
		rates[5] = buf[NUM_HEADER_BYTES + 7];
	}
	
	CFG_MSG_SetRate::CFG_MSG_SetRate(void) {
		
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_MSG;
		_dataLen = 3; // Length of data
		msgClass = 0;
		msgID = 0;
		rate = 0;
	}
	
	void CFG_MSG_SetRate::encodeMsg(uint8_t* buf) {
		buf[NUM_HEADER_BYTES + 0] = msgClass;
		buf[NUM_HEADER_BYTES + 1] = msgID;
		buf[NUM_HEADER_BYTES + 2] = rate; // Messages rate on current port
		
		UBXMsg::encodeMsg(buf);
	}
	
	void CFG_MSG_SetRate::decodeMsg(uint8_t* buf) {
		msgClass = buf[NUM_HEADER_BYTES + 0];
		msgID = buf[NUM_HEADER_BYTES + 1];
		rate = buf[NUM_HEADER_BYTES + 2];
	}
	
	
}