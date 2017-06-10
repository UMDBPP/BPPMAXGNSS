#include "Arduino.h"
#include "BPPMAXGNSS.h"

namespace BPPGNSS {
	NAV_POSLLH_Solution::NAV_POSLLH_Solution(void){
		_msgClass = CLASS_NAV;
		_msgID = ID_NAV_POSLLH;
		_dataLen = 28;
		
		iTOW = 0;
		lon = 0;
		lat = 0;
		height = 0;
		hMSL = 0;
		hAcc = 0;
		vAcc = 0;
	
	
	}
	void NAV_POSLLH_Solution::encodeMsg(uint8_t* buf){
		uint8_t temp[4];
		
		// Encode iTow
		encodeU4(temp, iTOW);
		buf[NUM_HEADER_BYTES + 0] = temp[0];
		buf[NUM_HEADER_BYTES + 1] = temp[1];
		buf[NUM_HEADER_BYTES + 2] = temp[2];
		buf[NUM_HEADER_BYTES + 3] = temp[3];
		
		// Encode lon
		encodeI4(temp, lon);
		buf[NUM_HEADER_BYTES + 4] = temp[0];
		buf[NUM_HEADER_BYTES + 5] = temp[1];
		buf[NUM_HEADER_BYTES + 6] = temp[2];
		buf[NUM_HEADER_BYTES + 7] = temp[3];
		
		//Encode lat
		encodeI4(temp,lat);
		buf[NUM_HEADER_BYTES + 8] = temp[0];
		buf[NUM_HEADER_BYTES + 9] = temp[1];
		buf[NUM_HEADER_BYTES + 10] = temp[2];
		buf[NUM_HEADER_BYTES + 11] = temp[3];
		
		//Encode height
		encodeI4(temp,height);
		buf[NUM_HEADER_BYTES + 12] = temp[0];
		buf[NUM_HEADER_BYTES + 13] = temp[1];
		buf[NUM_HEADER_BYTES + 14] = temp[2];
		buf[NUM_HEADER_BYTES + 15] = temp[3];
		
		//Encode hMSL
		encodeI4(temp,hMSL);
		buf[NUM_HEADER_BYTES + 16] = temp[0];
		buf[NUM_HEADER_BYTES + 17] = temp[1];
		buf[NUM_HEADER_BYTES + 18] = temp[2];
		buf[NUM_HEADER_BYTES + 19] = temp[3];
		
		//Encode hAcc
		encodeU4(temp,hAcc);
		buf[NUM_HEADER_BYTES + 20] = temp[0];
		buf[NUM_HEADER_BYTES + 21] = temp[1];
		buf[NUM_HEADER_BYTES + 22] = temp[2];
		buf[NUM_HEADER_BYTES + 23] = temp[3];
		
		//Encode vAcc
		encodeU4(temp,hAcc);
		buf[NUM_HEADER_BYTES + 24] = temp[0];
		buf[NUM_HEADER_BYTES + 25] = temp[1];
		buf[NUM_HEADER_BYTES + 26] = temp[2];
		buf[NUM_HEADER_BYTES + 27] = temp[3];
		
		
	}
	void NAV_POSLLH_Solution::decodeMsg(uint8_t* buf){
	
		uint8_t temp[4];
		
		// Decode iTow
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		temp[1] = buf[NUM_HEADER_BYTES + 1];
		temp[2] = buf[NUM_HEADER_BYTES + 2];
		temp[3] = buf[NUM_HEADER_BYTES + 3];
		iTOW = decodeU4(temp);
		
		//Decode lon
		temp[0] = buf[NUM_HEADER_BYTES + 4];
		temp[1] = buf[NUM_HEADER_BYTES + 5];
		temp[2] = buf[NUM_HEADER_BYTES + 6];
		temp[3] = buf[NUM_HEADER_BYTES + 7];
		lon = decodeI4(temp);
		
		//Decode lat
		temp[0] = buf[NUM_HEADER_BYTES + 8];
		temp[1] = buf[NUM_HEADER_BYTES + 9];
		temp[2] = buf[NUM_HEADER_BYTES + 10];
		temp[3] = buf[NUM_HEADER_BYTES + 11];
		lat = decodeI4(temp);
		
		//Decode height
		temp[0] = buf[NUM_HEADER_BYTES + 12];
		temp[1] = buf[NUM_HEADER_BYTES + 13];
		temp[2] = buf[NUM_HEADER_BYTES + 14];
		temp[3] = buf[NUM_HEADER_BYTES + 15];
		height = decodeI4(temp);
		
		//Decode hMSL
		temp[0] = buf[NUM_HEADER_BYTES + 16];
		temp[1] = buf[NUM_HEADER_BYTES + 17];
		temp[2] = buf[NUM_HEADER_BYTES + 18];
		temp[3] = buf[NUM_HEADER_BYTES + 19];
		hMSL = decodeI4(temp);
	
		//Decode hAcc
		temp[0] = buf[NUM_HEADER_BYTES + 20];
		temp[1] = buf[NUM_HEADER_BYTES + 21];
		temp[2] = buf[NUM_HEADER_BYTES + 22];
		temp[3] = buf[NUM_HEADER_BYTES + 23];
		hAcc = decodeU4(temp);
		
		//Decode vAcc
		temp[0] = buf[NUM_HEADER_BYTES + 24];
		temp[1] = buf[NUM_HEADER_BYTES + 25];
		temp[2] = buf[NUM_HEADER_BYTES + 26];
		temp[3] = buf[NUM_HEADER_BYTES + 27];
		vAcc = decodeU4(temp);
	}
}