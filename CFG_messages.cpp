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

	CFG_NAV5_Poll::CFG_NAV5_Poll(void){

		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_NAV5;
		_dataLen = 2; // Length of data

	}
	void CFG_NAV5_Poll::encodeMsg(uint8_t* buf){

		//Poll has no Payload
	}
	void CFG_NAV5_Poll::decodeMsg(uint8_t* buf){
		//Poll has no payload
	}

	CFG_NAV5_EngineSettings::CFG_NAV5_EngineSettings(void){
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_MSG;
		_dataLen = 36; // Length of data

		//bools for mask
		for(int i = 7; i>=0;i--) ReservedBit0[i] = 0;
		dgpsMask = 0;
		staticHoldMask = 0;
		timeMask = 0;
		posMask = 0;
		drLim = 0;
		posFixMode = 0;
		minEl = 0;
		dyn = 0;

		dynMode1 = 0;
		fixMode = 0;
		fixedAlt = 0;
		fixedAltVar = 0;
		minElev = 0;
		drLimit = 0;
		pDop = 0;
		tDop = 0;
		pAcc = 0;
		tAcc = 0;
		staticHoldThresh = 0;
		dgpsTimeOut = 0;
		cnoThreshNumSVs = 0;
		cnoThresh = 0;
		reserved2 = 0;
		reserved3 = 0;
		reserved4 = 0;
	}

	void CFG_NAV5_EngineSettings::encodeMsg(uint8_t* buf){
		uint8_t temp[4];

		//Encode mask
		mask = ReservedBit0[7]<<15|ReservedBit0[6]<<14|ReservedBit0[5]<<13|ReservedBit0[4]<<12|ReservedBit0[3]<<11|ReservedBit0[2]<<10|ReservedBit0[1]<<9|ReservedBit0[0]<<8|dgpsMask<<7|staticHoldMask<<6|timeMask<<5|posMask<<4|drLim<<3|posFixMode<<2|minEl<<1|dyn;
		temp[0] = mask;
		buf[NUM_HEADER_BYTES + 0] = temp[0];

		//Encode dynMode1
		encodeU1(temp,dynMode1);
		buf[NUM_HEADER_BYTES + 1] = temp[0];

		//Encode fixMode
		encodeU1(temp,fixMode);
		buf[NUM_HEADER_BYTES + 2] = temp[0];

		//Encode fixedAlt
		encodeI4(temp,fixedAlt);
		buf[NUM_HEADER_BYTES + 3] = temp[0];
		buf[NUM_HEADER_BYTES + 4] = temp[1];
		buf[NUM_HEADER_BYTES + 5] = temp[2];
		buf[NUM_HEADER_BYTES + 6] = temp[3];

		//Encode fixedAltVar
		encodeU4(temp,fixedAltVar);
		buf[NUM_HEADER_BYTES + 7] = temp[0];
		buf[NUM_HEADER_BYTES + 8] = temp[1];
		buf[NUM_HEADER_BYTES + 9] = temp[2];
		buf[NUM_HEADER_BYTES + 10] = temp[3];

		//Encode minElev
		encodeI1(temp,minElev);
		buf[NUM_HEADER_BYTES + 11] = temp[0];

		//Encode drLimit
		encodeU1(temp,drLimit);
		buf[NUM_HEADER_BYTES + 12] = temp[0];

		//Encode pDop
		encodeU2(temp,pDop);
		buf[NUM_HEADER_BYTES + 13] = temp[0];
		buf[NUM_HEADER_BYTES + 14] = temp[1];

		//Encode tDop
		encodeU2(temp,tDop);
		buf[NUM_HEADER_BYTES + 15] = temp[0];
		buf[NUM_HEADER_BYTES + 16] = temp[1];

		//Encode pAcc
		encodeU2(temp,pAcc);
		buf[NUM_HEADER_BYTES + 17] = temp[0];
		buf[NUM_HEADER_BYTES + 18] = temp[1];

		//Encode tAcc
		encodeU2(temp,tAcc);
		buf[NUM_HEADER_BYTES + 19] = temp[0];
		buf[NUM_HEADER_BYTES + 20] = temp[1];

		//Encode staticHoldThresh
		encodeU1(temp,staticHoldThresh);
		buf[NUM_HEADER_BYTES + 21] = temp[0];

		//Encode dpgsTimeOut
		encodeU1(temp,dgpsTimeOut);
		buf[NUM_HEADER_BYTES + 22] = temp[0];

		//Encode cnoThreshNumSVs
		encodeU1(temp,cnoThreshNumSVs);
		buf[NUM_HEADER_BYTES + 23] = temp[0];

		//Encode cnoThresh
		encodeU1(temp,cnoThresh);
		buf[NUM_HEADER_BYTES + 24] = temp[0];

		//Encode reserved2
		encodeU2(temp,reserved2);
		buf[NUM_HEADER_BYTES + 25] = temp[0];
		buf[NUM_HEADER_BYTES + 26] = temp[1];

		//Encode reserved3
		encodeU4(temp,reserved3);
		buf[NUM_HEADER_BYTES + 27] = temp[0];
		buf[NUM_HEADER_BYTES + 28] = temp[1];
		buf[NUM_HEADER_BYTES + 29] = temp[2];
		buf[NUM_HEADER_BYTES + 30] = temp[3];

		//Encode reserved4
		encodeU4(temp,reserved4);
		buf[NUM_HEADER_BYTES + 31] = temp[0];
		buf[NUM_HEADER_BYTES + 32] = temp[1];
		buf[NUM_HEADER_BYTES + 33] = temp[2];
		buf[NUM_HEADER_BYTES + 34] = temp[3];

	}

	void CFG_NAV5_EngineSettings::decodeMsg(uint8_t* buf){
		uint8_t temp[4];

		//Decode mask
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		for(int i = 7;i>=0;i--) ReservedBit0[i] = temp[0] & (1<<(i+8));
		dgpsMask = temp[0] & (1<<7);
		staticHoldMask = temp[0] & (1<<6);
		timeMask = temp[0] & (1<<5);
		posMask = temp[0] & (1<<4);
		drLim = temp[0] & (1<<3);
		posFixMode = temp[0] & (1<<2);
		minEl = temp[0] & (1<<1);
		dyn = temp[0] & 1;
		mask = 0;
		for(int i = 0; i>8;i++)mask+=ReservedBit0[i];
		mask = mask + dgpsMask + staticHoldMask + timeMask + posMask + drLim + posFixMode + minEl + dyn;

		//Decode dynMode1
		temp[0] = buf[NUM_HEADER_BYTES + 1];
		dynMode1 = decodeU1(temp);

		//Decode fixMode
		temp[0] = buf[NUM_HEADER_BYTES + 2];
		fixMode = decodeU1(temp);

		//Decode fixedAlt
		temp[0] = buf[NUM_HEADER_BYTES + 3];
		temp[1] = buf[NUM_HEADER_BYTES + 4];
		temp[2] = buf[NUM_HEADER_BYTES + 5];
		temp[3] = buf[NUM_HEADER_BYTES + 6];
		fixedAlt = decodeI4(temp);

		//Decode fixedAltVar
		temp[0] = buf[NUM_HEADER_BYTES + 7];
		temp[1] = buf[NUM_HEADER_BYTES + 8];
		temp[2] = buf[NUM_HEADER_BYTES + 9];
		temp[3] = buf[NUM_HEADER_BYTES + 10];
		fixedAltVar = decodeU4(temp);

		//Decode minElev
		temp[0] = buf[NUM_HEADER_BYTES + 11];
		minElev = decodeI1(temp);

		//Decode drLimit
		temp[0] = buf[NUM_HEADER_BYTES + 12];
		drLimit = decodeU1(temp);

		//Decode pDop
		temp[0] = buf[NUM_HEADER_BYTES + 13];
		temp[1] = buf[NUM_HEADER_BYTES + 14];
		pDop = decodeU2(temp);

		//Decode tDop
		temp[0] = buf[NUM_HEADER_BYTES + 15];
		temp[1] = buf[NUM_HEADER_BYTES + 16];
		tDop = decodeU2(temp);

		//Decode pAcc
		temp[0] = buf[NUM_HEADER_BYTES + 17];
		temp[1] = buf[NUM_HEADER_BYTES + 18];
		pAcc = decodeU2(temp);

		//Decode tAcc
		temp[0] = buf[NUM_HEADER_BYTES + 19];
		temp[1] = buf[NUM_HEADER_BYTES + 20];
		tAcc = decodeU2(temp);

		//Decode staticHoldThresh
		temp[0] = buf[NUM_HEADER_BYTES + 21];
		staticHoldThresh = decodeU1(temp);

		//Decode dgpsTimeOut
		temp[0] = buf[NUM_HEADER_BYTES + 22];
		dgpsTimeOut = decodeU1(temp);

		//Decode cnoThreshNumSVs
		temp[0] = buf[NUM_HEADER_BYTES + 23];
		cnoThreshNumSVs = decodeU1(temp);

		//Decode cnoThresh
		temp[0] = buf[NUM_HEADER_BYTES + 24];
		cnoThresh = decodeU1(temp);

		//Decode reserved2
		temp[0] = buf[NUM_HEADER_BYTES + 25];
		temp[0] = buf[NUM_HEADER_BYTES + 26];
		reserved2 = decodeU2(temp);

		//Decode reserved3
		temp[0] = buf[NUM_HEADER_BYTES + 27];
		temp[0] = buf[NUM_HEADER_BYTES + 28];
		temp[0] = buf[NUM_HEADER_BYTES + 29];
		temp[0] = buf[NUM_HEADER_BYTES + 30];
		reserved3 = decodeU4(temp);

		//Decode reserved4
		temp[0] = buf[NUM_HEADER_BYTES + 31];
		temp[0] = buf[NUM_HEADER_BYTES + 32];
		temp[0] = buf[NUM_HEADER_BYTES + 33];
		temp[0] = buf[NUM_HEADER_BYTES + 34];
		reserved4 = decodeU4(temp);
	}

	CFG_NMEA_Poll::CFG_NMEA_Poll(void){
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_NMEA;
		_dataLen = 0; // Length of data

	}
	void CFG_NMEA_Poll::encodeMsg(uint8_t* buf){

		//No Payload

	}
	void CFG_NMEA_Poll::decodeMsg(uint8_t* buf){

		//No Payload

	}

	CFG_NMEA_Deprecated::CFG_NMEA_Deprecated(void){
		_msgClass = CLASS_CFG;
		_msgID = ID_CFG_NMEA;
		_dataLen = 4; // Length of data

		//bools for filter
		trackFilt = 0;
		gpsOnlyFilter = 0;
		dateFilt = 0;
		timeFilt = 0;
		mskPosFilt = 0;
		posFilt = 0;

		nmeaVersion = 0;
		numSV = 0;

		//bools for flags
		consider = 0;
		compat = 0;

	}

	void CFG_NMEA_Deprecated::encodeMsg(uint8_t* buf){
		uint8_t temp[4];

		//Encode filter
		filter = trackFilt<<5|gpsOnlyFilter<<4|dateFilt<<3|timeFilt<<2|mskPosFilt<<1|posFilt;
		temp[0] = filter;
		buf[NUM_HEADER_BYTES + 0] = temp[0];

		//Encode nmeaVersion
		encodeU1(temp,nmeaVersion);
		buf[NUM_HEADER_BYTES + 1] = temp[0];

		//Encode numSV
		encodeU1(temp,numSV);
		buf[NUM_HEADER_BYTES + 2] = temp[0];

		//Encode flags
		flags = consider<<1|compat;
		temp[0] = flags;
		buf[NUM_HEADER_BYTES + 3] = temp[0];
	}

	void CFG_NMEA_Deprecated::decodeMsg(uint8_t* buf){
		uint8_t temp[4];

		//Decode filter
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		trackFilt = temp[0] & (1<<5);
		gpsOnlyFilter = temp[0] & (1<<4);
		dateFilt = temp[0] & (1<<3);
		timeFilt = temp[0] & (1<<2);
		mskPosFilt = temp[0] & (1<<1);
		posFilt = temp[0] & 1;
		filter = trackFilt + gpsOnlyFilter + dateFilt + timeFilt + mskPosFilt + posFilt;

		//Decode nmeaVersion
		temp[0] = buf[NUM_HEADER_BYTES + 1];
		nmeaVersion = decodeU1(temp);

		//Decode numSV
		temp[0] = buf[NUM_HEADER_BYTES + 1];
		numSV = decodeU1(temp);

		//Decode flags
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		consider = temp[0] & (1<<1);
		compat = temp[0] & 1;
		flags = consider + compat;

	}
}
