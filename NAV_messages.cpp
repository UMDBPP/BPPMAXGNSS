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

	NAV_DOP_Dilution::NAV_DOP_Dilution(void){
		_msgClass = CLASS_NAV;
		_msgID = ID_NAV_DOP;
		_dataLen = 18;

		iTOW = 0;
		gDOP = 0;
		pDOP = 0;
		tDOP = 0;
		vDOP = 0;
		hDOP = 0;
		nDOP = 0;
		eDOP = 0;


	}

	void NAV_DOP_Dilution::encodeMsg(uint8_t* buf){
		uint8_t temp[4];

		// Encode iTow
		encodeU4(temp, iTOW);
		buf[NUM_HEADER_BYTES + 0] = temp[0];
		buf[NUM_HEADER_BYTES + 1] = temp[1];
		buf[NUM_HEADER_BYTES + 2] = temp[2];
		buf[NUM_HEADER_BYTES + 3] = temp[3];

		// Encode gDOP
		encodeU2(temp, gDOP);
		buf[NUM_HEADER_BYTES + 4] = temp[0];
		buf[NUM_HEADER_BYTES + 5] = temp[1];


		//Encode pDOP
		encodeU2(temp,pDOP);
		buf[NUM_HEADER_BYTES + 8] = temp[0];
		buf[NUM_HEADER_BYTES + 9] = temp[1];

		//Encode tDOP
		encodeU2(temp,tDOP);
		buf[NUM_HEADER_BYTES + 12] = temp[0];
		buf[NUM_HEADER_BYTES + 13] = temp[1];


		//Encode vDOP
		encodeU2(temp,vDOP);
		buf[NUM_HEADER_BYTES + 16] = temp[0];
		buf[NUM_HEADER_BYTES + 17] = temp[1];

		//Encode hDOP
		encodeU2(temp,hDOP);
		buf[NUM_HEADER_BYTES + 20] = temp[0];
		buf[NUM_HEADER_BYTES + 21] = temp[1];

		//Encode nDOP
		encodeU2(temp,nDOP);
		buf[NUM_HEADER_BYTES + 24] = temp[0];
		buf[NUM_HEADER_BYTES + 25] = temp[1];

		//Encode eDOP
		encodeU2(temp,eDOP);
		buf[NUM_HEADER_BYTES + 24] = temp[0];
		buf[NUM_HEADER_BYTES + 25] = temp[1];


	}
	void NAV_DOP_Dilution::decodeMsg(uint8_t* buf){

		uint8_t temp[4];

		// Decode iTow
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		temp[1] = buf[NUM_HEADER_BYTES + 1];
		temp[2] = buf[NUM_HEADER_BYTES + 2];
		temp[3] = buf[NUM_HEADER_BYTES + 3];
		iTOW = decodeU4(temp);

		//Decode gDOP
		temp[0] = buf[NUM_HEADER_BYTES + 4];
		temp[1] = buf[NUM_HEADER_BYTES + 5];
		gDOP = decodeU2(temp);

		//Decode pDOP
		temp[0] = buf[NUM_HEADER_BYTES + 6];
		temp[1] = buf[NUM_HEADER_BYTES + 7];
		pDOP = decodeU2(temp);

		//Decode tDOP
		temp[0] = buf[NUM_HEADER_BYTES + 8];
		temp[1] = buf[NUM_HEADER_BYTES + 9];
		tDOP = decodeU2(temp);

		//Decode vDOP
		temp[0] = buf[NUM_HEADER_BYTES + 10];
		temp[1] = buf[NUM_HEADER_BYTES + 11];
		vDOP = decodeU2(temp);

		//Decode hDOP
		temp[0] = buf[NUM_HEADER_BYTES + 12];
		temp[1] = buf[NUM_HEADER_BYTES + 13];
		hDOP = decodeU2(temp);

		//Decode nDOP
		temp[0] = buf[NUM_HEADER_BYTES + 14];
		temp[1] = buf[NUM_HEADER_BYTES + 15];
		nDOP = decodeU2(temp);

		//Decode eDOP
		temp[0] = buf[NUM_HEADER_BYTES + 16];
		temp[1] = buf[NUM_HEADER_BYTES + 17];
		eDOP = decodeU2(temp);
	}

NAV_PVT_Solution::NAV_PVT_Solution(void){
	_msgClass = CLASS_NAV;
	_msgID = ID_NAV_SOL;
	_dataLen = 84;

	iTOW = 0;
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	min = 0;
	sec = 0;

	//bools for valid
	validDate = 0;
	validTime = 0;
	fullyResolved = 0;

	//bools for flags
	gnssFixOK = 0;
	diffSoln = 0;
	psmState[0] = 0;
	psmState[1] = 0;
	psmState[2] = 0;

	tAcc = 0;
	nano = 0;
	fixType = 0;
	reserved1 = 0;
	numSV = 0;
	lon = 0;
	lat = 0;
	height = 0;
	hMSL = 0;
	hAcc = 0;
	vAcc = 0;
	velN = 0;
	velE = 0;
	velD = 0;
	gSpeed = 0;
	heading = 0;
	sAcc = 0;
	headingAcc = 0;
	pDOP = 0;

	reserved2 = 0; // Reserved Bitfield

	reserved3 = 0;
}

void NAV_PVT_Solution::encodeMsg(uint8_t* buf){
	uint8_t temp[4];

	//Encode iTOW
	encodeU4(temp, iTOW);
	buf[NUM_HEADER_BYTES + 0] = temp[0];
	buf[NUM_HEADER_BYTES + 1] = temp[1];
	buf[NUM_HEADER_BYTES + 2] = temp[2];
	buf[NUM_HEADER_BYTES + 3] = temp[3];

	//Encode year
	encodeU2(temp,year);
	buf[NUM_HEADER_BYTES + 4] = temp[0];
	buf[NUM_HEADER_BYTES + 5] = temp[1];

	//Encode month
	encodeU1(temp,month);
	buf[NUM_HEADER_BYTES + 6] = temp[0];

	//Encode day
	encodeU1(temp,day);
	buf[NUM_HEADER_BYTES + 7] = temp[0];

	//Encode hour
	encodeU1(temp,hour);
	buf[NUM_HEADER_BYTES + 8] = temp[0];

	//Encode min
	encodeU1(temp,min);
	buf[NUM_HEADER_BYTES + 9] = temp[0];

	//Encode sec
	encodeU1(temp,sec);
	buf[NUM_HEADER_BYTES + 10] = temp[0];

	//Encode valid
	valid = fullyResolved<<2|validTime<<1|validDate;
	temp[0] = valid;
	buf[NUM_HEADER_BYTES + 11] = temp[0];

	//Encode tAcc
	encodeU4(temp,tAcc);
	buf[NUM_HEADER_BYTES + 12] = temp[0];
	buf[NUM_HEADER_BYTES + 13] = temp[1];
	buf[NUM_HEADER_BYTES + 14] = temp[2];
	buf[NUM_HEADER_BYTES + 15] = temp[3];

	//Encode nano
	encodeI4(temp,nano);
	buf[NUM_HEADER_BYTES + 16] = temp[0];
	buf[NUM_HEADER_BYTES + 17] = temp[1];
	buf[NUM_HEADER_BYTES + 18] = temp[2];
	buf[NUM_HEADER_BYTES + 19] = temp[3];

	//Encode fixType
	encodeU1(temp,fixType);
	buf[NUM_HEADER_BYTES + 20] = temp[0];

	//Encode flags
	flags = psmState[2]<<4|psmState[1]<<3|psmState[0]<<2|diffSoln<<1|gnssFixOK;
	temp[0] = flags;
	buf[NUM_HEADER_BYTES + 21] = temp[0];

	//Encode reserved1
	encodeU1(temp,reserved1);
	buf[NUM_HEADER_BYTES + 22] = temp[0];

	//Encode numSV
	encodeU1(temp,numSV);
	buf[NUM_HEADER_BYTES + 23] = temp[0];

	//Encode lon
	encodeI4(temp,lon);
	buf[NUM_HEADER_BYTES + 24] = temp[0];
	buf[NUM_HEADER_BYTES + 25] = temp[1];
	buf[NUM_HEADER_BYTES + 26] = temp[2];
	buf[NUM_HEADER_BYTES + 27] = temp[3];

	//Encode lat
	encodeI4(temp,lat);
	buf[NUM_HEADER_BYTES + 28] = temp[0];
	buf[NUM_HEADER_BYTES + 29] = temp[1];
	buf[NUM_HEADER_BYTES + 30] = temp[2];
	buf[NUM_HEADER_BYTES + 31] = temp[3];

	//Encode height
	encodeI4(temp,height);
	buf[NUM_HEADER_BYTES + 32] = temp[0];
	buf[NUM_HEADER_BYTES + 33] = temp[1];
	buf[NUM_HEADER_BYTES + 34] = temp[2];
	buf[NUM_HEADER_BYTES + 35] = temp[3];

	//Encode hMSL
	encodeI4(temp,hMSL);
	buf[NUM_HEADER_BYTES + 36] = temp[0];
	buf[NUM_HEADER_BYTES + 37] = temp[1];
	buf[NUM_HEADER_BYTES + 38] = temp[2];
	buf[NUM_HEADER_BYTES + 39] = temp[3];

	//Encode hAcc
	encodeU4(temp,hAcc);
	buf[NUM_HEADER_BYTES + 40] = temp[0];
	buf[NUM_HEADER_BYTES + 41] = temp[1];
	buf[NUM_HEADER_BYTES + 42] = temp[2];
	buf[NUM_HEADER_BYTES + 43] = temp[3];

	//Encode vAcc
	encodeU4(temp,vAcc);
	buf[NUM_HEADER_BYTES + 44] = temp[0];
	buf[NUM_HEADER_BYTES + 45] = temp[1];
	buf[NUM_HEADER_BYTES + 46] = temp[2];
	buf[NUM_HEADER_BYTES + 47] = temp[3];

	//Encode velN
	encodeI4(temp,velN);
	buf[NUM_HEADER_BYTES + 48] = temp[0];
	buf[NUM_HEADER_BYTES + 49] = temp[1];
	buf[NUM_HEADER_BYTES + 50] = temp[2];
	buf[NUM_HEADER_BYTES + 51] = temp[3];

	//Encode velE
	encodeI4(temp,velE);
	buf[NUM_HEADER_BYTES + 52] = temp[0];
	buf[NUM_HEADER_BYTES + 53] = temp[1];
	buf[NUM_HEADER_BYTES + 54] = temp[2];
	buf[NUM_HEADER_BYTES + 55] = temp[3];

	//Encode velD
	encodeI4(temp,velD);
	buf[NUM_HEADER_BYTES + 56] = temp[0];
	buf[NUM_HEADER_BYTES + 57] = temp[1];
	buf[NUM_HEADER_BYTES + 57] = temp[2];
	buf[NUM_HEADER_BYTES + 59] = temp[3];

	//Encode gSpeed
	encodeI4(temp,gSpeed);
	buf[NUM_HEADER_BYTES + 60] = temp[0];
	buf[NUM_HEADER_BYTES + 61] = temp[1];
	buf[NUM_HEADER_BYTES + 62] = temp[2];
	buf[NUM_HEADER_BYTES + 63] = temp[3];

	//Encode heading
	encodeI4(temp,heading);
	buf[NUM_HEADER_BYTES + 64] = temp[0];
	buf[NUM_HEADER_BYTES + 65] = temp[1];
	buf[NUM_HEADER_BYTES + 66] = temp[2];
	buf[NUM_HEADER_BYTES + 67] = temp[3];

	//Encode sAcc
	encodeU4(temp,sAcc);
	buf[NUM_HEADER_BYTES + 68] = temp[0];
	buf[NUM_HEADER_BYTES + 69] = temp[1];
	buf[NUM_HEADER_BYTES + 70] = temp[2];
	buf[NUM_HEADER_BYTES + 71] = temp[3];

	//Encode headingAcc
	encodeU4(temp,headingAcc);
	buf[NUM_HEADER_BYTES + 72] = temp[0];
	buf[NUM_HEADER_BYTES + 73] = temp[1];
	buf[NUM_HEADER_BYTES + 74] = temp[2];
	buf[NUM_HEADER_BYTES + 75] = temp[3];

	//Encode pDOP
	encodeU2(temp,pDOP);
	buf[NUM_HEADER_BYTES + 76] = temp[0];
	buf[NUM_HEADER_BYTES + 77] = temp[1];

	//Encode reserved2
	encodeU2(temp,reserved2);
	buf[NUM_HEADER_BYTES + 78] = temp[0];
	buf[NUM_HEADER_BYTES + 79] = temp[1];

	//Encode reserved3
	encodeU4(temp,reserved3);
	buf[NUM_HEADER_BYTES + 80] = temp[0];
	buf[NUM_HEADER_BYTES + 81] = temp[1];
	buf[NUM_HEADER_BYTES + 82] = temp[2];
	buf[NUM_HEADER_BYTES + 83] = temp[3];

}

void NAV_PVT_Solution::decodeMsg(uint8_t* buf){
	uint8_t temp[4];

	// Decode iTow
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	iTOW = decodeU4(temp);

	//Decode year
	temp[0] = buf[NUM_HEADER_BYTES + 4];
	temp[1] = buf[NUM_HEADER_BYTES + 5];
	year = decodeU2(temp);

	//Decode month
	temp[0] = buf[NUM_HEADER_BYTES + 6];
	month = decodeU1(temp);

	//Decode day
	temp[0] = buf[NUM_HEADER_BYTES + 7];
	day = decodeU1(temp);

	//Decode hour
	temp[0] = buf[NUM_HEADER_BYTES + 8];
	hour = decodeU1(temp);

	//Decode min
	temp[0] = buf[NUM_HEADER_BYTES + 9];
	min = decodeU1(temp);

	//Decode sec
	temp[0] = buf[NUM_HEADER_BYTES + 10];
	sec = decodeU1(temp);

	//Decode valid
	temp[0] = buf[NUM_HEADER_BYTES + 11];
	fullyResolved = temp[0] & (1<<2);
	validTime = temp[0] & (1<<1);
	validDate = temp[0] & 1;
	valid = fullyResolved + validTime + validDate;

	//Decode tAcc
	temp[0] = buf[NUM_HEADER_BYTES + 12];
	temp[1] = buf[NUM_HEADER_BYTES + 13];
	temp[2] = buf[NUM_HEADER_BYTES + 14];
	temp[3] = buf[NUM_HEADER_BYTES + 15];
	tAcc = decodeU4(temp);

	//Decode nano
	temp[0] = buf[NUM_HEADER_BYTES + 16];
	temp[1] = buf[NUM_HEADER_BYTES + 17];
	temp[2] = buf[NUM_HEADER_BYTES + 18];
	temp[3] = buf[NUM_HEADER_BYTES + 19];
	nano = decodeI4(temp);

	//Decode fixType
	temp[0] = buf[NUM_HEADER_BYTES + 20];
	fixType = decodeU1(temp);

	// Decode flags
	temp[0] = buf[NUM_HEADER_BYTES + 21];
	psmState[2] = temp[0] & (1<<4);
	psmState[1] = temp[0] & (1<<3);
	psmState[0] = temp[0] & (1<<2);
	diffSoln = temp[0] & (1<<1);
	gnssFixOK = temp[0] & 1;
	flags = psmState[2] + psmState[1] + psmState[0] + diffSoln + gnssFixOK;

	//Decode reserved1
	temp[0] = buf[NUM_HEADER_BYTES + 22];
	reserved1 = decodeU1(temp);

	//Decode numSV
	temp[0] = buf[NUM_HEADER_BYTES + 23];
	numSV = decodeU1(temp);

	//Decode lon
	temp[0] = buf[NUM_HEADER_BYTES + 24];
	temp[1] = buf[NUM_HEADER_BYTES + 25];
	temp[2] = buf[NUM_HEADER_BYTES + 26];
	temp[3] = buf[NUM_HEADER_BYTES + 27];
	lon = decodeI4(temp);

	//Decode lat
	temp[0] = buf[NUM_HEADER_BYTES + 28];
	temp[1] = buf[NUM_HEADER_BYTES + 29];
	temp[2] = buf[NUM_HEADER_BYTES + 30];
	temp[3] = buf[NUM_HEADER_BYTES + 31];
	lat = decodeI4(temp);

	//Decode height
	temp[0] = buf[NUM_HEADER_BYTES + 32];
	temp[1] = buf[NUM_HEADER_BYTES + 33];
	temp[2] = buf[NUM_HEADER_BYTES + 34];
	temp[3] = buf[NUM_HEADER_BYTES + 35];
	height = decodeI4(temp);

	//Decode hMSL
	temp[0] = buf[NUM_HEADER_BYTES + 36];
	temp[1] = buf[NUM_HEADER_BYTES + 37];
	temp[2] = buf[NUM_HEADER_BYTES + 38];
	temp[3] = buf[NUM_HEADER_BYTES + 39];
	hMSL = decodeI4(temp);

	//Decode hAcc
	temp[0] = buf[NUM_HEADER_BYTES + 40];
	temp[1] = buf[NUM_HEADER_BYTES + 41];
	temp[2] = buf[NUM_HEADER_BYTES + 42];
	temp[3] = buf[NUM_HEADER_BYTES + 43];
	hAcc = decodeU4(temp);

	//Decode vAcc
	temp[0] = buf[NUM_HEADER_BYTES + 44];
	temp[1] = buf[NUM_HEADER_BYTES + 45];
	temp[2] = buf[NUM_HEADER_BYTES + 46];
	temp[3] = buf[NUM_HEADER_BYTES + 47];
	vAcc = decodeU4(temp);

	//Decode velN
	temp[0] = buf[NUM_HEADER_BYTES + 48];
	temp[1] = buf[NUM_HEADER_BYTES + 49];
	temp[2] = buf[NUM_HEADER_BYTES + 50];
	temp[3] = buf[NUM_HEADER_BYTES + 51];
	velN = decodeI4(temp);

	//Decode velE
	temp[0] = buf[NUM_HEADER_BYTES + 52];
	temp[1] = buf[NUM_HEADER_BYTES + 53];
	temp[2] = buf[NUM_HEADER_BYTES + 54];
	temp[3] = buf[NUM_HEADER_BYTES + 55];
	velE = decodeI4(temp);

	//Decode velD
	temp[0] = buf[NUM_HEADER_BYTES + 56];
	temp[1] = buf[NUM_HEADER_BYTES + 57];
	temp[2] = buf[NUM_HEADER_BYTES + 58];
	temp[3] = buf[NUM_HEADER_BYTES + 59];
	velD = decodeI4(temp);

	//Decode gSpeed
	temp[0] = buf[NUM_HEADER_BYTES + 60];
	temp[1] = buf[NUM_HEADER_BYTES + 61];
	temp[2] = buf[NUM_HEADER_BYTES + 62];
	temp[3] = buf[NUM_HEADER_BYTES + 63];
	gSpeed = decodeI4(temp);

	//Decode heading
	temp[0] = buf[NUM_HEADER_BYTES + 64];
	temp[1] = buf[NUM_HEADER_BYTES + 65];
	temp[2] = buf[NUM_HEADER_BYTES + 66];
	temp[3] = buf[NUM_HEADER_BYTES + 67];
	heading = decodeI4(temp);

	//Decode sAcc
	temp[0] = buf[NUM_HEADER_BYTES + 68];
	temp[1] = buf[NUM_HEADER_BYTES + 69];
	temp[2] = buf[NUM_HEADER_BYTES + 70];
	temp[3] = buf[NUM_HEADER_BYTES + 71];
	sAcc = decodeU4(temp);

	//Decode headingAcc
	temp[0] = buf[NUM_HEADER_BYTES + 72];
	temp[1] = buf[NUM_HEADER_BYTES + 73];
	temp[2] = buf[NUM_HEADER_BYTES + 74];
	temp[3] = buf[NUM_HEADER_BYTES + 75];
	vAcc = decodeU4(temp);

	//Decode pDOP
	temp[0] = buf[NUM_HEADER_BYTES + 76];
	temp[1] = buf[NUM_HEADER_BYTES + 77];
	temp[2] = buf[NUM_HEADER_BYTES + 78];
	temp[3] = buf[NUM_HEADER_BYTES + 79];
	pDOP = decodeU4(temp);

	//Decode reserved2
	temp[0] = buf[NUM_HEADER_BYTES + 80];
	temp[1] = buf[NUM_HEADER_BYTES + 81];
	reserved2 = decodeU2(temp);

	//Decode reserved3
	temp[0] = buf[NUM_HEADER_BYTES + 82];
	temp[1] = buf[NUM_HEADER_BYTES + 83];
	temp[2] = buf[NUM_HEADER_BYTES + 84];
	temp[3] = buf[NUM_HEADER_BYTES + 85];
	reserved3 = decodeU4(temp);

}

NAV_SOL_Solution::NAV_SOL_Solution(void){
	_msgClass = CLASS_NAV;
	_msgID = ID_NAV_SOL;
	_dataLen = 52;

// +Praise Jesus+
	iTOW = 0;
	fTOW = 0;
	week = 0;
	gpsFix = 0;

	//bools

	GPSfixOK = 0;
	DiffSoln = 0;
	WKNSET = 0;
	TOWSET = 0;

	ecefX = 0;
	ecefY = 0;
	ecefZ = 0;
	pAcc = 0;
	ecefVX = 0;
	ecefVY = 0;
	ecefVZ = 0;
	sAcc = 0;
	pDOP = 0;
	reserved1 = 0;
	numSV = 0;
	reserved2 = 0;

}

void NAV_SOL_Solution::encodeMsg(uint8_t* buf){
	uint8_t temp[4];

	// Encode iTow
	encodeU4(temp, iTOW);
	buf[NUM_HEADER_BYTES + 0] = temp[0];
	buf[NUM_HEADER_BYTES + 1] = temp[1];
	buf[NUM_HEADER_BYTES + 2] = temp[2];
	buf[NUM_HEADER_BYTES + 3] = temp[3];

	// Encode fTow
	encodeI4(temp, fTOW);
	buf[NUM_HEADER_BYTES + 4] = temp[0];
	buf[NUM_HEADER_BYTES + 5] = temp[1];
	buf[NUM_HEADER_BYTES + 6] = temp[2];
	buf[NUM_HEADER_BYTES + 7] = temp[3];

	// Encode week
	encodeI2(temp, week);
	buf[NUM_HEADER_BYTES + 8] = temp[0];
	buf[NUM_HEADER_BYTES + 9] = temp[1];

	// Encode gpsFix
	encodeU1(temp, gpsFix);
	buf[NUM_HEADER_BYTES + 12] = temp[0];

	// Encode flags
	flags = TOWSET<<3|WKNSET<<2|DiffSoln<<1|GPSfixOK;
	temp[0] = flags;
	buf[NUM_HEADER_BYTES + 13] = temp[0];

	// Encode ecefX
	encodeI4(temp, ecefX);
	buf[NUM_HEADER_BYTES + 14] = temp[0];
	buf[NUM_HEADER_BYTES + 15] = temp[1];
	buf[NUM_HEADER_BYTES + 16] = temp[2];
	buf[NUM_HEADER_BYTES + 17] = temp[3];

	// Encode ecefY
	encodeI4(temp, ecefY);
	buf[NUM_HEADER_BYTES + 18] = temp[0];
	buf[NUM_HEADER_BYTES + 19] = temp[1];
	buf[NUM_HEADER_BYTES + 20] = temp[2];
	buf[NUM_HEADER_BYTES + 21] = temp[3];

	// Encode ecefZ
	encodeI4(temp, ecefZ);
	buf[NUM_HEADER_BYTES + 22] = temp[0];
	buf[NUM_HEADER_BYTES + 23] = temp[1];
	buf[NUM_HEADER_BYTES + 24] = temp[2];
	buf[NUM_HEADER_BYTES + 25] = temp[3];

	// Encode pAcc
	encodeU4(temp, pAcc);
	buf[NUM_HEADER_BYTES + 26] = temp[0];
	buf[NUM_HEADER_BYTES + 27] = temp[1];
	buf[NUM_HEADER_BYTES + 28] = temp[2];
	buf[NUM_HEADER_BYTES + 29] = temp[3];

	// Encode ecefVX
	encodeI4(temp, ecefVX);
	buf[NUM_HEADER_BYTES + 30] = temp[0];
	buf[NUM_HEADER_BYTES + 31] = temp[1];
	buf[NUM_HEADER_BYTES + 32] = temp[2];
	buf[NUM_HEADER_BYTES + 33] = temp[3];

	// Encode ecefVY
	encodeI4(temp, ecefVY);
	buf[NUM_HEADER_BYTES + 34] = temp[0];
	buf[NUM_HEADER_BYTES + 35] = temp[1];
	buf[NUM_HEADER_BYTES + 36] = temp[2];
	buf[NUM_HEADER_BYTES + 37] = temp[3];

	// Encode ecefVZ
	encodeI4(temp, ecefVZ);
	buf[NUM_HEADER_BYTES + 38] = temp[0];
	buf[NUM_HEADER_BYTES + 39] = temp[1];
	buf[NUM_HEADER_BYTES + 40] = temp[2];
	buf[NUM_HEADER_BYTES + 41] = temp[3];

	// Encode sAcc
	encodeU4(temp, sAcc);
	buf[NUM_HEADER_BYTES + 42] = temp[0];
	buf[NUM_HEADER_BYTES + 43] = temp[1];
	buf[NUM_HEADER_BYTES + 44] = temp[2];
	buf[NUM_HEADER_BYTES + 45] = temp[3];

	// Encode pDOP
	encodeU2(temp, pDOP);
	buf[NUM_HEADER_BYTES + 46] = temp[0];
	buf[NUM_HEADER_BYTES + 47] = temp[1];

	// Encode reserved1
	encodeU1(temp, reserved1);
	buf[NUM_HEADER_BYTES + 48] = temp[0];

	// Encode numSV
	encodeU1(temp, numSV);
	buf[NUM_HEADER_BYTES + 49] = temp[0];

	//Encode reserved2
	encodeU4(temp, reserved2);
	buf[NUM_HEADER_BYTES + 50] = temp[0];
	buf[NUM_HEADER_BYTES + 51] = temp[1];
	buf[NUM_HEADER_BYTES + 52] = temp[2];
	buf[NUM_HEADER_BYTES + 53] = temp[3];

}

void NAV_SOL_Solution::decodeMsg(uint8_t* buf){
		uint8_t temp[4];

		// Decode iTow
		temp[0] = buf[NUM_HEADER_BYTES + 0];
		temp[1] = buf[NUM_HEADER_BYTES + 1];
		temp[2] = buf[NUM_HEADER_BYTES + 2];
		temp[3] = buf[NUM_HEADER_BYTES + 3];
		iTOW = decodeU4(temp);

		// Decode fTOW
		temp[0] = buf[NUM_HEADER_BYTES + 5];
		temp[1] = buf[NUM_HEADER_BYTES + 6];
		temp[2] = buf[NUM_HEADER_BYTES + 7];
		temp[3] = buf[NUM_HEADER_BYTES + 8];
		fTOW = decodeI4(temp);

		// Decode week
		temp[0] = buf[NUM_HEADER_BYTES + 9];
		temp[1] = buf[NUM_HEADER_BYTES + 10];
		week = decodeI2(temp);

		// Decode gpsFix
		temp[0] = buf[NUM_HEADER_BYTES + 11];
		gpsFix = decodeU1(temp);

		// Decode flags
		temp[0] = buf[NUM_HEADER_BYTES + 12];
		TOWSET = temp[0] & (1<<3);
		WKNSET = temp[0] & (1<<2);
		DiffSoln = temp[0] & (1<<1);
		GPSfixOK = temp[0] & 1;
		flags = TOWSET+WKNSET+DiffSoln+GPSfixOK;

		// Decode ecefX
		temp[0] = buf[NUM_HEADER_BYTES + 14];
		temp[1] = buf[NUM_HEADER_BYTES + 15];
		temp[2] = buf[NUM_HEADER_BYTES + 16];
		temp[3] = buf[NUM_HEADER_BYTES + 17];
		ecefX = decodeI4(temp);

		// Decode ecefY
		temp[0] = buf[NUM_HEADER_BYTES + 18];
		temp[1] = buf[NUM_HEADER_BYTES + 19];
		temp[2] = buf[NUM_HEADER_BYTES + 20];
		temp[3] = buf[NUM_HEADER_BYTES + 21];
		ecefY = decodeI4(temp);

		// Decode ecefZ
		temp[0] = buf[NUM_HEADER_BYTES + 22];
		temp[1] = buf[NUM_HEADER_BYTES + 23];
		temp[2] = buf[NUM_HEADER_BYTES + 24];
		temp[3] = buf[NUM_HEADER_BYTES + 25];
		ecefZ = decodeI4(temp);

		// Decode pAcc
		temp[0] = buf[NUM_HEADER_BYTES + 26];
		temp[1] = buf[NUM_HEADER_BYTES + 27];
		temp[2] = buf[NUM_HEADER_BYTES + 28];
		temp[3] = buf[NUM_HEADER_BYTES + 29];
		pAcc = decodeU4(temp);

		// Decode ecefVX
		temp[0] = buf[NUM_HEADER_BYTES + 30];
		temp[1] = buf[NUM_HEADER_BYTES + 31];
		temp[2] = buf[NUM_HEADER_BYTES + 32];
		temp[3] = buf[NUM_HEADER_BYTES + 33];
		ecefVX = decodeI4(temp);

		// Decode evefVY
		temp[0] = buf[NUM_HEADER_BYTES + 34];
		temp[1] = buf[NUM_HEADER_BYTES + 35];
		temp[2] = buf[NUM_HEADER_BYTES + 36];
		temp[3] = buf[NUM_HEADER_BYTES + 37];
		ecefVY = decodeI4(temp);

		// Decode ecefVZ
		temp[0] = buf[NUM_HEADER_BYTES + 38];
		temp[1] = buf[NUM_HEADER_BYTES + 39];
		temp[2] = buf[NUM_HEADER_BYTES + 40];
		temp[3] = buf[NUM_HEADER_BYTES + 41];
		ecefVZ = decodeI4(temp);

		// Decode sAcc
		temp[0] = buf[NUM_HEADER_BYTES + 42];
		temp[1] = buf[NUM_HEADER_BYTES + 43];
		temp[2] = buf[NUM_HEADER_BYTES + 44];
		temp[3] = buf[NUM_HEADER_BYTES + 45];
		sAcc = decodeU4(temp);

		// Decode pDOP
		temp[0] = buf[NUM_HEADER_BYTES + 46];
		temp[1] = buf[NUM_HEADER_BYTES + 47];
		pDOP = decodeU2(temp);

		// Decode reserved1
		temp[0] = buf[NUM_HEADER_BYTES + 48];
		reserved1 = decodeU1(temp);

		// Decode numSV
		temp[0] = buf[NUM_HEADER_BYTES + 49];
		numSV = decodeU1(temp);

		// Decode reserved2
		temp[0] = buf[NUM_HEADER_BYTES + 50];
		temp[1] = buf[NUM_HEADER_BYTES + 51];
		temp[2] = buf[NUM_HEADER_BYTES + 52];
		temp[3] = buf[NUM_HEADER_BYTES + 53];
		reserved2 = decodeU4(temp);

}
NAV_STATUS::NAV_STATUS(void){
	_msgClass = CLASS_NAV;
	_msgID = ID_NAV_STATUS;
	_dataLen = 16;

	iTOW = 0;
	gpsFix = 0;

	//Bools for flags
	gpsFixOK = 0;
	diffSoln = 0;
	wknSet = 0;
	towSet = 0;

	//Bools for fixStat
	dgpsIStat = 0;
	mapMatching[0] = 0;
	mapMatching[1] = 0;

	//Bools for flags2
	psmState[0] = 0;
	psmState[1] = 0;

	ttff = 0;
	msss = 0;

}
void NAV_STATUS::encodeMsg(uint8_t* buf){
	uint8_t temp[4];

	// Encode iTOW
	encodeU4(temp, iTOW);
	buf[NUM_HEADER_BYTES + 0] = temp[0];
	buf[NUM_HEADER_BYTES + 1] = temp[1];
	buf[NUM_HEADER_BYTES + 2] = temp[2];
	buf[NUM_HEADER_BYTES + 3] = temp[3];

	//Encode gpsFix
	encodeU1(temp,gpsFix);
	buf[NUM_HEADER_BYTES + 4] = temp[0];

	//Encode flags
	flags = towSet<<3|wknSet<<2|diffSoln<<1|gpsFixOK;
	temp[0] = flags;
	buf[NUM_HEADER_BYTES + 5] = temp[0];

	//Encode fixStat
	fixStat = mapMatching[1]<<7|mapMatching[0]<<6|dgpsIStat;
	temp[0] = fixStat;
	buf[NUM_HEADER_BYTES + 6] = temp[0];

	//Encode flags2
	flags2 = psmState[1]<<1|psmState[0];
	temp[0] = flags2;
	buf[NUM_HEADER_BYTES + 7] = temp[0];

	//Encode ttff
	encodeU4(temp,ttff);
	buf[NUM_HEADER_BYTES + 8] = temp[0];
	buf[NUM_HEADER_BYTES + 9] = temp[1];
	buf[NUM_HEADER_BYTES + 10] = temp[2];
	buf[NUM_HEADER_BYTES + 11] = temp[3];

	//Encode msss
	encodeU4(temp,msss);
	buf[NUM_HEADER_BYTES + 12] = temp[0];
	buf[NUM_HEADER_BYTES + 13] = temp[1];
	buf[NUM_HEADER_BYTES + 14] = temp[2];
	buf[NUM_HEADER_BYTES + 15] = temp[3];
}

void NAV_STATUS::decodeMsg(uint8_t* buf){
	uint8_t temp[4];

	//Decode iTOW
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	iTOW = decodeU4(temp);

	//Decode gpsFix
	temp[0] = buf[NUM_HEADER_BYTES + 4];
	gpsFix = decodeU1(temp);

	// Decode flags
	temp[0] = buf[NUM_HEADER_BYTES + 5];
	towSet = temp[0] & (1<<3);
	wknSet = temp[0] & (1<<2);
	diffSoln = temp[0] & (1<<1);
	gpsFixOK = temp[0] & 1;
	flags = towSet+wknSet+diffSoln+gpsFixOK;

	//Decode fixStat
	temp[0] = buf[NUM_HEADER_BYTES + 6];
	mapMatching[1] = temp[0] & (1<<7);
	mapMatching[0] = temp[0] & (1<<6);
	dgpsIStat = temp[0] & 1;
	fixStat = mapMatching[1] + mapMatching[2] + dgpsIStat;

	//Decode flags2
	temp[0] = buf[NUM_HEADER_BYTES + 7];
	psmState[1] = temp[0] & (1<<1);
	psmState[0] = temp[0] & 1;
	flags2 = psmState[1] + psmState[0];

	//Decode ttff;
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 0];
	temp[2] = buf[NUM_HEADER_BYTES + 0];
	temp[3] = buf[NUM_HEADER_BYTES + 0];
	ttff = decodeU4(temp);

	//Decode msss
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 0];
	temp[2] = buf[NUM_HEADER_BYTES + 0];
	temp[3] = buf[NUM_HEADER_BYTES + 0];
	msss = decodeU4(temp);
}

NAV_TIMEUTC::NAV_TIMEUTC(void){
	_msgClass = CLASS_NAV;
	_msgID = ID_NAV_TIMEUTC;
	_dataLen = 20;

	iTOW = 0;
	tAcc = 0;
	nano = 0;
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	min = 0;
	sec = 0;

	//bools for valid
	validTOW = 0;
	validWKN = 0;
	validUTC = 0;
}

void NAV_TIMEUTC::encodeMsg(uint8_t* buf){
	uint8_t temp[4];

	//Encode iTOW
	encodeU4(temp,iTOW);
	buf[NUM_HEADER_BYTES + 0] = temp[0];
	buf[NUM_HEADER_BYTES + 1] = temp[1];
	buf[NUM_HEADER_BYTES + 2] = temp[2];
	buf[NUM_HEADER_BYTES + 3] = temp[3];

	//Encode tAcc
	encodeU4(temp,tAcc);
	buf[NUM_HEADER_BYTES + 4] = temp[0];
	buf[NUM_HEADER_BYTES + 5] = temp[1];
	buf[NUM_HEADER_BYTES + 6] = temp[2];
	buf[NUM_HEADER_BYTES + 7] = temp[3];

	//Encode nano
	encodeI4(temp,nano);
	buf[NUM_HEADER_BYTES + 8] = temp[0];
	buf[NUM_HEADER_BYTES + 9] = temp[1];
	buf[NUM_HEADER_BYTES + 10] = temp[2];
	buf[NUM_HEADER_BYTES + 11] = temp[3];

	//Encode year
	encodeU2(temp,year);
	buf[NUM_HEADER_BYTES + 12] = temp[0];
	buf[NUM_HEADER_BYTES + 13] = temp[1];

	//Encode month
	encodeU1(temp,month);
	buf[NUM_HEADER_BYTES + 14] = temp[0];

	//Encode day
	encodeU1(temp,day);
	buf[NUM_HEADER_BYTES + 15] = temp[0];

	//Encode hour
	encodeU1(temp,hour);
	buf[NUM_HEADER_BYTES + 16] = temp[0];

	//Encode min
	encodeU1(temp,min);
	buf[NUM_HEADER_BYTES + 17] = temp[0];

	//Encode sec
	encodeU1(temp,sec);
	buf[NUM_HEADER_BYTES + 18] = temp[0];

	//Encode valid
	valid = validUTC<<2|validWKN<<1|validTOW;
	temp[0] = valid;
	buf[NUM_HEADER_BYTES + 19] = temp[0];

}

void NAV_TIMEUTC::decodeMsg(uint8_t* buf){
	uint8_t temp[4];

	// Decode iTOW
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	iTOW = decodeU4(temp);

	// Decode tAcc
	temp[0] = buf[NUM_HEADER_BYTES + 4];
	temp[1] = buf[NUM_HEADER_BYTES + 5];
	temp[2] = buf[NUM_HEADER_BYTES + 6];
	temp[3] = buf[NUM_HEADER_BYTES + 7];
	tAcc = decodeU4(temp);

	// Decode nano
	temp[0] = buf[NUM_HEADER_BYTES + 8];
	temp[1] = buf[NUM_HEADER_BYTES + 9];
	temp[2] = buf[NUM_HEADER_BYTES + 10];
	temp[3] = buf[NUM_HEADER_BYTES + 11];
	nano = decodeI4(temp);

	// Decode year
	temp[0] = buf[NUM_HEADER_BYTES + 12];
	temp[1] = buf[NUM_HEADER_BYTES + 13];
	year = decodeU2(temp);

	// Decode month
	temp[0] = buf[NUM_HEADER_BYTES + 14];
	month = decodeU1(temp);

	// Decode day
	temp[0] = buf[NUM_HEADER_BYTES + 15];
	month = decodeU1(temp);

	// Decode hour
	temp[0] = buf[NUM_HEADER_BYTES + 16];
	hour = decodeU1(temp);

	// Decode min
	temp[0] = buf[NUM_HEADER_BYTES + 17];
	min = decodeU1(temp);

	// Decode sec
	temp[0] = buf[NUM_HEADER_BYTES + 18];
	sec = decodeU1(temp);

	// Decode valid
	temp[0] = buf[NUM_HEADER_BYTES + 19];
	validUTC = temp[0] & (1<<2);
	validWKN = temp[0] & (1<<1);
	validTOW = temp[0] & 1;
	valid = validUTC + validWKN + validTOW;

}
NAV_VELNED::NAV_VELNED(void){
	_msgClass = CLASS_NAV;
	_msgID = ID_NAV_VELNED;
	_dataLen = 36;

	//Message-Specific fields

	iTOW = 0;
	velN = 0;
	velE = 0;
	velD = 0;
	speed = 0;
	gSpeed = 0;
	heading = 0;
	sAcc = 0;
	cAcc = 0;
}
void NAV_VELNED::encodeMsg(uint8_t* buf){
	uint8_t temp[4];

	//Encode iTOW
	encodeU4(temp,iTOW);
	buf[NUM_HEADER_BYTES + 0] = temp[0];
	buf[NUM_HEADER_BYTES + 1] = temp[1];
	buf[NUM_HEADER_BYTES + 2] = temp[2];
	buf[NUM_HEADER_BYTES + 3] = temp[3];

	//Encode velN
	encodeI4(temp,velN);
	buf[NUM_HEADER_BYTES + 4] = temp[0];
	buf[NUM_HEADER_BYTES + 5] = temp[1];
	buf[NUM_HEADER_BYTES + 6] = temp[2];
	buf[NUM_HEADER_BYTES + 7] = temp[3];

	//Encode velE
	encodeI4(temp,velE);
	buf[NUM_HEADER_BYTES + 8] = temp[0];
	buf[NUM_HEADER_BYTES + 9] = temp[1];
	buf[NUM_HEADER_BYTES + 10] = temp[2];
	buf[NUM_HEADER_BYTES + 11] = temp[3];

	//Encode velD
	encodeI4(temp,velD);
	buf[NUM_HEADER_BYTES + 12] = temp[0];
	buf[NUM_HEADER_BYTES + 13] = temp[1];
	buf[NUM_HEADER_BYTES + 14] = temp[2];
	buf[NUM_HEADER_BYTES + 15] = temp[3];

	//Encode speed
	encodeU4(temp,speed);
	buf[NUM_HEADER_BYTES + 16] = temp[0];
	buf[NUM_HEADER_BYTES + 17] = temp[1];
	buf[NUM_HEADER_BYTES + 18] = temp[2];
	buf[NUM_HEADER_BYTES + 19] = temp[3];

	//Encode gSpeed
	encodeU4(temp,gSpeed);
	buf[NUM_HEADER_BYTES + 20] = temp[0];
	buf[NUM_HEADER_BYTES + 21] = temp[1];
	buf[NUM_HEADER_BYTES + 22] = temp[2];
	buf[NUM_HEADER_BYTES + 23] = temp[3];

	//Encode heading
	encodeI4(temp,heading);
	buf[NUM_HEADER_BYTES + 24] = temp[0];
	buf[NUM_HEADER_BYTES + 25] = temp[1];
	buf[NUM_HEADER_BYTES + 26] = temp[2];
	buf[NUM_HEADER_BYTES + 27] = temp[3];

	//Encode sAcc
	encodeU4(temp,sAcc);
	buf[NUM_HEADER_BYTES + 28] = temp[0];
	buf[NUM_HEADER_BYTES + 29] = temp[1];
	buf[NUM_HEADER_BYTES + 30] = temp[2];
	buf[NUM_HEADER_BYTES + 31] = temp[3];

	//Encode cAcc
	encodeU4(temp,cAcc);
	buf[NUM_HEADER_BYTES + 32] = temp[0];
	buf[NUM_HEADER_BYTES + 33] = temp[1];
	buf[NUM_HEADER_BYTES + 34] = temp[2];
	buf[NUM_HEADER_BYTES + 35] = temp[3];

}

void NAV_VELNED::decodeMsg(uint8_t* buf){
	uint8_t temp[4];

	// Decode iTOW
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	iTOW = decodeU4(temp);

	// Decode velN
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	velN = decodeI4(temp);

	// Decode velE
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	velE = decodeI4(temp);

	// Decode velD
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	velD = decodeI4(temp);

	// Decode speed
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	speed = decodeU4(temp);

	// Decode gSpeed
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	gSpeed = decodeU4(temp);

	// Decode heading
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	heading = decodeI4(temp);

	// Decode sAcc
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	sAcc = decodeU4(temp);

	// Decode cAcc
	temp[0] = buf[NUM_HEADER_BYTES + 0];
	temp[1] = buf[NUM_HEADER_BYTES + 1];
	temp[2] = buf[NUM_HEADER_BYTES + 2];
	temp[3] = buf[NUM_HEADER_BYTES + 3];
	cAcc = decodeU4(temp);

}

}
