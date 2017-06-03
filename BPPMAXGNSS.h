/*
 * University of Maryland Balloon Payload Prgogram u-blox GPS Library
 * For use with u-blox MAX series GNSS modules
 *
 * Copyright (c) 2017 University of Maryland Space Systems Laboratory.
 *
 * Some parts of this library are derived from the HABDuino code by Anthony
 * Stirk <https://github.com/HABduino> under the provisions of the GNU GPL v3.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * See <http://www.gnu.org/licenses/>.
 */

#ifndef BPPGNSS_h
#define BPPGNSS_h

#include <Arduino.h>
#include <I2C.h>

#define GNSS_ADDRESS 66 // I2C address of u-blox MAX GNSS chip
#define GNSS_REGISTER 0xFE // I2C Register to read from on u-blox MAX GNSS chip
#define DEFAULT_BYTES_TO_READ 32

// Different interface modes for the GNSS chip
#define INTERFACE_I2C 0
#define INTERFACE_SERIAL 1
#define INTERFACE_SOFT_SERIAL 2

#define UBX_HEADER_1 0xB5
#define UBX_HEADER_2 0x62

#define NUM_HEADER_BYTES 6
#define NUM_CONTROL_BYTES 8	// The number of control (header plus checksum) in a message

#define CLASS_NAV 0x01
#define CLASS_ACK 0x05
#define CLASS_CFG 0x06

#define ID_CFG_MSG 0x01
#define ID_CFG_NAV5 0x24
#define ID_CFG_NMEA 0x17

#define ID_NAV_POSLLH 0x02
#define ID_NAV_DOP 0x04
#define ID_NAV_PVT 0x07
#define ID_NAV_SOL 0x06
#define ID_NAV_STATUS 0x03
#define ID_NAV_TIMEUTC 0x21
#define ID_NAV_VELNED 0x12

namespace BPPGNSS {

	void appendChecksum(uint8_t* msg, uint8_t msgLength);

	void encodeU4(uint8_t* buf, uint32_t ulongToPack);
	void encodeI4(uint8_t* buf, uint32_t longToPack);
	void encodeU2(uint8_t* buf, uint16_t ulongToPack);
	void encodeI2(uint8_t* buf, uint16_t longToPack);
	void encodeU1(uint8_t* buf, uint16_t ulongtoPack);
	void encodeI1(uint8_t* buf, uint16_t ulongtoPack);

	uint32_t decodeU4(uint8_t* buf);
	int32_t decodeI4(uint8_t* buf);
	uint16_t decodeU2(uint8_t* buf);
	int16_t decodeI2(uint8_t* buf);
	uint8_t decodeU1(uint8_t* buf);
	int8_t decodeI1(uint8_t* buf);


	class UBXMsg {
		public:
			uint8_t getDataLength(void);
			virtual void encodeMsg(uint8_t* buf);
			virtual void decodeMsg(uint8_t* buf) = 0; // Pure virtual
		protected:
			uint8_t _dataLen;
			uint8_t _msgClass;
			uint8_t _msgID;
	};
//////////////////////////////////////////////////
	class CFG_MSG_Poll : protected UBXMsg {
		public:
			CFG_MSG_Poll();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			// Message-specific fields
			uint8_t msgClass;
			uint8_t msgID;
	};

	class CFG_MSG_SetRates : protected UBXMsg {
		public:
			CFG_MSG_SetRates();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			// Message-specific fields
			uint8_t msgClass;
			uint8_t msgID;
			uint8_t rates[6];
	};

	class CFG_MSG_SetRate : protected UBXMsg {
		public:
			CFG_MSG_SetRate();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			// Message-specific fields
			uint8_t msgClass;
			uint8_t msgID;
			uint8_t rate;
	};

	class CFG_NAV5_Poll : protected UBXMsg{
		public:
			CFG_NAV5_Poll();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-specific fields
			//Poll has no payload

	};
	class CFG_NAV5_EngineSettings : protected UBXMsg{

	public:
		CFG_NAV5_EngineSettings();
		void encodeMsg(uint8_t* buf);
		void decodeMsg(uint8_t* buf);

		//Message-specific fields

		uint16_t mask; //Bitfield
		//bools for mask
		bool ReservedBit0[8];
		bool dgpsMask;
		bool staticHoldMask;
		bool timeMask;
		bool posMask;
		bool drLim;
		bool posFixMode;
		bool minEl;
		bool dyn;

		uint8_t dynMode1;
		uint8_t fixMode;
		int32_t fixedAlt;
		uint32_t fixedAltVar;
		int8_t minElev;
		uint8_t drLimit;
		uint16_t pDop;
		uint16_t tDop;
		uint16_t pAcc;
		uint16_t tAcc;
		uint8_t staticHoldThresh;
		uint8_t dgpsTimeOut;
		uint8_t cnoThreshNumSVs;
		uint8_t cnoThresh;
		uint16_t reserved2;
		uint32_t reserved3;
		uint32_t reserved4;


	};

	class CFG_NMEA_Poll : protected UBXMsg{
		public:
			CFG_NMEA_Poll();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific fields

			//No payload

	};

	class CFG_NMEA_Deprecated : protected UBXMsg{
		public:
			CFG_NMEA_Deprecated();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific fields

			uint8_t filter; //Bitfield
			//bools for filter
			bool trackFilt;
			bool gpsOnlyFilter;
			bool dateFilt;
			bool timeFilt;
			bool mskPosFilt;
			bool posFilt;

			uint8_t nmeaVersion;
			uint8_t numSV;

			uint8_t flags; //Bitfield
			//bools for flags
			bool consider;
			bool compat;



	};

///////////////////////////////////////////////////////
	class NAV_POSLLH_Solution : protected UBXMsg {
		public:
			NAV_POSLLH_Solution();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			// Message-specific fields
			uint32_t iTOW;
			int32_t lon;
			int32_t lat;
			int32_t height;
			int32_t hMSL;
			uint32_t hAcc;
			uint32_t vAcc;


	};

	class NAV_DOP_Dilution : protected UBXMsg {
		public:
		    NAV_DOP_Dilution();
		    void encodeMsg(uint8_t* buf);
		    void decodeMsg(uint8_t* buf);

		          //Message-specific fields
		    uint32_t iTOW;
		    uint16_t gDOP;
		    uint16_t pDOP;
		    uint16_t tDOP;
		    uint16_t vDOP;
		    uint16_t hDOP;
		    uint16_t nDOP;
		    uint16_t eDOP;
	};
  class NAV_PVT_Solution : protected UBXMsg{
    public:
      NAV_PVT_Solution();
      void encodeMsg(uint8_t* buf);
      void decodeMsg(uint8_t* buf);

      //Message-specific fields
      uint32_t iTOW;
      uint16_t year;
      uint8_t  month;
      uint8_t  day;
      uint8_t  hour;
      uint8_t  min;
      uint8_t  sec;

			uint8_t valid; // Bitfield
			//Bools for valid
			bool validDate;
			bool validTime;
			bool fullyResolved;

			uint32_t tAcc;
			int32_t nano;
			uint8_t fixType;

			uint8_t flags; // Bitfield
			//Bools for flags
			bool gnssFixOK;
			bool diffSoln;
			bool psmState[3];

			uint8_t reserved1;
			uint8_t numSV;
			int32_t lon;
			int32_t lat;
			int32_t height;
			int32_t hMSL;
			uint32_t hAcc;
			uint32_t vAcc;
			int32_t velN;
			int32_t velE;
			int32_t velD;
			int32_t gSpeed;
			int32_t heading;
			uint32_t sAcc;
			uint32_t headingAcc;
			uint16_t pDOP;

			uint16_t reserved2; // Bitfield (reserved, no specification for bitfield)

			uint32_t reserved3;
  };

  /*class NAV_SBAS_Data : protected UBXMsg{
    public:
      NAV_SBAS_Data();
      void encodeMsg(uint8_t* buf);
      void decodeMsg(uint8_t* buf);

      //Message-Specific fields

      uint32_t iTOW;
      uint8_t  geo;



  };*/

	class NAV_SOL_Solution : protected UBXMsg{
		public:
			NAV_SOL_Solution();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific field

			uint32_t iTOW;
			int32_t fTOW;
			int16_t week;
			uint8_t gpsFix;
			uint8_t flags;//Bitfield

			// bools for flags

			bool GPSfixOK;
			bool DiffSoln;
			bool WKNSET;
			bool TOWSET;

			int32_t ecefX;
			int32_t ecefY;
			int32_t ecefZ;
			uint32_t pAcc;
			int32_t ecefVX;
			int32_t ecefVY;
			int32_t ecefVZ;
			uint32_t sAcc;
			uint16_t pDOP;
			uint8_t reserved1;
			uint8_t numSV;
			uint32_t reserved2;
	};

	class NAV_STATUS : protected UBXMsg{
		public:
			NAV_STATUS();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific fields

			uint32_t iTOW;
			uint8_t gpsFix;

			//Bitfields

			uint8_t flags;

			//bools for flags
			bool gpsFixOK;
			bool diffSoln;
			bool wknSet;
			bool towSet;

			uint8_t fixStat;

			//bools for fixStat
			bool dgpsIStat;
			bool mapMatching[2];

			uint8_t flags2;

			//bools for flags2
			bool psmState[2];


			uint32_t ttff;
			uint32_t msss;


	};

	class NAV_TIMEUTC : protected UBXMsg{
		public:
			NAV_TIMEUTC();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific fields

			uint32_t iTOW;
			uint32_t tAcc;
			int32_t nano;
			uint16_t year;
			uint8_t month;
			uint8_t day;
			uint8_t hour;
			uint8_t min;
			uint8_t sec;

			uint8_t valid; //Bitfield
			//Bools for valid
			bool validTOW;
			bool validWKN;
			bool validUTC;

	};

	class NAV_VELNED : protected UBXMsg{
		public:
			NAV_VELNED();
			void encodeMsg(uint8_t* buf);
			void decodeMsg(uint8_t* buf);

			//Message-Specific fields

			uint32_t iTOW;
			int32_t velN;
			int32_t velE;
			int32_t velD;
			uint32_t speed;
			uint32_t gSpeed;
			int32_t heading;
			uint32_t sAcc;
			uint32_t cAcc;
	};



	class MAXGNSS {
		public:
			MAXGNSS(uint8_t interfaceMode);

			uint8_t readBytes(uint8_t* buffer, uint8_t length, uint16_t timeout);
			uint8_t bytesAvailable();
			uint8_t sendBytes(uint8_t* msg, uint8_t msgLength);
			bool getUBX_ACK(uint8_t *msg);


		private:
			uint8_t _interfaceMode; // Identifies the interface the GPS is on (I2C, Serial, etc.)

			uint8_t bytesAvailableI2C();
			uint8_t bytesAvailableSerial();
			uint8_t bytesAvailableSoftSerial();

			uint8_t sendBytesI2C(uint8_t* msg, uint8_t msgLength);
			uint8_t sendBytesSerial(uint8_t* msg, uint8_t msgLength);
			uint8_t sendBytesSoftSerial(uint8_t* msg, uint8_t msgLength);

			uint8_t readBytesI2C(uint8_t* buffer, uint8_t length, uint16_t timeout);
			uint8_t readBytesSerial(uint8_t* buffer, uint8_t length, uint16_t timeout);
			uint8_t readBytesSoftSerial(uint8_t* buffer, uint8_t length, uint16_t timeout);



	};

}
#endif
