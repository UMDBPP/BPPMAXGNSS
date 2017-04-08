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




namespace BPPGNSS {
	
	void appendChecksum(uint8_t* msg, uint8_t msgLength);
	
	void packU4(uint8_t* buf, uint32_t ulongToPack);
	void packL4(uint8_t* buf, uint32_t longToPack);	
	void packU2(uint8_t* buf, uint16_t ulongToPack);
	void packL2(uint8_t* buf, uint16_t longToPack);
	
	uint32_t unpackU4(uint8_t* buf);
	int32_t unpackL4(uint8_t* buf);	
	uint16_t unpackU2(uint8_t* buf);
	int16_t unpackL2(uint8_t* buf);
	
	
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
