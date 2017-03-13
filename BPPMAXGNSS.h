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

namespace BPPGNSS {
	
	class UBXMsg {
		public:
			uint8_t* msg;
			uint8_t getLength(void);
			virtual void createMsg(void);
		private:
			uint8_t len;
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
			
			void appendChecksum(uint8_t* msg, uint8_t msgLength);
		
	};
	
	
	
	
	
}
#endif
