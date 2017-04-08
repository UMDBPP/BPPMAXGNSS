#include "Arduino.h"
#include "BPPMAXGNSS.h"



namespace BPPGNSS {
MAXGNSS::MAXGNSS(uint8_t interfaceMode) {
	I2c.begin();
	_interfaceMode = interfaceMode;
}

bool MAXGNSS::getUBX_ACK(uint8_t *msg) { // Adapted from from HABDuino
  uint8_t b;
  uint8_t ackByteID = 0;
  const uint8_t ACK_PACKET_LENGTH = 10;
  const uint32_t TIMEOUT = 3000; // Milliseconds
  uint8_t ackPacket[ACK_PACKET_LENGTH];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = UBX_HEADER_1;	// header
  ackPacket[1] = UBX_HEADER_1;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = msg[2];	// ACK class
  ackPacket[7] = msg[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  appendChecksum(ackPacket, ACK_PACKET_LENGTH);

  while (true) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > TIMEOUT) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

void appendChecksum(uint8_t* msg, uint8_t msgLength)
{

	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	for(int i = 2; i< (msgLength - 2); i++)
	{
		CK_A = CK_A + msg[i];
		CK_B = CK_B + CK_A;
	}
	
	
	msg[msgLength - 2] = CK_A;
	msg[msgLength - 1] = CK_B;
}

uint8_t MAXGNSS::bytesAvailable() {
	switch (_interfaceMode) {
		case INTERFACE_I2C: {
			return bytesAvailableI2C();
		}
		case INTERFACE_SERIAL: {
			return 0; // TODO
		}
		case INTERFACE_SOFT_SERIAL: {
			return 0; // TODO
		}
		default: {
			return 0; // TODO
		}
		
	}
}

uint8_t MAXGNSS::bytesAvailableI2C() {
	return I2c.available();
}

uint8_t MAXGNSS::bytesAvailableSerial() {
	return 0; // TODO
}

uint8_t MAXGNSS::bytesAvailableSoftSerial() {
	return 0; // TODO
}


// Returns number of bytes sent
uint8_t MAXGNSS::sendBytes(uint8_t* msg, uint8_t length) {
	switch (_interfaceMode) {
		case INTERFACE_I2C: {
			return sendBytesI2C(msg, length);
		}
		case INTERFACE_SERIAL: {
			return 0; // TODO
		}
		case INTERFACE_SOFT_SERIAL: {
			return 0; // TODO
		}
		default: {
			return 0; // TODO
		}
		
	}
}

uint8_t MAXGNSS::sendBytesI2C(uint8_t* msg, uint8_t length) {
	I2c.begin();
	I2c.write(GNSS_ADDRESS, GNSS_REGISTER, 0XFF); // wakes GNSS
	delay(100);
	uint8_t bytesSent = I2c.write(GNSS_ADDRESS, GNSS_REGISTER, msg, length);
	I2c.end();
	return bytesSent;
}


// Returns number of bytes read
uint8_t MAXGNSS::readBytes(uint8_t* buffer, uint8_t length, uint16_t timeout = 3000) {
	switch (_interfaceMode) {
		case INTERFACE_I2C: {
			return readBytesI2C(buffer, length, timeout);
		}
		case INTERFACE_SERIAL: {
			return 0; // TODO
		}
		case INTERFACE_SOFT_SERIAL: {
			return 0; // TODO
		}
		default: {
			return 0; // TODO
		}
		
	}
}

// Returns the number of bytes read
uint8_t MAXGNSS::readBytesI2C(uint8_t* buffer, uint8_t length, uint16_t timeout = 3000) {
	unsigned long startTime = millis();
	uint8_t bytesRead = 0;
	while(bytesRead < length)
	{
		if((millis() - startTime) > timeout) { // Enforce the timeout
			break;
		}
		
		while(I2c.available()) { // Read the bytes in the buffer
			uint8_t b = I2c.receive();
			buffer[bytesRead] = b;
			bytesRead++;
		}
		
		if(I2c.available() == 0) { // If the buffer is empty, try to refill it
			I2c.read(GNSS_ADDRESS, GNSS_REGISTER, DEFAULT_BYTES_TO_READ); 
		}		
	}
	return bytesRead;
}

uint8_t MAXGNSS::readBytesSerial(uint8_t* buffer, uint8_t length, uint16_t timeout = 3000) {
	return 0; // TODO
}

uint8_t MAXGNSS::readBytesSoftSerial(uint8_t* buffer, uint8_t length, uint16_t timeout = 3000) {
	return 0; // TODO
}

void UBXMsg::encodeMsg(uint8_t* buf) {
		buf[0] = UBX_HEADER_1; // Header
		buf[1] = UBX_HEADER_2;
		buf[2] = _msgClass; // Class
		buf[3] = _msgID; // ID
		buf[4] = _dataLen; // Length
		buf[5] = 0x00; // Upper byte of length; always 0 in this implementation
		appendChecksum(buf, _dataLen + NUM_CONTROL_BYTES);
	
}
}

