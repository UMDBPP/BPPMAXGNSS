#include "Arduino.h"
#include "BPPMAXGNSS.h"

namespace BPPMAXGNSS
{
    uint8_t UBXMsg::getDataLength(void)
    {
        return _dataLen;
    }

    CFG_MSG_Poll::CFG_MSG_Poll(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_MSG;
        _dataLen = 2;    // Length of data

        //Message-specific fields

        msgClass = 0;
        msgID = 0;
    }

    void CFG_MSG_Poll::encodeMsg(uint8_t* buf)
    {
        buf[NUM_HEADER_BYTES + 0] = msgClass;
        buf[NUM_HEADER_BYTES + 1] = msgID;

        UBXMsg::encodeMsg(buf);
    }

    void CFG_MSG_Poll::decodeMsg(uint8_t* buf)
    {
        msgClass = buf[NUM_HEADER_BYTES + 0];
        msgID = buf[NUM_HEADER_BYTES + 1];
    }

    CFG_MSG_SetRates::CFG_MSG_SetRates(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_MSG;
        _dataLen = 8;    // Length of data
        msgClass = 0;
        msgID = 0;
        rates[0] = 0;
        rates[1] = 0;
        rates[2] = 0;
        rates[3] = 0;
        rates[4] = 0;
        rates[5] = 0;
    }

    void CFG_MSG_SetRates::encodeMsg(uint8_t* buf)
    {
        buf[NUM_HEADER_BYTES + 0] = msgClass;
        buf[NUM_HEADER_BYTES + 1] = msgID;
        buf[NUM_HEADER_BYTES + 2] = rates[0];    // Messages rates on each port
        buf[NUM_HEADER_BYTES + 3] = rates[1];
        buf[NUM_HEADER_BYTES + 4] = rates[2];
        buf[NUM_HEADER_BYTES + 5] = rates[3];
        buf[NUM_HEADER_BYTES + 6] = rates[4];
        buf[NUM_HEADER_BYTES + 7] = rates[5];

        UBXMsg::encodeMsg(buf);
    }

    void CFG_MSG_SetRates::decodeMsg(uint8_t* buf)
    {
        msgClass = buf[NUM_HEADER_BYTES + 0];
        msgID = buf[NUM_HEADER_BYTES + 1];
        rates[0] = buf[NUM_HEADER_BYTES + 2];
        rates[1] = buf[NUM_HEADER_BYTES + 3];
        rates[2] = buf[NUM_HEADER_BYTES + 4];
        rates[3] = buf[NUM_HEADER_BYTES + 5];
        rates[4] = buf[NUM_HEADER_BYTES + 6];
        rates[5] = buf[NUM_HEADER_BYTES + 7];
    }

    CFG_MSG_SetRate::CFG_MSG_SetRate(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_MSG;
        _dataLen = 3;    // Length of data
        msgClass = 0;
        msgID = 0;
        rate = 0;
    }

    void CFG_MSG_SetRate::encodeMsg(uint8_t* buf)
    {
        buf[NUM_HEADER_BYTES + 0] = msgClass;
        buf[NUM_HEADER_BYTES + 1] = msgID;
        buf[NUM_HEADER_BYTES + 2] = rate;    // Messages rate on current port

        UBXMsg::encodeMsg(buf);
    }

    void CFG_MSG_SetRate::decodeMsg(uint8_t* buf)
    {
        msgClass = buf[NUM_HEADER_BYTES + 0];
        msgID = buf[NUM_HEADER_BYTES + 1];
        rate = buf[NUM_HEADER_BYTES + 2];
    }

    CFG_NAV5_Poll::CFG_NAV5_Poll(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_NAV5;
        _dataLen = 2;    // Length of data

    }

    void CFG_NAV5_Poll::encodeMsg(uint8_t* buf)
    {
        //Poll has no Payload
    }

    void CFG_NAV5_Poll::decodeMsg(uint8_t* buf)
    {
        //Poll has no payload
    }

    CFG_NAV5_EngineSettings::CFG_NAV5_EngineSettings(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_MSG;
        _dataLen = 36;    // Length of data

        //bools for mask
        for (int i = 7; i >= 0; i--)
            ReservedBit0[i] = 0;
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

    void CFG_NAV5_EngineSettings::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode mask
        mask = ReservedBit0[7] << 15 | ReservedBit0[6] << 14 | ReservedBit0[5] << 13 | ReservedBit0[4] << 12 | ReservedBit0[3] << 11 | ReservedBit0[2] << 10 | ReservedBit0[1] << 9
                | ReservedBit0[0] << 8 | dgpsMask << 7 | staticHoldMask << 6 | timeMask << 5 | posMask << 4 | drLim << 3 | posFixMode << 2 | minEl << 1 | dyn;
        temp[0] = mask;
        buf[NUM_HEADER_BYTES + 0] = temp[0];

        //Encode dynMode1
        encodeU1(temp, dynMode1);
        buf[NUM_HEADER_BYTES + 1] = temp[0];

        //Encode fixMode
        encodeU1(temp, fixMode);
        buf[NUM_HEADER_BYTES + 2] = temp[0];

        //Encode fixedAlt
        encodeI4(temp, fixedAlt);
        buf[NUM_HEADER_BYTES + 3] = temp[0];
        buf[NUM_HEADER_BYTES + 4] = temp[1];
        buf[NUM_HEADER_BYTES + 5] = temp[2];
        buf[NUM_HEADER_BYTES + 6] = temp[3];

        //Encode fixedAltVar
        encodeU4(temp, fixedAltVar);
        buf[NUM_HEADER_BYTES + 7] = temp[0];
        buf[NUM_HEADER_BYTES + 8] = temp[1];
        buf[NUM_HEADER_BYTES + 9] = temp[2];
        buf[NUM_HEADER_BYTES + 10] = temp[3];

        //Encode minElev
        encodeI1(temp, minElev);
        buf[NUM_HEADER_BYTES + 11] = temp[0];

        //Encode drLimit
        encodeU1(temp, drLimit);
        buf[NUM_HEADER_BYTES + 12] = temp[0];

        //Encode pDop
        encodeU2(temp, pDop);
        buf[NUM_HEADER_BYTES + 13] = temp[0];
        buf[NUM_HEADER_BYTES + 14] = temp[1];

        //Encode tDop
        encodeU2(temp, tDop);
        buf[NUM_HEADER_BYTES + 15] = temp[0];
        buf[NUM_HEADER_BYTES + 16] = temp[1];

        //Encode pAcc
        encodeU2(temp, pAcc);
        buf[NUM_HEADER_BYTES + 17] = temp[0];
        buf[NUM_HEADER_BYTES + 18] = temp[1];

        //Encode tAcc
        encodeU2(temp, tAcc);
        buf[NUM_HEADER_BYTES + 19] = temp[0];
        buf[NUM_HEADER_BYTES + 20] = temp[1];

        //Encode staticHoldThresh
        encodeU1(temp, staticHoldThresh);
        buf[NUM_HEADER_BYTES + 21] = temp[0];

        //Encode dpgsTimeOut
        encodeU1(temp, dgpsTimeOut);
        buf[NUM_HEADER_BYTES + 22] = temp[0];

        //Encode cnoThreshNumSVs
        encodeU1(temp, cnoThreshNumSVs);
        buf[NUM_HEADER_BYTES + 23] = temp[0];

        //Encode cnoThresh
        encodeU1(temp, cnoThresh);
        buf[NUM_HEADER_BYTES + 24] = temp[0];

        //Encode reserved2
        encodeU2(temp, reserved2);
        buf[NUM_HEADER_BYTES + 25] = temp[0];
        buf[NUM_HEADER_BYTES + 26] = temp[1];

        //Encode reserved3
        encodeU4(temp, reserved3);
        buf[NUM_HEADER_BYTES + 27] = temp[0];
        buf[NUM_HEADER_BYTES + 28] = temp[1];
        buf[NUM_HEADER_BYTES + 29] = temp[2];
        buf[NUM_HEADER_BYTES + 30] = temp[3];

        //Encode reserved4
        encodeU4(temp, reserved4);
        buf[NUM_HEADER_BYTES + 31] = temp[0];
        buf[NUM_HEADER_BYTES + 32] = temp[1];
        buf[NUM_HEADER_BYTES + 33] = temp[2];
        buf[NUM_HEADER_BYTES + 34] = temp[3];
    }

    void CFG_NAV5_EngineSettings::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode mask
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        for (int i = 7; i >= 0; i--)
            ReservedBit0[i] = temp[0] & (1 << (i + 8));
        dgpsMask = temp[0] & (1 << 7);
        staticHoldMask = temp[0] & (1 << 6);
        timeMask = temp[0] & (1 << 5);
        posMask = temp[0] & (1 << 4);
        drLim = temp[0] & (1 << 3);
        posFixMode = temp[0] & (1 << 2);
        minEl = temp[0] & (1 << 1);
        dyn = temp[0] & 1;
        mask = 0;
        for (int i = 0; i > 8; i++)
            mask += ReservedBit0[i];
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
        temp[1] = buf[NUM_HEADER_BYTES + 28];
        temp[2] = buf[NUM_HEADER_BYTES + 29];
        temp[3] = buf[NUM_HEADER_BYTES + 30];
        reserved3 = decodeU4(temp);

        //Decode reserved4
        temp[0] = buf[NUM_HEADER_BYTES + 31];
        temp[1] = buf[NUM_HEADER_BYTES + 32];
        temp[2] = buf[NUM_HEADER_BYTES + 33];
        temp[3] = buf[NUM_HEADER_BYTES + 34];
        reserved4 = decodeU4(temp);
    }

    CFG_NMEA_Poll::CFG_NMEA_Poll(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_NMEA;
        _dataLen = 0;    // Length of data
    }

    void CFG_NMEA_Poll::encodeMsg(uint8_t* buf)
    {
        //No Payload

    }
    void CFG_NMEA_Poll::decodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    CFG_NMEA_Deprecated::CFG_NMEA_Deprecated(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_NMEA;
        _dataLen = 4;    // Length of data

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

    void CFG_NMEA_Deprecated::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode filter
        filter = trackFilt << 5 | gpsOnlyFilter << 4 | dateFilt << 3 | timeFilt << 2 | mskPosFilt << 1 | posFilt;
        temp[0] = filter;
        buf[NUM_HEADER_BYTES + 0] = temp[0];

        //Encode nmeaVersion
        encodeU1(temp, nmeaVersion);
        buf[NUM_HEADER_BYTES + 1] = temp[0];

        //Encode numSV
        encodeU1(temp, numSV);
        buf[NUM_HEADER_BYTES + 2] = temp[0];

        //Encode flags
        flags = consider << 1 | compat;
        temp[0] = flags;
        buf[NUM_HEADER_BYTES + 3] = temp[0];
    }

    void CFG_NMEA_Deprecated::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode filter
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        trackFilt = temp[0] & (1 << 5);
        gpsOnlyFilter = temp[0] & (1 << 4);
        dateFilt = temp[0] & (1 << 3);
        timeFilt = temp[0] & (1 << 2);
        mskPosFilt = temp[0] & (1 << 1);
        posFilt = temp[0] & 1;
        filter = trackFilt + gpsOnlyFilter + dateFilt + timeFilt + mskPosFilt + posFilt;

        //Decode nmeaVersion
        temp[0] = buf[NUM_HEADER_BYTES + 1];
        nmeaVersion = decodeU1(temp);

        //Decode numSV
        temp[0] = buf[NUM_HEADER_BYTES + 2];
        numSV = decodeU1(temp);

        //Decode flags
        temp[0] = buf[NUM_HEADER_BYTES + 3];
        consider = temp[0] & (1 << 1);
        compat = temp[0] & 1;
        flags = consider + compat;
    }

    CFG_NMEA::CFG_NMEA(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_NMEA;
        _dataLen = 12;    // Length of data

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

        //bools for gnssToFilter
        glonass = 0;
        qzss = 0;
        sbas = 0;
        gps = 0;

        svNumbering = 0;
        mainTalkerId = 0;
        gsvTalkerId = 0;
        reserved = 0;

    }

    void CFG_NMEA::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode filter
        filter = trackFilt << 5 | gpsOnlyFilter << 4 | dateFilt << 3 | timeFilt << 2 | mskPosFilt << 1 | posFilt;
        temp[0] = filter;
        buf[NUM_HEADER_BYTES + 0] = temp[0];

        //Encode nmeaVersion
        encodeU1(temp, nmeaVersion);
        buf[NUM_HEADER_BYTES + 1] = temp[0];

        //Encode numSV
        encodeU1(temp, numSV);
        buf[NUM_HEADER_BYTES + 2] = temp[0];

        //Encode flags
        flags = consider << 1 | compat;
        temp[0] = flags;
        buf[NUM_HEADER_BYTES + 3] = temp[0];

        //Encode gnssToFilter
        gnssToFilter = glonass << 5 | qzss << 4 | sbas << 1 | gps;
        temp[0] = gnssToFilter;
        buf[NUM_HEADER_BYTES + 4] = temp[0];

        //Encode svNumbering
        encodeU1(temp, svNumbering);
        buf[NUM_HEADER_BYTES + 5] = temp[0];

        //Encode mainTalkerId
        encodeU1(temp, mainTalkerId);
        buf[NUM_HEADER_BYTES + 6] = temp[0];

        //Encode gsvTalkerId
        encodeU1(temp, gsvTalkerId);
        buf[NUM_HEADER_BYTES + 7] = temp[0];

        //Encode reserved
        encodeU1(temp, reserved);
        buf[NUM_HEADER_BYTES + 8] = temp[0];
    }

    void CFG_NMEA::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode filter
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        trackFilt = temp[0] & (1 << 5);
        gpsOnlyFilter = temp[0] & (1 << 4);
        dateFilt = temp[0] & (1 << 3);
        timeFilt = temp[0] & (1 << 2);
        mskPosFilt = temp[0] & (1 << 1);
        posFilt = temp[0] & 1;
        filter = trackFilt + gpsOnlyFilter + dateFilt + timeFilt + mskPosFilt + posFilt;

        //Decode nmeaVersion
        temp[0] = buf[NUM_HEADER_BYTES + 1];
        nmeaVersion = decodeU1(temp);

        //Decode numSV
        temp[0] = buf[NUM_HEADER_BYTES + 2];
        numSV = decodeU1(temp);

        //Decode flags
        temp[0] = buf[NUM_HEADER_BYTES + 3];
        consider = temp[0] & (1 << 1);
        compat = temp[0] & 1;
        flags = consider + compat;

        //Decode gnssToFilter
        temp[0] = buf[NUM_HEADER_BYTES + 4];
        glonass = temp[0] & (1 << 5);
        qzss = temp[0] & (1 << 4);
        sbas = temp[0] & (1 << 1);
        gps = temp[0] & 1;
        gnssToFilter = glonass + qzss + sbas + gps;

        //Decode svNumbering
        temp[0] = buf[NUM_HEADER_BYTES + 5];
        svNumbering = decodeU1(temp);

        //Decode mainTalkerId
        temp[0] = buf[NUM_HEADER_BYTES + 6];
        mainTalkerId = decodeU1(temp);

        //Decode gsvTalkerId
        temp[0] = buf[NUM_HEADER_BYTES + 7];
        gsvTalkerId = decodeU1(temp);

        //Decode reserved1
        temp[0] = buf[NUM_HEADER_BYTES + 8];
        reserved = decodeU1(temp);
    }

    CFG_PRT_Poll_Used::CFG_PRT_Poll_Used(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_PRT;
        _dataLen = 0;    // Length of data

        //No Payload
    }

    void CFG_PRT_Poll_Used::encodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    void CFG_PRT_Poll_Used::decodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    CFG_PRT_Poll_IOPort::CFG_PRT_Poll_IOPort(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_PRT;
        _dataLen = 1;    // Length of data

        //Message-specific fields

        PortID = 0;
    }

    void CFG_PRT_Poll_IOPort::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode PortID
        encodeU1(temp, PortID);
        buf[NUM_HEADER_BYTES + 0] = temp[0];
    }

    void CFG_PRT_Poll_IOPort::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode PortID
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        PortID = decodeU1(temp);
    }

    CFG_PRT_UART::CFG_PRT_UART(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_PRT;
        _dataLen = 20;    // Length of data

        PortID = 0;
        reserved0 = 0;

        //bools for txReady
        for (int i = 0; i < 9; i++)
            thres[i] = 0;
        for (int i = 0; i < 5; i++)
            pin[i] = 0;
        pol = 0;
        en = 0;

        //bools for mode
        for (int i = 0; i < 2; i++)
            nStopBits[i] = 0;
        for (int i = 0; i < 3; i++)
            parity[i] = 0;
        for (int i = 0; i < 2; i++)
            charLen[i] = 0;
        reserved1 = 0;

        baudRate = 0;

        //bools for inProtoMask
        inRtcm = 0;
        inNmea = 0;
        inUbx = 0;

        //bools for outProtoMask
        outNmea = 0;
        outUbx = 0;

        //bools for flags
        extendedTxTimeout = 0;

        reserved5 = 0;
    }

    void CFG_PRT_UART::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode PortID
        encodeU1(temp, PortID);
        buf[NUM_HEADER_BYTES + 0] = temp[0];

        //Encode reserved0
        encodeU1(temp, reserved0);
        buf[NUM_HEADER_BYTES + 1] = temp[0];

        //Encode txReady
        txReady = thres[8] << 15 | thres[7] << 14 | thres[6] << 13 | thres[5] << 12 | thres[4] << 11 | thres[3] << 10 | thres[2] << 9 | thres[1] << 8 | thres[0] << 7 | pin[4] << 6 | pin[3] << 5
                | pin[2] << 4 | pin[1] << 3 | pin[0] << 2 | pol << 1 | en;
        temp[0] = txReady;
        buf[NUM_HEADER_BYTES + 2] = temp[0];
        // +Praise Jesus+
        //Encode mode
        mode = nStopBits[1] << 13 || nStopBits[0] << 12 | parity[2] << 11 | parity[1] << 10 | parity[0] << 9 | charLen[1] << 7 | charLen[0] << 6 | reserved1 << 4;
        temp[0] = mode;
        buf[NUM_HEADER_BYTES + 3] = temp[0];

        //Encode baudRate
        encodeU4(temp, baudRate);
        buf[NUM_HEADER_BYTES + 4] = temp[0];
        buf[NUM_HEADER_BYTES + 5] = temp[0];
        buf[NUM_HEADER_BYTES + 6] = temp[0];
        buf[NUM_HEADER_BYTES + 7] = temp[0];

        //Encode inProtoMask
        inProtoMask = inRtcm << 2 | inNmea << 1 | inUbx;
        temp[0] = inProtoMask;
        buf[NUM_HEADER_BYTES + 8] = temp[0];

        //Encode outProtoMask
        outProtoMask = outNmea << 1 | outUbx;
        temp[0] = outProtoMask;
        buf[NUM_HEADER_BYTES + 9] = temp[0];

        //Encode flags
        flags = extendedTxTimeout << 1;
        temp[0] = flags;
        buf[NUM_HEADER_BYTES + 10] = temp[0];

        //Encode reserved5
        encodeU2(temp, reserved5);
        buf[NUM_HEADER_BYTES + 11] = temp[0];
        buf[NUM_HEADER_BYTES + 12] = temp[0];
    }

    void CFG_PRT_UART::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode PortID
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        PortID = decodeU1(temp);

        //Decode reserved0
        temp[0] = buf[NUM_HEADER_BYTES + 1];
        reserved0 = decodeU1(temp);

        //Decode txReady
        temp[0] = buf[NUM_HEADER_BYTES + 2];
        txReady = 0;
        for (int i = 0; i < 9; i++)
        {
            thres[8 - i] = temp[0] & (1 << (15 - i));
            txReady = txReady + thres[8 - i];
        }
        for (int i = 0; i < 5; i++)
        {
            pin[4 - i] = temp[0] & (1 << (6 - i));
            txReady = txReady + pin[4 - i];
        }
        pol = temp[0] & (1 << 1);
        en = temp[0] & 1;
        txReady = txReady + pol + en;

        //Decode mode
        temp[0] = buf[NUM_HEADER_BYTES + 3];
        mode = 0;
        for (int i = 0; i < 2; i++)
        {
            nStopBits[1 - i] = temp[0] & (1 << (13 - i));
            mode = mode + nStopBits[1 - i];
        }
        for (int i = 0; i < 3; i++)
        {
            parity[2 - i] = temp[0] & (1 << (11 - i));
            mode = mode + parity[2 - i];
        }
        for (int i = 0; i < 2; i++)
        {
            charLen[1 - i] = temp[0] & (1 << (7 - i));
            mode = mode + charLen[1 - i];
        }
        reserved1 = temp[0] & (1 << 4);
        mode = mode + reserved1;

        //Decode baudRate
        temp[0] = buf[NUM_HEADER_BYTES + 4];
        temp[1] = buf[NUM_HEADER_BYTES + 5];
        temp[2] = buf[NUM_HEADER_BYTES + 6];
        temp[3] = buf[NUM_HEADER_BYTES + 7];
        baudRate = decodeU4(temp);

        //Decode inProtoMask
        temp[0] = buf[NUM_HEADER_BYTES + 8];
        inRtcm = temp[0] & (1 << 2);
        inNmea = temp[0] & (1 << 1);
        inUbx = temp[0] & 1;

        //Decode outProtoMask
        temp[0] = buf[NUM_HEADER_BYTES + 9];
        outNmea = temp[0] & (1 << 1);
        outUbx = temp[0] & 1;
        outProtoMask = outNmea + outUbx;

        //Decode flags
        temp[0] = buf[NUM_HEADER_BYTES + 10];
        extendedTxTimeout = temp[0] & (1 << 1);
        flags = extendedTxTimeout;

        //Decode reserved5
        temp[0] = buf[NUM_HEADER_BYTES + 11];
        temp[1] = buf[NUM_HEADER_BYTES + 12];
        reserved5 = decodeU2(temp);
    }

    CFG_RXM_Poll::CFG_RXM_Poll(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_RXM;
        _dataLen = 0;    // Length of data

        //No Payload
    }

    void CFG_RXM_Poll::encodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    void CFG_RXM_Poll::decodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    CFG_RXM_Configuration::CFG_RXM_Configuration(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_RXM;
        _dataLen = 2;    // Length of data

        reserved1 = 8;    //Note: Always set to 8
        lpmode = 0;
    }

    void CFG_RXM_Configuration::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode reserved1
        encodeU1(temp, reserved1);
        buf[NUM_HEADER_BYTES + 0] = temp[0];

        //Encode lpmode
        encodeU1(temp, lpmode);
        buf[NUM_HEADER_BYTES + 1] = temp[0];
    }

    void CFG_RXM_Configuration::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode reserved1
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        reserved1 = decodeU1(temp);

        //Decode lpmode
        temp[0] = buf[NUM_HEADER_BYTES + 1];
        lpmode = decodeU1(temp);
    }

    CFG_RATE_Poll::CFG_RATE_Poll(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_RATE;
        _dataLen = 0;    // Length of data
    }

    void CFG_RATE_Poll::encodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    void CFG_RATE_Poll::decodeMsg(uint8_t* buf)
    {
        //No Payload
    }

    CFG_RATE_Settings::CFG_RATE_Settings(void)
    {
        _msgClass = CLASS_CFG;
        _msgID = ID_CFG_RATE;
        _dataLen = 6;    // Length of data

        measRate = 0;
        navRate = 0;
        timeRef = 0;
    }

    void CFG_RATE_Settings::encodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Encode measRate
        encodeU2(temp, measRate);
        buf[NUM_HEADER_BYTES + 0] = temp[0];
        buf[NUM_HEADER_BYTES + 1] = temp[1];

        //Encode navRate
        encodeU2(temp, navRate);
        buf[NUM_HEADER_BYTES + 2] = temp[0];
        buf[NUM_HEADER_BYTES + 3] = temp[1];

        //Encode timeRef
        encodeU2(temp, timeRef);
        buf[NUM_HEADER_BYTES + 4] = temp[0];
        buf[NUM_HEADER_BYTES + 5] = temp[1];
    }

    void CFG_RATE_Settings::decodeMsg(uint8_t* buf)
    {
        uint8_t temp[4];

        //Decode measRate
        temp[0] = buf[NUM_HEADER_BYTES + 0];
        temp[1] = buf[NUM_HEADER_BYTES + 1];
        measRate = decodeU2(temp);

        //Decode navRate
        temp[0] = buf[NUM_HEADER_BYTES + 2];
        temp[1] = buf[NUM_HEADER_BYTES + 3];
        navRate = decodeU2(temp);

        //Decode timeRef
        temp[0] = buf[NUM_HEADER_BYTES + 4];
        temp[1] = buf[NUM_HEADER_BYTES + 5];
        timeRef = decodeU2(temp);
    }
}
